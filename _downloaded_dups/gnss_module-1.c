#include "gnss_module.h"
#include "hardware_config.h"
#include "mqtt_config.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "GNSS";

// GNSS 简易状态（供日志/后续逻辑参考）
volatile bool  gnss_has_fix   = false;   // 是否已定位
volatile int   gnss_fix_q     = 0;       // GGA fix质量(0无fix,1GPS,2DGPS,4RTK fix,5RTK float...)
volatile int   gnss_dim       = 0;       // GSA 维度(1无,2=2D,3=3D)
volatile int   gnss_sats      = 0;       // GGA 在用卫星数
volatile float gnss_hdop      = 0.0f;    // GGA/GSA HDOP

// 记录各句型是否已见及状态，用于综合摘要
volatile bool gnss_seen_rmc = false;
volatile bool gnss_seen_gga = false;
volatile bool gnss_seen_gsa = false;
volatile bool gnss_rmc_valid = false; // RMC: A=valid

// TTFF（首次定位用时）测量标志
static volatile bool       gnss_ttff_measuring = false;
static volatile TickType_t gnss_ttff_start     = 0;

// 待机/唤醒可视化：最近完成一行NMEA的时间与累计行数
volatile TickType_t gnss_last_sentence_tick;
volatile uint32_t   gnss_lines_seen;

// PMTK待机ACK检测标志
static volatile bool gnss_ack_pmtk161;
static volatile bool gnss_ack_pmtk225;

// 是否认为已进入待机（用于调试观测）
static volatile bool gnss_in_standby = false;

// UBX ACK/NAK 标志（仅关心 CFG-SLEEP 0x06/0x41）
static volatile bool gnss_ack_cfg_sleep = false;
static volatile bool gnss_nak_cfg_sleep = false;

/* Quectel 二进制协议 ACK（帧头 F1 D9，和 u-blox 的 B5 62 不是一回事）
 * ACK-ACK = 组 0x05 子 0x01，载荷 {被确认的组 ID, 子 ID}
 * ACK-NAK = 组 0x05 子 0x00，载荷同上
 * 见 LC26xZ/LC76xZ GNSS 协议规范 3.3 节 */
static volatile int  gnss_qack_group  = -1;
static volatile int  gnss_qack_sub    = -1;
static volatile bool gnss_qack_is_ack = false;
static volatile bool gnss_qack_seen   = false;

// 基准测试结束后，停止 GNSS 接收
volatile bool gnss_should_stop = false;

// 位置保存结构（供 MQTT 上报/业务循环使用）
gnss_location_t gnss_last_location;  // 最近一次从RMC解析到的位置信息（有效时 valid=true）
gnss_location_t gnss_saved_location; // 每次进入Backup前的快照

// GNSS控制函数
void gnss_enable(bool on)
{
    gpio_set_level(GNSS_EN_PIN, on ? 1 : 0);
}

// RESET_N 受三极管反相：
void gnss_reset_assert(void)   
{ 
    gpio_set_level(GNSS_RST_PIN, 1); // IO2=1 -> Q6导通 -> RESET_N=Low(复位有效)
} 

void gnss_reset_release(void)  
{ 
    gpio_set_level(GNSS_RST_PIN, 0); // IO2=0 -> Q6截止 -> RESET_N=High(释放)
}

void gnss_log_fix_summary_if_changed(void)
{
    // 使用静态快照，只有有变化时才打印，避免刷屏
    static bool  last_has_fix   = false;
    static int   last_fix_q     = -1;
    static int   last_dim       = -1;
    static int   last_sats      = -1;
    static float last_hdop      = -1.0f;
    static bool  last_rmc_valid = false;
    static bool  last_seen_rmc  = false;
    static bool  last_seen_gga  = false;
    static bool  last_seen_gsa  = false;

    bool changed = false;
    if (last_has_fix != gnss_has_fix || last_fix_q != gnss_fix_q || last_dim != gnss_dim ||
        last_sats != gnss_sats || last_hdop != gnss_hdop || last_rmc_valid != gnss_rmc_valid ||
        last_seen_rmc != gnss_seen_rmc || last_seen_gga != gnss_seen_gga || last_seen_gsa != gnss_seen_gsa) {
        changed = true;
    }

    if (!changed) return;

    // 构造摘要
    char rmc_str[32], gga_str[64], gsa_str[32];
    if (gnss_seen_rmc) {
        snprintf(rmc_str, sizeof(rmc_str), "%s", gnss_rmc_valid ? "VALID" : "INVALID");
    } else {
        snprintf(rmc_str, sizeof(rmc_str), "N/A");
    }

    if (gnss_seen_gga) {
        snprintf(gga_str, sizeof(gga_str), "fixQ=%d sats=%d HDOP=%.1f", gnss_fix_q, gnss_sats, gnss_hdop);
    } else {
        snprintf(gga_str, sizeof(gga_str), "N/A");
    }

    if (gnss_seen_gsa) {
        const char* dim_str = (gnss_dim == 3) ? "3D" : (gnss_dim == 2) ? "2D" : "NO";
        snprintf(gsa_str, sizeof(gsa_str), "Dim=%s HDOP=%.1f", dim_str, gnss_hdop);
    } else {
        snprintf(gsa_str, sizeof(gsa_str), "N/A");
    }

    const char* overall = gnss_has_fix ? "FIXED" : "NO FIX";
    GNSS_PARSE_LOG("FIX SUMMARY: RMC=%s | GGA:%s | GSA:%s => %s", rmc_str, gga_str, gsa_str, overall);

    // 更新快照
    last_has_fix   = gnss_has_fix;
    last_fix_q     = gnss_fix_q;
    last_dim       = gnss_dim;
    last_sats      = gnss_sats;
    last_hdop      = gnss_hdop;
    last_rmc_valid = gnss_rmc_valid;
    last_seen_rmc  = gnss_seen_rmc;
    last_seen_gga  = gnss_seen_gga;
    last_seen_gsa  = gnss_seen_gsa;
}

void gnss_reset_state_for_ttff(void)
{
    gnss_has_fix   = false;
    gnss_fix_q     = 0;
    gnss_dim       = 0;
    gnss_sats      = 0;
    gnss_hdop      = 0.0f;

    gnss_seen_rmc  = false;
    gnss_seen_gga  = false;
    gnss_seen_gsa  = false;
    gnss_rmc_valid = false;

    gnss_ttff_measuring = true;
    gnss_ttff_start     = xTaskGetTickCount();
}

void gnss_report_ttff_if_needed(void)
{
    if (!gnss_ttff_measuring || !gnss_has_fix) return;
    if (gnss_ttff_start == 0) return;
    uint32_t ms = (uint32_t)((xTaskGetTickCount() - gnss_ttff_start) * 1000 / configTICK_RATE_HZ);
    if (ms > 0) {
        ESP_LOGI(TAG, "TTFF: %u ms (%.2f s)", (unsigned)ms, ms / 1000.0f);
        gnss_ttff_measuring = false;
    }
}

// 发送一条 NMEA 命令到 GNSS（自动追加 \r\n）
void gnss_send_nmea(const char* sentence)
{
    if (!sentence) return;
    uart_write_bytes(GNSS_UART_NUM, sentence, strlen(sentence));
    uart_write_bytes(GNSS_UART_NUM, "\r\n", 2);
}

/* ===================== Quectel PAIR 协议 =====================
 * LC760Z 属于 LC76G 系列（Airoha 平台），低功耗用的是 $PAIR NMEA 命令，
 * 不是 u-blox 的 UBX 二进制。本文件早期那套 gnss_send_quectel_bin(0x06,0x41)
 * 是从别的模组带过来的，这颗模组不认，已停用。
 *
 *   $PAIR003        关 GNSS 子系统，CPU 进 Standby
 *   $PAIR002        从 PAIR003 恢复
 *   $PAIR650,<秒>   进 Backup/RTC 模式，13 µA @3.3V，星历保留，热启动 1 s
 *                   参数 0 = 一直睡，靠 RESET_N 唤醒
 *   $PAIR001,<cmd>,<result>   模组对以上命令的 ACK，result 0 = OK
 * ============================================================ */

// $PAIR001 ACK 捕获（由 gnss_task 的行解析写入）
static volatile int  gnss_pair_ack_cmd    = -1;
static volatile int  gnss_pair_ack_result = -1;
static volatile bool gnss_pair_ack_seen   = false;

/* 专有语句（$P 开头）捕获。协议探测时用来接收任意厂商的回应，
 * 不需要预先知道 ACK 的确切格式。 */
static volatile uint32_t gnss_prop_count = 0;
static char              gnss_last_prop[96];
// 见过哪些专有前缀
static volatile bool gnss_seen_pmtk = false;
static volatile bool gnss_seen_pair = false;
static volatile bool gnss_seen_pcas = false;
static volatile bool gnss_seen_pq   = false;
// 见过哪些 talker ID（$GN/$GP/$GL/$GA/$BD...）
static char gnss_talkers[8][3];
static volatile int gnss_talker_n = 0;

// NMEA 校验和：'$' 与 '*' 之间所有字符的异或
static uint8_t nmea_checksum(const char* body)
{
    uint8_t cs = 0;
    for (const char* p = body; *p; ++p) { cs = (uint8_t)(cs ^ (uint8_t)(*p)); }
    return cs;
}

// 发送 $<body>*<CS>\r\n，body 不含 '$'、'*' 和校验和（对外版本，供协议探测使用）
void gnss_send_nmea_cs(const char* body)
{
    char out[96];
    int n = snprintf(out, sizeof(out), "$%s*%02X\r\n", body, nmea_checksum(body));
    if (n <= 0 || n >= (int)sizeof(out)) {
        ESP_LOGE(TAG, "NMEA command too long: %s", body);
        return;
    }
    uart_write_bytes(GNSS_UART_NUM, out, (size_t)n);
}



// 发送 UBX 二进制消息（B5 62 + cls + id + len + payload + CK_A CK_B）
void gnss_send_ubx(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len)
{
    uint8_t hdr[6];
    hdr[0] = 0xB5; hdr[1] = 0x62; hdr[2] = cls; hdr[3] = id; hdr[4] = (uint8_t)(len & 0xFF); hdr[5] = (uint8_t)(len >> 8);
    uint8_t ckA = 0, ckB = 0;
    // 校验覆盖 cls/id/len/payload（不含 0xB5 0x62）
    for (int i = 2; i < 6; ++i) { ckA = (uint8_t)(ckA + hdr[i]); ckB = (uint8_t)(ckB + ckA); }
    for (uint16_t i = 0; i < len; ++i)      { ckA = (uint8_t)(ckA + payload[i]); ckB = (uint8_t)(ckB + ckA); }
    // 打印即将发送的 UBX 帧
    char hex[16 * 3]; int pos = 0;
    GNSS_UBX_LOG("TX UBX cls=0x%02X id=0x%02X len=%u", cls, id, (unsigned)len);
    // 头
    for (int i = 0; i < 6 && pos < (int)sizeof(hex) - 3; ++i) pos += snprintf(hex + pos, sizeof(hex) - pos, "%02X ", hdr[i]);
    GNSS_UBX_LOG("  hdr: %s", hex);
    // 负载
    if (len && payload) {
        pos = 0;
        for (uint16_t i = 0; i < len && pos < (int)sizeof(hex) - 3; ++i) pos += snprintf(hex + pos, sizeof(hex) - pos, "%02X ", payload[i]);
        GNSS_UBX_LOG("  payload: %s", hex);
    }
    // 校验
    GNSS_UBX_LOG("  cks: %02X %02X", ckA, ckB);

    // 发送
    uart_write_bytes(GNSS_UART_NUM, (const char*)hdr, sizeof(hdr));
    if (len && payload) uart_write_bytes(GNSS_UART_NUM, (const char*)payload, len);
    uint8_t cks[2] = { ckA, ckB };
    uart_write_bytes(GNSS_UART_NUM, (const char*)cks, 2);
}

// 发送 Quectel 专用二进制消息（F1 D9 + cls + id + len + payload + CK_A CK_B）
void gnss_send_quectel_bin(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len)
{
    uint8_t hdr[6];
    hdr[0] = 0xF1; hdr[1] = 0xD9; hdr[2] = cls; hdr[3] = id; hdr[4] = (uint8_t)(len & 0xFF); hdr[5] = (uint8_t)(len >> 8);
    uint8_t ckA = 0, ckB = 0;
    for (int i = 2; i < 6; ++i) { ckA = (uint8_t)(ckA + hdr[i]); ckB = (uint8_t)(ckB + ckA); }
    for (uint16_t i = 0; i < len; ++i)      { ckA = (uint8_t)(ckA + payload[i]); ckB = (uint8_t)(ckB + ckA); }

    // 调试打印
    GNSS_UBX_LOG("TX QBIN cls=0x%02X id=0x%02X len=%u", cls, id, (unsigned)len);
    {
        char hex[16 * 3]; int pos = 0;
        for (int i = 0; i < 6 && pos < (int)sizeof(hex) - 3; ++i) pos += snprintf(hex + pos, sizeof(hex) - pos, "%02X ", hdr[i]);
        GNSS_UBX_LOG("  hdr: %s", hex);
        if (len && payload) {
            pos = 0;
            for (uint16_t i = 0; i < len && pos < (int)sizeof(hex) - 3; ++i) pos += snprintf(hex + pos, sizeof(hex) - pos, "%02X ", payload[i]);
            GNSS_UBX_LOG("  payload: %s", hex);
        }
        GNSS_UBX_LOG("  cks: %02X %02X", ckA, ckB);
    }

    // 发送
    uart_write_bytes(GNSS_UART_NUM, (const char*)hdr, sizeof(hdr));
    if (len && payload) uart_write_bytes(GNSS_UART_NUM, (const char*)payload, len);
    uint8_t cks[2] = { ckA, ckB };
    uart_write_bytes(GNSS_UART_NUM, (const char*)cks, 2);
}

// 构造并发送 Quectel CFG-SLEEP（0x06,0x41）：payload = sleep_ms(LE,4B) + lp_mode(1B)
static inline void gnss_send_cfg_sleep_quectel(uint32_t sleep_ms, uint8_t low_power_mode)
{
    uint8_t p[5];
    p[0] = (uint8_t)(sleep_ms & 0xFF);
    p[1] = (uint8_t)((sleep_ms >> 8) & 0xFF);
    p[2] = (uint8_t)((sleep_ms >> 16) & 0xFF);
    p[3] = (uint8_t)((sleep_ms >> 24) & 0xFF);
    p[4] = low_power_mode; // 1=Standby, 3=Backup
    gnss_send_quectel_bin(0x06, 0x41, p, 5);
}

bool gnss_wait_for_quiet2(uint32_t quiet_ms, uint32_t timeout_ms)
{
    TickType_t start = xTaskGetTickCount();
    while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(timeout_ms)) {
        TickType_t since = xTaskGetTickCount() - gnss_last_sentence_tick;
        if (since >= pdMS_TO_TICKS(quiet_ms)) {
            return true;
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    return false;
}

bool gnss_wait_for_activity2(uint32_t timeout_ms)
{
    uint32_t start_lines = gnss_lines_seen;
    TickType_t start = xTaskGetTickCount();
    while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(timeout_ms)) {
        if (gnss_lines_seen != start_lines) {
            return true;
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    return false;
}

/* ---- 方案 E：VCC 保持供电，模组进 Backup（13 µA），靠 RESET_N 唤醒热启动 ----
 * 保持 U21 (LP5907) 使能不变，所以 VCC 和 V_BCKP 都还在，星历不丢。
 * 相比彻底断电（冷启动 ~28 s @36 mA），13 µA 的代价远小于省下的搜星时间。 */

/* 兜底默认值：万一 hardware_config.h 没同步到，这里也能编过。
 * CFG-SLEEP 执行后模组立刻休眠，本来就发不出 ACK（同手册里 CFG-SIMPLERST
 * 的软件复位"系统命令，无 ACK"），所以等一小会儿就够。 */
#ifndef GNSS_SLEEP_ACK_TIMEOUT_MS
#define GNSS_SLEEP_ACK_TIMEOUT_MS 500
#endif
#ifndef GNSS_QBIN_ACK_TIMEOUT_MS
#define GNSS_QBIN_ACK_TIMEOUT_MS 1500
#endif
#ifndef GNSS_WAKE_TIMEOUT_MS
#define GNSS_WAKE_TIMEOUT_MS 5000
#endif
#ifndef GNSS_LP_USE_BACKUP
#define GNSS_LP_USE_BACKUP 1
#endif
#ifndef GNSS_BACKUP_SLEEP_SEC
#define GNSS_BACKUP_SLEEP_SEC 0
#endif

// 等待 Quectel 二进制 ACK-ACK（组/子 ID 匹配）
static bool gnss_wait_quectel_ack(uint8_t group, uint8_t sub, uint32_t timeout_ms)
{
    TickType_t t0 = xTaskGetTickCount();
    while ((xTaskGetTickCount() - t0) < pdMS_TO_TICKS(timeout_ms)) {
        if (gnss_qack_seen && gnss_qack_group == group && gnss_qack_sub == sub) {
            uint32_t ms = (uint32_t)((xTaskGetTickCount() - t0) * portTICK_PERIOD_MS);
            if (gnss_qack_is_ack) {
                ESP_LOGI(TAG, "[QBIN] ACK for %02X/%02X in %lu ms", group, sub, (unsigned long)ms);
                return true;
            }
            ESP_LOGW(TAG, "[QBIN] NAK for %02X/%02X in %lu ms", group, sub, (unsigned long)ms);
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    ESP_LOGW(TAG, "[QBIN] no ACK for %02X/%02X within %lu ms",
             group, sub, (unsigned long)timeout_ms);
    return false;
}

// 发送 CFG-SLEEP (0x06 0x41)：载荷 = uint32 休眠毫秒(0=一直睡) + uint8 模式(1=Standby,3=Backup)
static void gnss_send_cfg_sleep(uint32_t sleep_ms, uint8_t mode)
{
    uint8_t p[5];
    p[0] = (uint8_t)(sleep_ms & 0xFF);
    p[1] = (uint8_t)((sleep_ms >> 8) & 0xFF);
    p[2] = (uint8_t)((sleep_ms >> 16) & 0xFF);
    p[3] = (uint8_t)((sleep_ms >> 24) & 0xFF);
    p[4] = mode;
    gnss_qack_seen = false;
    gnss_qack_group = gnss_qack_sub = -1;
    gnss_ack_cfg_sleep = false;
    gnss_nak_cfg_sleep = false;
    ESP_LOGI(TAG, "[QBIN] TX -> CFG-SLEEP (06/41) sleep_ms=%lu mode=%u (%s)",
             (unsigned long)sleep_ms, mode,
             (mode == 3) ? "Backup" : (mode == 1) ? "Standby" : "?");
    gnss_send_quectel_bin(0x06, 0x41, p, sizeof(p));
}

// 使能星历自动存 Flash（CFG-EPHSAVE 0x06 0x10），让 Backup 唤醒后能热启动
bool gnss_enable_ephemeris_save(void)
{
    uint8_t p[1] = { 1 };
    gnss_qack_seen = false;
    ESP_LOGI(TAG, "[QBIN] TX -> CFG-EPHSAVE (06/10) enable=1");
    gnss_send_quectel_bin(0x06, 0x10, p, sizeof(p));
    return gnss_wait_quectel_ack(0x06, 0x10, GNSS_QBIN_ACK_TIMEOUT_MS);
}

bool gnss_ensure_standby_fast(void)
{
    /* 探测窗口：GNSS 在跑的时候是 26~56 句/秒，400 ms 能看到 10~22 句，
     * 判「有没有在输出」绰绰有余。跳过轮次里我们从头到尾没碰过 GNSS，
     * 它本该一直在 Backup —— 这里只是花 400 ms 把这件事确认掉，
     * 而不是花 4 s 走完整的进 Backup 流程。 */
    uint32_t pre = gnss_lines_seen;
    vTaskDelay(pdMS_TO_TICKS(GNSS_STANDBY_PROBE_MS));
    uint32_t n = gnss_lines_seen - pre;

    if (n == 0) {
        gnss_in_standby = true;
        ESP_LOGI(TAG, "GNSS 探测 %d ms 无输出 -> 仍在 Backup，跳过 CFG-SLEEP",
                 GNSS_STANDBY_PROBE_MS);
        return true;
    }

    /* 它醒着 —— 不管为什么，必须放回 Backup，否则整个睡眠窗口 40 mA。 */
    ESP_LOGW(TAG, "GNSS 探测到 %lu 句/%d ms -> 它是醒的，走完整流程放回 Backup",
             (unsigned long)n, GNSS_STANDBY_PROBE_MS);
    return gnss_enter_standby2();
}

bool gnss_enter_standby2(void)
{
    gnss_in_standby = false;

    const uint8_t mode = GNSS_LP_USE_BACKUP ? 3 : 1;
    ESP_LOGI(TAG, "===== GNSS low-power: entering %s =====",
             GNSS_LP_USE_BACKUP ? "BACKUP (CFG-SLEEP mode=3)" : "STANDBY (CFG-SLEEP mode=1)");
    ESP_LOGI(TAG, "  VCC stays ON (IO%d unchanged) so ephemeris/RTC survive", GNSS_EN_PIN);

    /* 判据说明：
     * CFG-SLEEP 执行后模组立刻睡过去，**发不出 ACK**——和手册里 CFG-SIMPLERST
     * 的软件复位一样是"系统命令，无 ACK"。所以不能拿 ACK 当必要条件。
     * 真正可靠的判据是**状态转变**：发命令前模组在持续输出，发完彻底停下。
     * 这样既能确认命令生效，又能和"模组本来就不说话"的假阳性区分开。 */

    // 1) 先量一下发命令之前的输出速率
    uint32_t pre0 = gnss_lines_seen;
    vTaskDelay(pdMS_TO_TICKS(1000));
    uint32_t rate_before = gnss_lines_seen - pre0;
    ESP_LOGI(TAG, "  发送前输出速率  : %lu 句/秒", (unsigned long)rate_before);

    uint32_t lines_before = gnss_lines_seen;
    TickType_t t0 = xTaskGetTickCount();

    // 2) 发命令。ACK 等一小会儿即可，收不到是预期行为
    gnss_send_cfg_sleep(GNSS_BACKUP_SLEEP_SEC * 1000U, mode);
    bool acked = gnss_wait_quectel_ack(0x06, 0x41, GNSS_SLEEP_ACK_TIMEOUT_MS);

    // 3) 确认真的安静下来，并持续安静
    bool quiet = gnss_wait_for_quiet2(1500, 5000);
    uint32_t lines_after_quiet = gnss_lines_seen;
    vTaskDelay(pdMS_TO_TICKS(2000));
    bool stays_quiet = (gnss_lines_seen == lines_after_quiet);
    uint32_t elapsed = (uint32_t)((xTaskGetTickCount() - t0) * portTICK_PERIOD_MS);

    ESP_LOGI(TAG, "----- GNSS low-power entry report -----");
    ESP_LOGI(TAG, "  发送前在输出    : %s (%lu 句/秒)   <== 关键项 1",
             rate_before > 0 ? "YES" : "NO", (unsigned long)rate_before);
    ESP_LOGI(TAG, "  发送后已静默    : %s              <== 关键项 2",
             (quiet && stays_quiet) ? "YES" : "NO");
    ESP_LOGI(TAG, "  ACK from module : %s (CFG-SLEEP 本就不回 ACK，仅供参考)",
             acked ? "YES" : "NO");
    ESP_LOGI(TAG, "  NMEA lines before/after : %lu / %lu",
             (unsigned long)lines_before, (unsigned long)gnss_lines_seen);
    ESP_LOGI(TAG, "  elapsed         : %lu ms", (unsigned long)elapsed);

    if (rate_before == 0) {
        // 模组本来就不出数据 —— 老代码的假阳性就是这个情况
        gnss_in_standby = false;
        ESP_LOGW(TAG, "  RESULT          : UNCONFIRMED - 发命令前模组就没在输出，");
        ESP_LOGW(TAG, "                    静默说明不了任何问题，不能算进入低功耗");
    } else if (quiet && stays_quiet) {
        gnss_in_standby = true;
        ESP_LOGI(TAG, "  RESULT          : ENTERED - 输出从 %lu 句/秒降到 0，命令生效",
                 (unsigned long)rate_before);
        ESP_LOGI(TAG, "                    此时量 VCC_LC760Z 应为 uA 量级");
    } else {
        gnss_in_standby = false;
        ESP_LOGW(TAG, "  RESULT          : FAILED - 模组仍在输出，命令没被接受");
    }
    ESP_LOGI(TAG, "---------------------------------------");
    return gnss_in_standby;
}

/* ==================== GNSS 低功耗协议探测 ====================
 * 目的：让模组自己告诉我们它认哪套命令，而不是靠猜。
 * 做法是"发一条、看有没有任何 $P 回应、看 NMEA 有没有停"，
 * 不依赖预先知道各厂商 ACK 的确切格式。
 * 约 40 秒跑完一轮，全部结论打在串口上。
 * =========================================================== */

typedef struct {
    const char *body;      // 不含 $ 和校验和
    const char *vendor;
    const char *desc;
} gnss_probe_cmd_t;

/* 版本/身份查询。LC26xZ/LC76xZ 用的是 PQTM 专有语句，
 * 其余几条留着做交叉验证——如果哪天换了模组，一眼能看出是哪家的。 */
static const gnss_probe_cmd_t s_probe_version[] = {
    { "PQTMVERNO",  "Quectel LC26xZ/LC76xZ", "查询版本（本板应答这条）" },
    { "PMTK605",    "MediaTek",              "查询固件版本" },
    { "PAIR021",    "Airoha/Quectel LC76G",  "查询固件版本" },
    { "PCAS06,0",   "CASIC/Allystar",        "查询版本信息" },
};

// 发一条命令，等 timeout_ms，返回是否收到任何新的 $P 回应
static bool probe_send_and_listen(const char *body, uint32_t timeout_ms, char *resp, size_t resp_sz)
{
    uint32_t prop_before = gnss_prop_count;
    gnss_last_prop[0] = '\0';

    gnss_send_nmea_cs(body);

    TickType_t t0 = xTaskGetTickCount();
    while ((xTaskGetTickCount() - t0) < pdMS_TO_TICKS(timeout_ms)) {
        if (gnss_prop_count != prop_before) {
            vTaskDelay(pdMS_TO_TICKS(200));   // 再等等，可能还有后续句
            if (resp && resp_sz) {
                strncpy(resp, gnss_last_prop, resp_sz - 1);
                resp[resp_sz - 1] = '\0';
            }
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    if (resp && resp_sz) { resp[0] = '\0'; }
    return false;
}

void gnss_lp_probe_run(void)
{
    char resp[96];

    ESP_LOGI(TAG, " ");
    ESP_LOGI(TAG, "============================================================");
    ESP_LOGI(TAG, "         GNSS 低功耗协议探测 (GNSS Low-Power Probe)");
    ESP_LOGI(TAG, "============================================================");

    /* ---- 1/4 观察模组在说什么 ---- */
    ESP_LOGI(TAG, "[1/4] 观察模组输出 5 秒...");
    gnss_talker_n = 0;
    gnss_seen_pmtk = gnss_seen_pair = gnss_seen_pcas = gnss_seen_pq = false;
    uint32_t lines0 = gnss_lines_seen;
    uint32_t prop0  = gnss_prop_count;
    vTaskDelay(pdMS_TO_TICKS(5000));
    uint32_t lines = gnss_lines_seen - lines0;
    uint32_t props = gnss_prop_count - prop0;

    ESP_LOGI(TAG, "      NMEA 句数        : %lu (%.1f 句/秒)",
             (unsigned long)lines, lines / 5.0f);
    if (lines == 0) {
        ESP_LOGE(TAG, "      模组没有任何输出，后面的探测没有意义。");
        ESP_LOGE(TAG, "      先查 VCC_LC760Z / RESET_N / R34 / R36。");
        ESP_LOGI(TAG, "============================================================");
        return;
    }
    char talkers[32] = {0};
    int tp = 0;
    for (int k = 0; k < gnss_talker_n && tp < (int)sizeof(talkers) - 4; ++k) {
        tp += snprintf(talkers + tp, sizeof(talkers) - tp, "%s ", gnss_talkers[k]);
    }
    ESP_LOGI(TAG, "      Talker ID        : %s", talkers[0] ? talkers : "(none)");
    ESP_LOGI(TAG, "      专有语句($P)     : %lu 条  PMTK=%d PAIR=%d PCAS=%d PQ=%d",
             (unsigned long)props, gnss_seen_pmtk, gnss_seen_pair, gnss_seen_pcas, gnss_seen_pq);
    if (gnss_last_prop[0]) {
        ESP_LOGI(TAG, "      最后一条专有语句 : %s", gnss_last_prop);
    }

    /* ---- 2/4 问它是谁 ---- */
    ESP_LOGI(TAG, " ");
    ESP_LOGI(TAG, "[2/4] 版本/身份查询（看哪家的命令有回应）");
    const char *vendor_guess = NULL;
    for (unsigned i = 0; i < sizeof(s_probe_version)/sizeof(s_probe_version[0]); ++i) {
        const gnss_probe_cmd_t *c = &s_probe_version[i];
        bool got = probe_send_and_listen(c->body, 1500, resp, sizeof(resp));
        ESP_LOGI(TAG, "      $%-12s (%-16s) -> %s",
                 c->body, c->vendor, got ? resp : "无回应");
        if (got && !vendor_guess) { vendor_guess = c->vendor; }
        vTaskDelay(pdMS_TO_TICKS(300));
    }

    /* ---- 3/4 星历存储 + 二进制 CFG-SLEEP 实测 ---- */
    ESP_LOGI(TAG, " ");
    ESP_LOGI(TAG, "[3/4] 二进制 CFG-SLEEP (0x06 0x41) 实测");

    ESP_LOGI(TAG, "      先使能星历存 Flash（热启动的前提）");
    bool eph_ok = gnss_enable_ephemeris_save();
    ESP_LOGI(TAG, "      CFG-EPHSAVE ACK  : %s", eph_ok ? "YES" : "NO");
    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_LOGI(TAG, "      进入 %s 模式...", GNSS_LP_USE_BACKUP ? "Backup(3)" : "Standby(1)");
    bool entered = gnss_enter_standby2();

    uint32_t wake_ms = 0;
    bool woke = false;
    if (entered) {
        vTaskDelay(pdMS_TO_TICKS(3000));      // 睡一会儿，方便你这时候量电流
        TickType_t tw = xTaskGetTickCount();
        woke = gnss_wake_from_standby2();
        wake_ms = (uint32_t)((xTaskGetTickCount() - tw) * portTICK_PERIOD_MS);

        // 唤醒后测一次重新出定位的时间，用来判断星历有没有保住
        ESP_LOGI(TAG, "      等待唤醒后重新定位（最多 60 秒，用于判断热/冷启动）...");
        gnss_last_location.valid = false;
        TickType_t tf = xTaskGetTickCount();
        while ((xTaskGetTickCount() - tf) < pdMS_TO_TICKS(60000)) {
            if (gnss_last_location.valid) { break; }
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        uint32_t fix_ms = (uint32_t)((xTaskGetTickCount() - tf) * portTICK_PERIOD_MS);
        if (gnss_last_location.valid) {
            ESP_LOGI(TAG, "      重新定位耗时     : %lu ms  (<5s 说明是热启动，星历保住了)",
                     (unsigned long)fix_ms);
        } else {
            ESP_LOGW(TAG, "      60 秒内没有重新定位（可能在室内，或星历丢失走了冷启动）");
        }
    }

    /* ---- 4/4 结论 ---- */
    ESP_LOGI(TAG, " ");
    ESP_LOGI(TAG, "[4/4] 结论");
    ESP_LOGI(TAG, "      平台推测         : %s",
             vendor_guess ? vendor_guess : "NMEA 查询无应答（本系列正常，它主要用二进制协议）");
    ESP_LOGI(TAG, "      CFG-EPHSAVE      : %s", eph_ok ? "ACK" : "无 ACK");
    ESP_LOGI(TAG, "      CFG-SLEEP 进入   : %s", entered ? "成功（收到 ACK 且串口静默）" : "失败");
    if (entered) {
        ESP_LOGI(TAG, "      唤醒             : %s, %lu ms",
                 woke ? "成功" : "失败", (unsigned long)wake_ms);
        ESP_LOGI(TAG, "      >>> 低功耗链路打通。睡眠期量 VCC_LC760Z 应为 uA 量级");
    } else {
        ESP_LOGW(TAG, "      >>> 没进去。看上面 [QBIN] 那几行：有没有收到 ACK-ACK/ACK-NAK");
    }
    ESP_LOGI(TAG, "============================================================");
    ESP_LOGI(TAG, " ");
}

bool gnss_wake_from_standby2(void)
{
    ESP_LOGI(TAG, "===== GNSS wake-up =====");
    TickType_t t0 = xTaskGetTickCount();
    uint32_t lines_before = gnss_lines_seen;
    const char *method = "";

#if GNSS_LP_USE_BACKUP
    /* 手册 3.4.10 备注：Backup 模式下模块只能通过休眠时间结束或硬件重启唤醒。
     * 我们把休眠时间设为 0（一直睡），所以只能靠 RESET_N。 */
    method = "RESET_N pulse";
    ESP_LOGI(TAG, "  method: %s on IO%d (>=100ms low) - Backup 只能硬件复位唤醒",
             method, GNSS_RST_PIN);
    gnss_reset_assert();
    vTaskDelay(pdMS_TO_TICKS(120));
    gnss_reset_release();
    vTaskDelay(pdMS_TO_TICKS(10));
    uart_flush_input(GNSS_UART_NUM);
#else
    /* 手册 3.4.10 备注：Standby 模式可通过 UART 发送任意命令唤醒（靠电平变化），
     * 也可以硬件复位。先试 UART，不行再复位。 */
    method = "UART wake byte";
    ESP_LOGI(TAG, "  method: %s (Standby 可由 UART 电平变换唤醒)", method);
    const uint8_t wake_seq[3] = {0xFF, '\r', '\n'};
    uart_write_bytes(GNSS_UART_NUM, (const char*)wake_seq, sizeof(wake_seq));
    vTaskDelay(pdMS_TO_TICKS(200));
    if (!gnss_wait_for_activity2(1000)) {
        method = "UART wake byte + RESET_N";
        ESP_LOGI(TAG, "  UART 唤醒无响应，改用 RESET_N 脉冲");
        gnss_reset_assert();
        vTaskDelay(pdMS_TO_TICKS(120));
        gnss_reset_release();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    uart_flush_input(GNSS_UART_NUM);
#endif

    bool ok = gnss_wait_for_activity2(GNSS_WAKE_TIMEOUT_MS);
    uint32_t ms = (uint32_t)((xTaskGetTickCount() - t0) * portTICK_PERIOD_MS);

    ESP_LOGI(TAG, "----- GNSS wake-up report -----");
    ESP_LOGI(TAG, "  method          : %s", method);
    ESP_LOGI(TAG, "  NMEA resumed    : %s", ok ? "YES" : "NO");
    ESP_LOGI(TAG, "  time to 1st line: %lu ms", (unsigned long)ms);
    ESP_LOGI(TAG, "  NMEA lines before/after : %lu / %lu",
             (unsigned long)lines_before, (unsigned long)gnss_lines_seen);
    ESP_LOGI(TAG, "-------------------------------");

    if (ok) {
        gnss_in_standby = false;
        return true;
    }
    ESP_LOGW(TAG, "No NMEA after wake within %d ms", GNSS_WAKE_TIMEOUT_MS);
    return false;
}

bool gnss_enter_backup_via_cfg(void)
{
    ESP_LOGI(TAG, "Send Quectel CFG-SLEEP (0x06,0x41): sleep_ms=0, mode=3 (Backup)");
    gnss_ack_cfg_sleep = false;
    gnss_nak_cfg_sleep = false;
    gnss_send_cfg_sleep_quectel(0 /*forever*/, 3 /*Backup*/);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    
    if (gnss_wait_for_quiet2(1500, 5000)) {
        uint32_t lines_before = gnss_lines_seen;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        if (gnss_lines_seen == lines_before) {
            ESP_LOGI(TAG, "Backup entered (UART quiet and stays quiet)");
            gnss_in_standby = true;
            return true;
        }
        ESP_LOGW(TAG, "Quiet detected after CFG-SLEEP but activity resumed -> not backup");
    } else {
        ESP_LOGW(TAG, "No quiet after CFG-SLEEP within timeout");
    }
    return false;
}

void gnss_exit_backup_via_reset(void)
{
    ESP_LOGI(TAG, "Exit Backup via RESET_N low pulse (>=100ms), inverted driver on IO%d", GNSS_RST_PIN);
    gnss_reset_assert();
    vTaskDelay(120 / portTICK_PERIOD_MS); // 稍高于 100ms
    gnss_reset_release();
    vTaskDelay(10 / portTICK_PERIOD_MS);  // 释放后保持 ≥10ms
    uart_flush_input(GNSS_UART_NUM);
    gnss_in_standby = false;
}

// 将 NMEA 行按逗号分割为字段，保留空字段
static int nmea_split_fields(char* s, char* fields[], int max_fields)
{
    if (!s || max_fields <= 0) return 0;
    int count = 0;
    char* p = s;
    fields[count++] = p;
    for (char* q = s; *q != '\0'; ++q) {
        if (*q == '*') { // 截断校验和部分
            *q = '\0';
            break;
        }
        if (*q == ',') {
            *q = '\0';
            if (count < max_fields) fields[count++] = q + 1; // 允许空字段
        }
    }
    return count;
}

static double nmea_coord_to_deg(const char* s, char hemi, bool is_lat)
{
    if (!s || !*s) return 0.0;
    double v = atof(s);
    if (v == 0.0) return 0.0;
    // ddmm.mmmm（lat: dd；lon: ddd）
    int deg = (int)(v / 100.0);
    double minutes = v - (deg * 100.0);
    double deg_val = deg + minutes / 60.0;
    if ((is_lat && (hemi == 'S')) || (!is_lat && (hemi == 'W'))) deg_val = -deg_val;
    return deg_val;
}

void gnss_parse_and_log_rmc(const char* nmea)
{
    // 查找以 '$' 开头、任意两字母 talker、紧随其后的 "RMC," 的句子
    const char* rmc = NULL;
    for (const char* p = nmea; (p = strchr(p, '$')) != NULL; ++p) {
        if (p[1] && p[2] && p[3] && p[4] && p[5] && p[6]) {
            if (p[3] == 'R' && p[4] == 'M' && p[5] == 'C' && p[6] == ',') { rmc = p; break; }
        } else {
            break;
        }
    }
    if (!rmc) return;

    // 拷贝一份行缓冲
    char line[128];
    size_t i = 0;
    while (rmc[i] && rmc[i] != '\n' && i < sizeof(line) - 1) { line[i] = rmc[i]; i++; }
    line[i] = '\0';

    // 用自定义分割（保留空字段）
    char tmp[128];
    strncpy(tmp, line, sizeof(tmp)-1); tmp[sizeof(tmp)-1] = '\0';
    char* fields[20] = {0};
    int count = nmea_split_fields(tmp, fields, 20);
    if (count <= 8) return; // 至少需要到索引8

    const char* time_utc = fields[1];
    const char* status   = fields[2];
    const char* lat      = fields[3];
    const char* lat_hemi = fields[4];
    const char* lng      = fields[5];
    const char* lng_hemi = fields[6];
    const char* spd_kn   = fields[7];
    const char* crs_deg  = fields[8];

    GNSS_PARSE_LOG("RMC: time=%s status=%s lat=%s%s lng=%s%s spd(kn)=%s crs=%s", 
             time_utc, status, lat, lat_hemi, lng, lng_hemi, spd_kn, crs_deg);

    // RMC 状态：A=有效，V=无效
    bool rmc_fix = (status && status[0] == 'A');
    static bool last_rmc_fix = false;
    if (rmc_fix != last_rmc_fix) {
        ESP_LOGI(TAG, "RMC fix state changed: %s", rmc_fix ? "FIXED" : "NO FIX");
        last_rmc_fix = rmc_fix;
    }
    if (rmc_fix) {
        gnss_has_fix = true;
        // 解析坐标/速度/航向并保存
        if (lat && *lat && lat_hemi && *lat_hemi && lng && *lng && lng_hemi && *lng_hemi) {
            double lat_deg = nmea_coord_to_deg(lat, lat_hemi[0], true);
            double lon_deg = nmea_coord_to_deg(lng, lng_hemi[0], false);
            double spd = (spd_kn && *spd_kn) ? atof(spd_kn) : 0.0;
            double crs = (crs_deg && *crs_deg) ? atof(crs_deg) : 0.0;
            gnss_last_location.lat_deg = lat_deg;
            gnss_last_location.lon_deg = lon_deg;
            gnss_last_location.spd_kn  = spd;
            gnss_last_location.crs_deg = crs;
            strncpy(gnss_last_location.time_utc, time_utc ? time_utc : "", sizeof(gnss_last_location.time_utc)-1);
            gnss_last_location.time_utc[sizeof(gnss_last_location.time_utc)-1] = '\0';
            gnss_last_location.valid = true;
        }
    }
    gnss_seen_rmc = true;
    gnss_rmc_valid = rmc_fix;
    gnss_log_fix_summary_if_changed();
    gnss_report_ttff_if_needed();
}

void gnss_parse_and_log_gga(const char* nmea)
{
    // 匹配 $xxGGA, 任意 talker
    const char* gga = NULL;
    for (const char* p = nmea; (p = strchr(p, '$')) != NULL; ++p) {
        if (p[1] && p[2] && p[3] && p[4] && p[5] && p[6]) {
            if (p[3] == 'G' && p[4] == 'G' && p[5] == 'A' && p[6] == ',') { gga = p; break; }
        } else {
            break;
        }
    }
    if (!gga) return;

    char line[128];
    size_t i = 0;
    while (gga[i] && gga[i] != '\n' && i < sizeof(line) - 1) { line[i] = gga[i]; i++; }
    line[i] = '\0';

    char tmp[128];
    strncpy(tmp, line, sizeof(tmp)-1); tmp[sizeof(tmp)-1] = '\0';
    char* fields[20] = {0};
    int count = nmea_split_fields(tmp, fields, 20);
    if (count <= 8) return; // 需要至少到索引8

    int   fixq = fields[6] ? atoi(fields[6]) : 0;
    int   sats = fields[7] ? atoi(fields[7]) : 0;
    float hdop = fields[8] ? (float)atof(fields[8]) : 0.0f;

    bool prev_fix = gnss_has_fix;
    gnss_fix_q = fixq;
    gnss_sats  = sats;
    gnss_hdop  = hdop;
    gnss_has_fix = (fixq > 0);

    GNSS_PARSE_LOG("GGA: fixQ=%d sats=%d HDOP=%.1f -> %s",
             fixq, sats, hdop, gnss_has_fix ? "FIXED" : "NO FIX");

    if (prev_fix != gnss_has_fix) {
        ESP_LOGI(TAG, "FIX state changed by GGA: %s", gnss_has_fix ? "FIXED" : "NO FIX");
    }
    gnss_seen_gga = true;
    gnss_log_fix_summary_if_changed();
    gnss_report_ttff_if_needed();
}

void gnss_parse_and_log_gsa(const char* nmea)
{
    // 匹配 $xxGSA, 任意 talker
    const char* gsa = NULL;
    for (const char* p = nmea; (p = strchr(p, '$')) != NULL; ++p) {
        if (p[1] && p[2] && p[3] && p[4] && p[5] && p[6]) {
            if (p[3] == 'G' && p[4] == 'S' && p[5] == 'A' && p[6] == ',') { gsa = p; break; }
        } else {
            break;
        }
    }
    if (!gsa) return;

    char line[160];
    size_t i = 0;
    while (gsa[i] && gsa[i] != '\n' && i < sizeof(line) - 1) { line[i] = gsa[i]; i++; }
    line[i] = '\0';

    char tmp[160];
    strncpy(tmp, line, sizeof(tmp)-1); tmp[sizeof(tmp)-1] = '\0';
    char* fields[24] = {0};
    int count = nmea_split_fields(tmp, fields, 24);
    if (count <= 2) return;

    int dim = fields[2] ? atoi(fields[2]) : 0; // 1=无,2=2D,3=3D
    float hdop = (count > 16 && fields[16]) ? (float)atof(fields[16]) : 0.0f;

    gnss_dim = dim;
    if (hdop > 0.0f) gnss_hdop = hdop;

    const char* dim_str = (dim == 3) ? "3D" : (dim == 2) ? "2D" : "NO FIX";
    GNSS_PARSE_LOG("GSA: dim=%s (val=%d) HDOP=%.1f", dim_str, dim, gnss_hdop);

    if (dim >= 2) gnss_has_fix = true;
    gnss_seen_gsa = true;
    gnss_log_fix_summary_if_changed();
    gnss_report_ttff_if_needed();
}

/* 置位后 gnss_task 自行退出。调试保持模式用：ESP32 要彻底安静下来。 */
static volatile bool s_gnss_stop = false;
void gnss_task_stop_request(void) { s_gnss_stop = true; }

void gnss_task(void* pv)
{
    // 读 GNSS NMEA 数据并打印，优先显示 RMC
    uint8_t buf[256];
    char line[256];
    size_t line_len = 0;

    // 以"上次完成一行"的时间为心跳参考（避免有乱码字节时误判为空闲）
    TickType_t last_line_tick = xTaskGetTickCount();
    size_t bytes_since_line = 0;   // 自上次换行以来累计收到的字节数
    uint8_t sample[16];            // 最近一批字节的样本（用于调试波特率不匹配）
    int sample_len = 0;

    // 初始化待机/唤醒观察变量
    gnss_last_sentence_tick = last_line_tick;
    gnss_lines_seen = 0;
    gnss_ack_pmtk161 = false;
    gnss_ack_pmtk225 = false;

    // 简易 UBX 帧解析状态机（用于捕获 ACK-ACK/ACK-NAK）
    enum {UBX_SYNC1, UBX_SYNC2, UBX_CLASS, UBX_ID, UBX_LEN1, UBX_LEN2, UBX_PAYLOAD, UBX_CKA, UBX_CKB} ubx_state = UBX_SYNC1;
    uint8_t ubx_cls=0, ubx_id=0; uint16_t ubx_len=0, ubx_pos=0; uint8_t ubx_ckA=0, ubx_ckB=0; uint8_t ubx_buf[256];

    // Quectel 二进制帧解析状态机（帧头 F1 D9，Fletcher-8 校验）
    enum {Q_S1, Q_S2, Q_GRP, Q_SUB, Q_L1, Q_L2, Q_PL, Q_C1, Q_C2} q_state = Q_S1;
    uint8_t q_grp=0, q_sub=0; uint16_t q_len=0, q_pos=0;
    uint8_t q_c1=0, q_c2=0, q_rx_c1=0; uint8_t q_buf[64];

    while (1) {
        if (s_gnss_stop) {
            ESP_LOGW(TAG, "gnss_task 收到停止请求，退出（调试保持模式）");
            vTaskDelete(NULL);
            return;
        }
        if (gnss_should_stop) {
            ESP_LOGI(TAG, "Stopping GNSS receiver after benchmark");
            vTaskDelete(NULL);
        }
        int len = uart_read_bytes(GNSS_UART_NUM, buf, sizeof(buf), 200 / portTICK_PERIOD_MS);
        if (len > 0) {
            // 更新样本与计数
            int n = (len < (int)sizeof(sample)) ? len : (int)sizeof(sample);
            memcpy(sample, buf, n);
            sample_len = n;
            bytes_since_line += len;

            for (int i = 0; i < len; ++i) {
                char c = (char)buf[i];

                // ---- Quectel 二进制帧解析（F1 D9）----
                {
                    uint8_t qb = (uint8_t)c;
                    switch (q_state) {
                        case Q_S1: if (qb == 0xF1) { q_state = Q_S2; } break;
                        case Q_S2:
                            if (qb == 0xD9) { q_state = Q_GRP; q_c1 = q_c2 = 0; }
                            else { q_state = (qb == 0xF1) ? Q_S2 : Q_S1; }
                            break;
                        case Q_GRP: q_grp = qb; q_c1 += qb; q_c2 += q_c1; q_state = Q_SUB; break;
                        case Q_SUB: q_sub = qb; q_c1 += qb; q_c2 += q_c1; q_state = Q_L1;  break;
                        case Q_L1:  q_len  = qb; q_c1 += qb; q_c2 += q_c1; q_state = Q_L2; break;
                        case Q_L2:
                            q_len |= ((uint16_t)qb << 8); q_c1 += qb; q_c2 += q_c1;
                            if (q_len > sizeof(q_buf)) { q_state = Q_S1; }
                            else { q_pos = 0; q_state = q_len ? Q_PL : Q_C1; }
                            break;
                        case Q_PL:
                            q_buf[q_pos++] = qb; q_c1 += qb; q_c2 += q_c1;
                            if (q_pos >= q_len) { q_state = Q_C1; }
                            break;
                        case Q_C1: q_rx_c1 = qb; q_state = Q_C2; break;
                        case Q_C2:
                            if (q_rx_c1 == q_c1 && qb == q_c2) {
                                // 校验通过。ACK-ACK = 05/01，ACK-NAK = 05/00
                                if (q_grp == 0x05 && q_len >= 2) {
                                    gnss_qack_group  = q_buf[0];
                                    gnss_qack_sub    = q_buf[1];
                                    gnss_qack_is_ack = (q_sub == 0x01);
                                    gnss_qack_seen   = true;
                                    ESP_LOGI(TAG, "[QBIN] %s <- for msg %02X/%02X",
                                             gnss_qack_is_ack ? "ACK-ACK" : "ACK-NAK",
                                             q_buf[0], q_buf[1]);
                                    if (q_buf[0] == 0x06 && q_buf[1] == 0x41) {
                                        if (gnss_qack_is_ack) { gnss_ack_cfg_sleep = true; }
                                        else                  { gnss_nak_cfg_sleep = true; }
                                    }
                                }
                            }
                            q_state = Q_S1;
                            break;
                    }
                }

                // UBX 解析
                uint8_t b = (uint8_t)c;
                switch (ubx_state) {
                    case UBX_SYNC1: if (b == 0xB5) { ubx_state = UBX_SYNC2; ubx_ckA = ubx_ckB = 0; } break;
                    case UBX_SYNC2: if (b == 0x62) { ubx_state = UBX_CLASS; } else { ubx_state = UBX_SYNC1; } break;
                    case UBX_CLASS: ubx_cls = b; ubx_ckA += b; ubx_ckB += ubx_ckA; ubx_state = UBX_ID; break;
                    case UBX_ID:    ubx_id  = b; ubx_ckA += b; ubx_ckB += ubx_ckA; ubx_state = UBX_LEN1; break;
                    case UBX_LEN1:  ubx_len = b; ubx_ckA += b; ubx_ckB += ubx_ckA; ubx_state = UBX_LEN2; break;
                    case UBX_LEN2:  ubx_len |= ((uint16_t)b << 8); ubx_ckA += b; ubx_ckB += ubx_ckA;
                                     if (ubx_len > sizeof(ubx_buf)) { ubx_state = UBX_SYNC1; }
                                     else { ubx_pos = 0; ubx_state = (ubx_len ? UBX_PAYLOAD : UBX_CKA); }
                                     break;
                    case UBX_PAYLOAD:
                        ubx_buf[ubx_pos++] = b; ubx_ckA += b; ubx_ckB += ubx_ckA;
                        if (ubx_pos >= ubx_len) ubx_state = UBX_CKA;
                        break;
                    case UBX_CKA:
                        if (b == ubx_ckA) { ubx_state = UBX_CKB; }
                        else { ubx_state = UBX_SYNC1; }
                        break;
                    case UBX_CKB:
                        if (b == ubx_ckB) {
                            // 完整 UBX 帧
                            if (ubx_cls == 0x05 && ubx_id == 0x01 && ubx_len >= 2) { // ACK-ACK
                                uint8_t ack_cls = ubx_buf[0];
                                uint8_t ack_id  = ubx_buf[1];
                                GNSS_UBX_LOG("RX ACK-ACK for cls=0x%02X id=0x%02X", ack_cls, ack_id);
                                if (ack_cls == 0x06 && ack_id == 0x41) gnss_ack_cfg_sleep = true;
                            } else if (ubx_cls == 0x05 && ubx_id == 0x00 && ubx_len >= 2) { // ACK-NAK
                                uint8_t ack_cls = ubx_buf[0];
                                uint8_t ack_id  = ubx_buf[1];
                                GNSS_UBX_LOG("RX ACK-NAK for cls=0x%02X id=0x%02X", ack_cls, ack_id);
                                if (ack_cls == 0x06 && ack_id == 0x41) gnss_nak_cfg_sleep = true;
                            }
                        }
                        ubx_state = UBX_SYNC1;
                        break;
                }
                if (c == '\r' || c == '\n') {
                    line[line_len] = '\0';
                    if (line_len > 0) {
                        // 打印整句 NMEA（可由宏控制开关）
                        GNSS_RAW_LOG("%s", line);
                        // ---- 协议探测用的统计 ----
                        if (line[0] == '$') {
                            if (line[1] == 'P') {
                                // 专有语句：记下最后一条原文和前缀
                                gnss_prop_count++;
                                strncpy(gnss_last_prop, line, sizeof(gnss_last_prop) - 1);
                                gnss_last_prop[sizeof(gnss_last_prop) - 1] = '\0';
                                if (strncmp(line, "$PMTK", 5) == 0) { gnss_seen_pmtk = true; }
                                else if (strncmp(line, "$PAIR", 5) == 0) { gnss_seen_pair = true; }
                                else if (strncmp(line, "$PCAS", 5) == 0) { gnss_seen_pcas = true; }
                                else if (strncmp(line, "$PQ", 3) == 0)   { gnss_seen_pq   = true; }
                            } else if (line_len >= 6) {
                                // 标准语句：收集 talker ID（$GN / $GP / $GL / $GA / $BD ...）
                                bool known = false;
                                for (int k = 0; k < gnss_talker_n; ++k) {
                                    if (gnss_talkers[k][0] == line[1] && gnss_talkers[k][1] == line[2]) {
                                        known = true; break;
                                    }
                                }
                                if (!known && gnss_talker_n < 8) {
                                    gnss_talkers[gnss_talker_n][0] = line[1];
                                    gnss_talkers[gnss_talker_n][1] = line[2];
                                    gnss_talkers[gnss_talker_n][2] = '\0';
                                    gnss_talker_n++;
                                }
                            }
                        }

                        // Quectel PAIR ACK：$PAIR001,<cmd>,<result>
                        // 手写解析，不用 sscanf —— 它在本任务 4096 的栈上开销太大
                        if (strncmp(line, "$PAIR001,", 9) == 0) {
                            const char *p = line + 9;
                            int cmd = 0, res = 0;
                            bool ok = (*p >= '0' && *p <= '9');
                            while (*p >= '0' && *p <= '9') { cmd = cmd * 10 + (*p++ - '0'); }
                            if (ok && *p == ',') {
                                ++p;
                                ok = (*p >= '0' && *p <= '9');
                                while (*p >= '0' && *p <= '9') { res = res * 10 + (*p++ - '0'); }
                            } else {
                                ok = false;
                            }
                            if (ok) {
                                gnss_pair_ack_cmd    = cmd;
                                gnss_pair_ack_result = res;
                                gnss_pair_ack_seen   = true;
                                ESP_LOGI(TAG, "[PAIR] RX <- %s", line);
                            }
                        }
                        // 待机命令ACK检测
                        if (strstr(line, "$PMTK001,161,3") != NULL) {
                            gnss_ack_pmtk161 = true;
                            ESP_LOGI(TAG, "ACK PMTK161");
                        } else if (strstr(line, "$PMTK001,225,3") != NULL) {
                            gnss_ack_pmtk225 = true;
                            ESP_LOGI(TAG, "ACK PMTK225");
                        }
                        // 基于句型选择性解析，避免误判
                        if (line[0] == '$' && line_len > 6 && line[6] == ',') {
                            if (line[3] == 'R' && line[4] == 'M' && line[5] == 'C') {
                                gnss_parse_and_log_rmc(line);
                            } else if (line[3] == 'G' && line[4] == 'G' && line[5] == 'A') {
                                gnss_parse_and_log_gga(line);
                            } else if (line[3] == 'G' && line[4] == 'S' && line[5] == 'A') {
                                gnss_parse_and_log_gsa(line);
                            }
                        }
                    }
                    line_len = 0;
                    // 完成一行，重置基准
                    last_line_tick = xTaskGetTickCount();
                    gnss_last_sentence_tick = last_line_tick;
                    gnss_lines_seen++;
                    bytes_since_line = 0;
                    sample_len = 0;
                } else if (line_len < sizeof(line) - 1) {
                    line[line_len++] = c;
                } else {
                    // 行过长，丢弃重来
                    line_len = 0;
                }
            }
        }

        // 超过2秒没有"完成一行"，输出心跳
        if ((xTaskGetTickCount() - last_line_tick) >= pdMS_TO_TICKS(2000)) {
            if (bytes_since_line == 0) {
                if (gnss_in_standby) {
                    // 已确认进入低功耗，安静是预期行为。降频打印，避免刷屏
                    static uint32_t quiet_ticks = 0;
                    if ((quiet_ticks++ % 15) == 0) {   // 约每 30 秒一条
                        ESP_LOGI(TAG, "GNSS in low-power, UART quiet as expected (%lus)",
                                 (unsigned long)(quiet_ticks * 2));
                    }
                } else {
                    ESP_LOGI(TAG, "Waiting for NMEA on UART2 (TX=%d, RX=%d, %d)...", GNSS_UART_TX, GNSS_UART_RX, GNSS_BAUD);
                }
            } else {
                // 有数据但无换行，多半是波特率不匹配或线路噪声
                char hex[16 * 3 + 1];
                int pos = 0;
                for (int i = 0; i < sample_len && pos < (int)sizeof(hex) - 3; ++i) {
                    pos += snprintf(hex + pos, sizeof(hex) - pos, "%02X ", sample[i]);
                }
                hex[(pos < (int)sizeof(hex)) ? pos : ((int)sizeof(hex) - 1)] = '\0';
                GNSS_RAW_LOG("RX without CR/LF for 2s, bytes=%u, sample=[%s]", (unsigned)bytes_since_line, hex);
                ESP_LOGI(TAG, "RX without CR/LF for 2s, bytes=%u", (unsigned)bytes_since_line);
            }
            // 重置基准窗口
            last_line_tick = xTaskGetTickCount();
            bytes_since_line = 0;
            sample_len = 0;
        }

        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

void gnss_init(void)
{
    // 初始化 GNSS UART2 (LC760Z)
    uart_config_t gnss_uart_cfg = {
        .baud_rate = GNSS_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    ESP_LOGI(TAG, "UART2 (GNSS) init at %d baud", GNSS_BAUD);
    ESP_ERROR_CHECK(uart_driver_install(GNSS_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(GNSS_UART_NUM, &gnss_uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(GNSS_UART_NUM, GNSS_UART_TX, GNSS_UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_LOGI(TAG, "Pins set: TX=%d -> GNSS RXD, RX=%d <- GNSS TXD", GNSS_UART_TX, GNSS_UART_RX);

    // GNSS 使能脚（IO11 -> LP5907 EN）拉高
    /* 先写电平锁存器再切方向：深睡唤醒等同复位，输出锁存器复位值是 0，
     * 先 set_direction(OUTPUT) 会把 IO11 驱动成低，LP5907 当场关断，
     * GNSS 掉电冷启动 —— Backup 和 RTC 全丢。 */
    gnss_enable(true);                 // 先写锁存器
    gpio_set_direction(GNSS_EN_PIN, GPIO_MODE_OUTPUT);
    ESP_LOGI(TAG, "GNSS EN pin %d set HIGH", GNSS_EN_PIN);
    
    // GNSS 复位脚初始化
    gnss_reset_release(); // 释放复位状态
    gpio_set_direction(GNSS_RST_PIN, GPIO_MODE_OUTPUT);
}