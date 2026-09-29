#include "eg915_selftest.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include <string.h>

#include "hardware_config.h"
#include "mqtt_module.h"   // 复用 send_at_command / wait_for_ok / uart_read_response / modem_power_cycle / mqtt_init
#include "motion_sensor.h" // 运动传感器自测
#include "battery_module.h"// 电池/ADC 自测
#include "gnss_module.h"   // GNSS 自测（直接读UART）
#include "gpio_module.h"   // LED / 模组电源 / 围栏引脚

/* =====================================================================
 * Cattle_T 工厂自测
 * 相对 pe_board 的改动：
 *   - 删除 CAN / RS485 / IGN 光耦自测（本板无此硬件）
 *   - 模组由 EG915U 改为 EG800，上电前先使能 3.8V 供电 (IO48 低有效)
 *   - 电池 / IBL 合并为同一路 ADC (IO1)
 *   - 增加 LED、GNSS 状态脚、电子围栏安全状态检查
 * ===================================================================== */

static const char *TAG_ST = "SELFTEST";

// GUI 发送的握手 ACK，收到后输出一次自测结果
#define GUI_SELFTEST_ACK "!GUI_SELFTEST_ACK"

// 电池判定范围（单节锂电，单位 V）
#define SELFTEST_VBAT_MIN 3.00f
#define SELFTEST_VBAT_MAX 4.30f

/* 自测阶段是否给模组上电。
 * 双击 IO0 = 联网发 MQTT 时设为 0：自测全程不碰 3.8V 轨和 PWRKEY，
 * 让双击那一刻从"模组完全没开"开始，时序打点才量得到真实开机耗时，
 * 也才和深睡唤醒后的实际情况一致。 */
#ifndef SELFTEST_POWER_MODEM
#  define SELFTEST_POWER_MODEM (!BUTTON_DBLCLICK_MQTT_PUBLISH)
#endif

typedef struct {
    // Modem (EG800)
    bool modem_ok;
    bool modem_skipped;      // 本轮自测没给模组上电，EG800/SIM 两项不参与判定
    bool modem_status_pin;
    char modem_imei[32];
    char modem_iccid[32];
    // Motion
    bool motion_ok;
    float motion_mag;
    // GNSS
    bool gnss_uart_ok;
    int  gnss_bytes;
    int  gnss_3dfix;
    int  gnss_jam;
    // Battery / IBL
    bool battery_ok;
    float battery_v;
    uint32_t ibl_mv;
    // LED
    bool led_tested;
    // Fence（只检查安全状态，不放电）
    bool fence_safe;
    bool fence_charge_done_pin;
} selftest_report_t;

bool eg915_at_handshake_once(unsigned timeout_ms, int retries)
{
    for (int i = 1; i <= retries; ++i) {
        ESP_LOGI(TAG_ST, "Handshake attempt %d/%d: sending AT...", i, retries);
        send_at_command("AT");
        if (wait_for_ok(timeout_ms)) {
            ESP_LOGI(TAG_ST, "AT handshake OK on attempt %d", i);
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(300));
    }
    ESP_LOGW(TAG_ST, "AT handshake failed after %d attempts", retries);
    return false;
}

// 读取模组 IMEI
static bool modem_get_imei(char *imei_buf, size_t buf_size)
{
    if (!imei_buf || buf_size < 16) return false;

    memset(imei_buf, 0, buf_size);
    ESP_LOGI(TAG_ST, "Getting IMEI...");
    send_at_command("AT+GSN");

    char resp[128];
    int len = uart_read_response(resp, sizeof(resp), 1000);
    if (len > 0) {
        char *p = resp;
        while (*p) {
            if (*p >= '0' && *p <= '9') {
                int digits = 0;
                char *start = p;
                while (p[digits] >= '0' && p[digits] <= '9') digits++;
                if (digits == 15) {  // IMEI 为 15 位
                    strncpy(imei_buf, start, 15);
                    imei_buf[15] = '\0';
                    ESP_LOGI(TAG_ST, "IMEI: %s", imei_buf);
                    return true;
                }
                p += digits;
            } else {
                p++;
            }
        }
    }
    ESP_LOGW(TAG_ST, "Failed to get IMEI");
    strcpy(imei_buf, "N/A");
    return false;
}

// 读取 SIM 卡 ICCID
static bool modem_get_iccid(char *iccid_buf, size_t buf_size)
{
    if (!iccid_buf || buf_size < 20) return false;

    memset(iccid_buf, 0, buf_size);
    ESP_LOGI(TAG_ST, "Getting ICCID...");
    send_at_command("AT+CCID");

    char resp[128];
    int len = uart_read_response(resp, sizeof(resp), 1000);
    if (len > 0) {
        char *p = strstr(resp, "+CCID:");
        if (p) {
            p += 6;
            while (*p == ' ') p++;

            int digits = 0;
            while (p[digits] >= '0' && p[digits] <= '9') digits++;
            if (digits >= 19 && digits <= 20) {
                strncpy(iccid_buf, p, digits);
                iccid_buf[digits] = '\0';
                ESP_LOGI(TAG_ST, "ICCID: %s", iccid_buf);
                return true;
            }
        }
    }
    ESP_LOGW(TAG_ST, "Failed to get ICCID");
    strcpy(iccid_buf, "N/A");
    return false;
}

static void drain_boot_urcs(uint32_t ms)
{
    // 智能URC drain: 最多等 ms 毫秒，但连续200ms无数据就提前退出
    TickType_t t0 = xTaskGetTickCount();
    TickType_t last_data = t0;
    char buf[256];
    while ((xTaskGetTickCount() - t0) < pdMS_TO_TICKS(ms)) {
        int n = uart_read_bytes(UART_EG800_NUM, (uint8_t*)buf, sizeof(buf) - 1, pdMS_TO_TICKS(50));
        if (n > 0) {
            last_data = xTaskGetTickCount();
            buf[n] = '\0';
            ESP_LOGI(TAG_ST, "URC: %s", buf);
        }
        if ((xTaskGetTickCount() - last_data) > pdMS_TO_TICKS(200)) {
            ESP_LOGI(TAG_ST, "URC drain complete (no data for 200ms)");
            break;
        }
    }
}

static bool selftest_motion(selftest_report_t *r)
{
    static bool motion_inited = false;
    bool ok = true;
    if (!motion_inited) {
        ok = motion_sensor_init();
        motion_inited = ok;
    }
    if (!ok) {
        r->motion_ok = false;
        r->motion_mag = 0.0f;
        ESP_LOGW(TAG_ST, "Motion sensor init failed");
        return false;
    }
    lis2dh12_accel_data_t acc;
    if (lis2dh12_read_accel_data(&acc)) {
        float mag = lis2dh12_calculate_magnitude(&acc);
        r->motion_ok = true;
        r->motion_mag = mag;
        ESP_LOGI(TAG_ST, "Motion: X=%.3fg Y=%.3fg Z=%.3fg Mag=%.3fg", acc.x, acc.y, acc.z, mag);
        return true;
    }
    r->motion_ok = false;
    r->motion_mag = 0.0f;
    ESP_LOGW(TAG_ST, "Motion sensor read failed");
    return false;
}

static void selftest_battery(selftest_report_t *r)
{
    static bool bat_inited = false;
    if (!bat_inited) { battery_init(); bat_inited = true; }

    r->ibl_mv    = read_ibl_voltage_mv();   // ADC 引脚上的实际电压
    r->battery_v = read_battery_voltage();  // 乘分压比后的 VBAT
    r->battery_ok = (r->battery_v >= SELFTEST_VBAT_MIN && r->battery_v <= SELFTEST_VBAT_MAX);

    ESP_LOGI(TAG_ST, "Battery: ADC=%lu mV, VBAT=%.2fV (ok=%d, expect %.2f-%.2fV)",
             (unsigned long)r->ibl_mv, r->battery_v, r->battery_ok,
             SELFTEST_VBAT_MIN, SELFTEST_VBAT_MAX);
}

static void selftest_gnss(selftest_report_t *r)
{
    static bool gnss_inited = false;
    if (!gnss_inited) { gnss_init(); gnss_inited = true; }
    gnss_enable(true);
    gnss_reset_release();
    vTaskDelay(pdMS_TO_TICKS(500));

    uint8_t buf[128];
    int total = 0;
    TickType_t t0 = xTaskGetTickCount();
    while ((xTaskGetTickCount() - t0) < pdMS_TO_TICKS(1500)) {
        int n = uart_read_bytes(GNSS_UART_NUM, buf, sizeof(buf), pdMS_TO_TICKS(100));
        if (n > 0) { total += n; if (total >= 32) break; }
    }
    r->gnss_bytes = total;
    r->gnss_uart_ok = (total > 0);
    r->gnss_3dfix = gnss_get_3dfix_level();
    r->gnss_jam   = gnss_get_jam_level();
    ESP_LOGI(TAG_ST, "GNSS UART bytes=%d (ok=%d), 3D_FIX=%d, JAM_IND=%d",
             total, r->gnss_uart_ok, r->gnss_3dfix, r->gnss_jam);
}

// LED 目视自检：两颗灯各闪两次，由产线目视/光感治具确认
static void selftest_leds(selftest_report_t *r)
{
    ESP_LOGI(TAG_ST, "LED blink test (ESP_LED IO%d, CONN_LED IO%d)...", LED_ESP_PIN, LED_CONN_PIN);
    for (int i = 0; i < 2; ++i) {
        led1_on();  vTaskDelay(pdMS_TO_TICKS(150));
        led1_off(); vTaskDelay(pdMS_TO_TICKS(150));
    }
    for (int i = 0; i < 2; ++i) {
        led3_on();  vTaskDelay(pdMS_TO_TICKS(150));
        led3_off(); vTaskDelay(pdMS_TO_TICKS(150));
    }
    r->led_tested = true;
}

// 电子围栏：只确认引脚处于安全状态并读取 DONE#，绝不触发放电
static void selftest_fence(selftest_report_t *r)
{
    fence_pins_safe_init();
    r->fence_safe = (gpio_get_level(FENCE_TRIG_PIN) == 0) &&
                    (gpio_get_level(FENCE_CHARGE_EN_PIN) == 0);
    r->fence_charge_done_pin = fence_charge_done();
    ESP_LOGI(TAG_ST, "Fence safe-state=%d, DONE#(low=done)=%d",
             r->fence_safe, r->fence_charge_done_pin);
}

static void build_json(const selftest_report_t *r, char *out, size_t out_sz)
{
    snprintf(out, out_sz,
        "SELFTEST SUMMARY:\n"
        "{\n"
        "  \"modem_ok\": %s,\n"
        "  \"modem_status_pin\": %s,\n"
        "  \"imei\": \"%s\",\n"
        "  \"iccid\": \"%s\",\n"
        "  \"motion\": { \"ok\": %s, \"mag\": %.3f },\n"
        "  \"gnss\": { \"uart_ok\": %s, \"bytes\": %d, \"fix3d\": %d, \"jam\": %d },\n"
        "  \"battery\": { \"ok\": %s, \"vbat\": %.2f, \"adc_mv\": %lu },\n"
        "  \"led\": { \"tested\": %s },\n"
        "  \"fence\": { \"safe\": %s, \"done\": %s }\n"
        "}\n",
        r->modem_ok ? "true" : "false",
        r->modem_status_pin ? "true" : "false",
        r->modem_imei,
        r->modem_iccid,
        r->motion_ok ? "true" : "false", r->motion_mag,
        r->gnss_uart_ok ? "true" : "false", r->gnss_bytes, r->gnss_3dfix, r->gnss_jam,
        r->battery_ok ? "true" : "false", r->battery_v, (unsigned long)r->ibl_mv,
        r->led_tested ? "true" : "false",
        r->fence_safe ? "true" : "false", r->fence_charge_done_pin ? "true" : "false"
    );
}

static bool overall_pass(const selftest_report_t *r)
{
    // 跳过模组这轮时，EG800/SIM 不参与总判定
    bool modem_part = r->modem_skipped ? true : r->modem_ok;
    return modem_part && r->motion_ok && r->gnss_uart_ok && r->battery_ok && r->fence_safe;
}

static void print_human_summary(const selftest_report_t *r)
{
    ESP_LOGI(TAG_ST, "================= SELFTEST RESULT =================");
    ESP_LOGI(TAG_ST, "EG800 (AT)  : %s (IMEI=%s)",
             r->modem_skipped ? "SKIP" : (r->modem_ok ? "PASS" : "FAIL"), r->modem_imei);
    ESP_LOGI(TAG_ST, "SIM/ICCID   : %s", r->modem_iccid);
    ESP_LOGI(TAG_ST, "Motion      : %s (|a|=%.3fg)", r->motion_ok ? "PASS" : "FAIL", r->motion_mag);
    ESP_LOGI(TAG_ST, "GNSS UART   : %s (bytes=%d, 3D_FIX=%d)", r->gnss_uart_ok ? "PASS" : "FAIL",
             r->gnss_bytes, r->gnss_3dfix);
    ESP_LOGI(TAG_ST, "Battery/ADC : %s (VBAT=%.2fV, ADC=%lu mV)", r->battery_ok ? "PASS" : "FAIL",
             r->battery_v, (unsigned long)r->ibl_mv);
    ESP_LOGI(TAG_ST, "LED         : %s", r->led_tested ? "BLINKED" : "SKIPPED");
    ESP_LOGI(TAG_ST, "Fence pins  : %s", r->fence_safe ? "SAFE" : "UNSAFE");
    ESP_LOGI(TAG_ST, "---------------------------------------------------");
    ESP_LOGI(TAG_ST, "OVERALL     : %s", overall_pass(r) ? "PASS" : "FAIL");
}

// 在 USB-JTAG 调试串口上输出格式化表格，便于人工查看
static void print_formatted_result(const selftest_report_t *r)
{
    bool overall = overall_pass(r);

    ESP_LOGI(TAG_ST, "========== FORMATTED REPORT START ==========");
    ESP_LOGI(TAG_ST, "+---------------------------------------------------------------+");
    ESP_LOGI(TAG_ST, "|         CATTLE TRACKER FACTORY SELF-TEST REPORT               |");
    ESP_LOGI(TAG_ST, "+--------------------+---------+--------------------------------+");
    ESP_LOGI(TAG_ST, "| Test Item          | Result  | Details                        |");
    ESP_LOGI(TAG_ST, "+--------------------+---------+--------------------------------+");
    ESP_LOGI(TAG_ST, "| EG800 Module       | %-7s | IMEI %s",
             r->modem_skipped ? "SKIP" : (r->modem_ok ? "PASS" : "FAIL"), r->modem_imei);
    ESP_LOGI(TAG_ST, "| SIM Card           | %-7s | ICCID %s",
             r->modem_skipped ? "SKIP"
                              : ((strcmp(r->modem_iccid, "N/A") != 0) ? "PASS" : "FAIL"),
             r->modem_iccid);
    ESP_LOGI(TAG_ST, "| Motion Sensor      | %-7s | Magnitude: %.3f g", r->motion_ok ? "PASS" : "FAIL", r->motion_mag);
    ESP_LOGI(TAG_ST, "| GNSS UART          | %-7s | %d bytes, 3D_FIX=%d", r->gnss_uart_ok ? "PASS" : "FAIL",
             r->gnss_bytes, r->gnss_3dfix);
    ESP_LOGI(TAG_ST, "| Battery (IBL/IO1)  | %-7s | VBAT %.2f V (ADC %lu mV)", r->battery_ok ? "PASS" : "FAIL",
             r->battery_v, (unsigned long)r->ibl_mv);
    ESP_LOGI(TAG_ST, "| LED                | %-7s | ESP_LED + CONN_LED blinked", r->led_tested ? "DONE" : "SKIP");
    ESP_LOGI(TAG_ST, "| Fence safe state   | %-7s | TRIG/CHARGE held low", r->fence_safe ? "PASS" : "FAIL");
    ESP_LOGI(TAG_ST, "+--------------------+---------+--------------------------------+");
    ESP_LOGI(TAG_ST, "| OVERALL RESULT: %-45s |", overall ? "ALL TESTS PASSED" : "SOME TESTS FAILED");
    ESP_LOGI(TAG_ST, "+---------------------------------------------------------------+");

    if (!overall) {
        ESP_LOGI(TAG_ST, "--- FAILURE DETAILS ---");
        if (!r->modem_ok && !r->modem_skipped)
            ESP_LOGI(TAG_ST, "  x EG800: no AT response - check 3.8V rail / PWRKEY / UART1");
        if (!r->motion_ok)    ESP_LOGI(TAG_ST, "  x Motion: read failed - check I2C (IO14/IO15)");
        if (!r->gnss_uart_ok) ESP_LOGI(TAG_ST, "  x GNSS: no data - check LP5907 EN (IO11) / UART2 (IO6,IO7)");
        if (!r->battery_ok)   ESP_LOGI(TAG_ST, "  x Battery: out of range - check R93/R94 divider on IO1");
        if (!r->fence_safe)   ESP_LOGI(TAG_ST, "  x Fence: pins not in safe state - CHECK BEFORE HANDLING");
    }
    ESP_LOGI(TAG_ST, "========== FORMATTED REPORT END ==========");
}

// 独立模式：JSON 直接打到 USB-JTAG 调试口
static void emit_summary_to_log(const selftest_report_t *r)
{
    char json_buf[640];
    build_json(r, json_buf, sizeof(json_buf));
    ESP_LOGI(TAG_ST, "%s", json_buf);
}

#if SELFTEST_USE_JIG
// 治具模式：JSON 通过 UART0 的 U0TXD/U0RXD(43/44) 发送给治具
static void emit_summary_over_uart0(const selftest_report_t *r)
{
    char json_buf[640];
    build_json(r, json_buf, sizeof(json_buf));
    if (json_buf[0] == '\0') return;

    uart_config_t cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    if (uart_driver_install(UART_NUM_0, 1024, 0, 0, NULL, 0) == ESP_OK) {
        uart_param_config(UART_NUM_0, &cfg);
        uart_set_pin(UART_NUM_0, U0TXD_PIN, U0RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        uart_write_bytes(UART_NUM_0, json_buf, strlen(json_buf));
        uart_wait_tx_done(UART_NUM_0, pdMS_TO_TICKS(200));
        uart_driver_delete(UART_NUM_0);
    }
}

// 在 UART0 上等待治具的 ACK。返回 true = 收到，false = 超时或串口不可用。
static bool wait_for_jig_ack(void)
{
    if (uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0) != ESP_OK) {
        ESP_LOGW(TAG_ST, "UART0 driver install failed; skipping ACK wait.");
        return false;
    }

    uart_config_t cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_param_config(UART_NUM_0, &cfg);
    uart_set_pin(UART_NUM_0, U0TXD_PIN, U0RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    const int ack_buf_len = 64;
    char ack_buf[ack_buf_len];
    int ack_pos = 0;
    memset(ack_buf, 0, sizeof(ack_buf));
    int wait_ms = 0;
    bool got_ack = false;

    const char *wait_msg = "Waiting for ACK: !GUI_SELFTEST_ACK\r\n";
    uart_write_bytes(UART_NUM_0, wait_msg, strlen(wait_msg));
    ESP_LOGI(TAG_ST, "[JIG][WAIT] DUT自测已完成，等待GUI发送ACK指令: %s", GUI_SELFTEST_ACK);

    while (1) {
        uint8_t ch;
        int n = uart_read_bytes(UART_NUM_0, &ch, 1, pdMS_TO_TICKS(100));
        wait_ms += 100;
        if (n == 1) {
            if (ack_pos < ack_buf_len - 1) {
                ack_buf[ack_pos++] = (char)ch;
                ack_buf[ack_pos] = '\0';
            } else {
                memmove(ack_buf, ack_buf + 1, ack_buf_len - 2);
                ack_buf[ack_buf_len - 2] = (char)ch;
                ack_buf[ack_buf_len - 1] = '\0';
            }
            if (strstr(ack_buf, GUI_SELFTEST_ACK)) {
                ESP_LOGI(TAG_ST, "[JIG][WAIT] 收到ACK，准备输出自测结果");
                got_ack = true;
                break;
            }
        }
        if (wait_ms % 1000 == 0) {
            ESP_LOGI(TAG_ST, "[JIG][WAIT] 正在等待ACK... 已等待%d ms", wait_ms);
        }
#if SELFTEST_ACK_TIMEOUT_MS > 0
        if (wait_ms >= SELFTEST_ACK_TIMEOUT_MS) {
            ESP_LOGW(TAG_ST, "[JIG][WAIT] 等待ACK超时(%d ms)，按无治具方式继续输出", wait_ms);
            break;
        }
#endif
    }

    uart_driver_delete(UART_NUM_0);
    return got_ack;
}
#endif // SELFTEST_USE_JIG

void Selftest_task(void *pv)
{
    (void)pv;
#if SELFTEST_USE_JIG
    ESP_LOGI(TAG_ST, "Self-test task starting (JIG mode: RUN -> WAIT ACK -> OUTPUT)...");
#else
    ESP_LOGI(TAG_ST, "Self-test task starting (STANDALONE mode: RUN -> OUTPUT)...");
#endif

    selftest_report_t rep = (selftest_report_t){0};

    // 0. 围栏引脚先进入安全状态（涉及高压，优先级最高）
    selftest_fence(&rep);

    // 1. 模组串口（mqtt_init 只建 UART1 和总线锁，不碰 3.8V 轨）
    mqtt_init();

#if SELFTEST_POWER_MODEM
    modem_power_enable(true);
    vTaskDelay(pdMS_TO_TICKS(200));

    rep.modem_status_pin = modem_is_powered();
    if (!rep.modem_status_pin) {
        ESP_LOGI(TAG_ST, "Modem STATUS low -> pulsing PWRKEY...");
        modem_power_cycle();
        vTaskDelay(pdMS_TO_TICKS(2000));
        rep.modem_status_pin = modem_is_powered();
    }
    ESP_LOGI(TAG_ST, "Draining boot URCs for 3s...");
    drain_boot_urcs(3000);

    // 2. 各项测试
    rep.modem_ok = eg915_at_handshake_once(500, 3);
    ESP_LOGI(TAG_ST, "EG800 AT: %s", rep.modem_ok ? "PASS" : "FAIL");

    if (rep.modem_ok) {
        modem_get_imei(rep.modem_imei, sizeof(rep.modem_imei));
        modem_get_iccid(rep.modem_iccid, sizeof(rep.modem_iccid));
    } else {
        strcpy(rep.modem_imei, "N/A");
        strcpy(rep.modem_iccid, "N/A");
    }
#else
    /* 自测不给模组上电：3.8V 轨保持关闭，PWRKEY 不动。
     * 模组开机交给双击 IO0 那条路径，这样每次双击都是真正的冷启动。 */
    modem_power_enable(false);
    rep.modem_skipped    = true;
    rep.modem_ok         = false;
    rep.modem_status_pin = modem_is_powered();
    strcpy(rep.modem_imei,  "SKIP");
    strcpy(rep.modem_iccid, "SKIP");
    ESP_LOGW(TAG_ST, "EG800/SIM 本轮跳过：自测不上电，双击 IO0 时才开机（冷启动计时）");

    /* 这几个只在上电分支里用，这里取一下地址避免 -Wunused-function */
    (void)drain_boot_urcs; (void)eg915_at_handshake_once;
    (void)modem_get_imei;  (void)modem_get_iccid;
#endif

    selftest_motion(&rep);
    selftest_battery(&rep);
    selftest_gnss(&rep);
    selftest_leds(&rep);

#if GNSS_LP_SELFTEST
    /* GNSS 低功耗协议探测。需要 gnss_task 在跑才能收到模组的回应，
     * 自测模式下平时不起这个任务，这里单独拉起来。 */
    if (rep.gnss_uart_ok) {
        static bool gnss_task_started = false;
        if (!gnss_task_started) {
            xTaskCreate(gnss_task, "gnss_task", 6144, NULL, 8, NULL);
            gnss_task_started = true;
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        gnss_lp_probe_run();
    } else {
        ESP_LOGW(TAG_ST, "GNSS 无输出，跳过低功耗协议探测");
    }
#endif

    // 3. 输出结果：表格 + 文字汇总 + JSON，全部走 USB-JTAG 调试口
    print_formatted_result(&rep);
    print_human_summary(&rep);
    emit_summary_to_log(&rep);

    // 4. 结果指示：全部通过点亮 ESP_LED，否则点亮 CONN_LED
    if (overall_pass(&rep)) {
        led1_on();  led3_off();
    } else {
        led1_off(); led3_on();
    }

#if SELFTEST_USE_JIG
    // 5. 治具模式：等 ACK，收到后把 JSON 发回 UART0
    wait_for_jig_ack();
    vTaskDelay(pdMS_TO_TICKS(100));
    emit_summary_over_uart0(&rep);
    ESP_LOGI(TAG_ST, "Self-test summary emitted to jig. Task exiting.");
#else
    ESP_LOGI(TAG_ST, "Self-test done (standalone, no jig). LED shows %s. Task exiting.",
             overall_pass(&rep) ? "PASS (ESP_LED)" : "FAIL (CONN_LED)");
#endif

    vTaskDelete(NULL);
}
