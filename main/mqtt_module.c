#include "mqtt_module.h"
#include "hardware_config.h"
#include "mqtt_config.h"
#include "gnss_module.h"
#include "gpio_module.h"
#include "battery_module.h"
#include "motion_sensor.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_log.h"
#include "esp_adc_cal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_sleep.h"
#include "driver/rtc_io.h"

static const char *TAG = "MQTT";

/* 模组总线互斥：周期上报任务和手动上报任务都要独占 UART1 + 模组电源，
 * 不加锁的话两边会同时发 AT，响应互相串味。mqtt_init() 里创建。 */
static SemaphoreHandle_t s_modem_mutex = NULL;

static bool modem_bus_lock(uint32_t timeout_ms)
{
    if (s_modem_mutex == NULL) { return true; }   // 尚未初始化，退化为不加锁
    TickType_t t = (timeout_ms == portMAX_DELAY) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTake(s_modem_mutex, t) == pdTRUE;
}

static void modem_bus_unlock(void)
{
    if (s_modem_mutex != NULL) { xSemaphoreGive(s_modem_mutex); }
}

// 全局状态变量
#if ENABLE_MQTT
static mqtt_state_t mqtt_state = MQTT_STATE_IDLE;
static bool apn_configured = false;
static uint16_t mqtt_msg_id = 1;
static bool mqtt_session_open = false;
#endif

// 简单的串口读互斥，避免URC轮询与同步等待抢读
static volatile bool uart_read_busy = false;

/* 调试保持模式：长按 IO0 进入之后，ESP32 不再对模组发任何 AT。
 * 定义在这里是因为 mqtt_poll_urc() 和 manual_publish_task() 都要看它。 */
static volatile bool s_modem_debug_hold = false;

// 外部依赖的ADC变量（需要从外部提供）
extern esp_adc_cal_characteristics_t *adc_chars;

/* ---------------- deviceId：绑定模组 IMEI ---------------- */
// 取不到 IMEI 时的回退值，便于在云端一眼看出是没读到而不是设备重名
static char s_device_id[24] = "CT-IMEI-UNKNOWN";
static bool s_device_id_valid = false;

const char* device_get_id(void)
{
    return s_device_id;
}

bool device_id_is_imei(void)
{
    return s_device_id_valid;
}

// 拼出 fleet/<deviceId>/<suffix>
void mqtt_build_topic(char *out, size_t out_sz, const char *suffix)
{
    snprintf(out, out_sz, "%s/%s/%s", MQTT_TOPIC_ROOT, device_get_id(), suffix);
}

bool device_fetch_imei(void)
{
    if (s_device_id_valid) {
        return true;   // 已经拿到过，不再重复问模组
    }

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
                if (digits == 15) {          // IMEI 固定 15 位
                    memcpy(s_device_id, start, 15);
                    s_device_id[15] = '\0';
                    s_device_id_valid = true;
                    ESP_LOGI(TAG, "deviceId bound to IMEI: %s", s_device_id);
                    return true;
                }
                p += digits;
            } else {
                p++;
            }
        }
    }

    ESP_LOGW(TAG, "Failed to read IMEI, deviceId stays '%s'", s_device_id);
    return false;
}

// AT命令函数
void send_at_command(const char* command)
{
    // 清空接收缓冲区
    uart_flush_input(UART_EG800_NUM);
    char cmd_with_crlf[256];
    snprintf(cmd_with_crlf, sizeof(cmd_with_crlf), "%s\r\n", command);
    uart_write_bytes(UART_EG800_NUM, cmd_with_crlf, strlen(cmd_with_crlf));
    ESP_LOGI("UART1", "Sent: %s", command);
}

bool wait_for_ok(uint32_t timeout_ms)
{
    char response[256];
    uint32_t start_time = xTaskGetTickCount();
    uart_read_busy = true;
    while ((xTaskGetTickCount() - start_time) < (timeout_ms / portTICK_PERIOD_MS)) {
        int len = uart_read_response(response, sizeof(response), 500);
        if (len > 0) {
            ESP_LOGI("UART1", "Received: %s", response);
            if (strstr(response, "OK")) {
                uart_read_busy = false;
                return true;
            }
            if (strstr(response, "ERROR")) {
                uart_read_busy = false;
                return false;
            }
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    ESP_LOGW(TAG, "Timeout waiting for OK");
    uart_read_busy = false;
    return false;
}

int uart_read_response(char* buffer, int max_len, int timeout_ms)
{
    int total_len = 0;
    int bytes_available;
    TickType_t start_time = xTaskGetTickCount();
    
    while ((xTaskGetTickCount() - start_time) < (timeout_ms / portTICK_PERIOD_MS)) {
        ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_EG800_NUM, (size_t*)&bytes_available));
        
        if (bytes_available > 0) {
            /* 只取"已经在缓冲区里"的字节，超时给 0。
             * 原来请求 max_len(≈500) 字节、超时 50ms —— 永远凑不满 500 字节，
             * 所以每次都实打实阻塞满 50ms。这是每条 AT 固定开销的一半来源。 */
            int want = bytes_available;
            if (want > max_len - total_len - 1) want = max_len - total_len - 1;
            int len = (want > 0) ? uart_read_bytes(UART_EG800_NUM,
                                                   (uint8_t*)buffer + total_len,
                                                   want, 0) : 0;
            if (len > 0) {
                total_len += len;
                buffer[total_len] = '\0';

                // 如果收到完整的响应（包含\r\n），提前返回
                if (strstr(buffer, "\r\n") || strstr(buffer, "OK") || strstr(buffer, "ERROR")) {
                    /* 收尾：把紧跟其后的内容（回显之后的 OK、OK 之后的 URC）一起收进来。
                     * 原来是硬等 100ms 再读一次 —— 另一半固定开销。
                     * 现在 10ms 一格地查，连续 TAIL_QUIET_MS 没有新字节就收尾，
                     * 上限仍是 TAIL_MAX_MS，语义不变，只是不等满。 */
                    const int TAIL_STEP_MS  = 10;
                    const int TAIL_MAX_MS   = 100;
                    const int TAIL_QUIET_MS = 20;
                    int waited = 0, quiet = 0;
                    while (waited < TAIL_MAX_MS && quiet < TAIL_QUIET_MS
                           && total_len < max_len - 1) {
                        vTaskDelay(TAIL_STEP_MS / portTICK_PERIOD_MS);
                        waited += TAIL_STEP_MS;
                        ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_EG800_NUM,
                                                                   (size_t*)&bytes_available));
                        if (bytes_available > 0) {
                            int w2 = bytes_available;
                            if (w2 > max_len - total_len - 1) w2 = max_len - total_len - 1;
                            int len2 = (w2 > 0) ? uart_read_bytes(UART_EG800_NUM,
                                                                  (uint8_t*)buffer + total_len,
                                                                  w2, 0) : 0;
                            if (len2 > 0) {
                                total_len += len2;
                                buffer[total_len] = '\0';
                                quiet = 0;          // 还有话说，重新计静默
                                continue;
                            }
                        }
                        quiet += TAIL_STEP_MS;
                    }
                    break;
                }
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    
    return total_len;
}

#if ENABLE_MQTT
static bool wait_for_contains(const char* expected, uint32_t timeout_ms)
{
    char response[512];
    uint32_t start_time = xTaskGetTickCount();
    uart_read_busy = true;
    while ((xTaskGetTickCount() - start_time) < (timeout_ms / portTICK_PERIOD_MS)) {
        int len = uart_read_response(response, sizeof(response), 500);
        if (len > 0) {
            ESP_LOGI("UART1", "Received: %s", response);
            if (strstr(response, expected)) return true;
            if (strstr(response, "ERROR")) return false;
            // 常见MQTT错误URC
            if (strstr(response, "+QMTOPEN: 0,1") || strstr(response, "+QMTOPEN: 0,2")) return false;
            if (strstr(response, "+QMTCONN: 0,1") || strstr(response, "+QMTCONN: 0,2") ||
                strstr(response, "+QMTCONN: 0,3") || strstr(response, "+QMTCONN: 0,4") ||
                strstr(response, "+QMTCONN: 0,5")) return false;
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    ESP_LOGW(TAG, "Timeout waiting for: %s", expected);
    uart_read_busy = false;
    return false;
}

static bool wait_for_ok_then_contains(const char* expected_urc, uint32_t timeout_ms)
{
    char response[512];
    uint32_t start_time = xTaskGetTickCount();
    bool got_ok = false;
    uart_read_busy = true;
    while ((xTaskGetTickCount() - start_time) < (timeout_ms / portTICK_PERIOD_MS)) {
        int len = uart_read_response(response, sizeof(response), 500);
        if (len > 0) {
            ESP_LOGI("UART1", "Received: %s", response);
            if (!got_ok && strstr(response, "OK")) {
                got_ok = true; // 已收到OK，继续等待URC
            }
            if (strstr(response, expected_urc)) return true;
            if (strstr(response, "ERROR")) return false;
            // 常见MQTT错误URC
            if (strstr(response, "+QMTOPEN: 0,1") || strstr(response, "+QMTOPEN: 0,2")) return false;
            if (strstr(response, "+QMTCONN: 0,1") || strstr(response, "+QMTCONN: 0,2") ||
                strstr(response, "+QMTCONN: 0,3") || strstr(response, "+QMTCONN: 0,4") ||
                strstr(response, "+QMTCONN: 0,5")) return false;
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    ESP_LOGW(TAG, "Timeout waiting for URC: %s", expected_urc);
    uart_read_busy = false;
    return false;
}

/* ============ 开机/联网时序：等事件，不盲等 ============
 * 这几个函数全部建立在现有的 send_at_command / wait_for_ok / uart_read_response
 * 之上，不改 AT 收发内核。
 * ====================================================== */

/* 等 UART 静默。模组开机会连着吐 RDY / Call Ready / PB Done 等 URC，
 * 与其死等固定秒数，不如等它自己说完：连续 quiet_ms 没新字节就返回。 */
static void drain_until_quiet(uint32_t quiet_ms, uint32_t max_ms)
{
    char buf[256];
    TickType_t t0 = xTaskGetTickCount();
    TickType_t last_rx = t0;
    while ((xTaskGetTickCount() - t0) < pdMS_TO_TICKS(max_ms)) {
        int len = uart_read_bytes(UART_EG800_NUM, (uint8_t*)buf, sizeof(buf) - 1,
                                  pdMS_TO_TICKS(20));
        if (len > 0) {
            buf[len] = '\0';
            last_rx = xTaskGetTickCount();
            if (strstr(buf, "RDY") || strstr(buf, "Call Ready") ||
                strstr(buf, "SMS Ready") || strstr(buf, "PB Done")) {
                ESP_LOGI(TAG, "  开机 URC: %s", buf);
            }
        } else if ((xTaskGetTickCount() - last_rx) >= pdMS_TO_TICKS(quiet_ms)) {
            break;
        }
    }
}

/* 等模组开机后吐出 RDY。取代"打完 PWRKEY 盲等 2 秒"。 */
static bool wait_boot_rdy(uint32_t timeout_ms)
{
    char buf[256];
    char acc[512];
    size_t acc_len = 0;
    acc[0] = '\0';
    TickType_t t0 = xTaskGetTickCount();
    while ((xTaskGetTickCount() - t0) < pdMS_TO_TICKS(timeout_ms)) {
        int len = uart_read_bytes(UART_EG800_NUM, (uint8_t*)buf, sizeof(buf) - 1,
                                  pdMS_TO_TICKS(20));
        if (len > 0) {
            buf[len] = '\0';
            if (acc_len + len < sizeof(acc) - 1) {
                memcpy(acc + acc_len, buf, len);
                acc_len += len;
                acc[acc_len] = '\0';
            } else {
                acc_len = 0; acc[0] = '\0';       // 溢出就重来，只关心是否出现 RDY
            }
            if (strstr(acc, "RDY")) { return true; }
        }
    }
    return false;
}

/* 反复发 AT 直到模组应答。比"盲等 N 秒让模组稳定"准确：
 * 模组什么时候能收命令，它自己说了算。 */
static bool wait_modem_responsive(uint32_t timeout_ms)
{
    TickType_t t0 = xTaskGetTickCount();
    int tries = 0;
    while ((xTaskGetTickCount() - t0) < pdMS_TO_TICKS(timeout_ms)) {
        tries++;
        send_at_command("AT");
        if (wait_for_ok(500)) {
            ESP_LOGI(TAG, "  模组已响应 AT（第 %d 次，用时 %lu ms）", tries,
                     (unsigned long)((xTaskGetTickCount() - t0) * portTICK_PERIOD_MS));
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    ESP_LOGW(TAG, "  模组 %lu ms 内没有响应 AT", (unsigned long)timeout_ms);
    return false;
}

/* 轮询一条查询命令，直到响应里出现 want（或 want2）。用于等 SIM 就绪、等注网。 */
static bool poll_until(const char *cmd, const char *want, const char *want2,
                       uint32_t timeout_ms, uint32_t interval_ms, const char *what)
{
    char resp[256];
    TickType_t t0 = xTaskGetTickCount();
    TickType_t last_note = t0;
    int polls = 0;

    while ((xTaskGetTickCount() - t0) < pdMS_TO_TICKS(timeout_ms)) {
        polls++;
        send_at_command(cmd);
        int n = uart_read_response(resp, sizeof(resp), 1500);
        if (n > 0 && (strstr(resp, want) || (want2 && strstr(resp, want2)))) {
            ESP_LOGI(TAG, "  %s 就绪，用时 %lu ms（查询 %d 次）", what,
                     (unsigned long)((xTaskGetTickCount() - t0) * portTICK_PERIOD_MS), polls);
            return true;
        }
        // 长步骤每 5 秒报一次进度，免得刷屏也免得看不到在干嘛
        if ((xTaskGetTickCount() - last_note) >= pdMS_TO_TICKS(5000)) {
            last_note = xTaskGetTickCount();
            for (int k = 0; k < n; ++k) {
                if (resp[k] == '\r' || resp[k] == '\n') { resp[k] = ' '; }
            }
            ESP_LOGI(TAG, "  等 %s ... 已 %lu 秒，最近响应: %.60s", what,
                     (unsigned long)((xTaskGetTickCount() - t0) * portTICK_PERIOD_MS / 1000),
                     n > 0 ? resp : "(无)");
        }
        vTaskDelay(pdMS_TO_TICKS(interval_ms));
    }
    ESP_LOGW(TAG, "  %s 等待超时（%lu ms，查询 %d 次）", what,
             (unsigned long)timeout_ms, polls);
    return false;
}

/* ---- 时序打点：每一步花多久，直接从串口读 ---- */
#define AT_SEQ_MAX_STEPS 16
static struct { const char *name; uint32_t ms; } s_steps[AT_SEQ_MAX_STEPS];
static int        s_step_n = 0;
static TickType_t s_step_t0, s_seq_t0;

static void seq_begin(void) { s_step_n = 0; s_seq_t0 = s_step_t0 = xTaskGetTickCount(); }
static void seq_step_reset(void) { s_step_t0 = xTaskGetTickCount(); }

static void step(const char *name)
{
    TickType_t now = xTaskGetTickCount();
    uint32_t ms = (uint32_t)((now - s_step_t0) * portTICK_PERIOD_MS);
    s_step_t0 = now;
    if (s_step_n < AT_SEQ_MAX_STEPS) {
        s_steps[s_step_n].name = name;
        s_steps[s_step_n].ms   = ms;
        s_step_n++;
    }
    ESP_LOGI("TIMING", "  %-26s %5lu ms", name, (unsigned long)ms);
}

static void seq_report(const char *title)
{
    uint32_t total = (uint32_t)((xTaskGetTickCount() - s_seq_t0) * portTICK_PERIOD_MS);
    ESP_LOGW("TIMING", "======== %s 耗时汇总 ========", title);
    for (int i = 0; i < s_step_n; ++i) {
        ESP_LOGW("TIMING", "  %-26s %6lu ms  %3d%%", s_steps[i].name,
                 (unsigned long)s_steps[i].ms,
                 total ? (int)(s_steps[i].ms * 100 / total) : 0);
    }
    ESP_LOGW("TIMING", "  %-26s %6lu ms", "总计", (unsigned long)total);
}

/* ---- 网络诊断：把模组当前的 APN / 运营商 / 制式 / 信号打出来 ----
 * 纯查询，不改任何配置。4 条 AT 约 120 ms，对上报耗时影响可忽略。
 * 放在注网成功之后：这时 COPS/QNWINFO 才有有效值。
 * 不想要的话把 MODEM_NET_DIAG 定义成 0。 */
#ifndef MODEM_NET_DIAG
#define MODEM_NET_DIAG 1
#endif

#if MODEM_NET_DIAG
static void net_diag_one(const char *cmd, const char *what)
{
    char resp[256];
    send_at_command(cmd);
    int n = uart_read_response(resp, sizeof(resp), 2000);
    if (n <= 0) { ESP_LOGW(TAG, "  [诊断] %-10s %s -> 无响应", what, cmd); return; }
    /* 换行压成空格，一行一条好看 */
    for (int k = 0; k < n; ++k) {
        if (resp[k] == '\r' || resp[k] == '\n') { resp[k] = ' '; }
    }
    ESP_LOGI(TAG, "  [诊断] %-10s %s", what, resp);
}

static void net_diag_dump(void)
{
    ESP_LOGI(TAG, "---- 网络诊断（只读，不改配置）----");
    net_diag_one("AT+CGDCONT?", "APN/PDP");   // context 1 实际用的 APN
    net_diag_one("AT+COPS?",    "运营商");     // 注册到谁、什么制式
    net_diag_one("AT+QNWINFO",  "频段");       // 制式/频段，为后面锁频做准备
    net_diag_one("AT+CSQ",      "信号");       // rssi,ber  (99,99 = 未知)
    ESP_LOGI(TAG, "-----------------------------------");
}
#else
static void net_diag_dump(void) { }
#endif
bool mqtt_init_network(void)
{
    ESP_LOGI(TAG, "Initializing 4G network...");
    seq_step_reset();   // 每次 attempt 重新计时

    /* 0. 等模组把开机 URC 说完。原来是死等 3 秒，现在静默 300 ms 就走。 */
    drain_until_quiet(MODEM_DRAIN_QUIET_MS, MODEM_DRAIN_MAX_MS);
    step("清开机 URC");

    /* 1. 等模组能收命令。
     * 原来是"AT 握手最多 5 次"+ 后面 CFUN 之后再盲等 3 秒"让模组稳定"，
     * 现在合并成一件事：一直发 AT 直到它回 OK —— 回了就说明真的稳定了。 */
    if (!wait_modem_responsive(MODEM_AT_READY_TIMEOUT_MS)) {
        ESP_LOGW(TAG, "Module not responding to AT command");
        return false;
    }
    step("等模组响应 AT");

    // 2. 关回显 + 读 IMEI
    send_at_command("ATE0");
    wait_for_ok(1000);
    device_fetch_imei();
    step("ATE0 + 读 IMEI");

    /* 3. 射频功能。先查再设 —— 刚上电通常本来就是 CFUN=1，
     * 无条件发 CFUN=1 会让模组重跑一遍射频初始化，白白多花几秒。 */
    send_at_command("AT+CFUN?");
    bool cfun_ok = wait_for_contains("+CFUN: 1", 2000);
    if (!cfun_ok) {
        ESP_LOGI(TAG, "  CFUN 不是 1，设置全功能模式...");
        send_at_command("AT+CFUN=1");
        if (!wait_for_ok(15000)) {
            ESP_LOGW(TAG, "Failed to set full functionality mode");
            return false;
        }
    } else {
        ESP_LOGI(TAG, "  CFUN 已经是 1，跳过设置");
    }
    step("CFUN 查/设");

    /* 4. 等 SIM 就绪。原来只发一次 CPIN? 就往下走，模组还没准备好也不管。 */
    if (!poll_until("AT+CPIN?", "READY", NULL,
                    MODEM_SIM_TIMEOUT_MS, 150, "SIM 卡")) {
        ESP_LOGW(TAG, "SIM card not ready");
    }
    step("等 SIM 就绪");

    // 5. APN（只在第一次设）
    if (strlen(APN_NAME) > 0 && !apn_configured) {
        char apn_cmd[128];
        snprintf(apn_cmd, sizeof(apn_cmd), "AT+CGDCONT=1,\"IP\",\"%s\"", APN_NAME);
        send_at_command(apn_cmd);
        if (!wait_for_ok(5000)) {
            ESP_LOGW(TAG, "Failed to set APN");
            return false;
        }
        apn_configured = true;
        ESP_LOGI(TAG, "  APN 已配置");
    }

    /* 6. 等注册上网。
     * 原代码没有这一步，没注上网就直接发 CGACT，会一路卡到 CGACT 的 30 秒
     * 超时然后整轮判失败重来。先确认注册再激活，失败也能早点知道原因。
     * +CEREG: n,1 = 已注册本网，n,5 = 漫游注册。 */
    if (!poll_until("AT+CEREG?", ",1", ",5",
                    MODEM_REG_TIMEOUT_MS, 400, "网络注册")) {
        ESP_LOGW(TAG, "未能在超时内注册到网络（信号 / SIM / 天线？）");
        return false;
    }
    step("等注网");

    /* 注网成功后 dump 一次：APN 是什么、注到哪家、什么频段、信号多少。 */
    net_diag_dump();
    step("网络诊断");

    /* 7. 激活 PDP。先查，已经激活就跳过。 */
    send_at_command("AT+CGACT?");
    if (wait_for_contains("+CGACT: 1,1", 2000)) {
        ESP_LOGI(TAG, "  PDP 已激活，跳过");
    } else {
        send_at_command("AT+CGACT=1,1");
        if (!wait_for_ok(30000)) {
            ESP_LOGW(TAG, "Failed to activate PDP context");
            return false;
        }
    }
    step("激活 PDP");

    ESP_LOGI(TAG, "4G network initialized successfully");
    return true;
}

/* force=false：没有会话就直接返回，不发无用的 QMTDISC/QMTCLOSE。
 * force=true ：重试路径用，socket 可能半开，必须强拆。 */
static bool do_disconnect(bool force)
{
    if (!mqtt_session_open && !force) {
        ESP_LOGI(TAG, "无 MQTT 会话，跳过 QMTDISC/QMTCLOSE");
        return true;
    }

    ESP_LOGI(TAG, "Closing MQTT connection...");

    // 1. 断开MQTT客户端连接
    send_at_command("AT+QMTDISC=0");
    bool disc_ok = wait_for_ok_then_contains("+QMTDISC: 0,0", 5000);

    // 2. 关闭MQTT网络连接
    send_at_command("AT+QMTCLOSE=0");
    bool close_ok = wait_for_ok_then_contains("+QMTCLOSE: 0,0", 5000);

    /* 收到 +QMTCLOSE: 0,0 就说明真的关完了，不需要再盲等 1 秒。
     * 只有在没等到 URC（可能是 ERROR / 超时）时才给模组一点缓冲。 */
    if (!close_ok) {
        ESP_LOGW(TAG, "未收到 QMTCLOSE URC（disc=%d close=%d），补 200ms 缓冲",
                 disc_ok, close_ok);
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }

    ESP_LOGI(TAG, "MQTT connection closed");
    mqtt_session_open = false;
    return true;
}

bool mqtt_disconnect_broker(void)
{
    return do_disconnect(false);
}

bool mqtt_connect_broker(void)
{
    ESP_LOGI(TAG, "Connecting to MQTT broker...");
    
    // 最多尝试3次连接
    for (int attempt = 1; attempt <= 3; ++attempt) {
        ESP_LOGI(TAG, "Attempt %d/3: Cleaning up and connecting...", attempt);
        /* 第 1 次：模组刚开机、没有会话时直接跳过清理。
         * 第 2/3 次：上一轮可能把 socket 留成半开，强制拆掉再来。 */
        bool cleaned = (mqtt_session_open || attempt > 1);
        do_disconnect(attempt > 1);
        if (cleaned) {
            vTaskDelay(300 / portTICK_PERIOD_MS);   // 只在真拆过连接时给缓冲
        }

        // 1. 配置MQTT服务器版本
        ESP_LOGI(TAG, "Step 1: Setting MQTT version...");
        send_at_command("AT+QMTCFG=\"version\",0,4");
        if (!wait_for_ok(3000)) {
            ESP_LOGW(TAG, "Failed to set MQTT version (attempt %d)", attempt);
            goto connect_fail_delay;
        }
        
        // 2. 设置MQTT保持连接时间
        ESP_LOGI(TAG, "Step 2: Setting keepalive interval...");
        char keepalive_cmd[64];
        snprintf(keepalive_cmd, sizeof(keepalive_cmd), "AT+QMTCFG=\"keepalive\",0,%d", MQTT_KEEPALIVE);
        send_at_command(keepalive_cmd);
        if (!wait_for_ok(3000)) {
            ESP_LOGW(TAG, "Failed to set keepalive (attempt %d)", attempt);
            goto connect_fail_delay;
        }
        
        // 3. 打开MQTT网络连接
        ESP_LOGI(TAG, "Step 3: Opening MQTT network connection...");
        char open_cmd[256];
        snprintf(open_cmd, sizeof(open_cmd), "AT+QMTOPEN=0,\"%s\",%d", MQTT_BROKER_HOST, MQTT_BROKER_PORT);
        send_at_command(open_cmd);
        if (!wait_for_ok_then_contains("+QMTOPEN: 0,0", 15000)) {
            ESP_LOGW(TAG, "Failed to open MQTT network connection (attempt %d)", attempt);
            goto connect_fail_delay;
        }
        
        // 4. 连接MQTT客户端
        ESP_LOGI(TAG, "Step 4: Connecting MQTT client...");
        char conn_cmd[256];
        snprintf(conn_cmd, sizeof(conn_cmd), "AT+QMTCONN=0,\"%s\",\"%s\",\"%s\"", 
                 MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD);
        send_at_command(conn_cmd);
        if (!wait_for_ok_then_contains("+QMTCONN: 0,0,0", 10000)) {
            ESP_LOGW(TAG, "Failed to connect MQTT client (attempt %d)", attempt);
            goto connect_fail_delay;
        }

        ESP_LOGI(TAG, "Connected to MQTT broker successfully (attempt %d)", attempt);
        step("连 broker");
        mqtt_session_open = true;
        return true;

connect_fail_delay:
        mqtt_disconnect_broker();
        mqtt_session_open = false;
        if (attempt < 3) {
            ESP_LOGW(TAG, "Connect attempt %d failed, retry after delay...", attempt);
            vTaskDelay(2000 / portTICK_PERIOD_MS);
        }
    }

    ESP_LOGW(TAG, "All connection attempts failed");
    return false;
}

bool mqtt_publish_message(const char* topic, const char* message)
{
    ESP_LOGI(TAG, "Publishing to topic: %s", topic);
    
    char pub_cmd[512];
    snprintf(pub_cmd, sizeof(pub_cmd), "AT+QMTPUBEX=0,0,%d,0,\"%s\",%d", 
             MQTT_QOS_LEVEL, topic, (int)strlen(message));
    send_at_command(pub_cmd);
    
    if (!wait_for_contains(">", 3000)) {
        return false;
    }
    
    // 发送消息内容
    uart_write_bytes(UART_EG800_NUM, message, strlen(message));

    // 等待发布完成URC
    if (!wait_for_contains("+QMTPUBEX: 0,0,0", 5000)) {
        ESP_LOGW(TAG, "Publish ack not received for msgId=0");
        return false;
    }
    
    ESP_LOGI(TAG, "Message published successfully");
    step("发布消息");
    return true;
}

void mqtt_poll_urc(void)
{
    if (mqtt_state != MQTT_STATE_CONNECTED) {
        return;
    }
    if (uart_read_busy || s_modem_debug_hold) {
        return;
    }

    size_t bytes_available = 0;
    ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_EG800_NUM, &bytes_available));
    if (bytes_available == 0) {
        return;
    }

    char buf[512];
    int len = uart_read_bytes(UART_EG800_NUM, (uint8_t*)buf, sizeof(buf) - 1, 20 / portTICK_PERIOD_MS);
    if (len <= 0) {
        return;
    }
    buf[len] = '\0';
    ESP_LOGI(TAG, "URC buffer: %s", buf);

    // 连接状态上报：1=已断开
    if (strstr(buf, "+QMTSTAT: 0,1")) {
        ESP_LOGW(TAG, "+QMTSTAT indicates disconnected by peer/network (0,1). Cleaning up...");
        mqtt_disconnect_broker();
        mqtt_state = MQTT_STATE_DISCONNECTED;
    }
}
#endif // ENABLE_MQTT

void modem_power_cycle(void)
{
    /* Cattle_T: EG800。3.8V 轨默认是关的（MODEM_RAIL_ON_AT_BOOT=0），
     * 这里先开轨并等稳压建立，再打 PWRKEY 脉冲。
     * IO33(PWRKEY) 经 Q8 反相，置 1 相当于把 PWRKEY 拉低。 */
    ESP_LOGI("MODEM", "Power-cycle EG800: 开 3.8V 轨 -> PWRKEY 脉冲 -> 等 RDY");
    modem_power_enable(true);
    vTaskDelay(MODEM_RAIL_SETTLE_MS / portTICK_PERIOD_MS);

    /* PWRKEY 脉冲宽度：EG800 规格是拉低 ≥500ms，这里默认 600ms。
     * 万一 600ms 边界不够（等不到 RDY 且 STATUS 仍为 0），自动补一发
     * MODEM_PWRKEY_PULSE_RETRY_MS 的长脉冲，不至于整轮上报失败。 */
    uint32_t pulse = MODEM_PWRKEY_PULSE_MS;
    bool rdy = false;
    uint32_t ms = 0;

    for (int attempt = 1; attempt <= 2; ++attempt) {
        gpio_set_level(QT_POWER_PIN, 0);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        gpio_set_level(QT_POWER_PIN, 1);                   // 置 1 = 模组 ~PWRKEY 拉低
        vTaskDelay(pulse / portTICK_PERIOD_MS);
        gpio_set_level(QT_POWER_PIN, 0);                   // 释放

        /* 等模组自己吐 RDY，而不是盲等固定秒数。 */
        TickType_t t0 = xTaskGetTickCount();
#if ENABLE_MQTT
        rdy = wait_boot_rdy(MODEM_BOOT_RDY_TIMEOUT_MS);
#else
        /* ENABLE_MQTT=0 时没有 UART1 收发，只能退回固定等待 */
        vTaskDelay(MODEM_BOOT_RDY_TIMEOUT_MS / 4 / portTICK_PERIOD_MS);
        rdy = false;
#endif
        ms = (uint32_t)((xTaskGetTickCount() - t0) * portTICK_PERIOD_MS);

        if (rdy) {
            ESP_LOGI("MODEM", "收到 RDY，模组已开机（脉冲 %lu ms，RDY 用时 %lu ms）",
                     (unsigned long)pulse, (unsigned long)ms);
            return;
        }
        if (modem_is_powered()) {
            ESP_LOGW("MODEM", "没收到 RDY 但 STATUS=1，认为已开机（脉冲 %lu ms）",
                     (unsigned long)pulse);
            return;
        }
        if (attempt == 1) {
            ESP_LOGW("MODEM", "%lu ms 脉冲没能开机（RDY 无 / STATUS=0），补一发 %d ms 长脉冲",
                     (unsigned long)pulse, MODEM_PWRKEY_PULSE_RETRY_MS);
            pulse = MODEM_PWRKEY_PULSE_RETRY_MS;
        }
    }

    ESP_LOGW("MODEM", "两次 PWRKEY 都没等到 RDY，继续（后面用 AT 轮询兜底）。STATUS(IO%d)=%d",
             MODEM_STATUS_PIN, gpio_get_level(MODEM_STATUS_PIN));
}

static void modem_power_off_soft(void)
{
    ESP_LOGI("MODEM", "Soft power off EG800 (AT+CFUN=0)");
#if ENABLE_MQTT
    send_at_command("AT+CFUN=0");
    wait_for_ok(10000);
#endif
}

/* 彻底关掉模组：先让它自己正常注销关机（AT+QPOWD），再断 3.8V 轨。
 *
 * 为什么仍然要先 QPOWD，而不是直接断电：断电是硬掉电，模组来不及向网络注销，
 * 基站侧还挂着这个上下文，下次入网要走完整流程、耗时更长；SIM 文件系统也有被
 * 写坏的风险。所以正常路径永远是 QPOWD -> 等 POWERED DOWN -> 断轨。
 *
 * 改版硬件（MODEM_EN_ACTIVE_HIGH=1）之后断轨是真断：IO48 高有效直驱 U27 的 EN，
 * 深睡期间 IO48 变高阻，板上的下拉把 EN 按在地上，3.8V 轨整个休眠期都是断的。
 * 初版硬件是反的（EN 上拉到 5V0），浮空会让轨自己恢复通电，见 POWER_BUDGET.md。 */
static void modem_power_down_full(void)
{
#if ENABLE_MQTT
    ESP_LOGI("MODEM", "Full power down: AT+QPOWD=1 ...");
    send_at_command("AT+QPOWD=1");
    // 模组会回 OK 然后输出 POWERED DOWN，给它时间正常注销
    if (!wait_for_contains("POWERED DOWN", MODEM_QPOWD_TIMEOUT_MS)) {
        ESP_LOGW("MODEM", "没等到 POWERED DOWN，退回 AT+CFUN=0");
        modem_power_off_soft();
    }
#endif
    modem_power_enable(false);   // 断 3.8V 轨
    ESP_LOGI("MODEM", "Modem powered down, 3.8V rail disabled");
}

#if ENABLE_MQTT
// 电压与 SOC 统一由 battery_module 提供，这里不再另起一份实现。

// 唤醒模组之前采下的空载电压。模组入网/发射时电流冲击会把电池拉低，
// 用那时的读数换算 SOC 会明显偏低，所以在上电前先记一笔。
static float s_vbat_idle = 0.0f;

void battery_snapshot_idle(void)
{
    s_vbat_idle = read_battery_voltage();
    ESP_LOGI("BAT", "Idle VBAT snapshot: %.2f V (SOC %d%%)",
             s_vbat_idle, estimate_soc_from_voltage(s_vbat_idle));
}

// 构造电池 JSON。放成独立函数是为了能在联网拿到 IMEI 之后再调用。
static int build_battery_json(char *payload, size_t payload_sz)
{
    // 电压取板上 IO1 (IBL) 的实测值：ADC 读数 × 分压比 1.303。
    // 优先用唤醒模组之前的空载快照，拿不到才现场读一次。
    float voltage = (s_vbat_idle > 0.5f) ? s_vbat_idle : read_battery_voltage();
    int soc = estimate_soc_from_voltage(voltage);

    float temperature = 31.4f;
    int health = 96;
    int cycleCount = 182;
    float estimatedRangeKm = 120.5f;
    const char* chargingStatus = "discharging";
    
    // GPS数据
    double speed_kmh = gnss_last_location.spd_kn * 1.852;
    int heading = (int)(gnss_last_location.crs_deg + 0.5);
    float altitude = 156.0f;
    int accuracy = 5;
    
    // 时间戳
    uint64_t ts_ms = (uint64_t)xTaskGetTickCount() * 1000 / configTICK_RATE_HZ;

    /* 没有有效定位时整段省掉 gps 字段，而不是发 0,0。
     * 后端对 gps 缺失是容忍的（telemetryData.gps = undefined，日志打 "GPS: none"），
     * 但 0,0 会在地图上画出一个几内亚湾的假点。 */
    char gps_part[192];
    if (gnss_last_location.valid) {
        snprintf(gps_part, sizeof(gps_part),
            ",\"gps\":{"
                "\"lat\":%.6f,"
                "\"lng\":%.6f,"
                "\"speed\":%.1f,"
                "\"heading\":%d,"
                "\"altitude\":%.1f,"
                "\"accuracy\":%d"
            "}",
            gnss_last_location.lat_deg,
            gnss_last_location.lon_deg,
            speed_kmh,
            heading,
            altitude,
            accuracy);
    } else {
        gps_part[0] = '\0';
    }

    /* fleet manager 读的是**顶层**字段（src/services/mqtt-service.ts:
     * handleBatteryMessage → data.soc / data.voltage / data.temperature /
     * data.health / data.cycleCount / data.estimatedRangeKm /
     * data.chargingStatus / data.alerts / data.gps.*）。
     * 所以电池各项不能再包在 "battery": {} 里，且字段名是 estimatedRangeKm。
     * deviceId 与 timestamp 后端并不读取（设备身份取自主题、时间戳用服务器本地时间），
     * 这里保留只是为了排查时方便对照。 */
    int n = snprintf(payload, payload_sz,
    "{"
        "\"deviceId\":\"%s\","
        "\"timestamp\":%llu,"
        "\"soc\":%d,"
        "\"voltage\":%.2f,"
        "\"temperature\":%.1f,"
        "\"health\":%d,"
        "\"cycleCount\":%d,"
        "\"estimatedRangeKm\":%.1f,"
        "\"chargingStatus\":\"%s\","
        "\"alerts\":[]"
        "%s"
    "}",
        device_get_id(),
        ts_ms,
        soc,
        voltage,
        temperature,
        health,
        cycleCount,
        estimatedRangeKm,
        chargingStatus,
        gps_part);
    return n;
}

bool publish_battery_payload(void)
{
    char payload[512];
    char topic[80];

    for (int attempt = 1; attempt <= MQTT_SEND_MAX_RETRY; ++attempt) {
        ESP_LOGI("BAT", "MQTT publish attempt %d/%d", attempt, MQTT_SEND_MAX_RETRY);
        if (!mqtt_init_network()) continue;
        if (!mqtt_connect_broker()) continue;

        // IMEI 是在 mqtt_init_network() 里读到的，主题和 payload 都要等它就位
        mqtt_build_topic(topic, sizeof(topic), MQTT_TOPIC_SUB_BATTERY);
        int n = build_battery_json(payload, sizeof(payload));
        if (n <= 0 || n >= (int)sizeof(payload)) {
            ESP_LOGE("BAT", "Battery JSON build failed");
            mqtt_disconnect_broker();
            return false;
        }

        ESP_LOGI("BAT", "Topic: %s\nPayload:\n%s", topic, payload);
        bool ok = mqtt_publish_message(topic, payload);
        mqtt_disconnect_broker();
        if (ok) return true;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    return false;
}

// 注意：fleet manager 只订阅 fleet/+/battery、fleet/+/status、fleet/chargenode/+，
// 没有订阅 fleet/+/gps。GPS 数据是随 battery payload 的 "gps" 字段一起上去的，
// 本函数目前没有调用点，保留备用。
bool publish_gps_payload(void)
{
    if (!gnss_last_location.valid) {
        ESP_LOGW("GPS", "No valid GPS location to publish");
        return false;
    }
    
    char payload[256];
    uint64_t ts_ms = (uint64_t)xTaskGetTickCount() * 1000 / configTICK_RATE_HZ;
    double speed_kmh = gnss_last_location.spd_kn * 1.852;
    int heading = (int)(gnss_last_location.crs_deg + 0.5);
    
    int n = snprintf(payload, sizeof(payload),
    "{\n"
    "  \"deviceId\": \"%s\",\n"
    "  \"timestamp\": %llu,\n"
    "  \"lat\": %.6f,\n"
    "  \"lng\": %.6f,\n"
    "  \"speed\": %.1f,\n"
    "  \"heading\": %d,\n"
    "  \"altitude\": 156.0,\n"
    "  \"accuracy\": 5\n"
    "}",
        device_get_id(),
        ts_ms,
        gnss_last_location.lat_deg,
        gnss_last_location.lon_deg,
        speed_kmh,
        heading);
        
    if (n <= 0 || n >= (int)sizeof(payload)) {
        ESP_LOGE("GPS", "GPS JSON build failed");
        return false;
    }

    char topic[80];

    for (int attempt = 1; attempt <= MQTT_SEND_MAX_RETRY; ++attempt) {
        ESP_LOGI("GPS", "MQTT publish attempt %d/%d", attempt, MQTT_SEND_MAX_RETRY);
        if (!mqtt_init_network()) continue;
        if (!mqtt_connect_broker()) continue;
        mqtt_build_topic(topic, sizeof(topic), MQTT_TOPIC_SUB_GPS);
        ESP_LOGI("GPS", "Topic: %s\nPayload:\n%s", topic, payload);
        bool ok = mqtt_publish_message(topic, payload);
        mqtt_disconnect_broker();
        if (ok) return true;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    return false;
}

/* ================= 手动上报（IO0 双击触发） =================
 * 不管有没有 GNSS 定位都发一条。跑在独立的一次性任务里，原因有两个：
 *   1) 整个流程最长可能几分钟，不能占着按键任务不放；
 *   2) MQTT 那一串函数的局部缓冲区加起来要 2.5KB 以上，按键任务的栈不够。
 */
static volatile bool s_manual_pub_running = false;

static void manual_publish_task(void *pv)
{
    (void)pv;
    bool powered_here = false;

    if (s_modem_debug_hold) {
        ESP_LOGW("PUB", "模组处于调试保持模式，忽略手动上报");
        s_manual_pub_running = false;
        vTaskDelete(NULL);
        return;
    }

    // 不等待：周期任务正在用模组的话，这次手动请求直接放弃，避免两个任务抢 UART1
    if (!modem_bus_lock(0)) {
        ESP_LOGW("PUB", "Cycle task is using the modem, manual publish skipped");
        led_blink(LED_CONN_PIN, 1, 800, 200);
        s_manual_pub_running = false;
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI("PUB", "===== Manual publish (IO0 double-click) =====");
    ESP_LOGI("PUB", "GNSS fix: %s", gnss_last_location.valid ? "valid" : "none (gps field omitted)");

    // 模组上电前先采空载电压
    battery_snapshot_idle();

    led_blink(LED_CONN_PIN, GPS_UPLOAD_BLINK_TIMES,
              GPS_UPLOAD_BLINK_ON_MS, GPS_UPLOAD_BLINK_OFF_MS);

    seq_begin();    // 无论模组开没开都要开始计时，否则总计是从上电算起的假数
    if (!modem_is_powered()) {
        ESP_LOGI("PUB", "Modem is off, powering up...");
        modem_power_cycle();
        powered_here = true;
        step("模组开机");
    } else {
        ESP_LOGI("PUB", "Modem already on (STATUS=1)，跳过开机");
        step("模组开机(已开,跳过)");
    }

    bool ok = publish_battery_payload();
    ESP_LOGI("PUB", "Manual publish %s", ok ? "SUCCESS" : "FAILED");
    seq_report(ok ? "手动上报（成功）" : "手动上报（失败）");

    // 结果指示：成功闪 3 下，失败长亮一下
    if (ok) {
        led_blink(LED_CONN_PIN, 3, 120, 120);
    } else {
        led_blink(LED_CONN_PIN, 1, 1000, 200);
    }

    // 只关我们自己开起来的模组，避免打断周期任务的状态
    if (powered_here) {
        modem_power_off_soft();
    }

    modem_bus_unlock();
    s_manual_pub_running = false;
    vTaskDelete(NULL);
}

/* ===================== 模组调试保持模式 =====================
 * 长按 IO0 进入。只做一件事：确认模组开着；没开就打一次 PWRKEY。
 * 之后 ESP32 什么都不做 —— 不发 AT、不碰 GNSS、不闪灯、不再动任何引脚，
 * 把板子完整交给你用模组的 USB 口调试。只能靠复位退出。
 * ============================================================ */
static void modem_debug_hold_task(void *pv)
{
    (void)pv;

    /* 永久占住模组总线锁，这样双击的手动上报会自己跳过，不来抢 UART1。 */
    modem_bus_lock(portMAX_DELAY);
    s_modem_debug_hold = true;

    ESP_LOGW("HOLD", "=== 模组调试保持模式 ===");

    // 1. 确保 3.8V 轨打开
    modem_power_enable(true);
    vTaskDelay(pdMS_TO_TICKS(200));

    // 2. 看 STATUS：为 0 就是没开机，打一次 PWRKEY
    int st = gpio_get_level(MODEM_STATUS_PIN);
    ESP_LOGW("HOLD", "STATUS(IO%d) = %d", MODEM_STATUS_PIN, st);

    if (st == 0) {
        ESP_LOGW("HOLD", "模组未开机，打 PWRKEY：IO%d 置 1 保持 2 秒再释放", QT_POWER_PIN);
        gpio_set_level(QT_POWER_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(2000));
        gpio_set_level(QT_POWER_PIN, 0);

        // 等模组起来，每秒看一次 STATUS，最多 10 秒
        for (int i = 1; i <= 10; ++i) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            st = gpio_get_level(MODEM_STATUS_PIN);
            ESP_LOGW("HOLD", "  +%d s  STATUS = %d", i, st);
            if (st) { break; }
        }
    } else {
        ESP_LOGW("HOLD", "模组已经开着，不打 PWRKEY");
    }

    ESP_LOGW("HOLD", "%s", st ? "模组已开机，交给 USB 侧调试"
                              : "STATUS 仍为 0，模组没起来（查 3.8V 轨 / PWRKEY 到 J10.7 的通路）");
    ESP_LOGW("HOLD", "ESP32 停止一切操作。退出请复位板子。");

    // 3. 到此为止，什么都不做
    vTaskSuspend(NULL);
    while (1) { vTaskDelay(portMAX_DELAY); }
}

void mqtt_modem_debug_hold(void)
{
    if (s_modem_debug_hold) {
        ESP_LOGW("HOLD", "已经在调试保持模式了");
        return;
    }
    xTaskCreate(modem_debug_hold_task, "modem_hold", 3072, NULL, 6, NULL);
}

void mqtt_manual_publish_request(void)
{
    if (s_manual_pub_running) {
        ESP_LOGW("PUB", "Manual publish already in progress, ignoring");
        return;
    }
    s_manual_pub_running = true;
    if (xTaskCreate(manual_publish_task, "manual_pub", 6144, NULL, 7, NULL) != pdPASS) {
        s_manual_pub_running = false;
        ESP_LOGE("PUB", "Failed to create manual publish task");
    }
}
#else  // !ENABLE_MQTT
void mqtt_manual_publish_request(void)
{
    ESP_LOGW("PUB", "Manual publish ignored: ENABLE_MQTT=0");
}
#endif // ENABLE_MQTT

#if ENABLE_DEEP_SLEEP
/* 进入 deep sleep。不返回：醒来等同于复位，从 app_main 重新跑一遍。
 * 跨睡眠不保留任何 RAM 状态——IMEI 每轮重读一次（模组本来就要重启），
 * APN 同理，成本可以忽略。 */
static void enter_deep_sleep_cycle(uint32_t seconds)
{
    ESP_LOGI("SLEEP", "ESP32 deep sleep %lu 秒，醒来后从 app_main 重新开始",
             (unsigned long)seconds);

    /* 加速度计挂在 3V3_ALWAYS_ON 上，MCU 睡了它不会跟着睡。
     * 必须显式写 power-down，否则高分辨率 100 Hz 会一直吃 ~185 µA
     * （电池侧约 360 µA，比整机其余底噪加起来还多）。 */
    motion_sensor_power_down();

    // 锁住 RTC GPIO 的电平（GNSS 供电必须保持，围栏引脚必须保持低）
    deep_sleep_prepare_pins();

    esp_sleep_enable_timer_wakeup((uint64_t)seconds * 1000000ULL);

#if DEEPSLEEP_WAKE_ON_BUTTON
    /* IO0 低电平唤醒。
     * 坑：IO0 的高电平只靠 gpio_init() 里的内部上拉，板上没有外部上拉电阻
     * （原理图 IO0 只接 SW2 到地 + 测试点 TP6）。进 deep sleep 后数字域的上拉
     * 会被关闭，引脚浮空 -> 读成低 -> ext0 立刻触发 -> 刚睡下就醒，死循环。
     * 所以必须显式打开 RTC 域的上拉，并关掉下拉。 */
    rtc_gpio_pullup_en((gpio_num_t)BUTTON_PIN);
    rtc_gpio_pulldown_dis((gpio_num_t)BUTTON_PIN);

    int io0 = gpio_get_level(BUTTON_PIN);
    if (io0 == 0) {
        // 按键卡住或被按着，此时挂 ext0 必然立刻唤醒，干脆不挂
        ESP_LOGW("SLEEP", "IO%d 当前为低，跳过按键唤醒源（否则会立刻被唤醒）", BUTTON_PIN);
        ESP_LOGI("SLEEP", "唤醒源：仅定时器 %lu 秒", (unsigned long)seconds);
    } else {
        esp_sleep_enable_ext0_wakeup((gpio_num_t)BUTTON_PIN, 0);
        ESP_LOGI("SLEEP", "唤醒源：定时器 %lu 秒 + IO%d 按键（RTC 上拉已开，当前电平=%d）",
                 (unsigned long)seconds, BUTTON_PIN, io0);
    }
#else
    ESP_LOGI("SLEEP", "唤醒源：定时器 %lu 秒", (unsigned long)seconds);
#endif

    vTaskDelay(pdMS_TO_TICKS(100));   // 让日志吐完
    esp_deep_sleep_start();
}
#endif // ENABLE_DEEP_SLEEP

void gps_mqtt_cycle_task(void* pv)
{
    ESP_LOGI("GPS", "gps_mqtt_cycle_task started: sleep=%d min, fix timeout=%d s, mqtt retries=%d",
             GPS_CYCLE_SLEEP_MIN, GPS_FIX_TIMEOUT_SEC, MQTT_SEND_MAX_RETRY);

    while (1) {
        // 1) 启动 GNSS。若本次是从 deep sleep 醒来，模组还在 Backup 里，
        //    必须用 RESET_N 脉冲叫醒它（Backup 只能硬件复位唤醒）。
        bool powered_modem = false;
        gnss_enable(true);
        gnss_reset_release();

#if ENABLE_DEEP_SLEEP
        esp_sleep_wakeup_cause_t wc = esp_sleep_get_wakeup_cause();
        if (wc == ESP_SLEEP_WAKEUP_TIMER || wc == ESP_SLEEP_WAKEUP_EXT0) {
            ESP_LOGI("GPS", "从 deep sleep 醒来，先把 GNSS 从 Backup 唤醒");
            gnss_wake_from_standby2();
        }
#endif

        // 2) 连续模式下等待 GNSS 定位
        gnss_last_location.valid = false;
        gnss_reset_state_for_ttff();
        ESP_LOGI("GPS", "Waiting GNSS fix (timeout %ds)...", GPS_FIX_TIMEOUT_SEC);
        TickType_t t0 = xTaskGetTickCount();
        while ((xTaskGetTickCount() - t0) < pdMS_TO_TICKS(GPS_FIX_TIMEOUT_SEC * 1000)) {
            if (gnss_last_location.valid) { break; }
            vTaskDelay(200 / portTICK_PERIOD_MS);
        }

        // 2.1) 定位成功：在串口打一段完整的定位信息
        if (gnss_last_location.valid) {
            uint32_t ttff_ms = (uint32_t)((xTaskGetTickCount() - t0) * portTICK_PERIOD_MS);
            ESP_LOGI("GPS", "================ GNSS FIX ACQUIRED ================");
            ESP_LOGI("GPS", "  UTC       : %s", gnss_last_location.time_utc);
            ESP_LOGI("GPS", "  Latitude  : %.6f deg", gnss_last_location.lat_deg);
            ESP_LOGI("GPS", "  Longitude : %.6f deg", gnss_last_location.lon_deg);
            ESP_LOGI("GPS", "  Speed     : %.2f kn (%.1f km/h)",
                     gnss_last_location.spd_kn, gnss_last_location.spd_kn * 1.852);
            ESP_LOGI("GPS", "  Course    : %.1f deg", gnss_last_location.crs_deg);
            ESP_LOGI("GPS", "  Satellites: %d, HDOP %.1f, fixQ %d, dim %dD",
                     gnss_sats, gnss_hdop, gnss_fix_q, gnss_dim);
            ESP_LOGI("GPS", "  Time to fix: %lu ms", (unsigned long)ttff_ms);
            ESP_LOGI("GPS", "===================================================");
#if ENABLE_DEEP_SLEEP
            app_note_fix_ok();
#endif
        }

        // 3) 若拿到定位，则唤醒 EG800 并进行 MQTT
#if ENABLE_MQTT
        if (gnss_last_location.valid) {
            // 独占模组总线，手动上报任务此时会主动让路
            modem_bus_lock(portMAX_DELAY);

            // 模组还没上电，此刻是空载，先把电池电压记下来
            battery_snapshot_idle();

            // 上报前闪一下 CONN_LED，指示"准备上云"
            led_blink(LED_CONN_PIN, GPS_UPLOAD_BLINK_TIMES,
                      GPS_UPLOAD_BLINK_ON_MS, GPS_UPLOAD_BLINK_OFF_MS);

            ESP_LOGI("GPS", "GNSS fix acquired, bringing up EG800 for MQTT publish");
            /* 与双击手动上报走同一套判断：深睡醒来时 3.8V 轨一定是关的，
             * 但 ENABLE_DEEP_SLEEP=0 的连续运行模式下可能已经开着，别重复打 PWRKEY。 */
            seq_begin();
            if (!modem_is_powered()) {
                ESP_LOGI("MODEM", "Wake EG800: PWRKEY pulse to publish");
                modem_power_cycle();
                step("模组开机");
            } else {
                ESP_LOGI("MODEM", "模组已在运行（STATUS=1），跳过开机");
                step("模组开机(已开,跳过)");
            }
            powered_modem = true;

            bool pub_ok = publish_battery_payload();
            ESP_LOGI("BAT", "Publish %s", pub_ok ? "success" : "failed");
            seq_report(pub_ok ? "周期上报（成功）" : "周期上报（失败）");
#if ENABLE_DEEP_SLEEP
            if (pub_ok) { app_note_publish_ok(); }
#endif
            // 注意：锁要持有到第 4 步关掉模组之后再放
        } else {
            ESP_LOGW("GPS", "No GNSS fix within %ds, skip EG800/MQTT and go to sleep", GPS_FIX_TIMEOUT_SEC);
        }
#endif

        /* 4) 进入省电（方案 E）
         *    GNSS -> Backup（VCC 保持，星历留存，热启动）
         *    模组 -> QPOWD 正常关机 + 断 3.8V 轨
         *    ESP32 -> deep sleep                                      */
        ESP_LOGI("GPS", "===== 进入省电：GNSS Backup + 模组关机 + ESP deep sleep %d 分钟 =====",
                 GPS_CYCLE_SLEEP_MIN);

        gnss_enter_standby2();

        if (powered_modem) {
            modem_power_down_full();
            modem_bus_unlock();   // 与第 3 步的 lock 配对，关完模组才放锁
        } else {
            ESP_LOGI("MODEM", "本轮没开过模组，只确保 3.8V 轨是关的");
            modem_power_enable(false);
        }

#if ENABLE_DEEP_SLEEP
        enter_deep_sleep_cycle(GPS_CYCLE_SLEEP_MIN * 60U);
        // enter_deep_sleep_cycle() 不返回，醒来是复位、从 app_main 重新开始
#else
        // 不用 deep sleep 时退回原来的行为：任务级延时 + 唤醒 GNSS
        vTaskDelay(GPS_CYCLE_SLEEP_MIN * 60 * 1000 / portTICK_PERIOD_MS);
        ESP_LOGI("GNSS_LP", "Wake GNSS from backup");
        gnss_wake_from_standby2();
#endif
    }
}

void mqtt_init(void)
{
    /* 幂等：自测模式下 app_main 和 Selftest_task 都会调到这里，
     * 重复 uart_driver_install() 返回 ESP_ERR_INVALID_STATE，会被
     * ESP_ERROR_CHECK 直接 abort。 */
    static bool inited = false;
    if (inited) {
        ESP_LOGI("UART", "mqtt_init() 已初始化过，跳过");
        return;
    }
    inited = true;

    if (s_modem_mutex == NULL) {
        s_modem_mutex = xSemaphoreCreateMutex();
    }

    // 初始化 UART1 (普通轮询模式)
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    ESP_LOGI("UART", "UART1 initialized at 115200 baud with polling mode");
    
    // 安装UART驱动，不使用中断
    ESP_ERROR_CHECK(uart_driver_install(UART_EG800_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_EG800_NUM, &uart_config));
    // uart_set_pin(port, tx_io, rx_io, rts, cts)
    ESP_ERROR_CHECK(uart_set_pin(UART_EG800_NUM, MODEM_UART_TX_PIN, MODEM_UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_LOGI("UART", "Modem UART pins: TX=IO%d -> EG800 RXD, RX=IO%d <- EG800 TXD",
             MODEM_UART_TX_PIN, MODEM_UART_RX_PIN);
}

mqtt_state_t get_mqtt_state(void)
{
#if ENABLE_MQTT
    return mqtt_state;
#else
    return MQTT_STATE_IDLE;
#endif
}

void set_mqtt_state(mqtt_state_t state)
{
#if ENABLE_MQTT
    mqtt_state = state;
#endif
}