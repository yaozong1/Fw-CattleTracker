#include <stdio.h>
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/gpio.h"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcpp"
#include "driver/adc.h"
#include "driver/gptimer.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "string.h"
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "esp_adc_cal.h"
#pragma GCC diagnostic pop
#include "mqtt_config.h"

// 统一默认开关：若外部未定义，则开启 MQTT
#ifndef ENABLE_MQTT
#define ENABLE_MQTT 1
#endif

// 通用 UNUSED_ATTR，避免静态函数/变量未用告警
#ifndef UNUSED_ATTR
#define UNUSED_ATTR __attribute__((unused, used))
#endif

// 硬件引脚与常量（若外部未定义则提供默认值）
#ifndef LED_PIN_EN_GNSS
#define LED_PIN_EN_GNSS 46  // IO46（同时作为 GNSS 使能脚）
#endif
#ifndef LED_PIN_RST_GNSS
#define LED_PIN_RST_GNSS 47  // IO47 -> 通过三极管反相驱动 RESET_N
#endif
#ifndef QT_POWER_PIN
#define QT_POWER_PIN 18      // EG915U PWRKEY 控制脚
#endif

#ifndef LED_PIN_1
#define LED_PIN_1 1
#endif
#ifndef LED_PIN_3
#define LED_PIN_3 3
#endif
#ifndef BUTTON_PIN
#define BUTTON_PIN 0
#endif
#ifndef LED_PIN_EN_4V
#define LED_PIN_EN_4V 4
#endif
#ifndef LED_PIN_EN_GPS
#define LED_PIN_EN_GPS 5
#endif
#ifndef LED_PIN_EN_3V3_LDO
#define LED_PIN_EN_3V3_LDO 6
#endif
#ifndef RS_EN
#define RS_EN 7
#endif

// UART & 缓冲区定义
#ifndef EX_UART_NUM
#define EX_UART_NUM UART_NUM_2   // GNSS 使用 UART2
#endif
#ifndef BUF_SIZE
#define BUF_SIZE 1024
#endif

#ifndef UART_EG915U_NUM
#define UART_EG915U_NUM UART_NUM_1 // EG915U模组 UART (IO11, IO12)
#endif
#ifndef EG915U_TX_PIN
#define EG915U_TX_PIN 11  // ESP32的TX连接到EG915U
#endif
#ifndef EG915U_RX_PIN
#define EG915U_RX_PIN 12  // ESP32的RX连接到EG915U
#endif
#ifndef EG915U_POWER
#define EG915U_POWER 48   // IO48 读取 EG915U 上电状态
#endif

// 电池主题
#ifndef MQTT_TOPIC_BATTERY
#define MQTT_TOPIC_BATTERY "fleet/PE-001/battery"
#endif

// ADC 配置
#define ADC1_CHANNEL ADC1_CHANNEL_0  // IO1 对应 ADC1_CHANNEL_0
#define NO_OF_SAMPLES 64  // 采样数

static esp_adc_cal_characteristics_t *adc_chars;
static gptimer_handle_t timer_handle = NULL;

static volatile bool log_output = false;
// 添加全局标志位
static volatile bool send_csq_flag = false;
static volatile bool send_at_flag = false;

// MQTT相关标志位
static volatile bool mqtt_connect_flag = false;
static volatile bool mqtt_publish_flag = false;

// APN设置标志位（确保只设置一次）
#if ENABLE_MQTT
static bool UNUSED_ATTR apn_configured = false;
#endif

// MQTT连接状态
typedef enum {
    MQTT_STATE_IDLE,
    MQTT_STATE_INIT_NETWORK,
    MQTT_STATE_CONNECTING,
    MQTT_STATE_CONNECTED,
    MQTT_STATE_DISCONNECTED,
    MQTT_STATE_ERROR
} mqtt_state_t;
#if ENABLE_MQTT
static mqtt_state_t UNUSED_ATTR mqtt_state = MQTT_STATE_IDLE;
#endif

// MQTT发布报文标识符（用于QoS1/2），范围1..65535，0无效
#if ENABLE_MQTT
static uint16_t mqtt_msg_id = 1;
#endif
// 简单的串口读互斥，避免URC轮询与同步等待抢读
static volatile bool uart_read_busy = false;

// 当前是否已建立MQTT会话（用于优雅断开）
#if ENABLE_MQTT
static bool UNUSED_ATTR mqtt_session_open = false;
#endif

// ---------------- GNSS (LC760Z) ----------------
// GNSS UART 使用 UART2, 引脚: RX=IO42, TX=IO43，IO46 拉高使能
#define GNSS_UART_NUM EX_UART_NUM
// GNSS 波特率（根据模块设置调整）
#define GNSS_BAUD 115200
// 注意：用户连线为 LC760Z RXD <- ESP32 IO42 (ESP32 TX), LC760Z TXD -> ESP32 IO45 (ESP32 RX)
#define GNSS_UART_TX 42  // ESP32 TX 输出到 GNSS RXD (IO42)
#define GNSS_UART_RX 45  // ESP32 RX 接 GNSS TXD (IO45)
#define GNSS_EN_PIN LED_PIN_EN_GNSS
// 复用 IO47 作为 GNSS RESET_N 控制：注意硬件为三极管反相，IO47=1 等效 RESET_N=低（有效）
#define GNSS_RST_PIN LED_PIN_RST_GNSS

static void gnss_enable(bool on)
{
    gpio_set_level(GNSS_EN_PIN, on ? 1 : 0);
}

// RESET_N 受三极管反相：
static inline void gnss_reset_assert(void)   { gpio_set_level(GNSS_RST_PIN, 1); } // IO47=1 -> RESET_N=Low(有效)
static inline void gnss_reset_release(void)  { gpio_set_level(GNSS_RST_PIN, 0); } // IO47=0 -> RESET_N=High(无效)

// GNSS 原始 NMEA 打印总开关（0=关闭，1=打开）
#ifndef GNSS_RAW_LOG_ENABLED
#define GNSS_RAW_LOG_ENABLED 0
#endif

#if GNSS_RAW_LOG_ENABLED
#define GNSS_RAW_LOG(fmt, ...) ESP_LOGI("GNSS_RAW", fmt, ##__VA_ARGS__)
#else
#define GNSS_RAW_LOG(...) do { } while (0)
#endif

// UBX 调试日志（与 NMEA 分开，便于过滤）
#ifndef GNSS_UBX_LOG_ENABLED
#define GNSS_UBX_LOG_ENABLED 1
#endif
#if GNSS_UBX_LOG_ENABLED
#define GNSS_UBX_LOG(fmt, ...) ESP_LOGI("GNSS_UBX", fmt, ##__VA_ARGS__)
#else
#define GNSS_UBX_LOG(...) do { } while (0)
#endif

// ---------- 提前声明与配置（供后续早期代码使用） ----------
// AT / MQTT 前向声明（避免隐式声明）
void send_at_command(const char* command);
static bool wait_for_ok(uint32_t timeout_ms);
static int uart_read_response(char* buffer, int max_len, int timeout_ms);
#if ENABLE_MQTT
static bool wait_for_contains(const char* expected, uint32_t timeout_ms);
static bool wait_for_ok_then_contains(const char* expected_urc, uint32_t timeout_ms);
static bool mqtt_init_network(void);
static bool mqtt_connect_broker(void);
static bool mqtt_publish_message(const char* topic, const char* message);
static bool mqtt_disconnect_broker(void);
#endif

// 先提供 AT 基础函数定义，避免后续使用时出现隐式声明/顺序问题
void send_at_command(const char* command)
{
    // 清空接收缓冲区
    uart_flush_input(UART_EG915U_NUM);
    char cmd_with_crlf[256];
    snprintf(cmd_with_crlf, sizeof(cmd_with_crlf), "%s\r\n", command);
    uart_write_bytes(UART_EG915U_NUM, cmd_with_crlf, strlen(cmd_with_crlf));
    ESP_LOGI("UART1", "Sent: %s", command);
    // 等待发送完成
    ESP_ERROR_CHECK(uart_wait_tx_done(UART_EG915U_NUM, 100 / portTICK_PERIOD_MS));
}

// 仅等待 OK/ERROR 的简单响应（在此提前定义，供早期调用）
static bool UNUSED_ATTR wait_for_ok(uint32_t timeout_ms)
{
    char response[512];
    uint32_t start_time = xTaskGetTickCount();
    uart_read_busy = true;
    while ((xTaskGetTickCount() - start_time) < (timeout_ms / portTICK_PERIOD_MS)) {
        int len = uart_read_response(response, sizeof(response), 500);
        if (len > 0) {
            ESP_LOGI("UART1", "Received: %s", response);
            if (strstr(response, "OK")) { uart_read_busy = false; return true; }
            if (strstr(response, "ERROR")) { uart_read_busy = false; return false; }
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    ESP_LOGW("MQTT", "Timeout waiting for OK");
    uart_read_busy = false;
    return false;
}

// ---------- GPS 上报周期参数（需在 gps_mqtt_cycle_task 与 publish_gps_payload 之前） ----------
#ifndef GPS_CYCLE_SLEEP_MIN
#define GPS_CYCLE_SLEEP_MIN 6              // 睡眠 6 分钟
#endif
#ifndef GPS_FIX_TIMEOUT_SEC
#define GPS_FIX_TIMEOUT_SEC 300            // 5 分钟定位超时
#endif
#ifndef MQTT_SEND_MAX_RETRY
#define MQTT_SEND_MAX_RETRY 3              // MQTT 发布最多重试 3 次
#endif
#ifndef MQTT_TOPIC_GPS
#define MQTT_TOPIC_GPS "fleet/PE-001/gps"
#endif

// ---------- 位置保存结构（供 MQTT 上报/业务循环使用） ----------
typedef struct {
    bool   valid;
    double lat_deg;   // 十进制度，北正南负
    double lon_deg;   // 十进制度，东正西负
    double spd_kn;    // 节
    double crs_deg;   // 航向角
    char   time_utc[16]; // hhmmss.sss（原样保存）
} gnss_location_t;

static gnss_location_t gnss_last_location;  // 最近一次从RMC解析到的位置信息（有效时 valid=true）
static gnss_location_t gnss_saved_location; // 每次进入Backup前的快照

// ---- 前向声明（本文件稍后定义的内部函数）----
bool gnss_wait_for_quiet2(uint32_t quiet_ms, uint32_t timeout_ms);
bool gnss_wait_for_activity2(uint32_t timeout_ms);
bool gnss_enter_standby2(void);
bool gnss_wake_from_standby2(void);
bool gnss_enter_backup_via_cfg(void);
void gnss_exit_backup_via_reset(void);
// 提前声明 standby 进入函数，避免早期调用时报隐式声明
static bool gnss_enter_standby_quectel(uint32_t sleep_ms);

// ---------------- 低功耗 GNSS 测试参数 ----------------
#ifndef ENABLE_GNSS_LP
#define ENABLE_GNSS_LP 0              // 关闭旧的低功耗循环，改用周期上报任务
#endif
#ifndef GNSS_LP_OFF_SEC
#define GNSS_LP_OFF_SEC 60            // 关断时长（秒）
#endif
#ifndef GNSS_TTFF_TIMEOUT_SEC
#define GNSS_TTFF_TIMEOUT_SEC 120     // TTFF 超时（秒）
#endif
#ifndef GNSS_LP_ON_HOLD_SEC
#define GNSS_LP_ON_HOLD_SEC 15        // Fix 后继续保持运行时间（秒）
#endif

// 低功耗方式：0=拉低EN完全断电；1=发送命令进入待机/备份（保持VCC/Vbackup）
#ifndef GNSS_LP_METHOD
#define GNSS_LP_METHOD 2
#endif

// 可选：2=使用 UBX CFG-SLEEP 进入 Backup，RESET_N 低脉冲退出

// GNSS 简易状态（供日志/后续逻辑参考）
static volatile bool  gnss_has_fix   = false;   // 是否已定位
static volatile int   gnss_fix_q     = 0;       // GGA fix质量(0无fix,1GPS,2DGPS,4RTK fix,5RTK float...)
static volatile int   gnss_dim       = 0;       // GSA 维度(1无,2=2D,3=3D)
static volatile int   gnss_sats      = 0;       // GGA 在用卫星数
static volatile float gnss_hdop      = 0.0f;    // GGA/GSA HDOP

// 记录各句型是否已见及状态，用于综合摘要
static volatile bool gnss_seen_rmc = false;
static volatile bool gnss_seen_gga = false;
static volatile bool gnss_seen_gsa = false;
static volatile bool gnss_rmc_valid = false; // RMC: A=valid

// TTFF（首次定位用时）测量标志
static volatile bool       gnss_ttff_measuring = false;
static volatile TickType_t gnss_ttff_start     = 0;

// 待机/唤醒可视化：最近完成一行NMEA的时间与累计行数
static volatile TickType_t gnss_last_sentence_tick;
static volatile uint32_t   gnss_lines_seen;
// PMTK待机ACK检测标志
static volatile bool gnss_ack_pmtk161;
static volatile bool gnss_ack_pmtk225;
// 是否认为已进入待机（用于调试观测）
static volatile bool gnss_in_standby = false;
// UBX ACK/NAK 标志（仅关心 CFG-SLEEP 0x06/0x41）
static volatile bool gnss_ack_cfg_sleep = false;
static volatile bool gnss_nak_cfg_sleep = false;
// 基准测试结束后，停止 GNSS 接收
static volatile bool gnss_should_stop = false;



static void gnss_log_fix_summary_if_changed(void)
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
    if (gnss_has_fix != last_has_fix) changed = true;
    if (gnss_fix_q != last_fix_q) changed = true;
    if (gnss_dim != last_dim) changed = true;
    if (gnss_sats != last_sats) changed = true;
    if ((last_hdop < 0 && gnss_hdop >= 0)) changed = true;
    else if (last_hdop >= 0) {
        float __diff = gnss_hdop - last_hdop;
        if (__diff < 0) __diff = -__diff;
        if (__diff > 0.05f) changed = true;
    }
    if (gnss_rmc_valid != last_rmc_valid) changed = true;
    if (gnss_seen_rmc != last_seen_rmc || gnss_seen_gga != last_seen_gga || gnss_seen_gsa != last_seen_gsa) changed = true;

    if (!changed) return;

    const char* overall = gnss_has_fix ? "FIXED" : "NO FIX";
    const char* dim_str = (gnss_dim == 3) ? "3D" : (gnss_dim == 2) ? "2D" : "N/A";

    // 构造各子状态字符串（未见则显示 N/A）
    char rmc_str[16];
    snprintf(rmc_str, sizeof(rmc_str), "%s", gnss_seen_rmc ? (gnss_rmc_valid ? "A(valid)" : "V(invalid)") : "N/A");

    char gga_str[48];
    if (gnss_seen_gga) {
        snprintf(gga_str, sizeof(gga_str), "Q=%d Sats=%d HDOP=%.1f", gnss_fix_q, gnss_sats, gnss_hdop);
    } else {
        snprintf(gga_str, sizeof(gga_str), "N/A");
    }

    char gsa_str[32];
    if (gnss_seen_gsa) {
        snprintf(gsa_str, sizeof(gsa_str), "Dim=%s HDOP=%.1f", dim_str, gnss_hdop);
    } else {
        snprintf(gsa_str, sizeof(gsa_str), "N/A");
    }

    ESP_LOGI("GNSS", "FIX SUMMARY: RMC=%s | GGA:%s | GSA:%s => %s", rmc_str, gga_str, gsa_str, overall);

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

static inline void gnss_reset_state_for_ttff(void)
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

// ---------- EG915U 电源控制（通过 QT_POWER_PIN 序列） ----------
static void modem_power_cycle(void)
{
    // 根据你现有 app_main 中的按键逻辑，这里复用相同的上电序列
    ESP_LOGI("MODEM", "Power-cycle EG915U via PWRKEY sequence");
    gpio_set_level(QT_POWER_PIN, 0);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    gpio_set_level(QT_POWER_PIN, 1);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    gpio_set_level(QT_POWER_PIN, 0);
}

// （send_at_command 与 wait_for_ok 已在文件前部定义）

static void modem_power_off_soft(void)
{
    ESP_LOGI("MODEM", "Soft power off EG915U (AT+CFUN=0)");
#if ENABLE_MQTT
    send_at_command("AT+CFUN=0");
    wait_for_ok(10000);
#endif
}

// ---------- 电池 JSON 构造与发布 ----------
#if ENABLE_MQTT
static float read_battery_voltage(void)
{
    // 简易读取：用 ADC 原始值换算电压（mV），再换 V；如有分压，外部用宏修正
    uint32_t adc_reading = 0;
    for (int i = 0; i < NO_OF_SAMPLES; i++) adc_reading += adc1_get_raw(ADC1_CHANNEL);
    adc_reading /= NO_OF_SAMPLES;
    uint32_t mv = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars); // mV
#ifndef VBAT_DIVIDER_RATIO
#define VBAT_DIVIDER_RATIO 2.0f
#endif
    return (float)mv / 1000.0f * VBAT_DIVIDER_RATIO;
}

static int estimate_soc_from_voltage(float v)
{
    // 非线性电池曲线此处用近似阶梯/线性，后续可替换为查表/卡尔曼
    if (v >= 12.6f) { return 100; }
    if (v <= 10.5f) { return 0; }
    float soc = (v - 10.5f) / (12.6f - 10.5f) * 100.0f;
    if (soc < 0) {
        soc = 0;
    }
    if (soc > 100) {
        soc = 100;
    }
    return (int)(soc + 0.5f);
}

static bool publish_battery_payload(void)
{
    char payload[512];
    // 采集
    // 按需求临时固定数值
    float voltage = 4.2f;
    static int current_soc = 96;
    // 每次交替 +1 / -1（无需随机数）
    static int toggle = 0; // 0 -> +1, 1 -> -1
    int delta = toggle ? -1 : 1;
    toggle ^= 1;
    current_soc += delta;
    if (current_soc < 0) current_soc = 0;
    if (current_soc > 100) current_soc = 100;
    int soc = current_soc;
    // 温度/健康/循环等暂时用占位或传感器对接后替换
    float temperature = 31.4f;
    int health = 96;
    int cycleCount = 182;
    float estimatedRangeKm = 120.5f;
    const char* chargingStatus = "discharging";
    // GPS 块（来自最近一次 GNSS RMC/GGA/GSA）
    double speed_kmh = gnss_last_location.spd_kn * 1.852; // 节->km/h
    int heading = (int)(gnss_last_location.crs_deg + 0.5);
    double altitude = 0.0; // 如需要，可从 GGA 拓展存储高度
    int accuracy = (int)(gnss_hdop > 0 ? gnss_hdop : 5);

    long long ts_ms = esp_timer_get_time() / 1000; // us -> ms

    // 生成 JSON
    int n = snprintf(payload, sizeof(payload),
        "{\n"
        "  \"device\": \"%s\",\n"
        "  \"ts\": %lld,\n"
        "  \"soc\": %d,\n"
        "  \"voltage\": %.2f,\n"
        "  \"temperature\": %.1f,\n"
        "  \"health\": %d,\n"
        "  \"cycleCount\": %d,\n"
        "  \"estimatedRangeKm\": %.1f,\n"
        "  \"chargingStatus\": \"%s\",\n"
        "  \"alerts\": [],\n"
        "  \"gps\": {\n"
        "    \"lat\": %.6f,\n"
        "    \"lng\": %.6f,\n"
        "    \"speed\": %.1f,\n"
        "    \"heading\": %d,\n"
        "    \"altitude\": %.1f,\n"
        "    \"accuracy\": %d\n"
    "  }\n"
    "}",
    "PE-001", // 改为对应设备 ID
        ts_ms,
        soc,
        voltage,
        temperature,
        health,
        cycleCount,
        estimatedRangeKm,
        chargingStatus,
        gnss_last_location.lat_deg,
        gnss_last_location.lon_deg,
        speed_kmh,
        heading,
        altitude,
        accuracy);
    if (n <= 0 || n >= (int)sizeof(payload)) {
        ESP_LOGE("BAT", "Battery JSON build failed");
        return false;
    }

    for (int attempt = 1; attempt <= MQTT_SEND_MAX_RETRY; ++attempt) {
        ESP_LOGI("BAT", "MQTT publish attempt %d/%d", attempt, MQTT_SEND_MAX_RETRY);
        if (!mqtt_init_network()) continue;
        if (!mqtt_connect_broker()) continue;
        // 打印即将发送的主题与完整 payload，便于与后台对照
        ESP_LOGI("BAT", "Topic: %s\nPayload:\n%s", MQTT_TOPIC_BATTERY, payload);
        bool ok = mqtt_publish_message(MQTT_TOPIC_BATTERY, payload);
        mqtt_disconnect_broker();
        if (ok) return true;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    return false;
}
#endif

// ---------- 主周期任务：定位 -> MQTT 上报 -> 睡眠 6 分钟 ----------
static void gps_mqtt_cycle_task(void* pv)
{
    ESP_LOGI("GPS", "gps_mqtt_cycle_task started: sleep=%d min, fix timeout=%d s, mqtt retries=%d",
             GPS_CYCLE_SLEEP_MIN, GPS_FIX_TIMEOUT_SEC, MQTT_SEND_MAX_RETRY);

    while (1) {
        // 1) 仅启动 GNSS，先尝试拿定位；未定位则不唤醒 EG915、不进行 MQTT
        bool powered_modem = false;
        // 启用 GNSS 供定位
        gnss_enable(true);
        gnss_reset_release();

        // 2) 连续模式下等待 GNSS 定位（RMC 有效）或超时
        gnss_last_location.valid = false;
        gnss_reset_state_for_ttff();
        ESP_LOGI("GPS", "Waiting GNSS fix (timeout %ds)...", GPS_FIX_TIMEOUT_SEC);
        TickType_t t0 = xTaskGetTickCount();
        while ((xTaskGetTickCount() - t0) < pdMS_TO_TICKS(GPS_FIX_TIMEOUT_SEC * 1000)) {
            if (gnss_last_location.valid) { break; }
            vTaskDelay(200 / portTICK_PERIOD_MS);
        }

        // 3) 若拿到定位，则唤醒 EG915 并进行 MQTT；否则跳过 MQTT 直接进入休眠
#if ENABLE_MQTT
        if (gnss_last_location.valid) {
            ESP_LOGI("GPS", "GNSS fix acquired, bringing up EG915 for MQTT publish");
            ESP_LOGI("MODEM", "Wake EG915U: about to PWRKEY pulse (power-cycle) to publish");
            modem_power_cycle();
            powered_modem = true;
            vTaskDelay(2000 / portTICK_PERIOD_MS);

            bool pub_ok = publish_battery_payload();
            ESP_LOGI("BAT", "Publish %s", pub_ok ? "success" : "failed");
        } else {
            ESP_LOGW("GPS", "No GNSS fix within %ds, skip EG915/MQTT and go to sleep", GPS_FIX_TIMEOUT_SEC);
        }
#endif

        // 4) 进入省电：GNSS -> Standby（CFG-SLEEP mode=1，sleep=6分钟），Modem（如已上电）-> 软关机
        ESP_LOGI("GPS", "Enter sleep: GNSS standby + modem CFUN=0 for %d min", GPS_CYCLE_SLEEP_MIN);
        gnss_enter_standby_quectel(GPS_CYCLE_SLEEP_MIN * 60 * 1000U);
        if (powered_modem) {
            modem_power_off_soft();
        } else {
            ESP_LOGI("MODEM", "Skip modem CFUN=0: modem wasn't powered this cycle");
        }

        // 进入睡眠间隔：简单延时（系统仍在运行，如需深睡可扩展 esp_light_sleep_start）
        vTaskDelay(GPS_CYCLE_SLEEP_MIN * 60 * 1000 / portTICK_PERIOD_MS);

        // 5) 从 Standby 唤醒 GNSS；下一轮再按是否有定位决定是否上电 Modem
        // 即将唤醒 GNSS（LC760Z）：通过 UART 发送 0xFF + CRLF 退出 Standby
        ESP_LOGI("GNSS_LP", "Wake GNSS: about to exit Standby (TX 0xFF + CRLF)");
        (void)gnss_wake_from_standby2();
        // 下一轮循环再决定是否 power_cycle Modem（仅在有定位时才上电）
    }
}

static void gnss_report_ttff_if_needed(void)
{
    if (gnss_ttff_measuring && gnss_has_fix) {
        TickType_t now = xTaskGetTickCount();
        uint32_t ms = (uint32_t)((now - gnss_ttff_start) * 1000 / configTICK_RATE_HZ);
        ESP_LOGI("GNSS", "TTFF: %u ms (%.2f s)", (unsigned)ms, ms / 1000.0f);
        gnss_ttff_measuring = false;
    }
}

// 发送一条 NMEA 命令到 GNSS（自动追加 \r\n）
static void gnss_send_nmea(const char* sentence)
{
    if (!sentence) return;
    uart_write_bytes(GNSS_UART_NUM, sentence, strlen(sentence));
    uart_write_bytes(GNSS_UART_NUM, "\r\n", 2);
}

// 发送 UBX 二进制消息（B5 62 + cls + id + len + payload + CK_A CK_B）
static void gnss_send_ubx(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len)
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
// 注意：校验算法与 UBX 相同（覆盖 cls/id/len/payload，不含帧头 F1 D9）
static void gnss_send_quectel_bin(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len)
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

// （前向声明已在文件前部提供）

// ---------------- TTFF 基准测试与 Standby（Quectel）辅助 ----------------
// 先提供 quiet/activity 等待函数的定义，供后续 Standby/Benchmark 调用
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
// 基准测试开关：1=在 gnss_lp_task 开始处自动对比 PowerOff/Backup/Standby 三种方式的 TTFF
#ifndef GNSS_TTFF_BENCH
#define GNSS_TTFF_BENCH 0
#endif
#ifndef GNSS_TTFF_BENCH_CYCLES
#define GNSS_TTFF_BENCH_CYCLES 2
#endif

// （上移到文件前部：GPS 上报周期参数 与 MQTT 主题 宏定义）

// 使用 Quectel CFG-SLEEP 进入 Standby（mode=1），sleep_ms 指定待机时间
static bool gnss_enter_standby_quectel(uint32_t sleep_ms)
{
    ESP_LOGI("GNSS_LP", "Send Quectel CFG-SLEEP Standby: sleep_ms=%u, mode=1", (unsigned)sleep_ms);
    gnss_send_cfg_sleep_quectel(sleep_ms, 1); // 1=Standby
    // 要求 ≥1.5s 静默并在随后的 1s 内保持静默
    if (gnss_wait_for_quiet2(1500, 5000)) {
        uint32_t lines_before = gnss_lines_seen;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        if (gnss_lines_seen == lines_before) {
            ESP_LOGI("GNSS_LP", "Standby entered (UART quiet and stays quiet)");
            return true;
        }
        ESP_LOGW("GNSS_LP", "Quiet detected but activity resumed -> not standby");
    } else {
        ESP_LOGW("GNSS_LP", "No quiet after Standby CFG-SLEEP within timeout");
    }
    return false;
}

#if GNSS_TTFF_BENCH
typedef struct {
    uint32_t count;
    uint32_t timeouts;
    uint64_t sum_ms;
    uint32_t min_ms;
    uint32_t max_ms;
} ttff_stats_t;

static bool ttff_wait_for_fix_and_get(uint32_t timeout_s, uint32_t* out_ms)
{
    gnss_reset_state_for_ttff();
    TickType_t start = xTaskGetTickCount();
    while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(timeout_s * 1000)) {
        if (gnss_has_fix) {
            uint32_t ms = (uint32_t)((xTaskGetTickCount() - gnss_ttff_start) * 1000 / configTICK_RATE_HZ);
            if (out_ms) *out_ms = ms;
            return true;
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    return false;
}

static void ttff_stats_add(ttff_stats_t* s, bool ok, uint32_t ms)
{
    if (!ok) { s->timeouts++; return; }
    s->count++;
    s->sum_ms += ms;
    if (s->min_ms == 0 || ms < s->min_ms) s->min_ms = ms;
    if (ms > s->max_ms) s->max_ms = ms;
}

static void ttff_stats_log(const char* tag, const ttff_stats_t* s)
{
    if (s->count == 0) {
        ESP_LOGW("GNSS_BENCH", "%s: 无成功样本，超时=%u", tag, (unsigned)s->timeouts);
        return;
    }
    double avg = (double)s->sum_ms / (double)s->count;
    ESP_LOGI("GNSS_BENCH", "%s: 样本=%u 成功=%u 超时=%u | min=%ums max=%ums avg=%.1fms",
             tag, (unsigned)(s->count + s->timeouts), (unsigned)s->count, (unsigned)s->timeouts,
             (unsigned)s->min_ms, (unsigned)s->max_ms, avg);
}

// 运行单轮 TTFF 测试
static bool bench_run_one(int method, uint32_t* out_ttff_ms)
{
    switch (method) {
        case 0: { // 完全断电（冷启动）
            ESP_LOGI("GNSS_BENCH", "[PowerOff] EN LOW for %ds", GNSS_LP_OFF_SEC);
            gnss_enable(false);
            vTaskDelay(GNSS_LP_OFF_SEC * 1000 / portTICK_PERIOD_MS);
            ESP_LOGI("GNSS_BENCH", "[PowerOff] EN HIGH, start TTFF");
            gnss_enable(true);
            vTaskDelay(250 / portTICK_PERIOD_MS);
            uart_flush_input(GNSS_UART_NUM);
            return ttff_wait_for_fix_and_get(GNSS_TTFF_TIMEOUT_SEC, out_ttff_ms);
        }
        case 2: { // Backup（热启动）
            ESP_LOGI("GNSS_BENCH", "[Backup] Entering (sleep %ds)", GNSS_LP_OFF_SEC);
            (void)gnss_enter_backup_via_cfg();
            vTaskDelay(GNSS_LP_OFF_SEC * 1000 / portTICK_PERIOD_MS);
            ESP_LOGI("GNSS_BENCH", "[Backup] Exit via RESET_N low pulse");
            gnss_exit_backup_via_reset();
            vTaskDelay(50 / portTICK_PERIOD_MS);
            uart_flush_input(GNSS_UART_NUM);
            ESP_LOGI("GNSS_BENCH", "[Backup] Start TTFF");
            return ttff_wait_for_fix_and_get(GNSS_TTFF_TIMEOUT_SEC, out_ttff_ms);
        }
        case 1: { // Standby（热启动）
            ESP_LOGI("GNSS_BENCH", "[Standby] Entering (sleep %ds)", GNSS_LP_OFF_SEC);
            (void)gnss_enter_standby_quectel(GNSS_LP_OFF_SEC * 1000U);
            vTaskDelay(GNSS_LP_OFF_SEC * 1000 / portTICK_PERIOD_MS);
            (void)gnss_wake_from_standby2();
            vTaskDelay(50 / portTICK_PERIOD_MS);
            uart_flush_input(GNSS_UART_NUM);
            ESP_LOGI("GNSS_BENCH", "[Standby] Start TTFF");
            return ttff_wait_for_fix_and_get(GNSS_TTFF_TIMEOUT_SEC, out_ttff_ms);
        }
        default:
            return false;
    }
}
#endif // GNSS_TTFF_BENCH

// ---------------- 位置保存结构与工具函数 ----------------
// （gnss_location_t 与相关全局变量已上移至文件前部，避免在早期使用时无声明）

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

// 等待在指定时间窗内NMEA静默（定义已前移）
// 等待出现新NMEA行（定义已前移）

// 进入待机：先PMTK161,0；若不安静，再试PMTK225,4。返回是否静默
bool gnss_enter_standby2(void)
{
    // 改为使用 Quectel CFG-SLEEP 的 Standby 模式（mode=1）
    gnss_in_standby = false;
    bool ok = gnss_enter_standby_quectel(GNSS_LP_OFF_SEC * 1000U);
    if (ok) gnss_in_standby = true;
    return ok;
}

// 唤醒：发任意字节并等待NMEA恢复
bool gnss_wake_from_standby2(void)
{
    ESP_LOGI("GNSS_LP", "Wake: TX 0xFF CRLF");
    const uint8_t wake_seq[3] = {0xFF, '\r', '\n'};
    uart_write_bytes(GNSS_UART_NUM, (const char*)wake_seq, sizeof(wake_seq));
    vTaskDelay(200 / portTICK_PERIOD_MS);
    uart_flush_input(GNSS_UART_NUM);

    if (gnss_wait_for_activity2(3000)) {
        ESP_LOGI("GNSS_LP", "NMEA resumed after wake");
        gnss_in_standby = false;
        return true;
    }
    ESP_LOGW("GNSS_LP", "No NMEA after wake within 3s");
    return false;
}

// 使用 UBX CFG-SLEEP (class=0x06, id=0x41) 进入 Backup（按所附资料）；
// 注意：具体 payload 根据芯片协议确定，这里默认无 payload 版本；若需要，可在此处填充 payload。
bool gnss_enter_backup_via_cfg(void)
{
    // 根据截图：Quectel 专用二进制帧头 F1 D9，payload=休眠时间(4B LE) + 低功耗模式(1B)
    // 进入 Backup：mode=3；休眠时间=0 表示一直休眠，靠 RESET_N 唤醒
    ESP_LOGI("GNSS_LP", "Send Quectel CFG-SLEEP (0x06,0x41): sleep_ms=0, mode=3 (Backup)");
    gnss_ack_cfg_sleep = false; // 对于 F1 D9 帧，不期望 UBX ACK
    gnss_nak_cfg_sleep = false;
    gnss_send_cfg_sleep_quectel(0 /*forever*/, 3 /*Backup*/);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    // 不等待 ACK：直接通过串口静默来判定是否进入备份
    // 进入 Backup 后 UART 应变为 invalid，使用更严格的静默判定
    if (gnss_wait_for_quiet2(1500, 5000)) {
        uint32_t lines_before = gnss_lines_seen;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        if (gnss_lines_seen == lines_before) {
            ESP_LOGI("GNSS_LP", "Backup entered (UART quiet and stays quiet)");
            gnss_in_standby = true;
            return true;
        }
        ESP_LOGW("GNSS_LP", "Quiet detected after CFG-SLEEP but activity resumed -> not backup");
    } else {
        ESP_LOGW("GNSS_LP", "No quiet after CFG-SLEEP within timeout");
    }
    return false;
}

// 通过 RESET_N 低脉冲退出 Backup：保持低≥100ms（因硬件反相，IO47=High 即 RESET_N=Low）
void gnss_exit_backup_via_reset(void)
{
    ESP_LOGI("GNSS_LP", "Exit Backup via RESET_N low pulse (>=100ms), inverted driver on IO%d", GNSS_RST_PIN);
    gnss_reset_assert();
    vTaskDelay(120 / portTICK_PERIOD_MS); // 稍高于 100ms
    gnss_reset_release();
    vTaskDelay(10 / portTICK_PERIOD_MS);  // 释放后保持 ≥10ms
    uart_flush_input(GNSS_UART_NUM);
    gnss_in_standby = false;
}

// 将 NMEA 行按逗号分割为字段，保留空字段（例如 ",," 之间的空字符串）
// 处理到 '*' 或字符串结束为止；在分割过程中会在原串上写入'\0'
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

// 简单的 NMEA RMC 解析（只提取时间、定位状态、经纬度、速度、航向）
static void gnss_parse_and_log_rmc(const char* nmea)
{
    // 期望句子形如: $xxRMC,hhmmss.sss,A,llll.ll,N,yyyyy.yy,E,spd,crs,ddmmyy,... (xx 为任意 talker: GP/GN/GA/BD/...)
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

    ESP_LOGI("GNSS", "RMC: time=%s status=%s lat=%s%s lng=%s%s spd(kn)=%s crs=%s", 
             time_utc, status, lat, lat_hemi, lng, lng_hemi, spd_kn, crs_deg);

    // RMC 状态：A=有效，V=无效
    bool rmc_fix = (status && status[0] == 'A');
    static bool last_rmc_fix = false;
    if (rmc_fix != last_rmc_fix) {
        ESP_LOGI("GNSS", "RMC fix state changed: %s", rmc_fix ? "FIXED" : "NO FIX");
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

// 解析 GGA：fix质量/卫星数/HDOP
static void gnss_parse_and_log_gga(const char* nmea)
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

    ESP_LOGI("GNSS", "GGA: fixQ=%d sats=%d HDOP=%.1f -> %s",
             fixq, sats, hdop, gnss_has_fix ? "FIXED" : "NO FIX");

    if (prev_fix != gnss_has_fix) {
        ESP_LOGI("GNSS", "FIX state changed by GGA: %s", gnss_has_fix ? "FIXED" : "NO FIX");
    }
    gnss_seen_gga = true;
    gnss_log_fix_summary_if_changed();
    gnss_report_ttff_if_needed();
}

// 解析 GSA：2D/3D 维度与 HDOP（可选）
static void gnss_parse_and_log_gsa(const char* nmea)
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
    ESP_LOGI("GNSS", "GSA: dim=%s (val=%d) HDOP=%.1f", dim_str, dim, gnss_hdop);

    if (dim >= 2) gnss_has_fix = true;
    gnss_seen_gsa = true;
    gnss_log_fix_summary_if_changed();
    gnss_report_ttff_if_needed();
}

// （移除了损坏的低功耗循环与基准代码，ENABLE_GNSS_LP 默认关闭）

static void gnss_task(void* pv)
{
    // 读 GNSS NMEA 数据并打印，优先显示 RMC
    uint8_t buf[256];
    char line[256];
    size_t line_len = 0;

    // 以“上次完成一行”的时间为心跳参考（避免有乱码字节时误判为空闲）
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

    while (1) {
        if (gnss_should_stop) {
            ESP_LOGI("GNSS", "Stopping GNSS receiver after benchmark");
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
                        // 待机命令ACK检测
                        if (strstr(line, "$PMTK001,161,3") != NULL) {
                            gnss_ack_pmtk161 = true;
                            ESP_LOGI("GNSS", "ACK PMTK161");
                        } else if (strstr(line, "$PMTK001,225,3") != NULL) {
                            gnss_ack_pmtk225 = true;
                            ESP_LOGI("GNSS", "ACK PMTK225");
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

        // 超过2秒没有“完成一行”，输出心跳（即使有字节但无CR/LF，也会提示）
        if ((xTaskGetTickCount() - last_line_tick) >= pdMS_TO_TICKS(2000)) {
            if (bytes_since_line == 0) {
                ESP_LOGI("GNSS", "Waiting for NMEA on UART2 (TX=%d, RX=%d, %d)...", GNSS_UART_TX, GNSS_UART_RX, GNSS_BAUD);
            } else {
                // 有数据但无换行，多半是波特率不匹配或线路噪声；打印样本帮助定位
                char hex[16 * 3 + 1];
                int pos = 0;
                for (int i = 0; i < sample_len && pos < (int)sizeof(hex) - 3; ++i) {
                    pos += snprintf(hex + pos, sizeof(hex) - pos, "%02X ", sample[i]);
                }
                hex[(pos < (int)sizeof(hex)) ? pos : ((int)sizeof(hex) - 1)] = '\0';
                GNSS_RAW_LOG("RX without CR/LF for 2s, bytes=%u, sample=[%s]", (unsigned)bytes_since_line, hex);
                ESP_LOGI("GNSS", "RX without CR/LF for 2s, bytes=%u", (unsigned)bytes_since_line);
            }
            // 重置基准窗口
            last_line_tick = xTaskGetTickCount();
            bytes_since_line = 0;
            sample_len = 0;
        }

        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

// 简单的UART读取函数（轮询模式）
static int uart_read_response(char* buffer, int max_len, int timeout_ms)
{
    int total_len = 0;
    int bytes_available;
    TickType_t start_time = xTaskGetTickCount();
    
    while ((xTaskGetTickCount() - start_time) < (timeout_ms / portTICK_PERIOD_MS)) {
        ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_EG915U_NUM, (size_t*)&bytes_available));
        
        if (bytes_available > 0) {
            int len = uart_read_bytes(UART_EG915U_NUM, (uint8_t*)buffer + total_len, 
                                     max_len - total_len - 1, 50 / portTICK_PERIOD_MS);
            if (len > 0) {
                total_len += len;
                buffer[total_len] = '\0';
                
                // 如果收到完整的响应（包含\r\n），提前返回
                if (strstr(buffer, "\r\n") || strstr(buffer, "OK") || strstr(buffer, "ERROR")) {
                    // 继续读取一小段时间，确保完整接收
                    vTaskDelay(100 / portTICK_PERIOD_MS);
                    
                    ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_EG915U_NUM, (size_t*)&bytes_available));
                    if (bytes_available > 0) {
                        len = uart_read_bytes(UART_EG915U_NUM, (uint8_t*)buffer + total_len, 
                                             max_len - total_len - 1, 50 / portTICK_PERIOD_MS);
                        if (len > 0) {
                            total_len += len;
                            buffer[total_len] = '\0';
                        }
                    }
                    break;
                }
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    
    return total_len;
}

// 简单的UART监控函数（用于调试）
#if 0
static void uart_monitor_task(void *pvParameters)
{
    char buffer[256];
    
    while (1) {
        int len = uart_read_response(buffer, sizeof(buffer), 1000); // 每秒检查一次
        if (len > 0) {
            ESP_LOGI("UART_MONITOR", "Unexpected data: %s", buffer);
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
#endif

// 定时器回调函数 - 每100ms执行一次
static bool IRAM_ATTR timer_callback_100ms(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    // 在这里添加你需要每100ms执行的代码
    // 注意：这是中断上下文，避免使用阻塞函数或长时间操作
    if(gpio_get_level(BUTTON_PIN) == 0)
    {
        log_output = true;
#if ENABLE_MQTT
        mqtt_connect_flag = true;  // 触发MQTT连接（主任务里将先等待一段时间）
#endif
    }
    // 示例：翻转状态、计数器等
    static uint32_t counter = 0;
    counter++;

    if (counter % 50 == 0)//  每5秒执行一次
    {
      //  send_at_flag = true; // 设置标志位
    }
    
    if (counter % 600 == 0)//  每60秒执行一次（600 * 100ms）
    {
#if ENABLE_MQTT
        mqtt_publish_flag = true; // 触发MQTT数据发送
#endif
    }
    
    // 如果需要在主任务中处理，可以使用队列或通知
    // 这里仅做简单示例，避免在ISR中打印
    
    return false; // 返回false表示不需要yield切换任务
}

// 初始化100ms定时器
static void init_timer_100ms(void)
{
    ESP_LOGI("TIMER", "Initializing 100ms timer");
    
    // 配置定时器
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1MHz, 1 tick = 1us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &timer_handle));
    
    // 配置报警事件（100ms = 100000us）
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = 100000, // 100ms
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true, // 自动重载，实现周期触发
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(timer_handle, &alarm_config));
    
    // 注册回调函数
    gptimer_event_callbacks_t cbs = {
        .on_alarm = timer_callback_100ms,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(timer_handle, &cbs, NULL));
    
    // 使能并启动定时器
    ESP_ERROR_CHECK(gptimer_enable(timer_handle));
    ESP_ERROR_CHECK(gptimer_start(timer_handle));
    
    ESP_LOGI("TIMER", "100ms timer started");
}

// 向EG915U发送AT指令的函数（移至文件前部定义）

// MQTT相关函数

// wait_for_ok 已在前部定义
// 等待包含指定关键字（URC、提示符等），同时拦截通用错误
static bool UNUSED_ATTR wait_for_contains(const char* expected, uint32_t timeout_ms)
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
    ESP_LOGW("MQTT", "Timeout waiting for: %s", expected);
    uart_read_busy = false;
    return false;
}
// 先等待OK，再等待指定URC（用于QMTOPEN/QMTCONN等异步结果）
static bool UNUSED_ATTR wait_for_ok_then_contains(const char* expected_urc, uint32_t timeout_ms)
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
    ESP_LOGW("MQTT", "Timeout waiting for URC: %s", expected_urc);
    uart_read_busy = false;
    return false;
}

// 初始化网络连接（4G）
#if ENABLE_MQTT
static bool mqtt_init_network(void)
{
    ESP_LOGI("MQTT", "Initializing 4G network...");
    
    // 0. 清理上电URC（如 RDY / Call Ready / SMS Ready），避免打断后续AT握手
    ESP_LOGI("MQTT", "Warm-up: draining boot URCs...");
    uint32_t drain_start = xTaskGetTickCount();
    char drain_buf[256];
    while ((xTaskGetTickCount() - drain_start) < (3000 / portTICK_PERIOD_MS)) { // 3秒内尽量清空
        int len = uart_read_bytes(UART_EG915U_NUM, (uint8_t*)drain_buf, sizeof(drain_buf) - 1, 50 / portTICK_PERIOD_MS);
        if (len > 0) {
            drain_buf[len] = '\0';
            // 只记录一次，避免刷屏
            if (strstr(drain_buf, "RDY") || strstr(drain_buf, "Call Ready") || strstr(drain_buf, "SMS Ready") || strstr(drain_buf, "PB Done")) {
                ESP_LOGI("MQTT", "Boot URC: %s", drain_buf);
            }
        } else {
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }
    }
    
    // 1. 检查模组状态
    ESP_LOGI("MQTT", "Step 1: Checking module status (AT handshake with retries)...");
    bool at_ok = false;
    for (int attempt = 1; attempt <= 5 && !at_ok; ++attempt) {
        send_at_command("AT");
        at_ok = wait_for_ok(1000);
        if (!at_ok) {
            ESP_LOGW("MQTT", "AT no response, retry %d/5", attempt);
            vTaskDelay(300 / portTICK_PERIOD_MS);
        }
    }
    if (!at_ok) {
        ESP_LOGW("MQTT", "Module not responding to AT command after retries");
        return false;
    }
    
    // 2. 关闭回显
    ESP_LOGI("MQTT", "Step 2: Disabling echo...");
    send_at_command("ATE0");
    wait_for_ok(3000);
    
    // 3. 设置全功能模式
    ESP_LOGI("MQTT", "Step 3: Setting full functionality mode...");
    send_at_command("AT+CFUN=1");
    if (!wait_for_ok(15000)) {  // 增加超时时间
        ESP_LOGW("MQTT", "Failed to set full functionality mode");
        return false;
    }
    
    // 等待模组稳定
    ESP_LOGI("MQTT", "Waiting for module to stabilize...");
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    
    // 4. 检查SIM卡状态
    ESP_LOGI("MQTT", "Step 4: Checking SIM card status...");
    send_at_command("AT+CPIN?");
    if (!wait_for_contains("READY", 5000)) {
        ESP_LOGW("MQTT", "SIM card not ready");
        // 继续尝试，有些卡可能需要更多时间
    }
    
    // 5. 设置APN（仅在APN_NAME不为空且未设置过时设置）
    if (strlen(APN_NAME) > 0 && !apn_configured) {
        ESP_LOGI("MQTT", "Step 5: Setting APN to %s (first time)...", APN_NAME);
        char apn_cmd[128];
        snprintf(apn_cmd, sizeof(apn_cmd), "AT+CGDCONT=1,\"IP\",\"%s\"", APN_NAME);
    send_at_command(apn_cmd);
    if (!wait_for_ok(5000)) {
            ESP_LOGW("MQTT", "Failed to set APN");
            return false;
        }
        apn_configured = true;  // 标记为已配置
        ESP_LOGI("MQTT", "APN configured successfully, will not set again");
    } else if (strlen(APN_NAME) > 0 && apn_configured) {
        ESP_LOGI("MQTT", "Step 5: Skipping APN setting (already configured this session)");
    } else {
        ESP_LOGI("MQTT", "Step 5: Skipping APN setting (APN_NAME is empty)");
    }
    
    // 6. 激活PDP上下文
    ESP_LOGI("MQTT", "Step 6: Activating PDP context...");
    send_at_command("AT+CGACT=1,1");
    if (!wait_for_ok(30000)) {
        ESP_LOGW("MQTT", "Failed to activate PDP context");
        return false;
    }
    
    // 7. 查询IP地址
    ESP_LOGI("MQTT", "Step 7: Getting IP address...");
    send_at_command("AT+CGPADDR=1");
    if (!wait_for_ok(5000)) {
        ESP_LOGW("MQTT", "Failed to get IP address, but continuing...");
        // 不返回false，因为有些模组可能不支持此命令
    }
    
    ESP_LOGI("MQTT", "4G network initialized successfully");
    return true;
}

// 关闭MQTT连接
static bool mqtt_disconnect_broker(void)
{
    ESP_LOGI("MQTT", "Closing MQTT connection...");
    // 以前如果认为没有会话就直接跳过，但模组侧可能仍保留连接（导致下一次 QMTOPEN 返回 identifier in use）。
    // 为保证彻底清理，这里即使 mqtt_session_open == false 也强制发送 QMTDISC / QMTCLOSE。
    if (!mqtt_session_open) {
        ESP_LOGI("MQTT", "Session flag = false, still force sending QMTDISC/QMTCLOSE for safety");
    }
    
    // 1. 断开MQTT客户端连接
    send_at_command("AT+QMTDISC=0");
    // 可能先OK再+QMTDISC，也可能只有URC
    if (!wait_for_ok_then_contains("+QMTDISC: 0,0", 5000)) {
        // 容忍失败（可能已断开）
    }
    
    // 2. 关闭MQTT网络连接
    send_at_command("AT+QMTCLOSE=0");
    if (!wait_for_ok_then_contains("+QMTCLOSE: 0,0", 5000)) {
        // 容忍失败（可能已关闭）
    }
    
    // 等待一段时间确保连接完全关闭
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    ESP_LOGI("MQTT", "MQTT connection closed");
    mqtt_session_open = false;
    return true;
}

// 连接MQTT服务器（带重连机制）
static bool mqtt_connect_broker(void)
{
    ESP_LOGI("MQTT", "Connecting to MQTT broker...");
    
    // 最多尝试3次连接，每次失败后先清理并延时再试
    for (int attempt = 1; attempt <= 3; ++attempt) {
        ESP_LOGI("MQTT", "Attempt %d/3: Cleaning up and connecting...", attempt);
        // 0. 关闭可能存在的旧连接，避免identifier被占用
        mqtt_disconnect_broker();
        vTaskDelay(500 / portTICK_PERIOD_MS);

        // 1. 配置MQTT服务器版本
        ESP_LOGI("MQTT", "Step 1: Setting MQTT version...");
        char server_cmd[128];
        snprintf(server_cmd, sizeof(server_cmd), "AT+QMTCFG=\"version\",0,4");
        send_at_command(server_cmd);
        if (!wait_for_ok(3000)) {
            ESP_LOGW("MQTT", "Failed to set MQTT version (attempt %d)", attempt);
            goto connect_fail_delay;
        }
        
        // 2. 设置MQTT保持连接时间
        ESP_LOGI("MQTT", "Step 2: Setting keepalive interval...");
        char keepalive_cmd[64];
        snprintf(keepalive_cmd, sizeof(keepalive_cmd), "AT+QMTCFG=\"keepalive\",0,%d", MQTT_KEEPALIVE);
        send_at_command(keepalive_cmd);
        if (!wait_for_ok(3000)) {
            ESP_LOGW("MQTT", "Failed to set keepalive (attempt %d)", attempt);
            goto connect_fail_delay;
        }
        
        // 3. 打开MQTT网络连接
        ESP_LOGI("MQTT", "Step 3: Opening MQTT network connection...");
        char open_cmd[256];
        snprintf(open_cmd, sizeof(open_cmd), "AT+QMTOPEN=0,\"%s\",%d", MQTT_BROKER_HOST, MQTT_BROKER_PORT);
        send_at_command(open_cmd);
        if (!wait_for_ok_then_contains("+QMTOPEN: 0,0", 15000)) {  // 先OK再等URC
            ESP_LOGW("MQTT", "Failed to open MQTT network connection (attempt %d)", attempt);
            goto connect_fail_delay;
        }
        
        // 4. 连接MQTT客户端
        ESP_LOGI("MQTT", "Step 4: Connecting MQTT client...");
        char conn_cmd[256];
        snprintf(conn_cmd, sizeof(conn_cmd), "AT+QMTCONN=0,\"%s\",\"%s\",\"%s\"", 
                 MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD);
        send_at_command(conn_cmd);
        if (!wait_for_ok_then_contains("+QMTCONN: 0,0,0", 10000)) {
            ESP_LOGW("MQTT", "Failed to connect MQTT client (attempt %d)", attempt);
            goto connect_fail_delay;
        }

        ESP_LOGI("MQTT", "Connected to MQTT broker successfully (attempt %d)", attempt);
        mqtt_session_open = true;
        return true;

connect_fail_delay:
        // 失败兜底：清理连接并延时再试
        mqtt_disconnect_broker();
        mqtt_session_open = false;
        if (attempt < 3) {
            ESP_LOGW("MQTT", "Connect attempt %d failed, retry after delay...", attempt);
            vTaskDelay(2000 / portTICK_PERIOD_MS); // 延时2秒再试
        }
    }

    ESP_LOGW("MQTT", "All connection attempts failed");
    return false;
}

// 发布MQTT消息
static bool mqtt_publish_message(const char* topic, const char* message)
{
    ESP_LOGI("MQTT", "Publishing to topic: %s", topic);
    
    // 发布消息：固定 msgId=0，便于当前阶段测试（QoS 建议使用 0）
    char pub_cmd[512];
    snprintf(pub_cmd, sizeof(pub_cmd), "AT+QMTPUBEX=0,0,%d,0,\"%s\",%d", 
             MQTT_QOS_LEVEL, topic, (int)strlen(message));
    send_at_command(pub_cmd);
    
    if (!wait_for_contains(">", 3000)) {
        return false;
    }
    
    // 发送消息内容
    uart_write_bytes(UART_EG915U_NUM, message, strlen(message));

    // 等待发布完成URC：+QMTPUBEX: 0,0,0（msgId 固定为 0）
    if (!wait_for_contains("+QMTPUBEX: 0,0,0", 5000)) {
        ESP_LOGW("MQTT", "Publish ack not received for msgId=0");
        return false;
    }
    
    ESP_LOGI("MQTT", "Message published successfully");
    return true;
}

// 轮询处理URC（异步上报），例如 +QMTSTAT: 0,1
static void mqtt_poll_urc(void)
{
    if (mqtt_state != MQTT_STATE_CONNECTED) {
        return;
    }
    if (uart_read_busy) {
        return; // 避免与同步等待竞争
    }

    size_t bytes_available = 0;
    ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_EG915U_NUM, &bytes_available));
    if (bytes_available == 0) {
        return;
    }

    char buf[512];
    int len = uart_read_bytes(UART_EG915U_NUM, (uint8_t*)buf, sizeof(buf) - 1, 20 / portTICK_PERIOD_MS);
    if (len <= 0) {
        return;
    }
    buf[len] = '\0';
    ESP_LOGI("MQTT", "URC buffer: %s", buf);

    // 连接状态上报：1=已断开（由服务器或网络原因）
    if (strstr(buf, "+QMTSTAT: 0,1")) {
        ESP_LOGW("MQTT", "+QMTSTAT indicates disconnected by peer/network (0,1). Cleaning up...");
        mqtt_disconnect_broker();
        mqtt_state = MQTT_STATE_DISCONNECTED;
    }

    // 可以在此扩展其他URC处理，如 +QMTRECV 订阅消息等
}
#endif // ENABLE_MQTT


void gpio_init(void)
{
    // GPIO 初始化
    // IO2 在此设计上作为 IBL (ADC) 使用；确保不被驱动，设置为浮空以利于 ADC 采样
    gpio_reset_pin(IBL_PIN);
    gpio_set_direction(IBL_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(IBL_PIN, GPIO_FLOATING);
    gpio_set_direction(LED_PIN_3, GPIO_MODE_OUTPUT);
    gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON_PIN, GPIO_PULLUP_ONLY);

    gpio_set_direction(QT_POWER_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(QT_POWER_PIN, 0);

    gpio_set_direction(LED_PIN_EN_4V, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_PIN_EN_GPS, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_PIN_EN_3V3_LDO, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_PIN_EN_GNSS, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_PIN_RST_GNSS, GPIO_MODE_OUTPUT);
    gpio_set_direction(RS_EN, GPIO_MODE_OUTPUT);

    gpio_set_direction(EG915U_POWER, GPIO_MODE_INPUT);
    

    gpio_set_level(LED_PIN_EN_4V, 1);
    gpio_set_level(LED_PIN_EN_GPS, 1);
    gpio_set_level(LED_PIN_EN_3V3_LDO, 1);
    gpio_set_level(LED_PIN_EN_GNSS, 1);
    gpio_set_level(LED_PIN_RST_GNSS, 0);
    gpio_set_level(RS_EN, 1);

}

void app_main(void)
{
    ESP_LOGI("APP", "app_main start");
    gpio_init();

    // 初始化 GNSS UART2 (LC760Z)
    uart_config_t gnss_uart_cfg = {
        .baud_rate = GNSS_BAUD,         // GNSS 波特率
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    ESP_LOGI("GNSS", "UART2 (GNSS) init at %d baud", GNSS_BAUD);
    ESP_ERROR_CHECK(uart_driver_install(GNSS_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(GNSS_UART_NUM, &gnss_uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(GNSS_UART_NUM, GNSS_UART_TX, GNSS_UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_LOGI("GNSS", "Pins set: TX=%d -> GNSS RXD, RX=%d <- GNSS TXD", GNSS_UART_TX, GNSS_UART_RX);

    // GNSS 使能脚（IO46）拉高
    gpio_set_direction(GNSS_EN_PIN, GPIO_MODE_OUTPUT);
    gnss_enable(true);
    ESP_LOGI("GNSS", "GNSS EN pin %d set HIGH", GNSS_EN_PIN);
    vTaskDelay(200 / portTICK_PERIOD_MS);

    // 初始化 UART1 (普通轮询模式)
    uart_config_t uart_config2 = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    ESP_LOGI("UART", "UART1 initialized at 115200 baud with polling mode");
    
    // 安装UART驱动，不使用中断
    ESP_ERROR_CHECK(uart_driver_install(UART_EG915U_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_EG915U_NUM, &uart_config2));
    ESP_ERROR_CHECK(uart_set_pin(UART_EG915U_NUM, EG915U_RX_PIN, EG915U_TX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    // 创建 GNSS 读取任务
    xTaskCreate(gnss_task, "gnss_task", 4096, NULL, 8, NULL);
    
// wait_for_ok 提前定义，避免重复定义

    
    // 初始化100ms定时器
    init_timer_100ms();

    // 初始化 ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL, ADC_ATTEN_DB_11);
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, adc_chars);

    // 等待一段时间让UART稳定
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    

    // 启动主业务：GPS->MQTT->睡眠 周期任务
    xTaskCreate(gps_mqtt_cycle_task, "gps_cycle", 4096, NULL, 9, NULL);

    while (1) {
        // 读取 ADC 值
        uint32_t adc_reading = 0;
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            adc_reading += adc1_get_raw(ADC1_CHANNEL);
        }
        adc_reading /= NO_OF_SAMPLES;
      //  uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
      //  ESP_LOGI("ADC", "Raw: %lu, Voltage: %lu mV", (unsigned long)adc_reading, (unsigned long)voltage);

        if(log_output == true)
        {
            if(gpio_get_level(EG915U_POWER) == 0)
            {
            ESP_LOGI("MODEM", "ON/OFF the modem");
            gpio_set_level(QT_POWER_PIN, 0);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            gpio_set_level(QT_POWER_PIN, 1);
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            gpio_set_level(QT_POWER_PIN, 0);
            }
            // 发送AT指令测试通信
            vTaskDelay(1000 / portTICK_PERIOD_MS); // 等待模组启动
            
            log_output = false;
        }

        if (send_at_flag) {
            send_at_flag = false;
            send_at_command("AT");
        }


        // GNSS 串口由 gnss_task 读取，这里不再做回环回显

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
