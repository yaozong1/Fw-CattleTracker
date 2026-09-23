#include "mqtt_module.h"
#include "hardware_config.h"
#include "mqtt_config.h"
#include "gnss_module.h"
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

static const char *TAG = "MQTT";

// 全局状态变量
#if ENABLE_MQTT
static mqtt_state_t mqtt_state = MQTT_STATE_IDLE;
static bool apn_configured = false;
static uint16_t mqtt_msg_id = 1;
static bool mqtt_session_open = false;
#endif

// 简单的串口读互斥，避免URC轮询与同步等待抢读
static volatile bool uart_read_busy = false;

// 外部依赖的ADC变量（需要从外部提供）
extern esp_adc_cal_characteristics_t *adc_chars;

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
            int len = uart_read_bytes(UART_EG800_NUM, (uint8_t*)buffer + total_len, 
                                     max_len - total_len - 1, 50 / portTICK_PERIOD_MS);
            if (len > 0) {
                total_len += len;
                buffer[total_len] = '\0';
                
                // 如果收到完整的响应（包含\r\n），提前返回
                if (strstr(buffer, "\r\n") || strstr(buffer, "OK") || strstr(buffer, "ERROR")) {
                    // 继续读取一小段时间，确保完整接收
                    vTaskDelay(100 / portTICK_PERIOD_MS);
                    
                    ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_EG800_NUM, (size_t*)&bytes_available));
                    if (bytes_available > 0) {
                        len = uart_read_bytes(UART_EG800_NUM, (uint8_t*)buffer + total_len, 
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

bool mqtt_init_network(void)
{
    ESP_LOGI(TAG, "Initializing 4G network...");
    
    // 0. 清理上电URC
    ESP_LOGI(TAG, "Warm-up: draining boot URCs...");
    uint32_t drain_start = xTaskGetTickCount();
    char drain_buf[256];
    while ((xTaskGetTickCount() - drain_start) < (3000 / portTICK_PERIOD_MS)) {
        int len = uart_read_bytes(UART_EG800_NUM, (uint8_t*)drain_buf, sizeof(drain_buf) - 1, 50 / portTICK_PERIOD_MS);
        if (len > 0) {
            drain_buf[len] = '\0';
            if (strstr(drain_buf, "RDY") || strstr(drain_buf, "Call Ready") || strstr(drain_buf, "SMS Ready") || strstr(drain_buf, "PB Done")) {
                ESP_LOGI(TAG, "Boot URC: %s", drain_buf);
            }
        } else {
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }
    }
    
    // 1. 检查模组状态
    ESP_LOGI(TAG, "Step 1: Checking module status (AT handshake with retries)...");
    bool at_ok = false;
    for (int attempt = 1; attempt <= 5 && !at_ok; ++attempt) {
        send_at_command("AT");
        at_ok = wait_for_ok(1000);
        if (!at_ok) {
            ESP_LOGW(TAG, "AT no response, retry %d/5", attempt);
            vTaskDelay(300 / portTICK_PERIOD_MS);
        }
    }
    if (!at_ok) {
        ESP_LOGW(TAG, "Module not responding to AT command after retries");
        return false;
    }
    
    // 2. 关闭回显
    ESP_LOGI(TAG, "Step 2: Disabling echo...");
    send_at_command("ATE0");
    wait_for_ok(3000);
    
    // 3. 设置全功能模式
    ESP_LOGI(TAG, "Step 3: Setting full functionality mode...");
    send_at_command("AT+CFUN=1");
    if (!wait_for_ok(15000)) {
        ESP_LOGW(TAG, "Failed to set full functionality mode");
        return false;
    }
    
    // 等待模组稳定
    ESP_LOGI(TAG, "Waiting for module to stabilize...");
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    
    // 4. 检查SIM卡状态
    ESP_LOGI(TAG, "Step 4: Checking SIM card status...");
    send_at_command("AT+CPIN?");
    if (!wait_for_contains("READY", 5000)) {
        ESP_LOGW(TAG, "SIM card not ready");
    }
    
    // 5. 设置APN
    if (strlen(APN_NAME) > 0 && !apn_configured) {
        ESP_LOGI(TAG, "Step 5: Setting APN to %s (first time)...", APN_NAME);
        char apn_cmd[128];
        snprintf(apn_cmd, sizeof(apn_cmd), "AT+CGDCONT=1,\"IP\",\"%s\"", APN_NAME);
        send_at_command(apn_cmd);
        if (!wait_for_ok(5000)) {
            ESP_LOGW(TAG, "Failed to set APN");
            return false;
        }
        apn_configured = true;
        ESP_LOGI(TAG, "APN configured successfully, will not set again");
    } else if (strlen(APN_NAME) > 0 && apn_configured) {
        ESP_LOGI(TAG, "Step 5: Skipping APN setting (already configured this session)");
    } else {
        ESP_LOGI(TAG, "Step 5: Skipping APN setting (APN_NAME is empty)");
    }
    
    // 6. 激活PDP上下文
    ESP_LOGI(TAG, "Step 6: Activating PDP context...");
    send_at_command("AT+CGACT=1,1");
    if (!wait_for_ok(30000)) {
        ESP_LOGW(TAG, "Failed to activate PDP context");
        return false;
    }
    
    // 7. 查询IP地址
    ESP_LOGI(TAG, "Step 7: Getting IP address...");
    send_at_command("AT+CGPADDR=1");
    if (!wait_for_ok(5000)) {
        ESP_LOGW(TAG, "Failed to get IP address, but continuing...");
    }
    
    ESP_LOGI(TAG, "4G network initialized successfully");
    return true;
}

bool mqtt_disconnect_broker(void)
{
    ESP_LOGI(TAG, "Closing MQTT connection...");
    if (!mqtt_session_open) {
        ESP_LOGI(TAG, "Session flag = false, still force sending QMTDISC/QMTCLOSE for safety");
    }
    
    // 1. 断开MQTT客户端连接
    send_at_command("AT+QMTDISC=0");
    if (!wait_for_ok_then_contains("+QMTDISC: 0,0", 5000)) {
        // 容忍失败
    }
    
    // 2. 关闭MQTT网络连接
    send_at_command("AT+QMTCLOSE=0");
    if (!wait_for_ok_then_contains("+QMTCLOSE: 0,0", 5000)) {
        // 容忍失败
    }
    
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    ESP_LOGI(TAG, "MQTT connection closed");
    mqtt_session_open = false;
    return true;
}

bool mqtt_connect_broker(void)
{
    ESP_LOGI(TAG, "Connecting to MQTT broker...");
    
    // 最多尝试3次连接
    for (int attempt = 1; attempt <= 3; ++attempt) {
        ESP_LOGI(TAG, "Attempt %d/3: Cleaning up and connecting...", attempt);
        mqtt_disconnect_broker();
        vTaskDelay(500 / portTICK_PERIOD_MS);

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
    return true;
}

void mqtt_poll_urc(void)
{
    if (mqtt_state != MQTT_STATE_CONNECTED) {
        return;
    }
    if (uart_read_busy) {
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
    // Cattle_T: EG800。IO48(MODEM_EN) 低有效，先确保 3.8V 供电已打开；
    // IO33(PWRKEY) 经 NPN 反相，置 1 相当于把 PWRKEY 拉低。
    ESP_LOGI("MODEM", "Power-cycle EG800 via PWRKEY sequence");
    gpio_set_level(MODEM_EN_PIN, 0);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    gpio_set_level(QT_POWER_PIN, 0);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    gpio_set_level(QT_POWER_PIN, 1);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    gpio_set_level(QT_POWER_PIN, 0);
}

static void modem_power_off_soft(void)
{
    ESP_LOGI("MODEM", "Soft power off EG800 (AT+CFUN=0)");
#if ENABLE_MQTT
    send_at_command("AT+CFUN=0");
    wait_for_ok(10000);
#endif
}

#if ENABLE_MQTT
static float read_battery_voltage(void)
{
    uint32_t adc_reading = 0;
    for (int i = 0; i < NO_OF_SAMPLES; i++) adc_reading += adc1_get_raw(ADC1_CHANNEL);
    adc_reading /= NO_OF_SAMPLES;
    uint32_t mv = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
    return (float)mv / 1000.0f * VBAT_DIVIDER_RATIO;
}

static int estimate_soc_from_voltage(float v)
{
    // Cattle_T：单节锂电池（阈值见 hardware_config.h）
    if (v >= VBAT_FULL_V)  { return 100; }
    if (v <= VBAT_EMPTY_V) { return 0; }
    float soc = (v - VBAT_EMPTY_V) / (VBAT_FULL_V - VBAT_EMPTY_V) * 100.0f;
    if (soc < 0) {
        soc = 0;
    }
    if (soc > 100) {
        soc = 100;
    }
    return (int)(soc + 0.5f);
}

bool publish_battery_payload(void)
{
    char payload[512];
    float voltage = 4.2f;
    static int current_soc = 96;
    
    // 每次交替 +1 / -1
    static int toggle = 0;
    int delta = toggle ? -1 : 1;
    toggle ^= 1;
    current_soc += delta;
    if (current_soc < 0) current_soc = 0;
    if (current_soc > 100) current_soc = 100;
    int soc = current_soc;
    
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
    
    int n = snprintf(payload, sizeof(payload),
    "{\n"
    "  \"deviceId\": \"%s\",\n"
    "  \"timestamp\": %llu,\n"
    "  \"battery\": {\n"
        "    \"soc\": %d,\n"
        "    \"voltage\": %.2f,\n"
        "    \"temperature\": %.1f,\n"
        "    \"health\": %d,\n"
        "    \"cycleCount\": %d,\n"
        "    \"estimatedRange\": %.1f,\n"
        "    \"chargingStatus\": \"%s\"\n"
    "  },\n"
        "  \"gps\": {\n"
        "    \"lat\": %.6f,\n"
        "    \"lng\": %.6f,\n"
        "    \"speed\": %.1f,\n"
        "    \"heading\": %d,\n"
        "    \"altitude\": %.1f,\n"
        "    \"accuracy\": %d\n"
    "  }\n"
    "}",
    "CT-001",
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
        ESP_LOGI("BAT", "Topic: %s\nPayload:\n%s", MQTT_TOPIC_BATTERY, payload);
        bool ok = mqtt_publish_message(MQTT_TOPIC_BATTERY, payload);
        mqtt_disconnect_broker();
        if (ok) return true;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    return false;
}

bool publish_gps_payload(void)
{
    // GPS数据已经包含在battery payload中，这里可以单独发送GPS数据
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
    "  \"deviceId\": \"CT-001\",\n"
    "  \"timestamp\": %llu,\n"
    "  \"lat\": %.6f,\n"
    "  \"lng\": %.6f,\n"
    "  \"speed\": %.1f,\n"
    "  \"heading\": %d,\n"
    "  \"altitude\": 156.0,\n"
    "  \"accuracy\": 5\n"
    "}",
        ts_ms,
        gnss_last_location.lat_deg,
        gnss_last_location.lon_deg,
        speed_kmh,
        heading);
        
    if (n <= 0 || n >= (int)sizeof(payload)) {
        ESP_LOGE("GPS", "GPS JSON build failed");
        return false;
    }

    for (int attempt = 1; attempt <= MQTT_SEND_MAX_RETRY; ++attempt) {
        ESP_LOGI("GPS", "MQTT publish attempt %d/%d", attempt, MQTT_SEND_MAX_RETRY);
        if (!mqtt_init_network()) continue;
        if (!mqtt_connect_broker()) continue;
        ESP_LOGI("GPS", "Topic: %s\nPayload:\n%s", MQTT_TOPIC_GPS, payload);
        bool ok = mqtt_publish_message(MQTT_TOPIC_GPS, payload);
        mqtt_disconnect_broker();
        if (ok) return true;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    return false;
}
#endif // ENABLE_MQTT

void gps_mqtt_cycle_task(void* pv)
{
    ESP_LOGI("GPS", "gps_mqtt_cycle_task started: sleep=%d min, fix timeout=%d s, mqtt retries=%d",
             GPS_CYCLE_SLEEP_MIN, GPS_FIX_TIMEOUT_SEC, MQTT_SEND_MAX_RETRY);

    while (1) {
        // 1) 仅启动 GNSS，先尝试拿定位
        bool powered_modem = false;
        gnss_enable(true);
        gnss_reset_release();

        // 2) 连续模式下等待 GNSS 定位
        gnss_last_location.valid = false;
        gnss_reset_state_for_ttff();
        ESP_LOGI("GPS", "Waiting GNSS fix (timeout %ds)...", GPS_FIX_TIMEOUT_SEC);
        TickType_t t0 = xTaskGetTickCount();
        while ((xTaskGetTickCount() - t0) < pdMS_TO_TICKS(GPS_FIX_TIMEOUT_SEC * 1000)) {
            if (gnss_last_location.valid) { break; }
            vTaskDelay(200 / portTICK_PERIOD_MS);
        }

        // 3) 若拿到定位，则唤醒 EG800 并进行 MQTT
#if ENABLE_MQTT
        if (gnss_last_location.valid) {
            ESP_LOGI("GPS", "GNSS fix acquired, bringing up EG800 for MQTT publish");
            ESP_LOGI("MODEM", "Wake EG800: about to PWRKEY pulse (power-cycle) to publish");
            modem_power_cycle();
            powered_modem = true;
            vTaskDelay(2000 / portTICK_PERIOD_MS);

            bool pub_ok = publish_battery_payload();
            ESP_LOGI("BAT", "Publish %s", pub_ok ? "success" : "failed");
        } else {
            ESP_LOGW("GPS", "No GNSS fix within %ds, skip EG800/MQTT and go to sleep", GPS_FIX_TIMEOUT_SEC);
        }
#endif

        // 4) 进入省电
        ESP_LOGI("GPS", "Enter sleep: GNSS standby + modem CFUN=0 for %d min", GPS_CYCLE_SLEEP_MIN);
        gnss_enter_standby2();
        if (powered_modem) {
            modem_power_off_soft();
        } else {
            ESP_LOGI("MODEM", "Skip modem CFUN=0: modem wasn't powered this cycle");
        }

        // 进入睡眠间隔
        vTaskDelay(GPS_CYCLE_SLEEP_MIN * 60 * 1000 / portTICK_PERIOD_MS);

        // 5) 从 Standby 唤醒 GNSS
        ESP_LOGI("GNSS_LP", "Wake GNSS: about to exit Standby (TX 0xFF + CRLF)");
        gnss_wake_from_standby2();
    }
}

void mqtt_init(void)
{
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