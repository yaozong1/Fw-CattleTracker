#ifndef MQTT_MODULE_H
#define MQTT_MODULE_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gnss_module.h"

// MQTT连接状态
typedef enum {
    MQTT_STATE_IDLE,
    MQTT_STATE_INIT_NETWORK,
    MQTT_STATE_CONNECTING,
    MQTT_STATE_CONNECTED,
    MQTT_STATE_DISCONNECTED,
    MQTT_STATE_ERROR
} mqtt_state_t;

// MQTT相关配置（从mqtt_config.h获取）
#ifndef APN_NAME
#define APN_NAME ""
#endif
#ifndef MQTT_KEEPALIVE
#define MQTT_KEEPALIVE 120
#endif
#ifndef MQTT_QOS_LEVEL
#define MQTT_QOS_LEVEL 0
#endif

// 设备标识：deviceId 绑定模组 IMEI。
// device_fetch_imei() 在网络初始化时自动调一次，成功后结果被缓存。
bool device_fetch_imei(void);
const char* device_get_id(void);
bool device_id_is_imei(void);

// 拼出 fleet/<deviceId>/<suffix>，deviceId 取自 IMEI
void mqtt_build_topic(char *out, size_t out_sz, const char *suffix);

// AT命令函数
void send_at_command(const char* command);
bool wait_for_ok(uint32_t timeout_ms);
int uart_read_response(char* buffer, int max_len, int timeout_ms);

// MQTT功能函数
bool mqtt_init_network(void);
bool mqtt_connect_broker(void);
bool mqtt_publish_message(const char* topic, const char* message);
bool mqtt_disconnect_broker(void);
void mqtt_poll_urc(void);

// 在唤醒模组之前采一次空载电池电压，供本轮上报使用
void battery_snapshot_idle(void);

// RTC 内存里的跨深睡计数器（实现在 main.c）
uint32_t app_get_boot_count(void);
void app_note_fix_ok(void);
void app_note_publish_ok(void);

// 手动上报：不管有没有 GNSS 定位都发一条当前 payload。
// 立即返回，实际工作在独立的一次性任务里完成。
void mqtt_manual_publish_request(void);

// 业务功能函数
bool publish_gps_payload(void);
bool publish_battery_payload(void);
void modem_power_cycle(void);

// GPS+MQTT周期任务
void gps_mqtt_cycle_task(void* pv);

// MQTT初始化
void mqtt_init(void);

// 获取MQTT状态
mqtt_state_t get_mqtt_state(void);
void set_mqtt_state(mqtt_state_t state);

#endif // MQTT_MODULE_H