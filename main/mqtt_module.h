#ifndef MQTT_MODULE_H
#define MQTT_MODULE_H

#include <stdint.h>
#include <stdbool.h>
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