#ifndef CAN_MODULE_H
#define CAN_MODULE_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/twai.h"

// CAN通信参数
#define CAN_BITRATE_125K    125000
#define CAN_BITRATE_250K    250000
#define CAN_BITRATE_500K    500000
#define CAN_BITRATE_1M      1000000

// 默认CAN参数
#define DEFAULT_CAN_BITRATE CAN_BITRATE_500K
#define CAN_RX_QUEUE_SIZE   20
#define CAN_TX_QUEUE_SIZE   20

// CAN消息类型
typedef enum {
    CAN_MSG_TYPE_DATA = 0,
    CAN_MSG_TYPE_REMOTE = 1
} can_msg_type_t;

// CAN帧格式
typedef enum {
    CAN_FRAME_STD = 0,      // 标准帧 (11位ID)
    CAN_FRAME_EXT = 1       // 扩展帧 (29位ID)
} can_frame_format_t;

// CAN消息结构
typedef struct {
    uint32_t identifier;           // CAN ID
    can_frame_format_t format;     // 标准帧或扩展帧
    can_msg_type_t type;          // 数据帧或远程帧
    uint8_t data_length;          // 数据长度 (0-8字节)
    uint8_t data[8];              // 数据内容
} can_message_t;

// CAN状态
typedef enum {
    CAN_STATE_STOPPED = 0,
    CAN_STATE_RUNNING,
    CAN_STATE_BUS_OFF,
    CAN_STATE_ERROR_WARNING,
    CAN_STATE_ERROR_PASSIVE
} can_state_t;

// CAN统计信息
typedef struct {
    uint32_t tx_count;            // 发送帧计数
    uint32_t rx_count;            // 接收帧计数
    uint32_t tx_error_count;      // 发送错误计数
    uint32_t rx_error_count;      // 接收错误计数
    uint32_t bus_off_count;       // 总线关闭计数
    uint32_t arbitration_lost_count; // 仲裁丢失计数
} can_stats_t;

// 初始化和控制函数
bool can_module_init(uint32_t bitrate);
bool can_module_start(void);
bool can_module_stop(void);
bool can_module_deinit(void);
bool can_transceiver_enable(bool enable);

// 消息收发函数
bool can_send_message(const can_message_t* message, uint32_t timeout_ms);
bool can_receive_message(can_message_t* message, uint32_t timeout_ms);
bool can_send_data(uint32_t id, const uint8_t* data, uint8_t length, uint32_t timeout_ms);
bool can_send_remote_frame(uint32_t id, uint8_t length, uint32_t timeout_ms);

// 状态和统计函数
can_state_t can_get_state(void);
bool can_get_stats(can_stats_t* stats);
void can_reset_stats(void);
bool can_is_bus_healthy(void);
bool can_force_reset(void);  // 强制复位 CAN 控制器状态

// 任务和处理函数
void can_task(void* pvParameters);
void can_test_loopback(void);
void can_test_send_periodic(void);

// 回调函数类型
typedef void (*can_rx_callback_t)(const can_message_t* message);
typedef void (*can_error_callback_t)(can_state_t state, uint32_t error_code);

// 回调函数注册
bool can_register_rx_callback(can_rx_callback_t callback);
bool can_register_error_callback(can_error_callback_t callback);

#endif // CAN_MODULE_H