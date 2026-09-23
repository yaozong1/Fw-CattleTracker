#ifndef TIMER_MODULE_H
#define TIMER_MODULE_H

#include <stdint.h>
#include <stdbool.h>

// 全局标志位声明
extern volatile bool log_output;
extern volatile bool send_at_flag;
extern volatile bool mqtt_connect_flag;
extern volatile bool mqtt_publish_flag;

// 定时器控制函数
void timer_init(void);
void timer_start(void);
void timer_stop(void);

// 定时器回调函数类型
typedef bool (*timer_callback_t)(void);

// 注册定时器回调
void timer_register_callback(timer_callback_t callback);

#endif // TIMER_MODULE_H