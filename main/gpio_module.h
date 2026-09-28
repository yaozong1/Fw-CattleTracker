#ifndef GPIO_MODULE_H
#define GPIO_MODULE_H

#include <stdint.h>
#include <stdbool.h>

// GPIO控制函数
void gpio_init(void);
void led_set(int led_pin, bool state);
bool button_is_pressed(void);
void power_control_init(void);

// LED控制函数（led1 = ESP_LED/IO46, led3 = CONN_LED/IO45）
void led1_on(void);
void led1_off(void);
void led3_on(void);
void led3_off(void);

// 闪烁指定 LED 若干次，结束后恢复闪烁之前的亮灭状态
void led_blink(int led_pin, int times, uint32_t on_ms, uint32_t off_ms);

// 蜂窝模组电源控制（IO48 低有效：0 = 打开 3.8V 供电）
void modem_power_enable(bool on);
bool modem_is_powered(void);

// GNSS 状态引脚
int  gnss_get_3dfix_level(void);
int  gnss_get_geofence_level(void);
int  gnss_get_jam_level(void);

// 电子围栏引脚安全状态（充电关闭、不触发）
void fence_pins_safe_init(void);
// LT3750 DONE# 为低电平表示充电完成
bool fence_charge_done(void);

/* ---- deep sleep 引脚保持 ----
 * ESP32-S3 只有 IO0~IO21 是 RTC GPIO，深睡期间只有它们能靠 hold 维持电平，
 * 其余引脚一律浮空。进 deep sleep 前调 prepare，醒来 gpio_init() 后调 release。 */
void deep_sleep_prepare_pins(void);
void deep_sleep_release_pins(void);

// 电子围栏电源轨（12V_EN + 5V0_ISO_EN）。
// 只开/关供电轨，CHARGE 与 TRIG 始终保持低电平，不会充电也不会放电。
void fence_rails_enable(bool on);
bool fence_rails_are_on(void);

// Trig 方波测试：拉高 on_ms、拉低 off_ms，重复 times 次，结束后保证停在低电平。
// 用于没接变压器时验证 PC817 / 可控硅驱动这段电路。
// CHARGE (IO9) 在整个过程中保持低电平，储能电容不会被充电。
void fence_trig_pulse_train(int times, uint32_t on_ms, uint32_t off_ms);

#endif // GPIO_MODULE_H
