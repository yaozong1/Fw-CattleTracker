#ifndef BUTTON_MODULE_H
#define BUTTON_MODULE_H

#include <stdbool.h>

/* =====================================================================
 * IO0 按键手势识别
 *   双击 -> 拉高 5V0_ISO_EN (IO12) 与 12V_EN (IO47)，ESP_LED 闪烁提示
 *   长按 -> 两路供电轨回到安全状态（可用 BUTTON_LONGPRESS_OFF 关闭此行为）
 *
 * 注意：只开关供电轨，CHARGE (IO9) 与 TRIG (IO13) 始终保持低电平，
 *       不会启动 LT3750 充电，也不会触发放电。
 * ===================================================================== */

// 启动按键监听任务
void button_module_start(void);

// 手动触发一次"双击"动作（调试用）
void button_trigger_fence_rails_on(void);

#endif // BUTTON_MODULE_H
