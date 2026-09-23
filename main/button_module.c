#include "button_module.h"
#include "hardware_config.h"
#include "gpio_module.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "BUTTON";

// 按键为低有效（SW2 接地 + 内部上拉）
static inline bool btn_down(void)
{
    return gpio_get_level(BUTTON_PIN) == 0;
}

// 等待按键稳定在指定状态，返回是否在超时前达成
// 同时通过 held_ms 回报实际等待时长
static bool wait_level(bool want_down, uint32_t timeout_ms, uint32_t *elapsed_ms)
{
    uint32_t t = 0;
    uint32_t stable = 0;
    while (t < timeout_ms) {
        vTaskDelay(pdMS_TO_TICKS(BUTTON_POLL_MS));
        t += BUTTON_POLL_MS;
        if (btn_down() == want_down) {
            stable += BUTTON_POLL_MS;
            if (stable >= BUTTON_DEBOUNCE_MS) {
                if (elapsed_ms) { *elapsed_ms = t; }
                return true;
            }
        } else {
            stable = 0;
        }
    }
    if (elapsed_ms) { *elapsed_ms = t; }
    return false;
}

void button_trigger_fence_rails_on(void)
{
    fence_rails_enable(true);
    ESP_LOGI(TAG, "Double-click: 5V0_ISO_EN + 12V_EN driven HIGH (charge stays LOW)");
    led_blink(LED_ESP_PIN, BUTTON_ACK_BLINK_TIMES, BUTTON_ACK_BLINK_ON_MS, BUTTON_ACK_BLINK_OFF_MS);

#if FENCE_TRIG_TEST_AFTER_DBLCLICK
    // 等供电轨建立，再对 Trig 发方波，验证 PC817 + 可控硅驱动电路
    vTaskDelay(pdMS_TO_TICKS(FENCE_TRIG_TEST_SETTLE_MS));
    fence_trig_pulse_train(FENCE_TRIG_TEST_PULSES,
                           FENCE_TRIG_TEST_ON_MS,
                           FENCE_TRIG_TEST_OFF_MS);
#endif
}

static void button_task(void *pv)
{
    (void)pv;
    ESP_LOGI(TAG, "Button task started on IO%d (double-click -> fence rails ON)", BUTTON_PIN);

    while (1) {
        // --- 等待第一次按下 ---
        if (!btn_down()) {
            vTaskDelay(pdMS_TO_TICKS(BUTTON_POLL_MS));
            continue;
        }
        // 去抖确认
        if (!wait_level(true, BUTTON_DEBOUNCE_MS * 3, NULL)) {
            continue;   // 抖动，丢弃
        }

        // --- 等待松开，顺便判断是否长按 ---
        uint32_t held = 0;
        bool released = wait_level(false, BUTTON_LONGPRESS_MS, &held);

        if (!released) {
#if BUTTON_LONGPRESS_OFF
            // 长按：回到安全状态
            fence_rails_enable(false);
            ESP_LOGI(TAG, "Long press: fence rails back to SAFE (both LOW)");
            led_blink(LED_ESP_PIN, 1, 600, 200);
#else
            ESP_LOGI(TAG, "Long press ignored (BUTTON_LONGPRESS_OFF=0)");
#endif
            // 等按键真正松开再继续，避免重复触发
            while (btn_down()) {
                vTaskDelay(pdMS_TO_TICKS(BUTTON_POLL_MS));
            }
            continue;
        }

        // --- 在窗口内等待第二次按下 ---
        uint32_t gap = 0;
        bool second = wait_level(true, BUTTON_DBLCLICK_WINDOW_MS, &gap);
        if (!second) {
            ESP_LOGD(TAG, "Single click ignored (no second press within %d ms)",
                     BUTTON_DBLCLICK_WINDOW_MS);
            continue;
        }

        // 双击成立
        button_trigger_fence_rails_on();

        // 等第二次按键松开，防止连击被拆成新的一轮
        while (btn_down()) {
            vTaskDelay(pdMS_TO_TICKS(BUTTON_POLL_MS));
        }
        vTaskDelay(pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS));
    }
}

void button_module_start(void)
{
    xTaskCreate(button_task, "button_task", 3072, NULL, 6, NULL);
}
