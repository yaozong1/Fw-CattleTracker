#include "gpio_module.h"
#include "hardware_config.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "GPIO";

// LED 当前状态（led_blink 结束后据此恢复）
static bool s_led_esp_on  = false;
static bool s_led_conn_on = false;

// 电子围栏供电轨状态
static bool s_fence_rails_on = false;

void gpio_init(void)
{
    /* ---- 指示灯 ---- */
    gpio_reset_pin(LED_ESP_PIN);
    gpio_set_direction(LED_ESP_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_ESP_PIN, 0);

    gpio_reset_pin(LED_CONN_PIN);
    gpio_set_direction(LED_CONN_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_CONN_PIN, 0);

    /* ---- 按键 ---- */
    gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON_PIN, GPIO_PULLUP_ONLY);

    /* ---- IBL / 电池采样 (IO1)：保持浮空，交给 ADC ---- */
    gpio_set_pull_mode(IBL_PIN, GPIO_FLOATING);

    /* ---- 蜂窝模组控制 ---- */
    gpio_set_direction(QT_POWER_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(QT_POWER_PIN, 0);            // PWRKEY 释放

    gpio_set_direction(MODEM_RESET_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(MODEM_RESET_PIN, 0);         // 不复位

    gpio_set_direction(MODEM_EN_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(MODEM_EN_PIN, 0);            // 低有效：打开 3.8V 供电

    gpio_set_direction(MODEM_STATUS_PIN, GPIO_MODE_INPUT);

    /* ---- GNSS ---- */
    gpio_set_direction(GNSS_EN_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(GNSS_RST_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(GNSS_RST_PIN, 0);            // 反相驱动，0 = 释放复位

    gpio_set_direction(GNSS_PPS_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(GNSS_3DFIX_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(GNSS_GEOFENCE_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(GNSS_JAMIND_PIN, GPIO_MODE_INPUT);

    /* ---- 加速度计中断 ---- */
    gpio_set_direction(MOTION_SENSOR_INT1_PIN, GPIO_MODE_INPUT);

    /* ---- 电子围栏：必须最先进入安全状态 ---- */
    fence_pins_safe_init();

    power_control_init();

    ESP_LOGI(TAG, "GPIO initialized (Cattle_T)");
}

void power_control_init(void)
{
    // 打开模组供电与 GNSS 供电，复位脚释放
    gpio_set_level(MODEM_EN_PIN, 0);   // 低有效
    gpio_set_level(GNSS_EN_PIN, 1);
    gpio_set_level(GNSS_RST_PIN, 0);

    ESP_LOGI(TAG, "Power control pins initialized (modem ON, GNSS ON)");
}

/* ============ 电子围栏 ============ */
void fence_pins_safe_init(void)
{
    gpio_reset_pin(FENCE_TRIG_PIN);
    gpio_set_direction(FENCE_TRIG_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(FENCE_TRIG_PIN, 0);          // 绝对不能默认触发

    gpio_reset_pin(FENCE_CHARGE_EN_PIN);
    gpio_set_direction(FENCE_CHARGE_EN_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(FENCE_CHARGE_EN_PIN, 0);     // LT3750 不充电

    gpio_set_direction(FENCE_CHARGE_DONE_PIN, GPIO_MODE_INPUT);

    gpio_reset_pin(FENCE_12V_EN_PIN);
    gpio_set_direction(FENCE_12V_EN_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(FENCE_12V_EN_PIN, 0);        // 12V 升压关闭

    gpio_reset_pin(FENCE_5V_ISO_EN_PIN);
    gpio_set_direction(FENCE_5V_ISO_EN_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(FENCE_5V_ISO_EN_PIN, 0);     // 隔离 5V 关闭

    s_fence_rails_on = false;

    ESP_LOGI(TAG, "Fence pins in SAFE state (charge off, no trigger)");
}

void fence_rails_enable(bool on)
{
    if (on) {
        // 先开控制侧的隔离 5V，再开 12V 升压
        gpio_set_level(FENCE_5V_ISO_EN_PIN, 1);
        gpio_set_level(FENCE_12V_EN_PIN, 1);
    } else {
        gpio_set_level(FENCE_12V_EN_PIN, 0);
        gpio_set_level(FENCE_5V_ISO_EN_PIN, 0);
    }
    // 无论如何，充电与触发保持关闭
    gpio_set_level(FENCE_CHARGE_EN_PIN, 0);
    gpio_set_level(FENCE_TRIG_PIN, 0);

    s_fence_rails_on = on;
    ESP_LOGI(TAG, "Fence rails (12V_EN IO%d + 5V0_ISO_EN IO%d) -> %s",
             FENCE_12V_EN_PIN, FENCE_5V_ISO_EN_PIN, on ? "ON" : "OFF");
}

bool fence_rails_are_on(void)
{
    return s_fence_rails_on;
}

void fence_trig_pulse_train(int times, uint32_t on_ms, uint32_t off_ms)
{
    // 保证不会在充电状态下触发
    gpio_set_level(FENCE_CHARGE_EN_PIN, 0);

    ESP_LOGI(TAG, "Trig pulse train on IO%d: %d cycles, %lu ms high / %lu ms low",
             FENCE_TRIG_PIN, times, (unsigned long)on_ms, (unsigned long)off_ms);

    for (int i = 1; i <= times; ++i) {
        gpio_set_level(FENCE_TRIG_PIN, 1);
        ESP_LOGI(TAG, "  Trig %d/%d HIGH", i, times);
        vTaskDelay(pdMS_TO_TICKS(on_ms));
        gpio_set_level(FENCE_TRIG_PIN, 0);
        ESP_LOGI(TAG, "  Trig %d/%d LOW", i, times);
        vTaskDelay(pdMS_TO_TICKS(off_ms));
    }

    // 收尾务必停在低电平
    gpio_set_level(FENCE_TRIG_PIN, 0);
    ESP_LOGI(TAG, "Trig pulse train done, IO%d held LOW", FENCE_TRIG_PIN);
}

bool fence_charge_done(void)
{
    // DONE# 低有效
    return gpio_get_level(FENCE_CHARGE_DONE_PIN) == 0;
}

/* ============ 模组电源 ============ */
void modem_power_enable(bool on)
{
    // IO48 -> Q5 -> LDO EN（反相），低电平打开供电
    gpio_set_level(MODEM_EN_PIN, on ? 0 : 1);
    ESP_LOGI(TAG, "Modem 3.8V rail %s", on ? "ON" : "OFF");
}

bool modem_is_powered(void)
{
    return gpio_get_level(MODEM_STATUS_PIN) == 1;
}

/* ============ GNSS 状态 ============ */
int gnss_get_3dfix_level(void)    { return gpio_get_level(GNSS_3DFIX_PIN); }
int gnss_get_geofence_level(void) { return gpio_get_level(GNSS_GEOFENCE_PIN); }
int gnss_get_jam_level(void)      { return gpio_get_level(GNSS_JAMIND_PIN); }

/* ============ LED / 按键 ============ */
void led_set(int led_pin, bool state)
{
    gpio_set_level(led_pin, state ? 1 : 0);
    if (led_pin == LED_ESP_PIN)  { s_led_esp_on = state; }
    if (led_pin == LED_CONN_PIN) { s_led_conn_on = state; }
}

void led_blink(int led_pin, int times, uint32_t on_ms, uint32_t off_ms)
{
    for (int i = 0; i < times; ++i) {
        gpio_set_level(led_pin, 1);
        vTaskDelay(pdMS_TO_TICKS(on_ms));
        gpio_set_level(led_pin, 0);
        vTaskDelay(pdMS_TO_TICKS(off_ms));
    }
    // 恢复闪烁之前的状态，避免把自测结果指示灯覆盖掉
    bool prev = (led_pin == LED_ESP_PIN) ? s_led_esp_on
              : (led_pin == LED_CONN_PIN) ? s_led_conn_on
              : false;
    gpio_set_level(led_pin, prev ? 1 : 0);
}

bool button_is_pressed(void)
{
    return gpio_get_level(BUTTON_PIN) == 0;
}

void led1_on(void)  { led_set(LED_ESP_PIN, true); }
void led1_off(void) { led_set(LED_ESP_PIN, false); }
void led3_on(void)  { led_set(LED_CONN_PIN, true); }
void led3_off(void) { led_set(LED_CONN_PIN, false); }
