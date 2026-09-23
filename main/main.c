#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include <string.h>

// 模块头文件
#include "hardware_config.h"
#include "gpio_module.h"
#include "gnss_module.h"
#include "mqtt_module.h"
#include "battery_module.h"
#include "timer_module.h"
#include "motion_sensor.h"
#include "button_module.h"
#include "eg915_selftest.h"

static const char *TAG = "APP";

void app_main(void)
{
    ESP_LOGI(TAG, "Cattle Tracker Application Starting...");

#if SELFTEST
    // 自测模式：仅初始化GPIO，然后创建自测任务
    gpio_init();

    xTaskCreate(Selftest_task, "selftest_task", 4096, NULL, 10, NULL);
    ESP_LOGI(TAG, "Self-test task created: selftest_task (SELFTEST=1)");

    // IO0 双击 -> 打开围栏供电轨（自测模式下也可用）
    button_module_start();
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
#else

    // 1. 初始化GPIO（含电子围栏引脚安全状态）
    gpio_init();

    // 1.1 IO0 按键手势：双击打开围栏供电轨，长按关回安全状态
    button_module_start();

    // 2. 初始化GNSS模块
    gnss_init();

    // 3. 初始化MQTT模块（EG800 UART1）
    mqtt_init();

    // 4. 初始化电池监控模块
    battery_init();

    // 5. 初始化定时器
    timer_init();
    timer_start();

    // 6. 初始化运动传感器
    if (motion_sensor_init()) {
        ESP_LOGI(TAG, "Motion sensor initialized successfully!");
        xTaskCreate(motion_sensor_task, "motion_task", 4096, NULL, 5, NULL);
    } else {
        ESP_LOGE(TAG, "Motion sensor initialization failed - continuing without it");
    }

    // 7. 启动GNSS任务
    xTaskCreate(gnss_task, "gnss_task", 4096, NULL, 8, NULL);

    // 8. 启动GPS+MQTT周期任务
    xTaskCreate(gps_mqtt_cycle_task, "gps_cycle", 4096, NULL, 9, NULL);

    // 等待系统稳定
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    ESP_LOGI(TAG, "All modules initialized, entering main loop");

    // 主循环
    while (1) {
        // 读取 ADC 值进行电池监控
        uint32_t adc_reading = read_adc_raw();
        uint32_t voltage_mv = read_adc_voltage_mv();

        static uint32_t last_battery_log = 0;
        uint32_t now = xTaskGetTickCount();
        if ((now - last_battery_log) > pdMS_TO_TICKS(10000)) { // 每10秒打印一次
            ESP_LOGI(TAG, "Battery: ADC=%lu, ADC_mV=%lu, VBAT=%.2fV",
                     (unsigned long)adc_reading, (unsigned long)voltage_mv,
                     read_battery_voltage());
            last_battery_log = now;
        }

        // 处理按键触发的事件：模组未上电时重新开机
        if (log_output) {
            if (!modem_is_powered()) {
                ESP_LOGI("MODEM", "Power cycling modem via button press");
                modem_power_cycle();
            }
            vTaskDelay(1000 / portTICK_PERIOD_MS); // 等待模组启动
            log_output = false;
        }

        // 处理AT命令标志
        if (send_at_flag) {
            send_at_flag = false;
            send_at_command("AT");
        }

        // 主循环延时
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
#endif // SELFTEST
}
