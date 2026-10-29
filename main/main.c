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
#include "esp_sleep.h"
#include "driver/gpio.h"
#include "eg915_selftest.h"
#include "tracking_policy.h"
#include "fence_bench.h"

static const char *TAG = "APP";

#if ENABLE_DEEP_SLEEP
/* RTC 慢速内存里的计数器，deep sleep 不会清零。
 * 串口在深睡时会断开枚举，重连后看这个数就知道中间醒过几轮。 */
RTC_DATA_ATTR static uint32_t s_boot_count = 0;
RTC_DATA_ATTR static uint32_t s_fix_ok_count = 0;
RTC_DATA_ATTR static uint32_t s_pub_ok_count = 0;

uint32_t app_get_boot_count(void)   { return s_boot_count; }
void app_note_fix_ok(void)          { s_fix_ok_count++; }
void app_note_publish_ok(void)      { s_pub_ok_count++; }
#endif


void app_main(void)
{
    ESP_LOGI(TAG, "Cattle Tracker Application Starting...");

#if FENCE_BENCH
    /* 围栏台架测试固件：只起 GPIO（含引脚安全态）+ 电池 ADC + 控制台。
     * 刻意不启动 GNSS / MQTT / 运动传感器 / gps_mqtt_cycle_task ——
     * 后者会在 30 秒后进深睡，测到一半板子就睡了。 */
    gpio_init();
    battery_init();
    fence_bench_start();

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
#elif SELFTEST
    // 自测模式：仅初始化GPIO，然后创建自测任务
    gpio_init();

    xTaskCreate(Selftest_task, "selftest_task", 4096, NULL, 10, NULL);
    ESP_LOGI(TAG, "Self-test task created: selftest_task (SELFTEST=1)");

#if BUTTON_DBLCLICK_MQTT_PUBLISH
    /* 双击要发 MQTT，得先有 UART1 和模组总线锁。
     * mqtt_init() 是幂等的，自测任务里再调一次无妨。 */
    mqtt_init();
    ESP_LOGW(TAG, "自测模式：双击 IO0 = 联网发送 MQTT（不是围栏测试）");
#endif

    // IO0 双击 -> 按 BUTTON_DBLCLICK_* 决定动作
    button_module_start();
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
#else

#if ENABLE_DEEP_SLEEP
    /* 先报告本次是怎么起来的。deep sleep 醒来等同复位，所以每轮都会走一遍
     * app_main，RAM 状态全部重置——IMEI、APN 这些每轮重新取。 */
    {
        esp_sleep_wakeup_cause_t wc = esp_sleep_get_wakeup_cause();
        s_boot_count++;

        ESP_LOGI(TAG, "==================================================");
        ESP_LOGI(TAG, " 第 %lu 次启动（RTC 计数器，deep sleep 不清零）",
                 (unsigned long)s_boot_count);
        switch (wc) {
            case ESP_SLEEP_WAKEUP_TIMER:
                ESP_LOGI(TAG, " 唤醒原因：deep sleep 定时器到期（正常上报轮次）");
                break;
            case ESP_SLEEP_WAKEUP_EXT0:
                ESP_LOGI(TAG, " 唤醒原因：IO%d 按键（手动触发）", BUTTON_PIN);
                break;
            default:
                ESP_LOGI(TAG, " 唤醒原因：上电 / 复位（首次启动，计数器已归零）");
                s_fix_ok_count = 0;
                s_pub_ok_count = 0;
                break;
        }
        ESP_LOGI(TAG, " 历史统计：定位成功 %lu 次，上报成功 %lu 次",
                 (unsigned long)s_fix_ok_count, (unsigned long)s_pub_ok_count);
#if ENABLE_TRACKING_POLICY
        ESP_LOGI(TAG, " 频率策略：已跑 %lu 轮，成功上报 %lu 次",
                 (unsigned long)track_cycles(), (unsigned long)track_publishes());
#endif
        ESP_LOGI(TAG, "==================================================");

        // 解除 deep sleep 期间的引脚保持，否则后面 gpio_set_level 写不进去
        deep_sleep_release_pins();

        /* 开机指示：ESP_LED 按启动次数闪（最多 5 下）。
         * 串口在深睡时会断开，这个闪灯是不依赖串口就能确认"确实醒了"的证据。 */
        gpio_reset_pin(LED_ESP_PIN);
        gpio_set_direction(LED_ESP_PIN, GPIO_MODE_OUTPUT);
        int blinks = (int)(s_boot_count > 5 ? 5 : s_boot_count);
        for (int i = 0; i < blinks; ++i) {
            gpio_set_level(LED_ESP_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(80));
            gpio_set_level(LED_ESP_PIN, 0);
            vTaskDelay(pdMS_TO_TICKS(120));
        }
    }
#endif

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
#if ENABLE_TRACKING_POLICY
        /* 启用运动驱动频率时不起这个轮询任务：它会和每轮唤醒的 5 秒采样
         * 抢同一条 I2C 总线，而且采样窗口本身已经覆盖了它的功能。 */
        ESP_LOGI(TAG, "运动驱动频率已启用，不启动 motion_sensor_task（避免 I2C 争用）");
#else
        xTaskCreate(motion_sensor_task, "motion_task", 4096, NULL, 5, NULL);
#endif
    } else {
        ESP_LOGE(TAG, "Motion sensor initialization failed - continuing without it");
    }

    // 7. 启动GNSS任务
    // 栈 6144：任务内局部缓冲约 800B，加上 ESP_LOGI 的 vprintf 路径，4096 偏紧
    xTaskCreate(gnss_task, "gnss_task", 6144, NULL, 8, NULL);

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
