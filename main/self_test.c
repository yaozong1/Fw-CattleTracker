#include "self_test.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include <string.h>

#include "hardware_config.h"
#include "gpio_module.h"
#include "motion_sensor.h"
#include "mqtt_module.h" // EG800 相关接口（send_at/等待OK/电源控制）
#include "battery_module.h"

/* =====================================================================
 * 简易台架自检（循环跑，方便实验室观察）
 * 工厂治具用的是 eg915_selftest.c 里的 Selftest_task。
 * Cattle_T 上已删除 CAN / RS485 测试项。
 * ===================================================================== */

static const char *TAG = "SELFTEST";

typedef struct {
    bool motion_ok;
    bool modem_ok;
    bool battery_ok;
    bool gnss_ok;
} selftest_result_t;

static void log_summary(const selftest_result_t *r)
{
    ESP_LOGI(TAG, "================ Self-Test Summary ================");
    ESP_LOGI(TAG, "Motion Sensor:  %s", r->motion_ok ? "PASS" : "FAIL");
    ESP_LOGI(TAG, "EG800 (AT):     %s", r->modem_ok ? "PASS" : "FAIL");
    ESP_LOGI(TAG, "GNSS UART:      %s", r->gnss_ok ? "PASS" : "FAIL");
    ESP_LOGI(TAG, "Battery ADC:    %s", r->battery_ok ? "PASS" : "FAIL");
    ESP_LOGI(TAG, "===================================================");
}

static bool test_motion(void)
{
    ESP_LOGI(TAG, "[MOTION] init...");
    if (!motion_sensor_init()) {
        ESP_LOGE(TAG, "[MOTION] init failed");
        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(100));
    lis2dh12_accel_data_t a;
    if (lis2dh12_read_accel_data(&a)) {
        ESP_LOGI(TAG, "[MOTION] Accel: X=%.3fg Y=%.3fg Z=%.3fg", a.x, a.y, a.z);
        return true;
    }
    ESP_LOGE(TAG, "[MOTION] read accel failed");
    return false;
}

static bool test_modem(void)
{
    ESP_LOGI(TAG, "[EG800] power cycle + AT...");
    modem_power_enable(true);
    vTaskDelay(pdMS_TO_TICKS(200));
    modem_power_cycle();
    // 等待模组上电完成并输出 RDY 等 URC
    vTaskDelay(pdMS_TO_TICKS(5000));

    send_at_command("AT");
    bool ok = wait_for_ok(2000);
    ESP_LOGI(TAG, "[EG800] AT result: %s (STATUS pin=%d)", ok ? "PASS" : "FAIL", modem_is_powered());
    return ok;
}

static bool test_battery(void)
{
    ESP_LOGI(TAG, "[BAT] init + read ADC...");
    battery_init();
    vTaskDelay(pdMS_TO_TICKS(10));
    uint32_t raw = read_adc_raw();
    uint32_t mv  = read_adc_voltage_mv();
    float vbat   = read_battery_voltage();
    ESP_LOGI(TAG, "[BAT] ADC=%lu, %lu mV, VBAT=%.2fV, SOC=%d%%",
             (unsigned long)raw, (unsigned long)mv, vbat, estimate_soc_from_voltage(vbat));
    return (raw > 0);
}

static bool test_gnss(void)
{
    ESP_LOGI(TAG, "[GNSS] enable + check 3D_FIX/JAM pins...");
    gpio_set_level(GNSS_EN_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI(TAG, "[GNSS] 3D_FIX=%d GEOFENCE=%d JAM=%d",
             gnss_get_3dfix_level(), gnss_get_geofence_level(), gnss_get_jam_level());
    return true;
}

void self_test_run(void)
{
    ESP_LOGI(TAG, "===== Cattle Tracker Self-Test Mode =====");
    gpio_init();

    // 打开基本电源通道（并确保围栏引脚安全）
    power_control_init();

    // 初始化 EG800 所在的 UART1（用于AT通信）
    mqtt_init();

    while (1) {
        selftest_result_t r = {0};

        r.motion_ok  = test_motion();
        r.modem_ok   = test_modem();
        r.gnss_ok    = test_gnss();
        r.battery_ok = test_battery();

        log_summary(&r);

        // 指示灯：ESP_LED 亮表示全部通过，CONN_LED 亮表示存在失败
        bool all_pass = r.motion_ok && r.modem_ok && r.gnss_ok && r.battery_ok;
        led1_off();
        led3_off();
        if (all_pass) {
            led1_on();
        } else {
            led3_on();
        }

        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
