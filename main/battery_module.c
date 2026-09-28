#include "battery_module.h"
#include "hardware_config.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_log.h"
#include <stdlib.h>

static const char *TAG = "BATTERY";

// ADC校准特性
esp_adc_cal_characteristics_t *adc_chars = NULL;

void battery_init(void)
{
    // Cattle_T：只有一路电池采样 IO1 (IBL, ADC1_CH0)
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL, ADC_ATTEN_DB_11);

    if (adc_chars == NULL) {
        adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    }
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, adc_chars);

    ESP_LOGI(TAG, "ADC initialized on IO%d (ADC1_CH0) for battery/IBL", IBL_PIN);
}

uint32_t read_adc_raw(void)
{
    uint32_t adc_reading = 0;
    for (int i = 0; i < NO_OF_SAMPLES; i++) {
        adc_reading += adc1_get_raw(ADC1_CHANNEL);
    }
    adc_reading /= NO_OF_SAMPLES;
    return adc_reading;
}

uint32_t read_adc_voltage_mv(void)
{
    uint32_t adc_reading = read_adc_raw();
    return esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
}

// ----- IBL helpers（与电池采样同一通道） -----
uint32_t read_ibl_raw(void)
{
    return read_adc_raw();
}

uint32_t read_ibl_voltage_mv(void)
{
    return read_adc_voltage_mv();
}

float read_battery_voltage(void)
{
    uint32_t mv = read_adc_voltage_mv();
    return (float)mv / 1000.0f * VBAT_DIVIDER_RATIO;
}

/* 单节锂电池 OCV → SOC 查表（静置电压，室温）。
 * 锂电放电平台很平，直接线性插值会在 3.7~3.9V 这段严重失真，
 * 所以用分段表 + 线性插值。端点跟 hardware_config.h 的 VBAT_FULL_V /
 * VBAT_EMPTY_V 对齐；换电芯的话改这张表即可。
 * 注意：表针对空载电压，带载时电池会被拉低，读数应在模组上电前采样。 */
typedef struct {
    float v;
    int   soc;
} ocv_point_t;

static const ocv_point_t s_ocv_table[] = {
    { VBAT_FULL_V, 100 },   // 4.15
    { 4.08f,        90 },
    { 4.00f,        80 },
    { 3.93f,        70 },
    { 3.87f,        60 },
    { 3.82f,        50 },
    { 3.79f,        40 },
    { 3.75f,        30 },
    { 3.70f,        20 },
    { 3.60f,        10 },
    { 3.45f,         5 },
    { VBAT_EMPTY_V,  0 },   // 3.30
};

#define OCV_TABLE_LEN ((int)(sizeof(s_ocv_table) / sizeof(s_ocv_table[0])))

int estimate_soc_from_voltage(float v)
{
    if (v >= s_ocv_table[0].v) { return 100; }
    if (v <= s_ocv_table[OCV_TABLE_LEN - 1].v) { return 0; }

    for (int i = 1; i < OCV_TABLE_LEN; ++i) {
        if (v >= s_ocv_table[i].v) {
            const ocv_point_t *hi = &s_ocv_table[i - 1];
            const ocv_point_t *lo = &s_ocv_table[i];
            float span = hi->v - lo->v;
            if (span <= 0.0f) { return lo->soc; }
            float k = (v - lo->v) / span;
            float soc = (float)lo->soc + k * (float)(hi->soc - lo->soc);
            if (soc < 0.0f)   { soc = 0.0f; }
            if (soc > 100.0f) { soc = 100.0f; }
            return (int)(soc + 0.5f);
        }
    }
    return 0;
}
