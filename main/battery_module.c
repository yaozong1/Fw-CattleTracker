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

int estimate_soc_from_voltage(float v)
{
    // 单节锂电池（CN3163 充电），近似线性估算
    if (v >= VBAT_FULL_V)  { return 100; }
    if (v <= VBAT_EMPTY_V) { return 0; }
    float soc = (v - VBAT_EMPTY_V) / (VBAT_FULL_V - VBAT_EMPTY_V) * 100.0f;
    if (soc < 0)   { soc = 0; }
    if (soc > 100) { soc = 100; }
    return (int)(soc + 0.5f);
}
