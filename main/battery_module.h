#ifndef BATTERY_MODULE_H
#define BATTERY_MODULE_H

#include <stdint.h>
#include <stdbool.h>

// ADC和电池相关函数
void battery_init(void);
float read_battery_voltage(void);
int estimate_soc_from_voltage(float voltage);
uint32_t read_adc_raw(void);
uint32_t read_adc_voltage_mv(void);

// IBL (IO1) —— Cattle_T 上电池分压与 IBL 共用同一路 ADC
uint32_t read_ibl_raw(void);
uint32_t read_ibl_voltage_mv(void);

// 兼容 pe_board 时期的命名（原先 IBL 在 IO2）
#define read_io2_raw        read_ibl_raw
#define read_io2_voltage_mv read_ibl_voltage_mv

#endif // BATTERY_MODULE_H
