#ifndef MOTION_SENSOR_H
#define MOTION_SENSOR_H

#include <stdint.h>
#include <stdbool.h>

// LIS2DH12TR寄存器地址
#define LIS2DH12_WHO_AM_I          0x0F
#define LIS2DH12_CTRL_REG1         0x20
#define LIS2DH12_CTRL_REG2         0x21
#define LIS2DH12_CTRL_REG3         0x22
#define LIS2DH12_CTRL_REG4         0x23
#define LIS2DH12_CTRL_REG5         0x24
#define LIS2DH12_CTRL_REG6         0x25
#define LIS2DH12_STATUS_REG        0x27
#define LIS2DH12_OUT_X_L           0x28
#define LIS2DH12_OUT_X_H           0x29
#define LIS2DH12_OUT_Y_L           0x2A
#define LIS2DH12_OUT_Y_H           0x2B
#define LIS2DH12_OUT_Z_L           0x2C
#define LIS2DH12_OUT_Z_H           0x2D

// LIS2DH12TR期望的WHO_AM_I值
#define LIS2DH12_WHO_AM_I_VALUE    0x33

// I2C地址 (SDO/SA0 pin状态决定)
#define LIS2DH12_I2C_ADDR_LOW      0x18  // SDO/SA0 = 0
#define LIS2DH12_I2C_ADDR_HIGH     0x19  // SDO/SA0 = 1

// 数据输出率配置
typedef enum {
    LIS2DH12_ODR_POWER_DOWN = 0x00,
    LIS2DH12_ODR_1HZ        = 0x10,
    LIS2DH12_ODR_10HZ       = 0x20,
    LIS2DH12_ODR_25HZ       = 0x30,
    LIS2DH12_ODR_50HZ       = 0x40,
    LIS2DH12_ODR_100HZ      = 0x50,
    LIS2DH12_ODR_200HZ      = 0x60,
    LIS2DH12_ODR_400HZ      = 0x70
} lis2dh12_odr_t;

// 满量程配置
typedef enum {
    LIS2DH12_FS_2G  = 0x00,
    LIS2DH12_FS_4G  = 0x10,
    LIS2DH12_FS_8G  = 0x20,
    LIS2DH12_FS_16G = 0x30
} lis2dh12_fs_t;

// 加速度数据结构
typedef struct {
    int16_t x;  // X轴原始数据
    int16_t y;  // Y轴原始数据
    int16_t z;  // Z轴原始数据
} lis2dh12_raw_data_t;

typedef struct {
    float x;    // X轴加速度 (g)
    float y;    // Y轴加速度 (g)
    float z;    // Z轴加速度 (g)
} lis2dh12_accel_data_t;

// 初始化和配置函数
bool motion_sensor_init(void);
bool lis2dh12_check_device_id(void);
bool lis2dh12_configure(lis2dh12_odr_t odr, lis2dh12_fs_t full_scale);

// 数据读取函数
bool lis2dh12_read_raw_data(lis2dh12_raw_data_t* data);
bool lis2dh12_read_accel_data(lis2dh12_accel_data_t* data);
bool lis2dh12_data_ready(void);

// 寄存器读写函数
bool lis2dh12_read_register(uint8_t reg_addr, uint8_t* data);
bool lis2dh12_write_register(uint8_t reg_addr, uint8_t data);

// 实用函数
float lis2dh12_calculate_magnitude(const lis2dh12_accel_data_t* data);
void motion_sensor_test(void);
void motion_sensor_task(void* pvParameters);
void motion_sensor_debug_test(void);  // 新增调试函数

#endif // MOTION_SENSOR_H