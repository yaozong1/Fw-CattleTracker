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


/* ---- 中断 / 低功耗相关寄存器（阶段一：睡眠布防用）---- */
#define LIS2DH12_REFERENCE         0x26
#define LIS2DH12_INT1_CFG          0x30
#define LIS2DH12_INT1_SRC          0x31
#define LIS2DH12_INT1_THS          0x32
#define LIS2DH12_INT1_DURATION     0x33
#define LIS2DH12_ACT_THS           0x3E
#define LIS2DH12_ACT_DUR           0x3F

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

/* 进 deep sleep 之前必须调用。
 * LIS2DH12 挂在 3V3_ALWAYS_ON 上，这条轨永远在线，MCU 睡了它还在采样。
 * 高分辨率模式 Idd 典型 185 µA，折到电池侧约 360 µA —— 静态电流的大头。
 * 写 CTRL_REG1 = 0x00 进 power-down 之后降到 0.5 µA。
 * 同时停掉 motion_sensor_task 的轮询，避免它在断电后继续读寄存器。 */
bool motion_sensor_power_down(void);


/* ================= 阶段一：锁存 bit + 5 秒采样 =================
 * 深睡期间 LIS2DH12 以 10 Hz 低功耗模式运行（datasheet Table 12: 3 µA），
 * 中断锁存（CTRL_REG5.LIR_INT1）。ESP32 醒来先读 INT1_SRC 拿到一个 bit：
 * "整个睡眠窗口内动过没有"。动过才值得再花 5 秒采样数次数。
 * 用不到 INT1 引脚（IO17）—— 锁存位在寄存器里，I2C 读就行。 */

/* 进深睡前调用：切到 10 Hz 低功耗 + 高通滤波 + INT1 阈值中断 + 锁存，
 * 并清掉配置过程中产生的旧锁存，让下一个睡眠窗口从干净状态开始。
 * 取代原来的 motion_sensor_power_down()。 */
bool motion_arm_for_sleep(void);

/* 醒来第一件事调用：读 INT1_SRC（读操作本身会清除锁存）。
 * 返回 true = 上一个睡眠窗口内发生过超阈值事件。
 * I2C 读失败时返回 true —— 往保守方向判，宁可多采一次样。 */
bool motion_read_and_clear_latch(void);

/* 醒着时调用：采样 window_ms，返回【活动占比】0~100（%）——
 * 窗口内 |‖a‖ − 1g| 超过 MOTION_EVENT_TH_MG 的样本比例。
 * 这是「有动作」的主判据：持续运动占比高，瞬态动作占比低。
 * edges_out / peak_mg_out 是诊断用的附加信息，可传 NULL。
 * I2C 失败返回 -1。
 * 注意它会改写 CTRL_REG1/4，调用顺序必须是：
 *   read_and_clear_latch -> activity_mg -> ...业务... -> arm_for_sleep -> 深睡 */
int motion_activity_mg(uint32_t window_ms, int *pct_out, int *peak_mg_out);

// 实用函数
float lis2dh12_calculate_magnitude(const lis2dh12_accel_data_t* data);
void motion_sensor_test(void);
void motion_sensor_task(void* pvParameters);
void motion_sensor_debug_test(void);  // 新增调试函数

#endif // MOTION_SENSOR_H