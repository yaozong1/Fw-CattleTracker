#include "motion_sensor.h"
#include "hardware_config.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "MOTION_SENSOR";

// I2C配置
#define I2C_MASTER_NUM           I2C_NUM_0
#define I2C_MASTER_FREQ_HZ       100000    // 100kHz
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define I2C_MASTER_TIMEOUT_MS    1000

// LIS2DH12TR默认I2C地址 (根据硬件连接确定)
// 根据日志显示，您的硬件使用0x19地址 (SDO/SA0接VCC)
static uint8_t lis2dh12_addr = LIS2DH12_I2C_ADDR_HIGH;
static lis2dh12_fs_t current_full_scale = LIS2DH12_FS_2G;

// I2C初始化
static esp_err_t i2c_master_init(void)
{
    // 首先尝试配置，如果失败可能是因为驱动已安装
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = MOTION_SENSOR_SDA_PIN,
        .scl_io_num = MOTION_SENSOR_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 50000,    // 降低到50kHz，提高兼容性
        .clk_flags = 0,
    };
    
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(err));
        return err;
    }
    
    // 尝试安装驱动
    err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    if (err == ESP_FAIL) {
        // 可能驱动已安装，尝试删除后重新安装
        ESP_LOGD(TAG, "I2C driver already installed, reinstalling...");
        i2c_driver_delete(I2C_MASTER_NUM);
        
        // 重新配置
        err = i2c_param_config(I2C_MASTER_NUM, &conf);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "I2C param reconfig failed: %s", esp_err_to_name(err));
            return err;
        }
        
        // 重新安装
        err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "I2C driver reinstall failed: %s", esp_err_to_name(err));
            return err;
        }
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(err));
        return err;
    }
    
    ESP_LOGI(TAG, "I2C master initialized on SDA=%d, SCL=%d at 50kHz", MOTION_SENSOR_SDA_PIN, MOTION_SENSOR_SCL_PIN);
    
    // 验证引脚状态
    int sda_level = gpio_get_level(MOTION_SENSOR_SDA_PIN);
    int scl_level = gpio_get_level(MOTION_SENSOR_SCL_PIN);
    ESP_LOGI(TAG, "Pin levels after init: SDA=%d, SCL=%d", sda_level, scl_level);
    
    return ESP_OK;
}

bool lis2dh12_read_register(uint8_t reg_addr, uint8_t* data)
{
    esp_err_t err;
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (lis2dh12_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (lis2dh12_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, data, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    
    err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    
    if (err != ESP_OK) {
        // 降低日志级别 - 地址探测时的失败是正常的
        if (reg_addr == LIS2DH12_WHO_AM_I) {
            ESP_LOGD(TAG, "Read WHO_AM_I from 0x%02X failed: %s", lis2dh12_addr, esp_err_to_name(err));
        } else {
            ESP_LOGE(TAG, "Read register 0x%02X failed: %s (addr=0x%02X)", 
                     reg_addr, esp_err_to_name(err), lis2dh12_addr);
            
            // 提供具体的错误诊断 (仅对非WHO_AM_I寄存器)
            switch (err) {
                case ESP_ERR_TIMEOUT:
                    ESP_LOGE(TAG, "I2C timeout - check if device is connected");
                    break;
                case ESP_FAIL:
                    ESP_LOGE(TAG, "I2C communication failed - check wiring and pullups");
                    break;
                case ESP_ERR_INVALID_STATE:
                    ESP_LOGE(TAG, "I2C driver not installed or configured incorrectly");
                    break;
                default:
                    ESP_LOGE(TAG, "Unknown I2C error");
                    break;
            }
        }
        return false;
    }
    
    return true;
}

bool lis2dh12_write_register(uint8_t reg_addr, uint8_t data)
{
    esp_err_t err;
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (lis2dh12_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    
    err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Write register 0x%02X failed: %s", reg_addr, esp_err_to_name(err));
        return false;
    }
    
    return true;
}

bool lis2dh12_check_device_id(void)
{
    uint8_t who_am_i;
    
    // 优化：先尝试更常见的地址0x19，然后再尝试0x18
    uint8_t addresses[] = {LIS2DH12_I2C_ADDR_HIGH, LIS2DH12_I2C_ADDR_LOW};
    const char* addr_names[] = {"0x19 (SDO=HIGH)", "0x18 (SDO=LOW)"};
    
    for (int i = 0; i < 2; i++) {
        lis2dh12_addr = addresses[i];
        ESP_LOGI(TAG, "Trying address %s...", addr_names[i]);
        
        if (lis2dh12_read_register(LIS2DH12_WHO_AM_I, &who_am_i)) {
            if (who_am_i == LIS2DH12_WHO_AM_I_VALUE) {
                ESP_LOGI(TAG, "✅ LIS2DH12 found at address 0x%02X, WHO_AM_I=0x%02X", lis2dh12_addr, who_am_i);
                return true;
            } else {
                ESP_LOGW(TAG, "Device at 0x%02X responded but WHO_AM_I=0x%02X (expected 0x%02X)", 
                         lis2dh12_addr, who_am_i, LIS2DH12_WHO_AM_I_VALUE);
            }
        } else {
            ESP_LOGD(TAG, "No response from address 0x%02X", lis2dh12_addr);
        }
        
        // 小延迟避免总线冲突
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    
    ESP_LOGE(TAG, "❌ LIS2DH12 not found at any expected address");
    return false;
}

bool lis2dh12_configure(lis2dh12_odr_t odr, lis2dh12_fs_t full_scale)
{
    // 配置CTRL_REG1: 设置数据输出率和使能X、Y、Z轴
    uint8_t ctrl_reg1 = odr | 0x07;  // 0x07 = 使能XYZ轴
    if (!lis2dh12_write_register(LIS2DH12_CTRL_REG1, ctrl_reg1)) {
        ESP_LOGE(TAG, "Failed to configure CTRL_REG1");
        return false;
    }
    
    // 配置CTRL_REG4: 设置满量程和高分辨率模式
    uint8_t ctrl_reg4 = full_scale | 0x08;  // 0x08 = 高分辨率模式
    if (!lis2dh12_write_register(LIS2DH12_CTRL_REG4, ctrl_reg4)) {
        ESP_LOGE(TAG, "Failed to configure CTRL_REG4");
        return false;
    }
    
    current_full_scale = full_scale;
    
    ESP_LOGI(TAG, "LIS2DH12 configured: ODR=0x%02X, FS=0x%02X", odr, full_scale);
    return true;
}

bool lis2dh12_data_ready(void)
{
    uint8_t status;
    if (!lis2dh12_read_register(LIS2DH12_STATUS_REG, &status)) {
        return false;
    }
    
    // 检查ZYXDA位 (bit 3) - 表示新的XYZ数据可用
    return (status & 0x08) != 0;
}

bool lis2dh12_read_raw_data(lis2dh12_raw_data_t* data)
{
    uint8_t raw_data[6];
    
    // 读取6个字节的原始数据 (X_L, X_H, Y_L, Y_H, Z_L, Z_H)
    for (int i = 0; i < 6; i++) {
        if (!lis2dh12_read_register(LIS2DH12_OUT_X_L + i, &raw_data[i])) {
            ESP_LOGE(TAG, "Failed to read acceleration data");
            return false;
        }
    }
    
    // 组合高低字节 (12位数据左对齐在16位中)
    data->x = (int16_t)((raw_data[1] << 8) | raw_data[0]) >> 4;
    data->y = (int16_t)((raw_data[3] << 8) | raw_data[2]) >> 4;
    data->z = (int16_t)((raw_data[5] << 8) | raw_data[4]) >> 4;
    
    return true;
}

bool lis2dh12_read_accel_data(lis2dh12_accel_data_t* data)
{
    lis2dh12_raw_data_t raw_data;
    
    if (!lis2dh12_read_raw_data(&raw_data)) {
        return false;
    }
    
    // 根据满量程设置转换为g值
    float sensitivity;
    switch (current_full_scale) {
        case LIS2DH12_FS_2G:  sensitivity = 1.0f;  break;  // 1 mg/LSB
        case LIS2DH12_FS_4G:  sensitivity = 2.0f;  break;  // 2 mg/LSB
        case LIS2DH12_FS_8G:  sensitivity = 4.0f;  break;  // 4 mg/LSB
        case LIS2DH12_FS_16G: sensitivity = 12.0f; break;  // 12 mg/LSB
        default: sensitivity = 1.0f; break;
    }
    
    // 转换为g值 (除以1000将mg转换为g)
    data->x = (float)raw_data.x * sensitivity / 1000.0f;
    data->y = (float)raw_data.y * sensitivity / 1000.0f;
    data->z = (float)raw_data.z * sensitivity / 1000.0f;
    
    return true;
}

float lis2dh12_calculate_magnitude(const lis2dh12_accel_data_t* data)
{
    return sqrtf(data->x * data->x + data->y * data->y + data->z * data->z);
}

void motion_sensor_test(void)
{
    ESP_LOGI(TAG, "Starting motion sensor test...");
    
    for (int i = 0; i < 10; i++) {
        if (lis2dh12_data_ready()) {
            lis2dh12_accel_data_t accel_data;
            if (lis2dh12_read_accel_data(&accel_data)) {
                float magnitude = lis2dh12_calculate_magnitude(&accel_data);
                ESP_LOGI(TAG, "Accel: X=%.3fg, Y=%.3fg, Z=%.3fg, Mag=%.3fg", 
                         accel_data.x, accel_data.y, accel_data.z, magnitude);
            }
        } else {
            ESP_LOGI(TAG, "No new data available");
        }
        
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    
    ESP_LOGI(TAG, "Motion sensor test completed");
}

// I2C诊断函数
static void i2c_scan_devices(void)
{
    ESP_LOGI(TAG, "Scanning I2C bus for devices...");
    int devices_found = 0;
    
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        
        esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
        i2c_cmd_link_delete(cmd);
        
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Found device at address 0x%02X", addr);
            devices_found++;
        }
        
        // 添加小延迟避免总线过载
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
    
    if (devices_found == 0) {
        ESP_LOGW(TAG, "No I2C devices found! Check wiring and power supply.");
        ESP_LOGW(TAG, "Expected LIS2DH12 at 0x18 or 0x19");
    } else {
        ESP_LOGI(TAG, "I2C scan completed, found %d device(s)", devices_found);
    }
}

bool motion_sensor_init(void)
{
    ESP_LOGI(TAG, "=== Initializing LIS2DH12 motion sensor ===");
    ESP_LOGI(TAG, "Using pins: SDA=%d, SCL=%d", MOTION_SENSOR_SDA_PIN, MOTION_SENSOR_SCL_PIN);
    
    // 检查引脚定义是否正确
    if (MOTION_SENSOR_SDA_PIN < 0 || MOTION_SENSOR_SDA_PIN > 48) {
        ESP_LOGE(TAG, "Invalid SDA pin: %d", MOTION_SENSOR_SDA_PIN);
        return false;
    }
    if (MOTION_SENSOR_SCL_PIN < 0 || MOTION_SENSOR_SCL_PIN > 48) {
        ESP_LOGE(TAG, "Invalid SCL pin: %d", MOTION_SENSOR_SCL_PIN);
        return false;
    }
    
    // 初始化I2C
    if (i2c_master_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C");
        return false;
    }
    
    // 等待传感器稳定
    ESP_LOGI(TAG, "Waiting for sensor stabilization...");
    vTaskDelay(200 / portTICK_PERIOD_MS);
    
    // 扫描I2C设备
    i2c_scan_devices();
    
    // 检查设备ID
    ESP_LOGI(TAG, "Checking device ID...");
    if (!lis2dh12_check_device_id()) {
        ESP_LOGE(TAG, "Failed to detect LIS2DH12");
        ESP_LOGE(TAG, "Troubleshooting steps:");
        ESP_LOGE(TAG, "1. Check I2C wiring (SDA=%d, SCL=%d)", MOTION_SENSOR_SDA_PIN, MOTION_SENSOR_SCL_PIN);
        ESP_LOGE(TAG, "2. Check power supply (3.3V)");
        ESP_LOGE(TAG, "3. Check if device address is correct");
        ESP_LOGE(TAG, "4. Check for I2C pullup resistors");
        ESP_LOGE(TAG, "5. Verify sensor is LIS2DH12TR (not similar model)");
        return false;
    }
    
    // 配置传感器: 100Hz输出率, ±2g满量程
    if (!lis2dh12_configure(LIS2DH12_ODR_100HZ, LIS2DH12_FS_2G)) {
        ESP_LOGE(TAG, "Failed to configure LIS2DH12");
        return false;
    }
    
    // 等待配置生效
    vTaskDelay(10 / portTICK_PERIOD_MS);
    
    ESP_LOGI(TAG, "LIS2DH12 motion sensor initialized successfully");
    return true;
}

void motion_sensor_task(void* pvParameters)
{
    ESP_LOGI(TAG, "Motion sensor task started");
    
    // 等待传感器稳定
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    while (1) {
        if (lis2dh12_data_ready()) {
            lis2dh12_accel_data_t accel_data;
            if (lis2dh12_read_accel_data(&accel_data)) {
                float magnitude = lis2dh12_calculate_magnitude(&accel_data);
                
                // 检测明显的运动 (阈值可调整)
                static float last_magnitude = 1.0f; // 地球重力约1g
                float motion_threshold = 0.1f; // 100mg阈值
                
                if (fabs(magnitude - last_magnitude) > motion_threshold) {
                    ESP_LOGI(TAG, "Motion detected! X=%.3fg, Y=%.3fg, Z=%.3fg, Mag=%.3fg", 
                             accel_data.x, accel_data.y, accel_data.z, magnitude);
                }
                
                last_magnitude = magnitude;
            }
        }
        
        // 100ms采样间隔
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void motion_sensor_debug_test(void)
{
    ESP_LOGI(TAG, "=== Motion Sensor Debug Test ===");
    
    // 1. 检查引脚定义
    ESP_LOGI(TAG, "Pin configuration:");
    ESP_LOGI(TAG, "  SDA: GPIO%d", MOTION_SENSOR_SDA_PIN);
    ESP_LOGI(TAG, "  SCL: GPIO%d", MOTION_SENSOR_SCL_PIN);
    
    // 2. 重置引脚
    ESP_LOGI(TAG, "Resetting GPIO pins...");
    gpio_reset_pin(MOTION_SENSOR_SDA_PIN);
    gpio_reset_pin(MOTION_SENSOR_SCL_PIN);
    
    // 3. 手动配置引脚为输入并检查电平
    gpio_set_direction(MOTION_SENSOR_SDA_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(MOTION_SENSOR_SCL_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(MOTION_SENSOR_SDA_PIN, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(MOTION_SENSOR_SCL_PIN, GPIO_PULLUP_ONLY);
    
    vTaskDelay(10 / portTICK_PERIOD_MS);
    
    int sda_level = gpio_get_level(MOTION_SENSOR_SDA_PIN);
    int scl_level = gpio_get_level(MOTION_SENSOR_SCL_PIN);
    ESP_LOGI(TAG, "Initial pin levels (with pullups): SDA=%d, SCL=%d", sda_level, scl_level);
    
    if (sda_level == 0 || scl_level == 0) {
        ESP_LOGW(TAG, "Warning: One or both pins are pulled low - check for shorts");
    }
    
    // 4. 尝试非常低速的I2C初始化
    ESP_LOGI(TAG, "Trying very low speed I2C (10kHz)...");
    
    // 安全地删除之前的实例
    esp_err_t delete_result = i2c_driver_delete(I2C_MASTER_NUM);
    if (delete_result != ESP_OK && delete_result != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "I2C driver delete warning: %s", esp_err_to_name(delete_result));
    }
    
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = MOTION_SENSOR_SDA_PIN,
        .scl_io_num = MOTION_SENSOR_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 10000,  // 10kHz - 极低速度
        .clk_flags = 0,
    };
    
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Debug I2C config failed: %s", esp_err_to_name(err));
        return;
    }
    
    err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Debug I2C install failed: %s", esp_err_to_name(err));
        return;
    }
    
    ESP_LOGI(TAG, "Low speed I2C initialized successfully");
    
    // 5. 尝试简单的地址扫描
    ESP_LOGI(TAG, "Scanning for LIS2DH12 addresses only...");
    
    uint8_t test_addresses[] = {0x18, 0x19};
    for (int i = 0; i < 2; i++) {
        uint8_t addr = test_addresses[i];
        
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        
        err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
        i2c_cmd_link_delete(cmd);
        
        ESP_LOGI(TAG, "Address 0x%02X: %s", addr, 
                 (err == ESP_OK) ? "RESPONDED" : esp_err_to_name(err));
        
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    
    ESP_LOGI(TAG, "=== Debug test completed ===");
}