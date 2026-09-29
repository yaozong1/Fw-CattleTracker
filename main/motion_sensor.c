#include "motion_sensor.h"
#include "hardware_config.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include "esp_sleep.h"

static const char *TAG = "MOTION_SENSOR";

/* 深睡唤醒等同复位，motion_sensor_init() 会重新跑一遍，而它里面的
 * lis2dh12_configure(100Hz, HR) 会把 CTRL_REG1/4 改掉 —— ODR 变 10 倍、
 * 工作模式从 LP 切到 HR，高通滤波器输出上就是一个大台阶，当场把中断锁存了。
 * 所以锁存位必须在任何重新配置【之前】读走，读到的值缓存在这里。 */
static uint8_t s_latch_src    = 0;
static bool    s_latch_cached = false;

/* 把 INT1_SRC 的位解出来，方便看是哪个轴哪个方向触发的 */
static void log_int1_src(const char *tag, uint8_t src)
{
    ESP_LOGI(TAG, "%s INT1_SRC=0x%02X [IA=%d ZH=%d ZL=%d YH=%d YL=%d XH=%d XL=%d]",
             tag, src,
             !!(src & 0x40), !!(src & 0x20), !!(src & 0x10),
             !!(src & 0x08), !!(src & 0x04), !!(src & 0x02), !!(src & 0x01));
}


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
    uint8_t ctrl_reg4 = full_scale | 0x88;  // 0x80 = BDU(高低字节同一采样), 0x08 = 高分辨率模式
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

/* 连读多个寄存器：子地址置 bit7 打开自动递增，一次 I2C 事务取完。
 * 六个输出寄存器必须在同一次事务里读完 —— 分成 6 次事务的话，
 * 50 kHz 下要 ~5 ms，这期间 100 Hz 的 ODR 会更新半次，
 * X/Y/Z 来自不同时刻，算出来的矢量差既失真又偏小。 */
static bool lis2dh12_read_burst(uint8_t reg_addr, uint8_t *buf, size_t len)
{
    if (len == 0) return false;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (lis2dh12_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr | 0x80, true);      /* bit7 = 自动递增 */
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (lis2dh12_addr << 1) | I2C_MASTER_READ, true);
    if (len > 1) i2c_master_read(cmd, buf, len - 1, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, buf + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return err == ESP_OK;
}

bool lis2dh12_read_raw_data(lis2dh12_raw_data_t* data)
{
    uint8_t raw_data[6];
    
    // 一次事务读完 6 个字节 (X_L, X_H, Y_L, Y_H, Z_L, Z_H)，保证三轴同时刻
    if (!lis2dh12_read_burst(LIS2DH12_OUT_X_L, raw_data, 6)) {
        ESP_LOGE(TAG, "Failed to read acceleration data");
        return false;
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
    
    /* 深睡唤醒时加速度计从未掉电，不需要等稳定，也不需要扫总线 ——
     * 这两项加起来每轮约 630 ms，是静止轮次里最容易省掉的一段。
     * 上电/复位启动时照常执行，调试手段不丢。 */
    bool cold = (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_UNDEFINED);

    if (cold) {
        ESP_LOGI(TAG, "Waiting for sensor stabilization...");
        vTaskDelay(MOTION_INIT_SETTLE_MS / portTICK_PERIOD_MS);
    }

#if MOTION_I2C_SCAN_ON_WAKE
    i2c_scan_devices();
#else
    if (cold) { i2c_scan_devices(); }
#endif
    
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
    
    /* 先把上一个睡眠窗口的锁存位读走，再动任何配置。
     * 顺序颠倒的话，下面 configure() 造成的滤波器瞬态会把锁存重新置上，
     * 读到的就永远是「动过」。 */
    if (lis2dh12_read_register(LIS2DH12_INT1_SRC, &s_latch_src)) {
        s_latch_cached = true;
        log_int1_src("开机先读锁存（重配置之前）", s_latch_src);
    } else {
        s_latch_cached = false;
        ESP_LOGW(TAG, "开机读 INT1_SRC 失败");
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

/* 置位后 motion_sensor_task 自行退出，不再碰 I2C。
 * 深睡唤醒等同复位，任务会重新创建，所以不需要再清回去。 */
static volatile bool s_motion_stop = false;

bool motion_sensor_power_down(void)
{
    s_motion_stop = true;
    // 给轮询任务一个周期退出，避免和这里的写操作撞在同一条 I2C 总线上
    vTaskDelay(pdMS_TO_TICKS(150));

    // CTRL_REG1 = 0x00：ODR = power-down，XYZ 全关
    bool ok = lis2dh12_write_register(LIS2DH12_CTRL_REG1, LIS2DH12_ODR_POWER_DOWN);
    if (ok) {
        ESP_LOGI(TAG, "LIS2DH12 -> power-down (CTRL_REG1=0x00)，Idd 185 µA -> 0.5 µA");
    } else {
        ESP_LOGW(TAG, "LIS2DH12 power-down 写寄存器失败，深睡期间它仍在采样");
    }
    return ok;
}


/* ================= 阶段一：锁存 bit + 5 秒采样 ================= */

bool motion_arm_for_sleep(void)
{
    s_motion_stop = true;
#if !ENABLE_TRACKING_POLICY
    /* 只有 motion_sensor_task 在跑时才需要等它退出让出 I2C；
     * 启用运动策略时那个任务根本没创建，这 150 ms 是白等的。 */
    vTaskDelay(pdMS_TO_TICKS(150));
#endif

    uint8_t dummy, rb;

    /* 1) 先把中断生成器关掉再改配置，免得改的过程中乱触发 */
    lis2dh12_write_register(LIS2DH12_INT1_CFG, 0x00);

    /* 2) CTRL_REG1 = 10 Hz + LPen(bit3) + XYZ。datasheet Table 12: 3 µA */
    if (!lis2dh12_write_register(LIS2DH12_CTRL_REG1, LIS2DH12_ODR_10HZ | 0x08 | 0x07)) return false;

    /* 3) CTRL_REG2：HPM=10（Normal mode，自动跟踪去直流）+ bit0 HP_IA1。
     *    原来用 HPM=00（靠读 REFERENCE 手动复位），对时序很敏感 ——
     *    10 Hz 下一个采样周期是 100 ms，写完 CTRL_REG1 立刻读 REFERENCE
     *    会把还没更新的无效值当成基准，滤波器减不掉重力，
     *    结果静止时 ZH 一直置位。改成自动跟踪就不依赖这个时序了。 */
    if (!lis2dh12_write_register(LIS2DH12_CTRL_REG2, 0x80 | 0x01)) return false;

    /* 4) CTRL_REG3 bit6 = I1_IA1：输出到 INT1 引脚（IO17），只作冗余 */
    if (!lis2dh12_write_register(LIS2DH12_CTRL_REG3, 0x40)) return false;

    /* 5) CTRL_REG4 = ±2g，低功耗模式下 HR 位必须为 0 */
    if (!lis2dh12_write_register(LIS2DH12_CTRL_REG4, LIS2DH12_FS_2G)) return false;

    /* 6) CTRL_REG5 bit3 = LIR_INT1：中断锁存，直到读 INT1_SRC 才清 */
    if (!lis2dh12_write_register(LIS2DH12_CTRL_REG5, 0x08)) return false;

    /* 7) 阈值。±2g 量程下 INT1_THS 是 16 mg/LSB */
    uint8_t ths = (uint8_t)(MOTION_LATCH_THS_MG / 16);
    if (ths == 0)  ths = 1;
    if (ths > 127) ths = 127;
    if (!lis2dh12_write_register(LIS2DH12_INT1_THS, ths)) return false;

    /* 8) 持续时间。1 LSB = 1/ODR = 100 ms @10Hz */
    uint8_t dur = (uint8_t)(MOTION_LATCH_DUR_MS / 100);
    if (dur > 127) dur = 127;
    if (!lis2dh12_write_register(LIS2DH12_INT1_DURATION, dur)) return false;

    /* 9) 关键：等模式切换 + 高通滤波器稳定。
     *    10 Hz 下一个样本 100 ms，HPCF=00 的截止频率约 ODR/50，
     *    给 5 个采样周期（500 ms）让滤波器把重力收敛掉。 */
    vTaskDelay(pdMS_TO_TICKS(MOTION_ARM_SETTLE_MS));
    lis2dh12_read_register(LIS2DH12_REFERENCE, &dummy);   /* 保险起见再复位一次 */
    vTaskDelay(pdMS_TO_TICKS(MOTION_ARM_REF_WAIT_MS));

    /* 10) 写回校验 —— 确认寄存器真的写进去了，而不是 I2C 静默失败 */
    bool cfg_ok = true;
    if (!lis2dh12_read_register(LIS2DH12_CTRL_REG1, &rb) || rb != (LIS2DH12_ODR_10HZ|0x08|0x07)) {
        ESP_LOGW(TAG, "CTRL_REG1 回读 0x%02X，期望 0x2F", rb); cfg_ok = false; }
    if (!lis2dh12_read_register(LIS2DH12_CTRL_REG2, &rb) || rb != 0x81) {
        ESP_LOGW(TAG, "CTRL_REG2 回读 0x%02X，期望 0x81（HPF 没生效！）", rb); cfg_ok = false; }
    if (!lis2dh12_read_register(LIS2DH12_CTRL_REG5, &rb) || rb != 0x08) {
        ESP_LOGW(TAG, "CTRL_REG5 回读 0x%02X，期望 0x08（锁存没开）", rb); cfg_ok = false; }

    /* 11) 使能中断生成器：任一轴 high 事件（OR 组合）
     *     bit5 ZHIE | bit3 YHIE | bit1 XHIE = 0x2A */
    if (!lis2dh12_write_register(LIS2DH12_INT1_CFG, 0x2A)) return false;

    /* 12) 反复清锁存直到它保持干净。滤波器刚使能时可能还有残余瞬态，
     *     清一次不够 —— 清到连续读两次都是 IA=0 才算布防成功。 */
    uint8_t src = 0xFF;
    int tries = 0;
    for (; tries < 10; ++tries) {
        vTaskDelay(pdMS_TO_TICKS(MOTION_ARM_CLEAR_STEP_MS));
        if (!lis2dh12_read_register(LIS2DH12_INT1_SRC, &src)) break;
        if ((src & 0x40) == 0) {
            /* 再确认一次，确保不是刚好读在两次触发之间 */
            vTaskDelay(pdMS_TO_TICKS(MOTION_ARM_CLEAR_STEP_MS));
            if (lis2dh12_read_register(LIS2DH12_INT1_SRC, &src) && (src & 0x40) == 0) break;
        }
    }

    if ((src & 0x40) != 0) {
        log_int1_src("布防后仍在触发（静止时不该如此）", src);
        ESP_LOGW(TAG, "清锁存 %d 次仍未干净 —— 检查 HPF 是否生效，或阈值 %u mg 是否偏低",
                 tries, (unsigned)MOTION_LATCH_THS_MG);
    } else {
        ESP_LOGI(TAG, "LIS2DH12 睡眠布防完成：10Hz 低功耗(~3uA) HPF自动 阈值 %u mg(THS=%u) "
                      "持续 %u ms(DUR=%u) 锁存已清（用了 %d 次）%s",
                 (unsigned)MOTION_LATCH_THS_MG, ths, (unsigned)MOTION_LATCH_DUR_MS, dur,
                 tries + 1, cfg_ok ? "" : " [配置回读有异常]");
    }
    return true;
}
bool motion_read_and_clear_latch(void)
{
    uint8_t src = 0;

    /* 正常路径：用 motion_sensor_init() 在重配置之前缓存下来的那个值。
     * 这里再读一次的话，读到的是 init 自己造成的瞬态，不是睡眠窗口的真实情况。 */
    if (s_latch_cached) {
        s_latch_cached = false;
        bool ia_c = (s_latch_src & 0x40) != 0;
        log_int1_src(ia_c ? "睡眠窗口：动过" : "睡眠窗口：全程静止", s_latch_src);
        return ia_c;
    }

    ESP_LOGW(TAG, "没有开机缓存的锁存值，现场读一次（可能已被重配置污染）");
    if (!lis2dh12_read_register(LIS2DH12_INT1_SRC, &src)) {
        ESP_LOGW(TAG, "读 INT1_SRC 失败，保守起见按\"动过\"处理");
        return true;
    }
    bool ia = (src & 0x40) != 0;   /* bit6 = IA (interrupt active) */
    log_int1_src(ia ? "睡眠窗口：动过" : "睡眠窗口：全程静止", src);
    return ia;
}

int motion_activity_mg(uint32_t window_ms, int *pct_out, int *peak_mg_out)
{
    /* ODR 开到 100 Hz，而采样间隔是 20 ms(50 Hz)：保证每次读到的都是新样本。
     * 帧差判据下读到重复样本会被算成「没动」，会系统性压低活动占比。
     * 常规(HR)模式，和 lis2dh12_read_accel_data() 的 1 mg/LSB 标定一致。
     * 这几秒 ESP32 本来就醒着(~40 mA)，传感器那几十 µA 无所谓。 */
    if (!lis2dh12_configure(LIS2DH12_ODR_100HZ, LIS2DH12_FS_2G)) return -1;
    vTaskDelay(pdMS_TO_TICKS(120));   /* 换 ODR 后丢掉过渡样本 */

    const uint32_t period_ms = MOTION_SAMPLE_PERIOD_MS;
    const float    th_g      = MOTION_DELTA_TH_MG / 1000.0f;
    const float    rel_g     = th_g * 0.5f;      /* 只影响诊断用的沿计数 */

    int      total = 0, active = 0, edges = 0, bad = 0;
    bool     above = false, first = true, have_prev = false;
    uint32_t last_ms = 0, t = 0;
    float    peak_d = 0.0f, peak_m = 0.0f;
    float    px = 0.0f, py = 0.0f, pz = 0.0f;
    float    sum_d = 0.0f;

    /* 逐秒明细：用来区分「幅度不够」和「动得断断续续」。
     * 两者都表现为占比偏低，但处理方式完全相反 —— 前者调阈值，
     * 后者说明窗口判据本身选错了。 */
    #define ACT_SEC_MAX 10
    int   sec_tot[ACT_SEC_MAX] = {0}, sec_act[ACT_SEC_MAX] = {0};
    float sec_pk[ACT_SEC_MAX]  = {0}, sec_sum[ACT_SEC_MAX] = {0};
    uint32_t t0_us = (uint32_t)esp_timer_get_time();

    while (t < window_ms) {
        lis2dh12_accel_data_t a;
        if (lis2dh12_read_accel_data(&a)) {
            /* 对照用：旧判据 |‖a‖ − 1g|，只进日志 */
            float mag = sqrtf(a.x * a.x + a.y * a.y + a.z * a.z);
            float m   = fabsf(mag - 1.0f);
            if (m > peak_m) peak_m = m;

            if (have_prev) {
                /* 主判据：相邻帧的矢量差。旋转会让重力在轴间转移，
                 * 模长不变但帧差很大 —— 这正是摇头/翻转要抓的信号。 */
                float dx = a.x - px, dy = a.y - py, dz = a.z - pz;
                float d  = sqrtf(dx * dx + dy * dy + dz * dz);
                if (d > peak_d) peak_d = d;
                total++;

                sum_d += d;
                int si = t / 1000; if (si >= ACT_SEC_MAX) si = ACT_SEC_MAX - 1;
                sec_tot[si]++; sec_sum[si] += d;
                if (d > sec_pk[si]) sec_pk[si] = d;

                if (d > th_g) {
                    active++;                      /* 这一帧算「在动」 */
                    sec_act[si]++;
                    if (!above) {
                        above = true;
                        if (first || (t - last_ms) >= MOTION_EVENT_DEBOUNCE_MS) {
                            edges++; last_ms = t; first = false;   /* 仅诊断 */
                        }
                    }
                } else if (above && d < rel_g) {
                    above = false;
                }
            }
            px = a.x; py = a.y; pz = a.z;
            have_prev = true;
        } else {
            if (++bad > 5) { ESP_LOGW(TAG, "采样期间 I2C 连续失败，放弃本次判定"); return -1; }
        }
        vTaskDelay(pdMS_TO_TICKS(period_ms));
        t += period_ms;
    }

    int pct  = total ? (active * 100) / total : 0;
    int mean = total ? (int)((sum_d / total) * 1000.0f + 0.5f) : 0;

    ESP_LOGI(TAG, "运动采样：%lu ms / %d 点 -> 平均帧差 %d mg（门限 %d mg）"
                  "｜峰值 %.0f mg，>%u mg 占比 %d%%，沿计数 %d 次｜(对照 |‖a‖-1g| 峰值 %.0f mg)",
             (unsigned long)window_ms, total, mean, MOTION_MEAN_DELTA_TH_MG,
             (double)(peak_d * 1000.0f), (unsigned)MOTION_DELTA_TH_MG, pct, edges,
             (double)(peak_m * 1000.0f));

    /* 逐秒明细 + 真实耗时（t 是累加出来的名义时间，不含 I2C 和调度开销） */
    {
        char line[180]; int n = 0; int secs = (window_ms + 999) / 1000;
        if (secs > ACT_SEC_MAX) secs = ACT_SEC_MAX;
        for (int i = 0; i < secs && n < (int)sizeof(line) - 28; i++) {
            n += snprintf(line + n, sizeof(line) - n, "%s%.0f/%.0f",
                          i ? " | " : "",
                          sec_tot[i] ? (double)(sec_sum[i] / sec_tot[i] * 1000.0f) : 0.0,
                          (double)(sec_pk[i] * 1000.0f));
        }
        ESP_LOGI(TAG, "  逐秒(平均/峰值 mg)：%s｜实际耗时 %lu ms",
                 line, (unsigned long)(((uint32_t)esp_timer_get_time() - t0_us) / 1000));
    }
    #undef ACT_SEC_MAX

    if (pct_out)     *pct_out     = pct;
    if (peak_mg_out) *peak_mg_out = (int)(peak_d * 1000.0f);
    return mean;
}
void motion_sensor_task(void* pvParameters)
{
    ESP_LOGI(TAG, "Motion sensor task started");

    // 等待传感器稳定
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    while (1) {
        if (s_motion_stop) {
            ESP_LOGI(TAG, "Motion sensor task stopping (deep sleep)");
            vTaskDelete(NULL);
            return;
        }
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