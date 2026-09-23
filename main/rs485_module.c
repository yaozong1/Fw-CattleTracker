#include "rs485_module.h"
#include "hardware_config.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"

static const char* TAG = "RS485";

#ifndef RS485_UART_NUM
#define RS485_UART_NUM UART_NUM_0
#endif

#ifndef RS485_TX_PIN
#define RS485_TX_PIN 10
#endif

#ifndef RS485_RX_PIN
#define RS485_RX_PIN 9
#endif

// 可选：DE/RE 方向控制脚（未定义则不使用方向切换）
// 高电平=发送，低电平=接收（常见收发器如 MAX485）
// #define RS485_DE_RE_PIN  -1

#define RS485_RX_BUF_SIZE 2048
#define RS485_TX_BUF_SIZE 1024

static bool s_rs485_inited = false;

static void rs485_set_tx_mode(bool tx)
{
#ifdef RS485_DE_RE_PIN
    if (RS485_DE_RE_PIN >= 0) {
        gpio_set_level(RS485_DE_RE_PIN, tx ? 1 : 0);
    }
#endif
}

bool rs485_init(uint32_t baudrate)
{
    if (s_rs485_inited) {
        ESP_LOGW(TAG, "RS485 already initialized");
        return true;
    }

    // 可选方向脚配置
#ifdef RS485_DE_RE_PIN
    if (RS485_DE_RE_PIN >= 0) {
        gpio_config_t io = {
            .pin_bit_mask = (1ULL << RS485_DE_RE_PIN),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        gpio_config(&io);
        // 默认接收模式
        rs485_set_tx_mode(false);
    }
#endif

    uart_config_t cfg = {
        .baud_rate = (int)baudrate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t ret;
    ret = uart_driver_install(RS485_UART_NUM, RS485_RX_BUF_SIZE, RS485_TX_BUF_SIZE, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(ret));
        return false;
    }

    ret = uart_param_config(RS485_UART_NUM, &cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed: %s", esp_err_to_name(ret));
        uart_driver_delete(RS485_UART_NUM);
        return false;
    }

    ret = uart_set_pin(RS485_UART_NUM, RS485_TX_PIN, RS485_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed: %s", esp_err_to_name(ret));
        uart_driver_delete(RS485_UART_NUM);
        return false;
    }

    s_rs485_inited = true;
    ESP_LOGI(TAG, "RS485 init OK: UART=%d, TX=%d, RX=%d, baud=%lu", RS485_UART_NUM, RS485_TX_PIN, RS485_RX_PIN, (unsigned long)baudrate);
    return true;
}

int rs485_write(const uint8_t* data, size_t len, uint32_t timeout_ticks)
{
    if (!s_rs485_inited || !data || len == 0) {
        return -1;
    }

    // 进入发送模式
    rs485_set_tx_mode(true);

    int written = uart_write_bytes(RS485_UART_NUM, (const char*)data, (size_t)len);
    if (written < 0) {
        // 退出到接收模式
        rs485_set_tx_mode(false);
        return -1;
    }

    // 等待发送完成
    (void)uart_wait_tx_done(RS485_UART_NUM, timeout_ticks);

    // 切回接收模式
    rs485_set_tx_mode(false);
    return written;
}

int rs485_read(uint8_t* buf, size_t maxlen, uint32_t timeout_ticks)
{
    if (!s_rs485_inited || !buf || maxlen == 0) {
        return -1;
    }
    int n = uart_read_bytes(RS485_UART_NUM, buf, maxlen, timeout_ticks);
    if (n < 0) {
        return -1;
    }
    return n;
}

void rs485_flush(void)
{
    if (!s_rs485_inited) return;
    uart_flush(RS485_UART_NUM);
}

void rs485_deinit(void)
{
    if (!s_rs485_inited) return;
    uart_driver_delete(RS485_UART_NUM);
    s_rs485_inited = false;
}
