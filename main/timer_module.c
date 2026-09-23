#include "timer_module.h"
#include "hardware_config.h"
#include "gpio_module.h"
#include "mqtt_module.h"
#include "driver/gptimer.h"
#include "esp_log.h"

static const char *TAG = "TIMER";

static gptimer_handle_t timer_handle = NULL;
static timer_callback_t user_callback = NULL;

// 全局标志位
volatile bool log_output = false;
volatile bool send_at_flag = false;
volatile bool mqtt_connect_flag = false;
volatile bool mqtt_publish_flag = false;

// 定时器回调函数 - 每100ms执行一次
static bool IRAM_ATTR timer_callback_100ms(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    // 检查按键状态
    // 注意：IO0 现在由 button_module 用于"双击打开围栏供电轨"的手势识别，
    // 若同时保留这里的按键钩子，双击会顺带把模组重启一次。
    // 需要恢复 pe_board 的"按键重启模组"行为时，把 BUTTON_LEGACY_MODEM_CYCLE 置 1。
#if BUTTON_LEGACY_MODEM_CYCLE
    if(button_is_pressed()) {
        log_output = true;
#if ENABLE_MQTT
        mqtt_connect_flag = true;
#endif
    }
#endif
    
    // 示例：翻转状态、计数器等
    static uint32_t counter = 0;
    counter++;

    if (counter % 50 == 0) { // 每5秒执行一次
        //send_at_flag = true; // 设置标志位
    }
    
    if (counter % 600 == 0) { // 每60秒执行一次（600 * 100ms）
#if ENABLE_MQTT
        mqtt_publish_flag = true; // 触发MQTT数据发送
#endif
    }
    
    // 调用用户回调
    if (user_callback) {
        user_callback();
    }
    
    return false; // 返回false表示不需要yield切换任务
}

void timer_init(void)
{
    ESP_LOGI(TAG, "Initializing 100ms timer");
    
    // 配置定时器
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1MHz, 1 tick = 1us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &timer_handle));
    
    // 配置报警事件（100ms = 100000us）
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = 100000, // 100ms
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true, // 自动重载，实现周期触发
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(timer_handle, &alarm_config));
    
    // 注册回调函数
    gptimer_event_callbacks_t cbs = {
        .on_alarm = timer_callback_100ms,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(timer_handle, &cbs, NULL));
    
    // 使能定时器
    ESP_ERROR_CHECK(gptimer_enable(timer_handle));
    
    ESP_LOGI(TAG, "100ms timer initialized");
}

void timer_start(void)
{
    if (timer_handle) {
        ESP_ERROR_CHECK(gptimer_start(timer_handle));
        ESP_LOGI(TAG, "Timer started");
    }
}

void timer_stop(void)
{
    if (timer_handle) {
        ESP_ERROR_CHECK(gptimer_stop(timer_handle));
        ESP_LOGI(TAG, "Timer stopped");
    }
}

void timer_register_callback(timer_callback_t callback)
{
    user_callback = callback;
}