#include "gpio_module.h"
#include "hardware_config.h"
#include "driver/gpio.h"
#include "esp_rom_gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "GPIO";

// LED 当前状态（led_blink 结束后据此恢复）
static bool s_led_esp_on  = false;
static bool s_led_conn_on = false;

// 电子围栏供电轨状态
static bool s_fence_rails_on = false;
static bool s_fence_iso_on   = false;
static bool s_fence_12v_on   = false;

void gpio_init(void)
{
    /* ---- 指示灯 ---- */
    gpio_reset_pin(LED_ESP_PIN);
    gpio_set_direction(LED_ESP_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_ESP_PIN, 0);

    gpio_reset_pin(LED_CONN_PIN);
    gpio_set_direction(LED_CONN_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_CONN_PIN, 0);

    /* ---- 按键 ---- */
    gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON_PIN, GPIO_PULLUP_ONLY);

    /* ---- IBL / 电池采样 (IO1)：保持浮空，交给 ADC ---- */
    gpio_set_pull_mode(IBL_PIN, GPIO_FLOATING);

    /* ---- 蜂窝模组控制 ---- */
    gpio_set_direction(QT_POWER_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(QT_POWER_PIN, 0);            // PWRKEY 释放

    gpio_set_direction(MODEM_RESET_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(MODEM_RESET_PIN, 0);         // 不复位

    gpio_set_direction(MODEM_EN_PIN, GPIO_MODE_OUTPUT);
    // 默认不通电：模组要到真正上报时才开轨，省掉搜星那一段的 LDO 空载电流
    gpio_set_level(MODEM_EN_PIN, MODEM_RAIL_ON_AT_BOOT ? MODEM_EN_ON_LEVEL
                                                       : MODEM_EN_OFF_LEVEL);

    /* IO35 <- TXB0108 B8（模组 STATUS 经电平转换），板上已有 R13 100K 下拉。
     * 这里必须把 ESP32 的内部上下拉全关掉：芯片复位后部分 pad 的内部上拉是打开的，
     * 45K 内部上拉对着 100K 外部下拉会形成分压，一方面读数变成 2.3V 的中间态，
     * 另一方面恒定漏 3.3V/145K ≈ 23 µA。外部电阻已经定了电平，内部不要插手。 */
    gpio_set_direction(MODEM_STATUS_PIN, GPIO_MODE_INPUT);
    gpio_pullup_dis(MODEM_STATUS_PIN);
    gpio_pulldown_dis(MODEM_STATUS_PIN);

    /* ---- GNSS ----
     * 顺序很关键：必须【先写电平、再切方向】。
     * 深睡唤醒等同芯片复位，GPIO 输出锁存器复位值是 0；如果先 set_direction(OUTPUT)，
     * IO11 会被驱动成低电平，LP5907 当场关断 —— GNSS 掉电冷启动，
     * Backup 状态和 RTC 时间全丢，唤醒后一直在满功率搜星（约 40 mA）。 */
    gpio_set_level(GNSS_EN_PIN, 1);             // 先把输出锁存器写成 1
    gpio_set_direction(GNSS_EN_PIN, GPIO_MODE_OUTPUT);

    gpio_set_level(GNSS_RST_PIN, 0);            // 反相驱动，0 = 释放复位
    gpio_set_direction(GNSS_RST_PIN, GPIO_MODE_OUTPUT);

    gpio_set_direction(GNSS_PPS_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(GNSS_3DFIX_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(GNSS_GEOFENCE_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(GNSS_JAMIND_PIN, GPIO_MODE_INPUT);

    /* ---- 加速度计中断 ---- */
    gpio_set_direction(MOTION_SENSOR_INT1_PIN, GPIO_MODE_INPUT);

    /* ---- 电子围栏：必须最先进入安全状态 ---- */
    fence_pins_safe_init();

    power_control_init();

    ESP_LOGI(TAG, "GPIO initialized (Cattle_T)");
}

void power_control_init(void)
{
    // GNSS 必须常电（Backup 靠 VCC 保持星历）；模组轨按 MODEM_RAIL_ON_AT_BOOT 决定
    modem_power_enable(MODEM_RAIL_ON_AT_BOOT ? true : false);
    gpio_set_level(GNSS_EN_PIN, 1);
    gpio_set_level(GNSS_RST_PIN, 0);

    ESP_LOGI(TAG, "Power control pins initialized (modem rail %s, GNSS ON)",
             MODEM_RAIL_ON_AT_BOOT ? "ON" : "OFF");
}

/* ============ 电子围栏 ============ */
/* 把一个"动作"引脚钉死在低电平。
 *
 * 不用 gpio_reset_pin()：它的实现是「输入 + 使能内部上拉(~45kΩ)」。
 * 对 IO9 (LT3750 CHARGE) 和 IO13 (PC817 Trig) 来说，那一瞬间等于主动置位 ——
 * 启动日志里 "GPIO[9] ... Pullup: 1" 就是这么来的。这两个脚板上都没有外部
 * 下拉（12V_EN 有 R77 10k、5V0_ISO_EN 有 R63 10k，这两个没有），所以这里
 * 必须自己把内部上拉关掉、换成内部下拉。
 *
 * 顺序同样关键：先写电平再切方向。深睡醒来后输出锁存器复位为 0，
 * 反过来会让引脚先输出一次毛刺（GNSS 电源那个坑就是这么来的）。 */
static void fence_pin_force_low(int pin)
{
    esp_rom_gpio_pad_select_gpio(pin);   /* 只选 GPIO 功能，不碰上下拉 */
    gpio_set_level(pin, 0);              /* 先写电平 */
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    gpio_set_level(pin, 0);
    gpio_pullup_dis(pin);
    gpio_pulldown_en(pin);               /* 万一哪天回到输入态，也是往下走 */
}

void fence_pins_safe_init(void)
{
    /* 两个"动作"脚：没有外部下拉，必须最先钉死 */
    fence_pin_force_low(FENCE_TRIG_PIN);        /* 绝对不能默认触发 */
    fence_pin_force_low(FENCE_CHARGE_EN_PIN);   /* LT3750 不充电 */

    /* DONE# 是 LT3750 的开漏输出，板上 R81 上拉。纯输入，不要加内部上拉 */
    esp_rom_gpio_pad_select_gpio(FENCE_CHARGE_DONE_PIN);
    gpio_set_direction(FENCE_CHARGE_DONE_PIN, GPIO_MODE_INPUT);
    gpio_pullup_dis(FENCE_CHARGE_DONE_PIN);
    gpio_pulldown_dis(FENCE_CHARGE_DONE_PIN);

    /* 两个电源使能脚：板上各有 10k 外部下拉兜底，这里也一并钉低 */
    fence_pin_force_low(FENCE_12V_EN_PIN);      /* 12V 升压关闭 */
    fence_pin_force_low(FENCE_5V_ISO_EN_PIN);   /* 隔离 5V 关闭 */

    s_fence_rails_on = false;
    s_fence_iso_on   = false;
    s_fence_12v_on   = false;

    ESP_LOGI(TAG, "Fence pins in SAFE state (charge off, no trigger, pulldown on IO%d/IO%d)",
             FENCE_CHARGE_EN_PIN, FENCE_TRIG_PIN);
}

/* ================= deep sleep 引脚状态 =================
 *
 * 深睡时数字 IO 整块断电，所有输出脚变高阻（悬空），电平由板上的电阻决定。
 * 这里只调 gpio_deep_sleep_hold_en()，**不做任何逐脚 gpio_hold_en()**。
 *
 * 为什么不用逐脚 hold：对 IO0~IO21 它走的是 RTC 域的锁存器，一旦使用，
 * RTC_PERIPH 电源域整个休眠期不能断电。实测代价 +405 µA（112 → 490 µA）。
 * 而 gpio_deep_sleep_hold_en() 走数字域，不依赖那个域，但它只管 IO22 以上的
 * 纯数字脚，对 RTC 脚无效——所以 IO0~IO21 的电平必须由外部电阻保证。
 *
 * 各脚在深睡期间靠什么定电平：
 *   IO11 GNSS_EN      100K 上拉到 3V3   -> 高，U21 保持使能，星历不丢（热启动）
 *   IO2  GNSS_RST     Q6(KRC106S) 内部基极下拉 -> 低，不复位
 *   IO12 5V0_ISO_EN   R63 10K 下拉      -> 低
 *   IO47 12V_EN       R77 10K 下拉      -> 低（非 RTC 脚，本来也 hold 不了）
 *   IO13 Trig         R89 220 接可控硅栅极网络 -> 低
 *   IO48 MODEM_EN     板上下拉（高有效）-> 低，3.8V 轨断开
 *   IO9  C_EN         ⚠ 目前没有任何电阻定电平，深睡期间悬空
 *
 * ⚠ IO9 待办：它直连 LT3750(U12) 的 CHARGE。现在 D25 翘着、U12 没有 VCC，
 *   所以悬空无害；**装回 D25 之前必须先给 IO9 补一颗 100K 对地下拉**，
 *   否则高压电容充电器有自启的可能。
 *
 * ⚠ DEEPSLEEP_WAKE_ON_BUTTON：ext0 唤醒同样要求 RTC_PERIPH 供电，
 *   打开它会把那 405 µA 原样加回来。见 hardware_config.h。
 *
 * 实测（1800 mAh 电池，3.7 V 侧）：深睡 112 µA，GNSS 保持 Backup，唤醒热启动。
 * ====================================================== */
#define RTC_GPIO_MAX_NUM 21

void deep_sleep_prepare_pins(void)
{
    ESP_LOGI(TAG, "Deep sleep: 设置引脚电平（靠板上电阻在休眠期保持）");

    // GNSS 供电保持，否则备份域星历丢失，唤醒后变冷启动
    gpio_set_level(GNSS_EN_PIN, 1);
    gpio_set_level(GNSS_RST_PIN, 0);

    // 围栏相关一律拉低
    gpio_set_level(FENCE_TRIG_PIN, 0);
    gpio_set_level(FENCE_CHARGE_EN_PIN, 0);
    gpio_set_level(FENCE_5V_ISO_EN_PIN, 0);
    gpio_set_level(FENCE_12V_EN_PIN, 0);

    // 模组 3.8V 轨断开；IO48 非 RTC 脚，休眠期靠板上的下拉保持关断
    modem_power_enable(false);
#if MODEM_EN_ACTIVE_HIGH
    gpio_pulldown_en(MODEM_EN_PIN);     // 多一层保险，和板上的下拉同向
    gpio_pullup_dis(MODEM_EN_PIN);
#else
    ESP_LOGW(TAG, "  IO%-2d (MODEM_EN) 低有效，深睡浮空 -> 3.8V 轨会自行恢复通电",
             MODEM_EN_PIN);
#endif

    // LED 关掉
    gpio_set_level(LED_ESP_PIN, 0);
    gpio_set_level(LED_CONN_PIN, 0);

    gpio_deep_sleep_hold_en();
}

void deep_sleep_release_pins(void)
{
    gpio_deep_sleep_hold_dis();
}

void fence_iso_enable(bool on)
{
    gpio_set_level(FENCE_5V_ISO_EN_PIN, on ? 1 : 0);
    s_fence_iso_on = on;
    ESP_LOGI(TAG, "Fence 5V0_ISO_EN (IO%d) -> %s", FENCE_5V_ISO_EN_PIN, on ? "ON" : "OFF");
}

void fence_12v_enable(bool on)
{
    gpio_set_level(FENCE_12V_EN_PIN, on ? 1 : 0);
    s_fence_12v_on = on;
    ESP_LOGI(TAG, "Fence 12V_EN (IO%d) -> %s", FENCE_12V_EN_PIN, on ? "ON" : "OFF");
}

bool fence_iso_is_on(void) { return s_fence_iso_on; }
bool fence_12v_is_on(void) { return s_fence_12v_on; }

void fence_rails_enable(bool on)
{
    if (on) {
        // 先开控制侧的隔离 5V，再开 12V 升压
        fence_iso_enable(true);
        fence_12v_enable(true);
    } else {
        fence_12v_enable(false);
        fence_iso_enable(false);
    }
    // 无论如何，充电与触发保持关闭
    gpio_set_level(FENCE_CHARGE_EN_PIN, 0);
    gpio_set_level(FENCE_TRIG_PIN, 0);

    s_fence_rails_on = on;
    ESP_LOGI(TAG, "Fence rails (12V_EN IO%d + 5V0_ISO_EN IO%d) -> %s",
             FENCE_12V_EN_PIN, FENCE_5V_ISO_EN_PIN, on ? "ON" : "OFF");
}

bool fence_rails_are_on(void)
{
    return s_fence_rails_on;
}

void fence_trig_pulse_train(int times, uint32_t on_ms, uint32_t off_ms)
{
    // 保证不会在充电状态下触发
    gpio_set_level(FENCE_CHARGE_EN_PIN, 0);

    ESP_LOGI(TAG, "Trig pulse train on IO%d: %d cycles, %lu ms high / %lu ms low",
             FENCE_TRIG_PIN, times, (unsigned long)on_ms, (unsigned long)off_ms);

    for (int i = 1; i <= times; ++i) {
        gpio_set_level(FENCE_TRIG_PIN, 1);
        ESP_LOGI(TAG, "  Trig %d/%d HIGH", i, times);
        vTaskDelay(pdMS_TO_TICKS(on_ms));
        gpio_set_level(FENCE_TRIG_PIN, 0);
        ESP_LOGI(TAG, "  Trig %d/%d LOW", i, times);
        vTaskDelay(pdMS_TO_TICKS(off_ms));
    }

    // 收尾务必停在低电平
    gpio_set_level(FENCE_TRIG_PIN, 0);
    ESP_LOGI(TAG, "Trig pulse train done, IO%d held LOW", FENCE_TRIG_PIN);
}

bool fence_charge_done(void)
{
    // DONE# 低有效
    return gpio_get_level(FENCE_CHARGE_DONE_PIN) == 0;
}

/* ============ 模组电源 ============ */
void modem_power_enable(bool on)
{
    // 极性由 MODEM_EN_ACTIVE_HIGH 决定，见 hardware_config.h
    gpio_set_level(MODEM_EN_PIN, on ? MODEM_EN_ON_LEVEL : MODEM_EN_OFF_LEVEL);
    ESP_LOGI(TAG, "Modem 3.8V rail %s (IO%d=%d)",
             on ? "ON" : "OFF", MODEM_EN_PIN,
             on ? MODEM_EN_ON_LEVEL : MODEM_EN_OFF_LEVEL);
}

bool modem_is_powered(void)
{
    return gpio_get_level(MODEM_STATUS_PIN) == 1;
}

/* ============ GNSS 状态 ============ */
int gnss_get_3dfix_level(void)    { return gpio_get_level(GNSS_3DFIX_PIN); }
int gnss_get_geofence_level(void) { return gpio_get_level(GNSS_GEOFENCE_PIN); }
int gnss_get_jam_level(void)      { return gpio_get_level(GNSS_JAMIND_PIN); }

/* ============ LED / 按键 ============ */
void led_set(int led_pin, bool state)
{
    gpio_set_level(led_pin, state ? 1 : 0);
    if (led_pin == LED_ESP_PIN)  { s_led_esp_on = state; }
    if (led_pin == LED_CONN_PIN) { s_led_conn_on = state; }
}

void led_blink(int led_pin, int times, uint32_t on_ms, uint32_t off_ms)
{
    for (int i = 0; i < times; ++i) {
        gpio_set_level(led_pin, 1);
        vTaskDelay(pdMS_TO_TICKS(on_ms));
        gpio_set_level(led_pin, 0);
        vTaskDelay(pdMS_TO_TICKS(off_ms));
    }
    // 恢复闪烁之前的状态，避免把自测结果指示灯覆盖掉
    bool prev = (led_pin == LED_ESP_PIN) ? s_led_esp_on
              : (led_pin == LED_CONN_PIN) ? s_led_conn_on
              : false;
    gpio_set_level(led_pin, prev ? 1 : 0);
}

bool button_is_pressed(void)
{
    return gpio_get_level(BUTTON_PIN) == 0;
}

void led1_on(void)  { led_set(LED_ESP_PIN, true); }
void led1_off(void) { led_set(LED_ESP_PIN, false); }
void led3_on(void)  { led_set(LED_CONN_PIN, true); }
void led3_off(void) { led_set(LED_CONN_PIN, false); }
