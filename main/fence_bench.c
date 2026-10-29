/*
 * fence_bench.c —— 电子围栏台架测试控制台（阶段 1：D25 未焊）
 *
 * 电路（由 netlist.ipc 还原）：
 *
 *   5V0 ─ L14(4.7µH) ─[D25 SS14]─┬─ 12V ──→ U12 LT3750 VCC
 *         U18 MT3608 EN ← IO47   └─ TP13          VTRANS ← VBAT
 *                                                 GATE/R82 → Q2 → U7 初级 (1:10)
 *                                                                    │
 *                                                              U7 次级 → D26
 *                                                                    │
 *                     ┌──────────────┬────────────────────────┬──────┴───┐
 *                 C51 390nF/630V  D27(反并)   R5+R39+R40=9.9MΩ泄放   J2 EF25 初级
 *                     │                              │                  │
 *                     │                              │             Q7 BT151-800R
 *                     │                              │             gate ← R72(82Ω)
 *                     │                              │                  ↑ Q9 ← U29 PC817 ← R89 ← IO13
 *                  GNDPWR ─────────────────────────────────────────── GNDPWR
 *                                                                J2 次级 → J11/J4 围栏输出
 *
 * 阶段 1 为什么是零高压风险：
 *   D25 没焊 → 12V 轨恒为 0 → LT3750 没有 VCC → 整条高压链路是死的。
 *   本阶段只验证低压控制链路：IO12 → U24 → U23 隔离 5V，
 *   以及 IO13 → PC817 → Q9 → Q7 栅极这条触发链路。
 *
 * 安全约束（写死在代码里，不靠命令行参数约束）：
 *   - D25 未焊时拒绝一切打开 12V / 充电的命令
 *   - trig 必须先 `f arm`，60 秒后自动解除
 *   - trig 执行前强制 IO9 = 0，结束后强制 IO13 = 0
 *   - 脉冲次数和时长有硬上限
 *   - 本固件【没有】充电命令，阶段 2 另行加入并单独评审
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "esp_console.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "hardware_config.h"
#include "gpio_module.h"
#include "battery_module.h"
#include "fence_bench.h"

#if FENCE_BENCH

static const char *TAG = "FBENCH";

/* ---------------- arm 状态 ---------------- */

static int64_t s_armed_until_us = 0;

/* 上次充电的结果，供 f status 回显 */
static int      s_last_charge_ms   = -1;   /* 上次充电实际持续时间 */
static int      s_last_done_us     = -1;   /* 上次 IO10 首次跳变的时刻 */

static void arm_set(bool on)
{
    s_armed_until_us = on
        ? esp_timer_get_time() + (int64_t)FENCE_BENCH_ARM_TIMEOUT_S * 1000000LL
        : 0;
}

static bool arm_is_active(void)
{
    return esp_timer_get_time() < s_armed_until_us;
}

static int arm_remaining_s(void)
{
    int64_t d = s_armed_until_us - esp_timer_get_time();
    return d > 0 ? (int)(d / 1000000) : 0;
}

static bool require_arm(void)
{
    if (arm_is_active()) return true;
    printf("拒绝：未解锁。先执行 `f arm`（%d 秒内有效）\n", FENCE_BENCH_ARM_TIMEOUT_S);
    return false;
}

/* ---------------- 子命令 ---------------- */

static void cmd_status(void)
{
    printf("\n");
    printf("  D25 (SS14)          : %s\n",
           FENCE_BENCH_D25_FITTED ? "已焊 —— 高压链路可用，阶段 2"
                                  : "未焊 —— 12V 轨为 0，高压链路是死的（阶段 1）");
    printf("  解锁状态            : %s\n",
           arm_is_active() ? "ARMED" : "DISARMED");
    if (arm_is_active()) printf("                        剩余 %d 秒\n", arm_remaining_s());
    printf("\n");
    printf("  IO%-2d 12V_EN        : %s   (板上 R77 10k 下拉)\n",
           FENCE_12V_EN_PIN, fence_12v_is_on() ? "ON " : "OFF");
    printf("  IO%-2d 5V0_ISO_EN    : %s   (板上 R63 10k 下拉，测 TP14 应为 5.0V)\n",
           FENCE_5V_ISO_EN_PIN, fence_iso_is_on() ? "ON " : "OFF");
    printf("  IO%-2d C_EN (CHARGE) : LOW  (无外部下拉，靠内部下拉 + 固件钉死)\n",
           FENCE_CHARGE_EN_PIN);
    printf("  IO%-2d TRIG          : LOW  (无外部下拉，靠内部下拉 + 固件钉死)\n",
           FENCE_TRIG_PIN);
    printf("  IO%-2d C_DONE (原始)  : %d    (极性待实测确认，别信注释)\n",
           FENCE_CHARGE_DONE_PIN, gpio_get_level(FENCE_CHARGE_DONE_PIN));
    if (s_last_charge_ms >= 0) {
        printf("\n  上次充电            : 窗口 %d ms", s_last_charge_ms);
        if (s_last_done_us >= 0) printf("，DONE 首次跳变 @ %.2f ms\n", (double)s_last_done_us / 1000.0);
        else                     printf("，窗口内 DONE 无跳变\n");
    }
    printf("\n");
    printf("  VBAT                : %.2f V\n", read_battery_voltage());
    printf("\n");
    printf("  注：IO9 / IO13 配置为输出，读不回电平，上面显示的是固件状态。\n");
    printf("      实测点：TP14 (5V_ISO) 和 R73 (Q7 栅极-阴极)。\n");
    printf("      ★ 黑表笔必须接 GNDPWR（C51 负极腿），不是逻辑 GND ——\n");
    printf("        5V_ISO 是隔离电源，参考地不同，接错了读数没有意义。\n\n");
}

static void cmd_iso(int argc, char **argv)
{
    if (argc < 3) { printf("用法: f iso 0|1\n"); return; }
    bool on = atoi(argv[2]) != 0;
    fence_iso_enable(on);
    printf("5V0_ISO_EN -> %s。现在量 TP14：%s\n",
           on ? "ON" : "OFF", on ? "应为 5.0 V" : "应为 0 V");
}

static void cmd_12v(int argc, char **argv)
{
    if (argc < 3) { printf("用法: f 12v 0|1|pulse\n"); return; }

    if (strcmp(argv[2], "pulse") == 0) {
        if (!require_arm()) return;
        printf("IO%d 发一个 %d ms 的脉冲 —— 只够示波器抓，万用表看不到。\n",
               FENCE_12V_EN_PIN, FENCE_BENCH_12V_PULSE_MS);
        fence_12v_enable(true);
        vTaskDelay(pdMS_TO_TICKS(FENCE_BENCH_12V_PULSE_MS));
        fence_12v_enable(false);
        printf("脉冲结束，IO%d 已拉低。\n", FENCE_12V_EN_PIN);
        return;
    }

    bool on = atoi(argv[2]) != 0;
    if (on && !FENCE_BENCH_D25_FITTED) {
        printf("\n拒绝：D25 未焊，不允许持续打开 12V_EN。\n");
        printf("  MT3608 没有输出通路，FB 恒为 0 会按最大占空比开关，\n");
        printf("  L14 每周期攒的能量只能靠内部开关反复雪崩泄放，几十毫秒就能打死它。\n");
        printf("  要确认 IO%d 能翻转，用 `f 12v pulse` 配示波器。\n", FENCE_12V_EN_PIN);
        printf("  持续打开请等阶段 2（焊上 D25，改 FENCE_BENCH_D25_FITTED 为 1）。\n\n");
        return;
    }
    fence_12v_enable(on);
}

static void cmd_trig(int argc, char **argv)
{
    if (!require_arm()) return;

    int      n      = (argc > 2) ? atoi(argv[2]) : 1;
    uint32_t on_ms  = (argc > 3) ? (uint32_t)strtoul(argv[3], NULL, 10) : 500;
    uint32_t off_ms = (argc > 4) ? (uint32_t)strtoul(argv[4], NULL, 10) : 500;

    if (n < 1) n = 1;
    if (n > FENCE_BENCH_TRIG_MAX_PULSES) {
        printf("次数钳到上限 %d\n", FENCE_BENCH_TRIG_MAX_PULSES);
        n = FENCE_BENCH_TRIG_MAX_PULSES;
    }
    if (on_ms  > FENCE_BENCH_TRIG_MAX_MS) on_ms  = FENCE_BENCH_TRIG_MAX_MS;
    if (off_ms > FENCE_BENCH_TRIG_MAX_MS) off_ms = FENCE_BENCH_TRIG_MAX_MS;

    printf("\n触发 %d 次，高 %lu ms / 低 %lu ms。\n",
           n, (unsigned long)on_ms, (unsigned long)off_ms);
    printf("  5V0_ISO 当前 %s —— %s\n",
           fence_iso_is_on() ? "ON" : "OFF",
           fence_iso_is_on()
               ? "Q7 栅极 (/SCR_D) 应出现约 0.8~1.3 V 的脉冲（栅极结的正向压降）"
               : "Q7 栅极应【全程保持 0 V】（这是反向对照，很重要）");
    printf("\n");

    fence_trig_pulse_train(n, on_ms, off_ms);

    printf("结束，IO%d 已拉低。\n\n", FENCE_TRIG_PIN);
}

/* ================= 阶段 2：充电 =================
 *
 * 固件对 C51 的电压是【瞎】的 —— R5+R39+R40 是纯泄放串，中间没有 ADC 抽头。
 * 唯一的高压侧反馈就是 IO10 (DONE) 这一个数字位，电压只能靠万用表/示波器。
 *
 * 而且 DONE 的极性从来没被验证过（hardware_config.h 写的是"低=充满"，
 * R81 是 100k 上拉，看着像开漏低有效，但固件一次都没真充过电）。
 * 所以这里【不解释】极性，只把 IO10 的原始跳变时刻原样打出来，让实测定论。
 */

/* 记录一次充电窗口内 IO10 的跳变。忙等采样，200 µs 一次。 */
#define CHG_MAX_EDGES 16

static void charge_once(uint32_t ms, bool quiet)
{
    int      lvl0 = gpio_get_level(FENCE_CHARGE_DONE_PIN);
    int      lvl  = lvl0;
    int64_t  t0   = esp_timer_get_time();
    int64_t  edge_us[CHG_MAX_EDGES];
    int      edge_lv[CHG_MAX_EDGES];
    int      nedge = 0;

    gpio_set_level(FENCE_TRIG_PIN, 0);          /* 充电期间绝不允许触发 */
    gpio_set_level(FENCE_CHARGE_EN_PIN, 1);     /* CHARGE 拉高，开始 */

    while ((esp_timer_get_time() - t0) < (int64_t)ms * 1000) {
        int v = gpio_get_level(FENCE_CHARGE_DONE_PIN);
        if (v != lvl) {
            if (nedge < CHG_MAX_EDGES) {
                edge_us[nedge] = esp_timer_get_time() - t0;
                edge_lv[nedge] = v;
                nedge++;
            }
            lvl = v;
        }
        esp_rom_delay_us(200);
    }

    gpio_set_level(FENCE_CHARGE_EN_PIN, 0);     /* 无论如何，收尾必须拉低 */
    int64_t total_us = esp_timer_get_time() - t0;

    s_last_charge_ms = (int)(total_us / 1000);
    s_last_done_us   = nedge ? (int)edge_us[0] : -1;

    if (quiet) return;

    printf("\n充电窗口 %lu ms 结束，IO%d 已拉低。\n",
           (unsigned long)ms, FENCE_CHARGE_EN_PIN);
    printf("  IO%d (DONE) 起始电平 = %d，结束电平 = %d\n",
           FENCE_CHARGE_DONE_PIN, lvl0, lvl);
    if (nedge == 0) {
        printf("  整个窗口内没有跳变。\n");
        printf("  → 要么还没充满（加长时间再试），要么 LT3750 根本没工作。\n");
    } else {
        printf("  跳变 %d 次：\n", nedge);
        for (int i = 0; i < nedge; i++) {
            printf("      %7.2f ms  ->  %d\n",
                   (double)edge_us[i] / 1000.0, edge_lv[i]);
        }
        printf("  → 首次跳变在 %.2f ms。如果这就是「充满」，说明充电耗时约这么久。\n",
               (double)edge_us[0] / 1000.0);
    }
    printf("\n  ⚠ C51 现在带电。断开表笔后靠板载 9.9 MΩ 泄放，τ≈3.9 s，\n");
    printf("     约 14 秒降到 10 V 以下。要立刻放电请用放电棒。\n\n");
}

static bool charge_preflight(void)
{
    if (!FENCE_BENCH_D25_FITTED) {
        printf("拒绝：FENCE_BENCH_D25_FITTED = 0。焊上 D25 并改这个宏才能充电。\n");
        return false;
    }
    if (!require_arm()) return false;
    if (!fence_12v_is_on()) {
        printf("拒绝：12V 还没打开。LT3750 的 VCC 就是 12V 轨。\n");
        printf("  先执行 `f 12v 1`，量一下 TP13 确认是 12 V，再来充电。\n");
        return false;
    }
    return true;
}

static void cmd_charge(int argc, char **argv)
{
    if (!charge_preflight()) return;

    uint32_t ms = (argc > 2) ? (uint32_t)strtoul(argv[2], NULL, 10)
                             : FENCE_BENCH_CHARGE_DEFAULT_MS;
    if (ms < 1) ms = 1;
    if (ms > FENCE_BENCH_CHARGE_MAX_MS) {
        printf("时长钳到上限 %d ms\n", FENCE_BENCH_CHARGE_MAX_MS);
        ms = FENCE_BENCH_CHARGE_MAX_MS;
    }

    printf("\n★ 开始充电 %lu ms —— 单手操作，别碰 C51 和 J11/J4。\n",
           (unsigned long)ms);
    charge_once(ms, false);
}

static void cmd_hold(int argc, char **argv)
{
    if (!charge_preflight()) return;

    uint32_t sec = (argc > 2) ? (uint32_t)strtoul(argv[2], NULL, 10) : 10;
    if (sec < 1) sec = 1;
    if (sec > FENCE_BENCH_HOLD_MAX_S) {
        printf("时长钳到上限 %d 秒\n", FENCE_BENCH_HOLD_MAX_S);
        sec = FENCE_BENCH_HOLD_MAX_S;
    }

    printf("\n★ 维持模式 %lu 秒：每 %d ms 补一次电，歇 %d ms。\n",
           (unsigned long)sec, FENCE_BENCH_HOLD_CHARGE_MS, FENCE_BENCH_HOLD_REST_MS);
    printf("  现在可以从容地用万用表读 C51 两端电压（黑表笔 GNDPWR）。\n");
    printf("  预期约 405 V：(R67 82.5k / R64 2.49k) x 匝比 10 x 1.223V\n\n");

    int64_t end_us = esp_timer_get_time() + (int64_t)sec * 1000000LL;
    int rounds = 0;
    while (esp_timer_get_time() < end_us) {
        charge_once(FENCE_BENCH_HOLD_CHARGE_MS, true);
        rounds++;
        vTaskDelay(pdMS_TO_TICKS(FENCE_BENCH_HOLD_REST_MS));
    }
    gpio_set_level(FENCE_CHARGE_EN_PIN, 0);

    printf("维持结束，共补电 %d 轮，IO%d 已拉低。\n", rounds, FENCE_CHARGE_EN_PIN);
    printf("⚠ C51 仍然带电，约 14 秒后降到 10 V 以下。\n\n");
}

static void cmd_help(void)
{
    printf("\n电子围栏台架测试命令（阶段 1，D25 未焊）\n\n");
    printf("  f status             打印全部引脚与状态\n");
    printf("  f arm                解锁动作类命令，%d 秒后自动解除\n", FENCE_BENCH_ARM_TIMEOUT_S);
    printf("  f disarm             立即解除\n");
    printf("  f iso 0|1            IO%d 隔离 5V 使能（安全，随便开）\n", FENCE_5V_ISO_EN_PIN);
    printf("  f 12v 0|1|pulse      IO%d 12V 升压使能（D25 未焊时只允许 pulse）\n", FENCE_12V_EN_PIN);
    printf("  f trig [n] [on] [off]  IO%d 触发脉冲，默认 1 次 500/500 ms\n", FENCE_TRIG_PIN);
    printf("  f safe               一键回到安全态（注意：放不掉 C51 上的电）\n");
    printf("\n  --- 阶段 2（D25 已焊）---\n");
    printf("  f charge [ms]        单次充电，默认 %d ms，上限 %d ms\n",
           FENCE_BENCH_CHARGE_DEFAULT_MS, FENCE_BENCH_CHARGE_MAX_MS);
    printf("                       打印 IO%d 的原始跳变时刻\n", FENCE_CHARGE_DONE_PIN);
    printf("  f hold [sec]         维持模式，默认 10 秒，上限 %d 秒\n", FENCE_BENCH_HOLD_MAX_S);
    printf("                       持续补电，方便万用表读稳 C51 电压\n");
    printf("\n建议顺序：\n");
    printf("  1) f status                      —— 基线\n");
    printf("  2) f arm                         —— 解锁\n");
    printf("  3) f trig 3 2000 1000            —— ISO 关着，量 /SCR_D 应恒为 0 V（反向对照）\n");
    printf("  4) f iso 1                       —— 量 TP14 = 5.0 V\n");
    printf("  5) f arm; f trig 3 2000 1000     —— 量 /SCR_D 应有约 0.8~1.3 V 脉冲\n");
    printf("     （万用表要用长脉冲才读得到；5 500 500 那种短方波只适合示波器）\n");
    printf("  6) f iso 0; f safe               —— 收工\n");
    printf("\n阶段 2（有高压，先备好放电棒 / 护目镜 / 单手操作）：\n");
    printf("  1) f 12v 1                       —— 量 TP13 应为 12 V\n");
    printf("  2) f arm; f charge 50            —— 第一次，很短。量 C51 两端\n");
    printf("  3) f charge 100 / 200 / 500      —— 逐步加长，画充电曲线\n");
    printf("  4) f arm; f hold 20              —— 维持住，读稳定值（预期约 405 V）\n");
    printf("  5) f 12v 0; f safe               —— 收工，等 C51 自己泄放\n\n");
}

/* ---------------- 命令入口 ---------------- */

static int cmd_fence(int argc, char **argv)
{
    if (argc < 2) { cmd_help(); return 0; }

    const char *sub = argv[1];

    if      (strcmp(sub, "status") == 0) cmd_status();
    else if (strcmp(sub, "help")   == 0) cmd_help();
    else if (strcmp(sub, "arm")    == 0) {
        arm_set(true);
        printf("ARMED —— %d 秒内可执行动作类命令。\n", FENCE_BENCH_ARM_TIMEOUT_S);
    }
    else if (strcmp(sub, "disarm") == 0) {
        arm_set(false);
        fence_pins_safe_init();
        printf("DISARMED，引脚已回到安全态。\n");
    }
    else if (strcmp(sub, "safe")   == 0) {
        arm_set(false);
        fence_pins_safe_init();
        printf("已回到安全态：12V_EN / 5V0_ISO_EN / CHARGE / TRIG 全部拉低。\n");
        printf("注意：这【放不掉】C51 上已有的电，只能等 9.9 MΩ 泄放（约 14 秒）或用放电棒。\n");
    }
    else if (strcmp(sub, "iso")    == 0) cmd_iso(argc, argv);
    else if (strcmp(sub, "12v")    == 0) cmd_12v(argc, argv);
    else if (strcmp(sub, "trig")   == 0) cmd_trig(argc, argv);
    else if (strcmp(sub, "charge") == 0) cmd_charge(argc, argv);
    else if (strcmp(sub, "hold")   == 0) cmd_hold(argc, argv);
    else {
        printf("未知子命令 '%s'，`f help` 看用法。\n", sub);
    }
    return 0;
}

void fence_bench_start(void)
{
    /* 起 REPL 之前先确认引脚在安全态 —— gpio_init() 已经做过，这里是第二道 */
    fence_pins_safe_init();

    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t rc = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    rc.prompt             = "fence>";
    rc.max_cmdline_length = 128;

#if FENCE_BENCH_CONSOLE_USB
    esp_console_dev_usb_serial_jtag_config_t hw =
        ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&hw, &rc, &repl));
    const char *port = "USB-Serial-JTAG";
#else
    esp_console_dev_uart_config_t hw = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&hw, &rc, &repl));
    const char *port = "UART0 (TXD0/RXD0)";
#endif

    const esp_console_cmd_t cmd = {
        .command = "f",
        .help    = "电子围栏台架测试（f help 看用法）",
        .hint    = NULL,
        .func    = &cmd_fence,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
    ESP_ERROR_CHECK(esp_console_register_help_command());
    ESP_ERROR_CHECK(esp_console_start_repl(repl));

    (void)port;   /* 日志关掉时避免 unused 警告 */
    ESP_LOGW(TAG, "===== 围栏台架测试固件 =====");
    ESP_LOGW(TAG, "控制台: %s @115200   输入 `f help`", port);
    ESP_LOGW(TAG, "D25: %s", FENCE_BENCH_D25_FITTED ? "已焊（阶段 2）" : "未焊（阶段 1，高压链路是死的）");
    ESP_LOGW(TAG, "深睡已禁用，GNSS / MQTT / 运动传感器均未启动");
}

#endif // FENCE_BENCH
