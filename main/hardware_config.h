#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

/* =====================================================================
 * Cattle Tracker (Cattle_T) 硬件配置
 * ---------------------------------------------------------------------
 * 主控   : ESP32-S3-MINI-1 (U25)
 * 蜂窝   : Quectel EG800  (J10)  —— 经 TXB0108 (U11) 1V8/3V3 电平转换
 * GNSS   : LC760Z (U20)   —— 供电由 LP5907 (U21) 使能控制
 * 加速度 : LIS2DH12TR (U13) I2C
 * 电子围栏: LT3750 (U12) 电容充电 + BT151 可控硅 + PC817 隔离触发
 *
 * 本板没有 CAN / RS485 / IGN / Immobilizer 光耦，相关代码已在移植时裁剪。
 * ===================================================================== */

/* ---------------- 按键 / 指示灯 ---------------- */
#ifndef BUTTON_PIN
#define BUTTON_PIN 0                 // IO0  -> SW2 到 GND，内部上拉，低有效
#endif

#ifndef LED_ESP_PIN
#define LED_ESP_PIN 46               // IO46 -> D3 (ESP_LED)，高有效
#endif

/* IO0 按键手势参数（双击 -> 打开围栏供电轨） */
#ifndef BUTTON_POLL_MS
#define BUTTON_POLL_MS 10            // 轮询周期
#endif
#ifndef BUTTON_DEBOUNCE_MS
#define BUTTON_DEBOUNCE_MS 30        // 去抖时间
#endif
#ifndef BUTTON_DBLCLICK_WINDOW_MS
#define BUTTON_DBLCLICK_WINDOW_MS 400 // 两次按下之间的最大间隔
#endif
#ifndef BUTTON_LONGPRESS_MS
#define BUTTON_LONGPRESS_MS 2000     // 长按判定门限
#endif
#ifndef BUTTON_LONGPRESS_OFF
#define BUTTON_LONGPRESS_OFF 1       // 1 = 长按把围栏供电轨关回安全状态
#endif
#ifndef BUTTON_ACK_BLINK_TIMES
#define BUTTON_ACK_BLINK_TIMES 3     // 动作确认闪烁次数
#endif
#ifndef BUTTON_ACK_BLINK_ON_MS
#define BUTTON_ACK_BLINK_ON_MS 120
#endif
#ifndef BUTTON_ACK_BLINK_OFF_MS
#define BUTTON_ACK_BLINK_OFF_MS 120
#endif
// 自测开关：1 = 只跑工厂自测任务，0 = 跑正常业务
// 定义提前到这里，因为下面的双击动作要跟着它走
#ifndef SELFTEST
#define SELFTEST 0
#endif

/* ---- IO0 双击的动作 ----
 *   业务模式 (SELFTEST=0) → 手动向 MQTT 发一条 payload，完全不碰可控硅
 *   自测模式 (SELFTEST=1) → 由 SELFTEST_DBLCLICK_MQTT 决定：
 *        1 = 联网发一条 MQTT（用来在台面上验证模组联网和上报）
 *        0 = 打开围栏供电轨 + Trig 方波（可控硅测试）
 * 想要别的组合就显式覆盖下面两个宏（同时置 1 也能编，两件事会依次做）。 */
#ifndef SELFTEST_DBLCLICK_MQTT
#define SELFTEST_DBLCLICK_MQTT 1
#endif

#ifndef BUTTON_DBLCLICK_FENCE_TEST
#define BUTTON_DBLCLICK_FENCE_TEST   (SELFTEST && !SELFTEST_DBLCLICK_MQTT)
#endif
#ifndef BUTTON_DBLCLICK_MQTT_PUBLISH
#define BUTTON_DBLCLICK_MQTT_PUBLISH (!SELFTEST || SELFTEST_DBLCLICK_MQTT)
#endif

/* 长按 IO0 的动作。
 *   1 = 进入模组调试保持模式：把模组开起来之后 ESP32 彻底不碰它
 *       （不发 AT、不 power cycle、不 CFUN=0、不断轨），交给模组自己的
 *       USB 口去调试。只能靠复位退出。自测模式下默认打开。
 *   0 = 保持原行为，看 BUTTON_LONGPRESS_OFF（长按把围栏供电轨关回安全态）*/
#ifndef BUTTON_LONGPRESS_MODEM_DEBUG
#define BUTTON_LONGPRESS_MODEM_DEBUG SELFTEST
#endif

// 围栏动作里是否包含 Trig 方波（0 = 只开供电轨，不触发可控硅）
#ifndef FENCE_TRIG_TEST_AFTER_DBLCLICK
#define FENCE_TRIG_TEST_AFTER_DBLCLICK 1
#endif
#ifndef FENCE_TRIG_TEST_PULSES
#define FENCE_TRIG_TEST_PULSES 5           // 重复次数
#endif
#ifndef FENCE_TRIG_TEST_ON_MS
#define FENCE_TRIG_TEST_ON_MS 500          // 高电平时长
#endif
#ifndef FENCE_TRIG_TEST_OFF_MS
#define FENCE_TRIG_TEST_OFF_MS 500         // 低电平时长（500+500 = 1Hz）
#endif
#ifndef FENCE_TRIG_TEST_SETTLE_MS
#define FENCE_TRIG_TEST_SETTLE_MS 200      // 供电轨建立后等待多久再发 Trig
#endif

// pe_board 时期"按一下 IO0 就重启模组"的行为。IO0 现在用于双击手势，
// 两者同时打开会让双击顺带重启模组，因此默认关闭。
#ifndef BUTTON_LEGACY_MODEM_CYCLE
#define BUTTON_LEGACY_MODEM_CYCLE 0
#endif
#ifndef LED_CONN_PIN
#define LED_CONN_PIN 45              // IO45 -> D4 (CONN_LED)，高有效
#endif

// 兼容旧代码的命名（pe_board 时期的 led1 / led3）
#ifndef LED_PIN_1
#define LED_PIN_1 LED_ESP_PIN
#endif
#ifndef LED_PIN_3
#define LED_PIN_3 LED_CONN_PIN
#endif

/* ---------------- 电池采样 (IBL) ---------------- */
// VBAT --[R93 100K]-- IO1 --[R94 330K]-- GND
// 分压比 = (100K + 330K) / 330K = 1.303
#ifndef IBL_PIN
#define IBL_PIN 1                    // IO1 = ADC1_CH0
#endif
#define ADC1_CHANNEL ADC1_CHANNEL_0  // IO1 对应 ADC1_CHANNEL_0
#ifndef NO_OF_SAMPLES
#define NO_OF_SAMPLES 64
#endif
#ifndef VBAT_DIVIDER_RATIO
#define VBAT_DIVIDER_RATIO 1.303f    // (100k+330k)/330k
#endif
// 单节锂电池（CN3163 太阳能充电）电量估算区间
#ifndef VBAT_EMPTY_V
#define VBAT_EMPTY_V 3.30f
#endif
#ifndef VBAT_FULL_V
#define VBAT_FULL_V 4.15f
#endif

/* ---------------- 运动传感器 LIS2DH12TR ---------------- */
#ifndef MOTION_SENSOR_SDA_PIN
#define MOTION_SENSOR_SDA_PIN 14     // IO14 -> SDA (4K7 上拉)
#endif
#ifndef MOTION_SENSOR_SCL_PIN
#define MOTION_SENSOR_SCL_PIN 15     // IO15 -> SCL (4K7 上拉)
#endif
#ifndef MOTION_SENSOR_INT1_PIN
#define MOTION_SENSOR_INT1_PIN 17    // IO17 <- ACC_INT (LIS2DH12 INT_1)
#endif

/* ---------------- GNSS (LC760Z, UART2) ---------------- */
#ifndef EX_UART_NUM
#define EX_UART_NUM UART_NUM_2
#endif
#ifndef BUF_SIZE
#define BUF_SIZE 1024
#endif

#define GNSS_UART_NUM EX_UART_NUM
#define GNSS_BAUD 115200
#define GNSS_UART_TX 7               // IO7  -> GNSS_RXD (ESP 发)
#define GNSS_UART_RX 6               // IO6  <- GNSS_TXD (ESP 收)

#ifndef GNSS_EN_PIN
#define GNSS_EN_PIN 11               // IO11 -> LP5907 EN，高=GNSS 供电
#endif
#ifndef GNSS_RST_PIN
// IO2 -> Q6(NPN) 基极 -> LC760Z RESET_N
// 注意是反相驱动：IO2 = 1 表示 RESET_N 被拉低（复位有效）
#define GNSS_RST_PIN 2
#endif
#ifndef GNSS_PPS_PIN
#define GNSS_PPS_PIN 8               // IO8  <- 1PPS
#endif
#ifndef GNSS_3DFIX_PIN
#define GNSS_3DFIX_PIN 5             // IO5  <- 3D_FIX
#endif
#ifndef GNSS_GEOFENCE_PIN
#define GNSS_GEOFENCE_PIN 3          // IO3  <- GEOFENCE
#endif
#ifndef GNSS_JAMIND_PIN
#define GNSS_JAMIND_PIN 4            // IO4  <- JAM_IND
#endif

// 兼容旧命名
#define LED_PIN_EN_GNSS  GNSS_EN_PIN
#define LED_PIN_RST_GNSS GNSS_RST_PIN

/* ---------------- 蜂窝模组 EG800 (UART1) ---------------- */
#ifndef UART_EG800_NUM
#define UART_EG800_NUM UART_NUM_1
#endif
// 兼容 pe_board 时期的命名
#ifndef UART_EG915U_NUM
#define UART_EG915U_NUM UART_EG800_NUM
#endif

// uart_set_pin(port, tx_io, rx_io, ...)
#define MODEM_UART_TX_PIN 42         // IO42 -> MODEM_RX_3V3 -> EG800 MAIN_RXD
#define MODEM_UART_RX_PIN 41         // IO41 <- MODEM_TX_3V3 <- EG800 MAIN_TXD

// 兼容旧命名（旧代码里 EG915U_RX_PIN 实际指"模组的 RX"，即 ESP 的 TX）
#define EG915U_RX_PIN MODEM_UART_TX_PIN
#define EG915U_TX_PIN MODEM_UART_RX_PIN

#ifndef QT_POWER_PIN
// IO33 -> Q8(NPN) -> EG800 PWRKEY（反相）：IO33 = 1 时 PWRKEY 被拉低
#define QT_POWER_PIN 33
#endif
#ifndef MODEM_RESET_PIN
// IO34 -> Q4(NPN) -> EG800 RESET_N（反相）：IO34 = 1 时复位有效
#define MODEM_RESET_PIN 34
#endif
#ifndef MODEM_EN_PIN
// IO48 -> U27(MIC29302, 3.8V LDO) 的 EN
#define MODEM_EN_PIN 48
#endif

/* MODEM_EN 的极性。
 *
 * 1 = 高有效（改版后的硬件）：IO48 直驱 U27 的 EN，EN 对地有下拉电阻。
 *     IO48 = 1 -> 3.8V 开；IO48 = 0 或悬空 -> 3.8V 关。
 *     关键好处：IO48 不是 RTC GPIO，deep sleep 期间必然变高阻，
 *     此时外部下拉把 EN 按在地上，3.8V 轨真正断开，不需要软件保持。
 *
 * 0 = 低有效（初版硬件）：IO48 -> Q5(KRC106S, 内含基极下拉) -> EN，R25 51k 上拉到 5V0。
 *     IO48 悬空 -> Q5 截止 -> EN 被拉高 -> 轨会自己恢复通电，软件按不住。
 *
 * 换回旧板只需把这里改成 0，其余代码不用动。 */
#ifndef MODEM_EN_ACTIVE_HIGH
#define MODEM_EN_ACTIVE_HIGH 1
#endif

#if MODEM_EN_ACTIVE_HIGH
#define MODEM_EN_ON_LEVEL   1
#define MODEM_EN_OFF_LEVEL  0
#else
#define MODEM_EN_ON_LEVEL   0
#define MODEM_EN_OFF_LEVEL  1
#endif

/* 3.8V 轨默认状态。
 * 1 = 开机即通电（初版行为，模组还没开机就白烧 LDO 静态电流）
 * 0 = 开机不通电，等到真要用模组（PWRKEY 之前）才开 —— 现在的默认 */
#ifndef MODEM_RAIL_ON_AT_BOOT
#define MODEM_RAIL_ON_AT_BOOT 0
#endif

// 开轨到 PWRKEY 之间的稳压建立时间
#ifndef MODEM_RAIL_SETTLE_MS
#define MODEM_RAIL_SETTLE_MS 100
#endif
#ifndef MODEM_STATUS_PIN
// IO35 <- STATUS_LED_3V3（EG800 STATUS 经 TXB0108 B8 输出），高=模组已开机
#define MODEM_STATUS_PIN 35
#endif
// 兼容旧命名：读取模组上电状态
#define EG915U_POWER MODEM_STATUS_PIN

// 硬件流控 / 状态线（当前固件未使用，保留定义）
#define MODEM_RTS_PIN 36             // IO36
#define MODEM_CTS_PIN 37             // IO37
#define MODEM_DCD_PIN 38             // IO38
#define MODEM_RI_PIN  39             // IO39
#define MODEM_DTR_PIN 40             // IO40

/* ---------------- 电子围栏高压模块 ---------------- */
// 注意：上电后必须保持安全状态（充电关闭、不触发）。
// 本次移植只做引脚安全初始化，不实现放电逻辑。
#ifndef FENCE_12V_EN_PIN
#define FENCE_12V_EN_PIN 47          // IO47 -> 12V_EN
#endif
#ifndef FENCE_5V_ISO_EN_PIN
#define FENCE_5V_ISO_EN_PIN 12       // IO12 -> 5V0_ISO_EN
#endif
#ifndef FENCE_CHARGE_EN_PIN
#define FENCE_CHARGE_EN_PIN 9        // IO9  -> LT3750 CHARGE（高有效）
#endif
#ifndef FENCE_CHARGE_DONE_PIN
#define FENCE_CHARGE_DONE_PIN 10     // IO10 <- LT3750 DONE#（低=充满，100K 上拉）
#endif
#ifndef FENCE_TRIG_PIN
#define FENCE_TRIG_PIN 13            // IO13 -> PC817 -> 可控硅触发（高=放电）
#endif

/* ---------------- 工厂治具 UART0 ---------------- */
// ESP32-S3 默认 U0TXD=GPIO43 / U0RXD=GPIO44，板上引到 TP1 / TP10
#ifndef U0TXD_PIN
#define U0TXD_PIN 43
#endif
#ifndef U0RXD_PIN
#define U0RXD_PIN 44
#endif

/* ---------------- 功能开关 ---------------- */
#ifndef ENABLE_MQTT
#define ENABLE_MQTT 1
#endif

#ifndef ENABLE_GNSS_LP
#define ENABLE_GNSS_LP 0
#endif

// 自测开关 SELFTEST 已在本文件前面（按键手势那一段之前）定义，
// 因为 FENCE_TRIG_TEST_AFTER_DBLCLICK 要跟随它。改值请改那一处。

// 是否使用产线治具：
//   0 = 独立模式。自测跑完直接在 USB-JTAG 调试口打印全部结果并点亮结果 LED，
//       不碰 UART0、不等 ACK（当前这版硬件没有治具，默认用这个）。
//   1 = 治具模式。跑完后在 UART0 (IO43/IO44) 等待 GUI 发送 !GUI_SELFTEST_ACK，
//       收到后再把 JSON 结果发回治具。
#ifndef SELFTEST_USE_JIG
#define SELFTEST_USE_JIG 0
#endif

// 治具模式下等待 ACK 的超时（毫秒）；0 = 一直等到收到为止
#ifndef SELFTEST_ACK_TIMEOUT_MS
#define SELFTEST_ACK_TIMEOUT_MS 15000
#endif

#ifndef UNUSED_ATTR
#define UNUSED_ATTR __attribute__((unused, used))
#endif

/* ---------------- GNSS 调试日志 ---------------- */
#ifndef GNSS_RAW_LOG_ENABLED
#define GNSS_RAW_LOG_ENABLED 0
#endif
#ifndef GNSS_UBX_LOG_ENABLED
#define GNSS_UBX_LOG_ENABLED 0
#endif
#ifndef GNSS_PARSE_LOG_ENABLED
#define GNSS_PARSE_LOG_ENABLED 0
#endif

#if GNSS_RAW_LOG_ENABLED
#define GNSS_RAW_LOG(fmt, ...) ESP_LOGI("GNSS_RAW", fmt, ##__VA_ARGS__)
#else
#define GNSS_RAW_LOG(...) do { } while (0)
#endif

#if GNSS_UBX_LOG_ENABLED
#define GNSS_UBX_LOG(fmt, ...) ESP_LOGI("GNSS_UBX", fmt, ##__VA_ARGS__)
#else
#define GNSS_UBX_LOG(...) do { } while (0)
#endif

#if GNSS_PARSE_LOG_ENABLED
#define GNSS_PARSE_LOG(fmt, ...) ESP_LOGI("GNSS", fmt, ##__VA_ARGS__)
#else
#define GNSS_PARSE_LOG(...) do { } while (0)
#endif

/* ---------------- 低功耗 / 上报周期 ---------------- */
#ifndef GNSS_LP_OFF_SEC
#define GNSS_LP_OFF_SEC 60
#endif
#ifndef GNSS_TTFF_TIMEOUT_SEC
#define GNSS_TTFF_TIMEOUT_SEC 120
#endif
#ifndef GNSS_LP_ON_HOLD_SEC
#define GNSS_LP_ON_HOLD_SEC 15
#endif
#ifndef GNSS_LP_METHOD
#define GNSS_LP_METHOD 2
#endif

/* ---- GNSS 低功耗（方案 E）----
 * LC760Z 属 LC76G 系列，用 $PAIR NMEA 命令，不是 UBX 二进制。
 *   1 = Backup/RTC ($PAIR650)，13 µA @3.3V，星历保留，RESET_N 唤醒后热启动 ~1 s
 *   0 = Standby   ($PAIR003)，$PAIR002 恢复
 * 两种都保持 U21 使能不动，VCC 与 V_BCKP 持续供电。 */
#ifndef GNSS_LP_USE_BACKUP
#define GNSS_LP_USE_BACKUP 1
#endif
// $PAIR650 的睡眠秒数，0 = 一直睡直到 RESET_N 唤醒（时序由 MCU 掌握，最可控）
#ifndef GNSS_BACKUP_SLEEP_SEC
#define GNSS_BACKUP_SLEEP_SEC 0
#endif
// 等待 $PAIR001 ACK 的超时（手册/论坛给的是 1 秒内回）
#ifndef GNSS_PAIR_ACK_TIMEOUT_MS
#define GNSS_PAIR_ACK_TIMEOUT_MS 1500
#endif
// CFG-SLEEP 的 ACK 等待时间。这条命令执行后模组立刻休眠，本来就发不出 ACK
// （同手册里 CFG-SIMPLERST 的软件复位"系统命令，无 ACK"），所以等一小会儿就够
#ifndef GNSS_SLEEP_ACK_TIMEOUT_MS
#define GNSS_SLEEP_ACK_TIMEOUT_MS 500
#endif
// 等待 Quectel 二进制 ACK-ACK 的超时
#ifndef GNSS_QBIN_ACK_TIMEOUT_MS
#define GNSS_QBIN_ACK_TIMEOUT_MS 1500
#endif
// 唤醒后等首条 NMEA 的超时
#ifndef GNSS_WAKE_TIMEOUT_MS
#define GNSS_WAKE_TIMEOUT_MS 5000
#endif

// 自测模式里跑 GNSS 低功耗协议探测（约 40 秒，输出一份完整报告）
// 确定了模组认哪条命令之后可以关掉
#ifndef GNSS_LP_SELFTEST
#define GNSS_LP_SELFTEST 1
#endif

/* ---- 方案 E：ESP32 deep sleep ----
 * 睡眠期把 ESP32 深睡，醒来等同复位、从 app_main 重新开始。
 * 注意 ESP32-S3 只有 IO0~IO21 是 RTC GPIO，深睡期间只有这些能靠 hold 保持电平。 */
#ifndef ENABLE_DEEP_SLEEP
#define ENABLE_DEEP_SLEEP 1
#endif
/* deep sleep 期间是否允许 IO0 按键唤醒（按下为低）。
 *
 * ⚠ 这个开关很贵：ext0 唤醒由 RTC_PERIPH 电源域实现，打开它这个域整个休眠期
 *   不能断电，实测要多耗约 400 uA——和当初那五个 gpio_hold_en() 是同一笔钱。
 *   深睡底噪 112 uA -> 约 510 uA，续航差 4.5 倍。
 *   （同样的原因，深睡引脚电平一律靠板上的电阻保持，不用 gpio_hold_en()，
 *     见 gpio_module.c 里 deep_sleep_prepare_pins() 上方的说明。）
 *
 *   周期 1 分钟时无所谓（醒着的时间占比本来就高）；
 *   周期拉到 10 分钟以上时，它会变成整机功耗的绝对大头。
 *
 *   替代方案：不挂按键唤醒，靠定时器醒来后在等定位阶段双击 IO0 手动上报。 */
#ifndef DEEPSLEEP_WAKE_ON_BUTTON
#define DEEPSLEEP_WAKE_ON_BUTTON 0
#endif


/* ================= 运动驱动的上报频率（阶段一） =================
 * 机制：深睡时 LIS2DH12 以 10 Hz 低功耗跑着并锁存中断（约 3 µA）。
 * 醒来先读一个 bit「整个睡眠窗口动过没有」，动过才花 5 秒采样数次数。
 *
 *   次数 >= 门限        -> 有动作：mode = HIGH，streak 清零
 *   锁存位 = 0          -> 确定静止：streak++
 *   锁存位 = 1 但次数不够 -> 存疑：streak 不变（连续存疑太多次才算静止）
 *   streak >= 3         -> mode = LOW
 *
 * HIGH / LOW 只影响唤醒周期；两种模式下都照常定位，定到位就上报。 */

#ifndef ENABLE_TRACKING_POLICY
#define ENABLE_TRACKING_POLICY 1      // 0 = 退回原来的固定周期行为
#endif

/* 唤醒周期（分钟）。
 * 测试期两者都设成 3，先验证状态机逻辑；验证完再拉开
 * （计划：HIGH -> 1，LOW -> 15 或 30）。 */
/* 固定唤醒周期（秒）。当前阶段不区分高低频：每 TRACK_CYCLE_SEC 醒一次，
 * 读一次锁存位做判定，只有判定为「有动作」才唤醒 GNSS 去定位上报。 */
/* ---- 醒着时长的压缩（静止轮次的功耗几乎全在这里）----
 * 静止不动时每轮要醒 7.2 s 却什么都不做，占空比 24%，这是现在的功耗大头。
 * 下面这些都是可调的延时上限；如果压得太短导致静止误判成"动过"，
 * 或者 GNSS 没能回到 Backup（睡眠电流从 150 µA 跳到 40 mA），
 * 把对应的值调回括号里的原值即可，不用改代码。 */

/* 深睡唤醒时跳过 I2C 总线扫描（纯诊断，约 430 ms）。
 * 设备识别由 lis2dh12_check_device_id() 独立完成，不依赖扫描。
 * 置 1 则每轮都扫。 */
#ifndef MOTION_I2C_SCAN_ON_WAKE
#define MOTION_I2C_SCAN_ON_WAKE 0
#endif

/* 深睡唤醒时加速度计从未掉电，不需要再等稳定（原 200 ms） */
#ifndef MOTION_INIT_SETTLE_MS
#define MOTION_INIT_SETTLE_MS 200
#endif

/* 睡眠布防各段延时。SETTLE 是给高通滤波器收敛的，压太短会让
 * 布防完立刻误锁存 —— 静止被判成"动过"就是这里出问题。（原 500 / 200 / 120） */
#ifndef MOTION_ARM_SETTLE_MS
#define MOTION_ARM_SETTLE_MS 300
#endif
#ifndef MOTION_ARM_REF_WAIT_MS
#define MOTION_ARM_REF_WAIT_MS 120
#endif
#ifndef MOTION_ARM_CLEAR_STEP_MS
#define MOTION_ARM_CLEAR_STEP_MS 100
#endif

/* 跳过轮次确认 GNSS 是否还在 Backup 的探测窗口。
 * GNSS 醒着时是 26~56 句/秒，400 ms 能看到 10~22 句，足够判定。
 * 探测到 0 句就认为仍在 Backup，跳过整套 CFG-SLEEP 流程（省约 4 s）。 */
#ifndef GNSS_STANDBY_PROBE_MS
#define GNSS_STANDBY_PROBE_MS 400
#endif

#ifndef TRACK_CYCLE_SEC
#define TRACK_CYCLE_SEC 30
#endif
#ifndef TRACK_FIX_TIMEOUT_S
#define TRACK_FIX_TIMEOUT_S 90
#endif

/* 下面这两组是 HIGH/LOW 双档用的，当前被 TRACK_CYCLE_SEC 取代，
 * 逻辑还在 tracking_policy.c 里（streak/doubt 照常维护），只是不再影响周期。 */
#ifndef TRACK_HIGH_PERIOD_MIN
#define TRACK_HIGH_PERIOD_MIN 1
#endif
#ifndef TRACK_LOW_PERIOD_MIN
#define TRACK_LOW_PERIOD_MIN  1
#endif

/* 定位超时（秒） */
#ifndef TRACK_FIX_TIMEOUT_HIGH_S
#define TRACK_FIX_TIMEOUT_HIGH_S 90
#endif
#ifndef TRACK_FIX_TIMEOUT_LOW_S
#define TRACK_FIX_TIMEOUT_LOW_S  90
#endif

/* ---- 醒着时的 5 秒采样窗口 ---- */
#ifndef MOTION_WINDOW_MS
#define MOTION_WINDOW_MS 5000
#endif
/* 主判据的门限：相邻两个采样点的加速度矢量差 ‖a[n] − a[n-1]‖ 超过这个值，
 * 这一帧算「在动」。单位 mg，采样间隔 MOTION_SAMPLE_PERIOD_MS。
 *
 * 为什么用「相邻帧差」而不是「模长偏离 1g」：
 *   ‖a‖ 是旋转不变量。板子绕轴翻转/摇晃时重力只是在三个轴之间重新分配，
 *   模长始终 ≈1 g，|‖a‖−1g| 几乎为 0 —— 用力晃动实测峰值只有 206 mg、
 *   活动占比 4%，完全漏判。牛低头/甩头/走路同样以旋转为主。
 *   帧差把姿态变化也算进来，旋转和平移都抓得到。
 *
 * 量级参考（50 Hz 采样）：静止噪声 <10 mg，手持慢晃 100~400 mg，
 * 用力摇 500 mg 以上。60 mg 是起点，现场标定。 */
#ifndef MOTION_DELTA_TH_MG
#define MOTION_DELTA_TH_MG 60
#endif

/* 主判据：5 秒窗口内帧差的【平均值】(mg)。超过这个值判为「有动作」。
 *
 * 为什么不用「超阈值样本占比」：占比丢掉了幅度信息，而且帧差的能量集中在
 * 运动换向的瞬间 —— 用力晃动实测占比也只有 19%，逐秒 22/14/18/26/18 很均匀，
 * 说明不是断断续续，而是这个统计量本身的上限就在 20% 附近。
 * 用平均帧差就没有这个问题：它对幅度和持续时间都单调，一个阈值就够。
 *
 * 量级参考（50 Hz 采样、HR 模式）：
 *   传感器噪声底 ~4 mg（静止时就是这个数，不是 0）
 *   轻微摇头/碰一下   10~15 mg
 *   手持用力晃动      20~40 mg
 * 标定方法：静止跑一轮记下平均值 a，真实运动跑一轮记下 b，取 sqrt(a*b)。 */
#ifndef MOTION_MEAN_DELTA_TH_MG
#define MOTION_MEAN_DELTA_TH_MG 20
#endif
/* 旧判据 |‖a‖ − 1g| 的门限，现在只用于日志里的对照输出，不参与判定。 */
#ifndef MOTION_EVENT_TH_MG
#define MOTION_EVENT_TH_MG 120
#endif
/* 去抖：两次计数之间至少间隔这么久。200 ms 等于上限 5 Hz，
 * 高于任何真实步频，防止一次晃动被数成好几次。 */
#ifndef MOTION_EVENT_DEBOUNCE_MS
#define MOTION_EVENT_DEBOUNCE_MS 200
#endif
/* 「有动作」的主判据：5 秒窗口里超过阈值的样本占比（%）。
 * 换掉原来的「上升沿次数」—— 那个判据靠迟滞复位，持续晃动时信号一直
 * 待在迟滞下沿之上，above 永不复位，整段晃动只数到 1 次。
 * 占比天生区分「持续 vs 瞬态」，不需要靠去抖和迟滞去凑：
 *   持续走路/晃动 60~90%  |  摇头、敲一下 5~15%  |  静止 0%
 * 现场标定就调这个值。 */
#ifndef MOTION_ACTIVE_PCT_THRESHOLD
#define MOTION_ACTIVE_PCT_THRESHOLD 30
#endif

/* 采样间隔。传感器 ODR 开到 100 Hz、这里每 20 ms 取一次，
 * 保证每次读到的都是新样本（帧差判据下读到重复样本会被算成「没动」）。
 * 5 秒窗口 = 250 个点。 */
#ifndef MOTION_SAMPLE_PERIOD_MS
#define MOTION_SAMPLE_PERIOD_MS 20
#endif

/* 上升沿次数只作诊断输出，不再参与判定（保留是为了和历史日志对照） */
#ifndef MOTION_COUNT_THRESHOLD
#define MOTION_COUNT_THRESHOLD 5
#endif

/* ---- 深睡布防（锁存中断）---- */
/* 锁存阈值要比上面的事件阈值更灵敏 —— 它只负责回答「有没有动过」，
 * 宁可多锁存（进入存疑分支再采样确认），也不要漏。
 * ±2g 量程下 INT1_THS 是 16 mg/LSB，96 mg -> THS = 6。 */
#ifndef MOTION_LATCH_THS_MG
#define MOTION_LATCH_THS_MG 96
#endif
/* 0 = 任何一次超阈值立刻锁存。1 LSB = 1/ODR = 100 ms @10Hz。 */
#ifndef MOTION_LATCH_DUR_MS
#define MOTION_LATCH_DUR_MS 0
#endif

/* ---- 状态机 ---- */
#ifndef TRACK_NO_MOTION_STREAK
#define TRACK_NO_MOTION_STREAK 3      // 连续几轮无动作退回 LOW
#endif
/* 连续这么多轮「存疑」（锁存说动过但次数不够）也当作一轮静止。
 * 没有这个上限的话，一头每轮都甩一下头的牛会永远卡在 HIGH。 */
#ifndef TRACK_DOUBT_MAX
#define TRACK_DOUBT_MAX 5
#endif

/* ---------------- 模组开机 / 联网时序 ----------------
 * 下面这些是**超时上限**，不是固定等待。
 * 原来的代码里有一堆 vTaskDelay 盲等（开机后 2 s、清 URC 3 s、"等模组稳定" 3 s），
 * 现在全部换成等事件：等 RDY、等 AT 回 OK、轮询 CPIN/CEREG 直到就绪。
 * 正常情况下远远用不到这些上限。 */

/* PWRKEY 脉冲宽度。
 * 实测结论（本板 EG800，rail-on 到 RDY 的总时间固定约 2.29 s）：
 *   脉冲 2000 ms -> 释放后 290 ms 收到 RDY，模组开机段合计 2490 ms
 *   脉冲 1000 ms -> 释放后 1290 ms 收到 RDY，模组开机段合计 2490 ms
 *   脉冲  600 ms -> 开不了机
 * 即 RDY 的时刻由 PWRKEY 的【下降沿】决定，与脉冲宽度无关，
 * 缩短脉冲一秒也省不下来。所以这里取回 2000 ms —— 时间代价为零，
 * 离 600 ms 那个失败点的余量最大。不要再为了'省时间'调小它。 */
#ifndef MODEM_PWRKEY_PULSE_MS
#define MODEM_PWRKEY_PULSE_MS 2000
#endif
#ifndef MODEM_PWRKEY_PULSE_RETRY_MS
#define MODEM_PWRKEY_PULSE_RETRY_MS 2000
#endif
// 脉冲之后等模组吐 "RDY"（取代原来盲等 2 秒）；等不到也不致命，后面 AT 轮询兜底
#ifndef MODEM_BOOT_RDY_TIMEOUT_MS
#define MODEM_BOOT_RDY_TIMEOUT_MS 8000
#endif
// 清开机 URC：连续静默这么久就认为模组说完了（取代原来死等 3000 ms）
#ifndef MODEM_DRAIN_QUIET_MS
#define MODEM_DRAIN_QUIET_MS 300
#endif
#ifndef MODEM_DRAIN_MAX_MS
#define MODEM_DRAIN_MAX_MS 3000
#endif
// 反复发 AT 等模组应答的上限（取代原来 5 次握手 + 盲等 3 秒"让模组稳定"）
#ifndef MODEM_AT_READY_TIMEOUT_MS
#define MODEM_AT_READY_TIMEOUT_MS 10000
#endif
// 轮询 AT+CPIN? 等 SIM 就绪的上限
#ifndef MODEM_SIM_TIMEOUT_MS
#define MODEM_SIM_TIMEOUT_MS 8000
#endif
/* 轮询 AT+CEREG? 等注册上网的上限。
 * 原代码没有这一步，没注上网就直接发 CGACT，会一路卡到 CGACT 的 30 秒超时。 */
#ifndef MODEM_REG_TIMEOUT_MS
#define MODEM_REG_TIMEOUT_MS 60000
#endif

// 等 AT+QPOWD 回 "POWERED DOWN" 的超时
#ifndef MODEM_QPOWD_TIMEOUT_MS
#define MODEM_QPOWD_TIMEOUT_MS 15000
#endif

#ifndef GPS_CYCLE_SLEEP_MIN
#define GPS_CYCLE_SLEEP_MIN 1
#endif
#ifndef GPS_FIX_TIMEOUT_SEC
#define GPS_FIX_TIMEOUT_SEC 60
#endif
#ifndef MQTT_SEND_MAX_RETRY
#define MQTT_SEND_MAX_RETRY 3
#endif

// 定位成功、准备上云前 CONN_LED (IO45) 的闪烁提示
#ifndef GPS_UPLOAD_BLINK_TIMES
#define GPS_UPLOAD_BLINK_TIMES 2
#endif
#ifndef GPS_UPLOAD_BLINK_ON_MS
#define GPS_UPLOAD_BLINK_ON_MS 150
#endif
#ifndef GPS_UPLOAD_BLINK_OFF_MS
#define GPS_UPLOAD_BLINK_OFF_MS 150
#endif

/* MQTT 主题统一在 mqtt_config.h 定义，避免宏重复定义 */

#endif // HARDWARE_CONFIG_H
