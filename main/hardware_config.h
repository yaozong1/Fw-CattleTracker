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

/* ---- IO0 双击的动作，两种模式互补 ----
 *   SELFTEST=1 → 打开围栏供电轨 + Trig 方波（可控硅测试），不发 MQTT
 *   SELFTEST=0 → 手动向 MQTT 发一条 payload，完全不碰可控硅
 * 想要别的组合就显式覆盖这两个宏（同时置 1 也能编，两件事会依次做）。 */
#ifndef BUTTON_DBLCLICK_FENCE_TEST
#define BUTTON_DBLCLICK_FENCE_TEST SELFTEST
#endif
#ifndef BUTTON_DBLCLICK_MQTT_PUBLISH
#define BUTTON_DBLCLICK_MQTT_PUBLISH (!SELFTEST)
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
