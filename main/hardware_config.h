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
/* 双击之后的 Trig 方波测试（没接变压器时验证 PC817 / 可控硅驱动电路） */
#ifndef FENCE_TRIG_TEST_AFTER_DBLCLICK
#define FENCE_TRIG_TEST_AFTER_DBLCLICK 1   // 0 = 双击只开供电轨，不发 Trig
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
// IO48 -> Q5(NPN) -> U27(3.8V LDO) EN，EN 由 R25 默认上拉。
// 反相 + 低有效：IO48 = 0 -> 3.8V 供电打开；IO48 = 1 -> 关掉模组供电。
#define MODEM_EN_PIN 48
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

// 自测开关：1 = 只跑工厂自测任务
#ifndef SELFTEST
#define SELFTEST 1
#endif

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

#ifndef GPS_CYCLE_SLEEP_MIN
#define GPS_CYCLE_SLEEP_MIN 6
#endif
#ifndef GPS_FIX_TIMEOUT_SEC
#define GPS_FIX_TIMEOUT_SEC 300
#endif
#ifndef MQTT_SEND_MAX_RETRY
#define MQTT_SEND_MAX_RETRY 3
#endif

/* MQTT 主题统一在 mqtt_config.h 定义，避免宏重复定义 */

#endif // HARDWARE_CONFIG_H
