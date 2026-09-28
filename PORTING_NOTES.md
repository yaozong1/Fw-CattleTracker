# pe_board → Cattle_T 移植说明

依据 `Cattle_T.kicad_sch` 原理图，将 pe_board 固件（selftest + function code）移植到 Cattle Tracker 新板。

## 1. 引脚映射（ESP32-S3-MINI-1, U25）

| 功能 | 网络名 | Cattle_T | pe_board |
|---|---|---|---|
| 按键 / BOOT | IO0 | IO0 | IO0 |
| 电池分压 (IBL) | IBL | IO1 (ADC1_CH0) | IO1 电池 + IO2 IBL |
| GNSS RESET（经 Q6 反相） | GNSS_RESET | IO2 | IO47 |
| GNSS GEOFENCE | GEOFENCE | IO3 | — |
| GNSS JAM_IND | JAM_IND | IO4 | — |
| GNSS 3D_FIX | 3D_FIX | IO5 | — |
| GNSS TXD → ESP RX | GNSS_TXD | IO6 | IO45 |
| ESP TX → GNSS RXD | GNSS_RXD | IO7 | IO42 |
| GNSS 1PPS | 1PPS | IO8 | — |
| 围栏充电使能 | C_EN | IO9 | — |
| 围栏充电完成(低有效) | C_DONE | IO10 | — |
| GNSS 供电使能 (LP5907 EN) | GNSS_EN | IO11 | IO46 |
| 隔离 5V 使能 | 5V0_ISO_EN | IO12 | — |
| 围栏放电触发 | Trig | IO13 | — |
| I2C SDA | SDA | IO14 | IO14 |
| I2C SCL | SCL | IO15 | IO15 |
| 加速度中断 | ACC_INT | IO17 | — |
| 12V 使能 | 12V_EN | IO47 | — |
| 模组 PWRKEY（经 Q8 反相） | PWRKEY | IO33 | IO18 |
| 模组 RESET（经 Q4 反相） | NRESET | IO34 | — |
| 模组供电使能（经 Q5，**低有效**） | Modem_EN | IO48 | — |
| 模组 STATUS（3V3 侧） | STATUS_LED_3V3 | IO35 | IO48 (输入) |
| 模组 RTS/CTS/DCD/RI/DTR | — | IO36..IO40 | — |
| 模组 TXD → ESP RX | MODEM_TX_3V3 | IO41 | IO11 |
| ESP TX → 模组 RXD | MODEM_RX_3V3 | IO42 | IO12 |
| CONN_LED | CONN_LED | IO45 | — |
| ESP_LED | ESP_LED | IO46 | IO2/IO3 |
| 工厂治具 UART0 | TXD0 / RXD0 | IO43 / IO44 (TP1/TP10) | 同 |

## 2. 主要改动

- **`hardware_config.h` 整体重写**，按上表定义引脚，并保留一批兼容宏（`LED_PIN_1/3`、`EG915U_*`、`UART_EG915U_NUM`、`read_io2_*`），旧代码不用逐处改名。
- **删除 CAN / RS485**：`can_module.c`、`rs485_module.c` 从 `main/CMakeLists.txt` 移除（文件保留但不参与编译），`main.c`、`self_test.c`、`eg915_selftest.c` 中的相关测试与主循环收发逻辑一并删除。
- **删除 IGN / Immobilizer 光耦**相关代码（`ign_test_detect_transition`、`im_test_toggle_task`）。
- **模组电源时序**：EG800 的 3.8V 由 IO48 经 Q5 控制，**低电平才是打开**。`gpio_init()` 与 `modem_power_cycle()` 都会先把 IO48 拉低再脉冲 PWRKEY。
- **模组串口方向已核对**：`uart_set_pin(port, tx_io, rx_io, ...)`，现在显式用 `MODEM_UART_TX_PIN=42` / `MODEM_UART_RX_PIN=41`（pe_board 的宏名与注释是反的，已在注释中说明）。
- **电池换算**：IO1 分压比改为 `(100K+330K)/330K = 1.303`；SOC 曲线从 12V 铅酸改为单节锂电（3.30–4.15 V），`mqtt_module.c` 内部那份拷贝也一起改了。
- **电子围栏引脚**：`fence_pins_safe_init()` 在 `gpio_init()` 最早阶段把 `Trig` / `C_EN` / `12V_EN` / `5V0_ISO_EN` 全部拉低，自测只读状态、**不触发放电**。充放电逻辑没有实现——那是新功能，不在这次移植范围内。
- **自测项**：EG800 AT + IMEI + ICCID、LIS2DH12、GNSS UART（含 3D_FIX / JAM_IND 电平）、电池/IBL、LED 闪烁、围栏安全状态。
- **自测输出模式**由 `hardware_config.h` 里的 `SELFTEST_USE_JIG` 控制：
  - `0`（默认，当前这版硬件）：跑完直接在 USB-JTAG 调试口打印格式化表格 + 文字汇总 + JSON，并点亮结果 LED（全通过 = ESP_LED，有失败 = CONN_LED），完全不碰 UART0，不等待任何握手。`idf.py monitor` 即可看到全部结果。
  - `1`：恢复产线治具流程，在 UART0 (IO43/IO44) 等 `!GUI_SELFTEST_ACK` 再回 JSON；等待超时由 `SELFTEST_ACK_TIMEOUT_MS` 控制（默认 15000 ms，设 0 = 一直等）。超时后按独立模式继续，不会再卡死。
  - 注意 **JSON 字段相对 pe_board 变了**，治具 GUI 启用治具模式时需要同步更新。
- 工程名 `pe_board` → `cattle_tracker`；MQTT ClientID / 主题前缀改为 `cattle/CT-001`。

## 2.1 IO0 按键手势（`button_module.c`，新增）

| 操作 | 动作 |
|---|---|
| **双击** | 先闪 3 下 ESP_LED 表示手势已识别，之后按模式二选一：<br>**SELFTEST=1** → `5V0_ISO_EN`(IO12) + `12V_EN`(IO47) 拉高 → 等 200 ms → `Trig`(IO13) 发 1 Hz 方波 5 个周期，结束停在低电平。**不发 MQTT**。<br>**SELFTEST=0** → 触发一次**手动 MQTT 上报**（异步，不管有没有定位）。**完全不碰可控硅，也不开供电轨**。 |
| **长按 ≥2 s** | 两路供电轨拉回低电平（安全状态），ESP_LED 长闪 1 下 |
| 单击 | 忽略 |

- **`C_EN` (IO9) 全程保持低电平**，LT3750 不充电、储能电容没有能量，所以 Trig 方波只是在空载下驱动 PC817 和可控硅门极，适合没接变压器时验证这段电路。每个沿都有串口日志，方便对照示波器。
- **两个动作互补，由 `SELFTEST` 自动切换**：

  ```c
  #define BUTTON_DBLCLICK_FENCE_TEST     SELFTEST     // 自测模式才测可控硅
  #define BUTTON_DBLCLICK_MQTT_PUBLISH   (!SELFTEST)  // 业务模式才手动上报
  ```

  想要别的组合就显式覆盖这两个宏，同时置 1 也能编（两件事依次做），同时置 0 则双击只闪灯。
- `FENCE_TRIG_TEST_AFTER_DBLCLICK`(1) 退化为围栏动作内部的子开关：置 0 就只开供电轨、不触发可控硅。
- Trig 测试参数：`FENCE_TRIG_TEST_PULSES`(5)、`FENCE_TRIG_TEST_ON_MS`(500)、`FENCE_TRIG_TEST_OFF_MS`(500)、`FENCE_TRIG_TEST_SETTLE_MS`(200)。
- 注意 `SELFTEST` 的定义已移到 `hardware_config.h` 靠前的位置（按键手势那一段之前），因为双击开关要引用它。
- 按键参数：`BUTTON_DBLCLICK_WINDOW_MS`(400)、`BUTTON_LONGPRESS_MS`(2000)、`BUTTON_DEBOUNCE_MS`(30)、`BUTTON_ACK_BLINK_*`。不想要长按关闭就把 `BUTTON_LONGPRESS_OFF` 置 0。
- ⚠️ 接上变压器和储能电容之后，这个 Trig 方波就会变成真正的放电动作。上高压前请先把 `FENCE_TRIG_TEST_AFTER_DBLCLICK` 置 0。
- 闪烁结束后会恢复 LED 原状态，不会盖掉自测的 PASS/FAIL 指示。
- 自测模式和正常运行模式都会启动这个任务。
- **注意**：pe_board 原来在 100 ms 定时器里挂了"按 IO0 就重启模组"的钩子，和双击手势冲突（双击会顺带重启模组），已用 `BUTTON_LEGACY_MODEM_CYCLE` 关闭（默认 0）。要恢复就置 1。
- 提醒：IO0 同时是 BOOT 引脚，**复位瞬间**按住会进下载模式；运行中按不受影响。

## 2.2 上报链路：与 fleet manager 对齐

后端代码：`process-engineering-fleet_test/src/services/mqtt-service.ts`

- 订阅：`fleet/+/battery`、`fleet/+/status`、`fleet/chargenode/+`（QoS 1）
- **设备 ID 取自主题第二段**（`topicParts[1]`），payload 里的 `deviceId` 后端不读
- **时间戳用服务器的 `Date.now()`**，payload 里的 `timestamp` 后端不读
- 电池字段全部读**顶层**：`soc / voltage / temperature / health / cycleCount / estimatedRangeKm / chargingStatus / alerts`，另加嵌套的 `gps.{lat,lng,speed,heading,altitude,accuracy}`

固件相应改为：

- 主题 `fleet/<IMEI>/battery`，运行时由 `mqtt_build_topic()` 拼出（`mqtt_config.h` 只留 `MQTT_TOPIC_ROOT` 和几个后缀）
- payload 拍平，`estimatedRange` 改名 `estimatedRangeKm`，补上 `alerts: []`，去掉换行缩进省流量（286 字节）
- payload 和主题都挪到"联网成功之后"再构造，确保 IMEI 已就位

**仍未对齐：broker。** 后端连的是 HiveMQ Cloud（`.env.local` 的 `NEXT_PUBLIC_MQTT_URL`，TLS 8884），固件连的是 `processengineeringsz.com:1883` 明文。两边不在同一个 broker 上，真机数据到不了后端。HiveMQ Cloud 不接受明文，固件要连上去需要在 EG800 上配 SSL（`AT+QMTCFG="SSL"` + `AT+QFUPL` 上传 CA + `AT+QSSLCFG`）。

## 2.2.1 手动上报（IO0 双击）

`mqtt_manual_publish_request()` 起一个一次性任务 `manual_pub`（栈 6144，优先级 7）：

1. 抢模组总线互斥锁，**不等待**——周期任务正在上报就直接放弃本次（CONN_LED 长亮一下提示）
2. 采空载电池电压 → CONN_LED 闪 2 下
3. `modem_is_powered()` 为假才 PWRKEY 上电，等 2 秒
4. 走和周期任务完全相同的 `publish_battery_payload()`
5. 结果指示：成功 CONN_LED 闪 3 下，失败长亮 1 秒
6. 只在"这次是自己开的模组"时才 `CFUN=0`，不打断周期任务的状态
7. 放锁

要点：

- **独立任务**的原因有两个：整个流程最坏几分钟，不能占着按键任务；MQTT 那串函数的局部缓冲区加起来超过 2.5 KB，按键任务的 3072 栈不够。
- **模组总线互斥**（`s_modem_mutex`，`mqtt_init()` 里创建）：周期任务从上电前一直持有到 `CFUN=0` 之后才放，手动任务以 0 超时去抢，抢不到就跳过。不加锁两个任务会同时往 UART1 发 AT，响应互相串味。
- **没有定位时整段省掉 `gps` 字段**，而不是发 `0,0`。后端对 gps 缺失是容忍的（日志打 `GPS: none`），但 0,0 会在地图上画出一个几内亚湾的假点。带 gps 286 字节，不带 189 字节。
- 开关：`BUTTON_DBLCLICK_MQTT_PUBLISH`（默认 1）。

## 2.3 上报链路其他改动

- `deviceId` 绑定模组 IMEI：`mqtt_init_network()` 里 `ATE0` 之后调一次 `device_fetch_imei()`（`AT+GSN`，取 15 位数字），结果缓存，后续复用。取不到时回退为 `"CT-IMEI-UNKNOWN"`，云端一眼能看出是没读到而不是设备重名。
- `battery.voltage` 改为板上实测：IO1 (IBL) 的 ADC 读数 × 分压比 1.303。
- `battery.soc` 改为由电压换算，用的是 `battery_module.c` 里的单节锂电 OCV 查表 + 线性插值（12 个点，端点对齐 `VBAT_FULL_V` 4.15 / `VBAT_EMPTY_V` 3.30）。换电芯改这张表即可。
- 电压在**唤醒模组之前**采样（`battery_snapshot_idle()`）：模组注网/发射时的电流冲击会把电池拉低 0.1~0.3 V，带载读数换算出来的 SOC 会明显偏低。
- `mqtt_module.c` 里原来那两份 `read_battery_voltage` / `estimate_soc_from_voltage` 的私有拷贝已删除，统一走 `battery_module`。
- 仍是常量的字段：`temperature / health / cycleCount / estimatedRange / chargingStatus / altitude / accuracy`。
- 定位成功后在串口打印一段完整信息：UTC、经纬度、节速与 km/h、航向、卫星数 / HDOP / fixQ / 几维定位、以及本轮 TTFF。
- 上报前 CONN_LED (IO45) 闪 2 下，参数见 `GPS_UPLOAD_BLINK_*`。

## 3. 原理图上需要你确认的几点

1. **IO35 (STATUS_LED_3V3)** 在图上只出现在 ESP32 这一侧；TXB0108 的 B8 只接了 R13 下拉，两者没有连线。固件按"IO35 = 模组 STATUS 输入"来写，如果实际没连通，自测里的 `modem_status_pin` 会恒为 0（不影响 AT 判定）。
2. **IO47 (12V_EN)** 同样只出现在 ESP32 一侧；MT3608 (U18) 的 EN 经 R77 直接到 GND，也就是 12V 升压默认关闭且无法用软件打开。
3. **LIS2DH12 的 SDO/SA0 (pin3) 悬空**，I2C 地址不确定。驱动会自动在 0x19 / 0x18 之间探测，但建议硬件上加确定的上/下拉。
4. **IBL 分压上限**：VBAT 4.2 V 时 ADC 端约 3.22 V，超过 ESP32-S3 在 11dB 衰减下的线性区（约 3.1 V）。满电附近读数会偏低，如果要准确测满电电压，建议把 R94 改小（例如 270K，比值 1.37，满电约 3.06 V）。
5. **围栏充放电时序**（LT3750 CHARGE / DONE#、SCR 触发脉冲宽度、两次放电最小间隔）还没有规格，需要你给出后再实现 `fence_module`。
