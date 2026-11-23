# Fw-CattleTracker

ESP32-S3 牛羊项圈：GNSS 定位 + LTE 上报 + 电子围栏高压块。
硬件代号 `Cattle_T`，原理图在同级的 `process-engineering-cattle-tracker`（KiCad）。

> 新会话从这里开始。按需读第 6 节的文档索引，不用一次全读。
>
> **最后核对：2026-11-20 / 分支 `p1-protocol`**（P1 第一步写完后更新）

---

## 1. ⚠ 当前仓库状态

**三个分支编出来的东西不一样，先看清在哪个分支：**

| 分支 | 编出来的固件 |
|---|---|
| `p1-protocol`（当前开发） | **生产固件**，`FENCE_BENCH = 0`，按 `MQTT_PROTOCOL.md`（ct/v1）上报 |
| `p0-bench` | P0 实验固件（`P0_BENCH = 1`），见那个分支里的 `P0_BENCH.md` |
| `connect-fleet` | 围栏台架测试固件（`FENCE_BENCH = 1`），旧的 `fleet/<IMEI>/battery` 上报 |

### 1.1 交接（每次切会话只更新这一节）

**上个会话停在哪**（2026-11-20）

- 和云端工程师分工：固件归我们，broker（TLS + 每台独立凭据）和正式后端归他。
  在他的 broker 就绪前，用 FleetManager（Y 盘，分支 `p1-protocol`）当测试平台。
- 定了协议 `MQTT_PROTOCOL.md`（英文为准）/ `MQTT_PROTOCOL_zh.md`：topic `ct/v1/{id}/…`，
  设备 ID = `CT-001` 这类编号。服务器只有刺激「总开关」，协议里没有任何触发放电的消息。
- **P1 第一步（上行）**：编号写入、status + telemetry、心跳、定位失败也上报、UTC 时间。
  FleetManager 加了 `/api/collar` 和 Cattle 标签页，已在本地跑通（用 MQTTX 发的假数据）。
- **P1 第二步（下行）**：连网后订阅 `config` / `stim` / `all/stim`，整条校验后存 NVS，
  `status.applied` / `status.err` 回执。FleetManager 加了 `/api/collar/control`（仅 admin）、
  设置面板、单台刺激开关（打开要确认）、全群总闸、审计日志 `collar:audit`。
- **两步都只在云端编译通过 + 主机单元测试（串口解析、校验逻辑），还没上板。**
- P0 实验固件在 `p0-bench` 分支，等台架测试（决定围栏判定放 GNSS 还是固件）。
- 围栏高压块带机已全部跑通（411 V / 4.84 ms，十连发约 50 ms），见 `FENCE_BRINGUP.md`。

**下一步（按建议顺序）**

1. **上板验证 P1**：写编号 → Cattle 页出现心跳 / 动作 / 定位失败；网页改配置 / 开关
   → 下次连网后显示 applied。重点看日志里 `recv/mode`、`+QMTSUB`、`+QMTRECV` 三处（未在 EG800 上验证过）。
2. P0 台架测试 → 定围栏方案 → `fence` 下行格式（协议 v1.1）。
3. OTA 分区（2MB → 8MB，要有线全擦重烧一次）。
4. 围栏边界测试：电池串 100 / 220 mΩ 跑 `f burst 10 1`，模拟老化/低温电芯 ESR。

**悬而未决**

- 出货的心跳间隔 / 检查周期（开发值 10 min / 60 s，下行做好后可远程改）。
- 加速度计中断唤醒（ext1）能否替代 60 s 轮询：等 P0 第 ① 项。
- 动作门限 20 mg 是台面上手晃标定的，挂到真牛身上必然要重调。

---

## 2. 构建

ESP-IDF **v5.3.2**，target `esp32s3`。

```
idf.py build
idf.py -p <PORT> flash monitor      # 退出 Ctrl+]
```

**坑**：`sdkconfig.defaults` 里**不能有中文注释**，会让 kconfig 解析失败、
`build/` 配置不出来。曾经因此被误判成"编译不过"。该文件已改名为
`_sdkconfig.defaults.disabled` 停用。

**日志双通道**：UART0 (GPIO43/44，模块丝印 TXD0/RXD0) 为主，
USB-Serial-JTAG 为次级。深睡会断开 USB 枚举，所以调试深睡循环时接 UART0。
（`CONFIG_ESP_CONSOLE_UART_DEFAULT` + `CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG`）

---

## 3. 三种固件形态

由 `hardware_config.h` 顶部的开关决定，`main.c` 里分支：

| 开关 | 形态 | 用途 |
|---|---|---|
| `FENCE_BENCH = 1` | 围栏台架 | 串口 `f` 命令集，手动充电/放电/连发 |
| `SELFTEST = 1` | 产测自检 | `eg915_selftest.c`，IO0 双击触发 |
| 两个都 0 | **生产固件** | 运动驱动的定位上报循环 |

生产固件内部还有第三个开关，它决定主循环的形态，比上表更影响实际行为：

| 开关 | 行为 |
|---|---|
| `ENABLE_TRACKING_POLICY = 1`（**必须**） | 运动驱动 + 心跳，见第 4 节 |
| `ENABLE_TRACKING_POLICY = 0` | P1 起**编译报错**：旧的固定周期路径已删除 |

`GPS_CYCLE_SLEEP_MIN`、`TRACK_CYCLE_SEC`、`TRACK_FIX_TIMEOUT_S` 在 P1 里**都不再生效**，
节奏参数一律走 `ct_cfg()`（默认值是 `hardware_config.h` 末尾的 `CT_CFG_*`）。

---

## 4. 生产固件的主循环

每 **`check_s`（默认 60 s）** 深睡唤醒一次。**深睡 = 复位**，RAM 全丢，
只有 `RTC_DATA_ATTR` 存活。上电 / 复位那一次先开 10 秒串口窗口（`id CT-001` 写编号，`go` 跳过）。

```
唤醒
 ├─ gpio_init()          ← 先写电平再切方向（见第 5 节）
 ├─ 读 LIS2DH12 的 INT1_SRC 锁存位   ← 必须在任何重配置之前读
 ├─ 锁存=0 → 判「确定静止」，跳过采样，直接睡
 └─ 锁存=1 → 5 秒运动采样（100Hz ODR / 20ms 取样）
      ├─ 平均帧差 >= motion_th_mg(20mg) → 「有动作」
      └─ 不够 → 「存疑」

track_decide() 决定上报原因：
  冷启动 → boot；有动作且距上次动作上报 >= motion_min_s(300) → motion；
  否则距上次上报 >= heartbeat_min(10) → heartbeat；其余不联网直接睡
要上报：按原因开 GNSS（motion/boot 90 s，heartbeat 30 s，0 = 不开）
  → 不管定没定到都上报（gps=null + fix_fail）→ ct_report()：status(retained) + telemetry
上报失败也重置计时（没信号时不每轮重试，免得耗光电池）
```

判据是**相邻帧的加速度矢量差** `‖a[n] − a[n−1]‖` 的平均值，不是 `|‖a‖ − 1g|`。
后者是旋转不变量，牛低头/甩头/走路都以旋转为主，会完全漏判（实测用力晃动
峰值只有 206 mg，活动占比 4%）。

决策逻辑全在 `tracking_policy.c` 的 `track_decide()`，是个纯函数，好测。

`mode` / `streak` / `doubt` 还在 `tracking_policy.c` 里维护，但已不影响任何行为，也不再上报，
只剩日志诊断用。

---

## 5. 硬件相关的坑（踩过的，别再踩）

### 5.1 GPIO 必须先写电平再切方向

深睡唤醒后输出锁存器复位为 0。先 `gpio_set_direction(OUTPUT)` 再
`gpio_set_level(1)`，引脚会先输出一次低电平毛刺。

曾经因此每次唤醒都把 GNSS 电源循环一次，表现为"第二轮睡眠停在 40 mA"。

### 5.2 `gpio_reset_pin()` 会打开内部上拉

它的实现是「输入 + 使能内部上拉(~45kΩ)」。对 IO9(LT3750 CHARGE) 和
IO13(围栏 TRIG) 这种没有外部下拉的"动作"脚，等于主动置位。
见 `gpio_module.c` 的 `fence_pin_force_low()`。

### 5.3 任何 RTC 域唤醒源 = +405 µA

`gpio_hold_en()` 和 `esp_sleep_enable_ext0_wakeup()` 都要求 RTC_PERIPH
整个休眠期供电。实测深睡电流 112 µA → 550 µA。

所以：**不用中断唤醒**，改成定时唤醒 + 读加速度计的锁存中断位。
深睡期间引脚电平靠**板上电阻**保证，不靠 hold。

### 5.4 `pdMS_TO_TICKS()` 会整除成 0

`CONFIG_FREERTOS_HZ = 100`，1 tick = 10 ms。`pdMS_TO_TICKS(5)` 和
`pdMS_TO_TICKS(2)` 都是 **0**。毫秒级以下的时序一律用 `esp_rom_delay_us()` 忙等。

### 5.5 LC760Z 的 Backup 模式只能硬件复位唤醒

`CFG-SLEEP` mode=3 进去之后，软件命令无效，必须给 `RESET_N` 一个脉冲。
`gnss_wake_from_standby2()` 要在 `do_gnss` 分支**无条件调用**，
上电和复位路径同样需要。

### 5.6 围栏电路不能用程控电源测

峰值 6.5 A / 3 ms，程控电源的引线电感、CC 恢复、环路带宽全都跟不上，
会给出系统性错误的结论。**必须用电池。** 详见 `FENCE_BRINGUP.md` 第 4 节。

---

## 6. 文档索引

| 文件 | 内容 |
|---|---|
| `FENCE_BRINGUP.md` | 围栏高压块：电路、验收数据、修掉的 bug、未完成项 |
| `POWER_BUDGET.md` | 功耗预算、深睡电流排查过程、供电架构问题、续航估算 |
| `固件调试操作手册.pdf` | **三种形态怎么切、每种下有哪些指令和操作**、现场判断速查 |
| `睡眠与运动唤醒机制.pdf` | 睡眠/唤醒机制的图文说明（给人看的版本） |
| `MQTT_PROTOCOL.md` | **设备与后端的协议契约**（ct/v1，英文为准，发给云端工程师的就是它） |
| `MQTT_PROTOCOL_zh.md` | 上面的中文对照版，改协议时两份一起改 |
| `PORTING_NOTES.md` | 从 pe_board 移植的记录。引脚映射表仍然准确，可当引脚速查用 |
| `围栏调试与固件架构.html` | 本轮会话的图文回顾：带机过程 + 软件架构图（给人看，不是给新会话读的） |

> `睡眠与运动唤醒机制.pdf` 的正文是最新的（30 s 周期、平均帧差判据、96 mg 硬件锁存都对），
> 但**第 10 节「待办」里的 IO9 下拉那条已经过期** —— 那颗电阻已经装了（实际用的 10 kΩ）。
> 以 `POWER_BUDGET.md` 1.2 节和本文件第 8 节为准。
>
> `BUSINESS_FLOW.md` 已删除：它描述的是改成运动驱动之前的 6 分钟固定周期流程，
> 里面的周期、定位超时、以及「唤醒源：定时器 + IO0 按键」全都不再成立，
> 最后那条还会误导人去打开 ext0（+405 µA）。内容已由第 4 节和那份 PDF 覆盖。

---

## 7. 模块地图（`main/`）

| 文件 | 职责 |
|---|---|
| `main.c` | 三种形态的分支入口 |
| `tracking_policy.c` | 运动判定的纯决策函数 + RTC 状态 |
| `motion_sensor.c` | LIS2DH12：睡眠布防、锁存读取、5 秒采样 |
| `gnss_module.c` | LC760Z：NMEA 解析、Backup 进出 |
| `mqtt_module.c` | EG800 AT 收发、注网、MQTT 发布（`mqtt_publish_ex` QoS1 + retain）、订阅收下行（`mqtt_subscribe_collect`）、周期任务 |
| `ct_proto.c` | ct/v1 协议：status / telemetry 的 JSON（cJSON）、UTC 校时、单调时钟、seq |
| `ct_config.c` | 运行配置 `ct_cfg()` + 下行校验应用（config / stim / all/stim，存 NVS key `dl`）。**只管许可，不调用任何放电函数** |
| `device_id.c` | 设备编号（NVS）、上电计数、上电 10 秒串口窗口 |
| `gpio_module.c` | 引脚初始化、围栏引脚安全态、电源控制 |
| `fence_bench.c` | 围栏台架控制台（`FENCE_BENCH=1` 才编译） |
| `eg915_selftest.c` | 产测自检（`SELFTEST=1` 走的就是它的 `Selftest_task`） |
| `self_test.c` | 另一套实验室循环自检。**在 CMakeLists 里参与编译，但当前没有任何地方调用它**，算死代码 |
| `battery_module.c` / `timer_module.c` / `button_module.c` | 电池 ADC / 定时器 / IO0 按键 |

不参与编译：`can_module.c`、`rs485_module.c`（本板无此硬件）、
`pe_original.c`（pe_board 历史参考）。

---

## 8. 待办

**硬件**

- 分区表改双 OTA 槽 + flash 配置 2MB → 8MB（需要有线全擦重烧一次）
- 高压侧加电压分压抽头到 ADC（现在固件对 C51 电压是瞎的）
- `MODEM_EN` 在 IO48（>21，深睡浮空 → 3.8V 轨自己通电），建议挪到 RTC GPIO

**软件**（按建议顺序，详见 1.1 交接）

1. 上板验证 P1（见 1.1）
2. 用 ext1 中断替代 60 s 轮询（等 P0 结论）
3. 蜂窝 OTA（EG800 走 `AT+QHTTP*`，需要二进制安全的 UART 路径 + 打开硬件流控）
4. NimBLE GATT + NVS 配置层，之后是 Flutter App

> 出货前的开关检查：`FENCE_BENCH = 0`、`SELFTEST = 0`、`P0_BENCH` 不存在或为 0。
> 节奏参数改 `CT_CFG_*` 默认值，或下行做好后由服务器下发。

**测试**

- 围栏在老化/低温电池下的边界（串 100/220 mΩ）
- 动作门限 `motion_th_mg` 的现场标定（挂到真牛身上调）
