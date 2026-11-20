# Fw-CattleTracker

ESP32-S3 牛羊项圈：GNSS 定位 + LTE 上报 + 电子围栏高压块。
硬件代号 `Cattle_T`，原理图在同级的 `process-engineering-cattle-tracker`（KiCad）。

> 新会话从这里开始。按需读第 6 节的文档索引，不用一次全读。
>
> **最后核对：2026-11-19 / `646af57`**（本文件的事实已逐条对过代码）

---

## 1. ⚠ 当前仓库状态

**`hardware_config.h` 里 `FENCE_BENCH = 1`** —— 现在编出来的是**围栏台架测试固件**，
不是生产固件。它不启动 GNSS / MQTT / 运动传感器，不进深睡，只开一个串口控制台。

要回到生产固件：把 `FENCE_BENCH` 改回 `0`。

> **分支 `p0-bench`**：`P0_BENCH = 1`（优先级最高），编出来的是 P0 实验固件，
> 用来给"围栏判定放 GNSS 还是放固件"拿实测数据。步骤和结果表见 `P0_BENCH.md`。

### 1.1 交接（每次切会话只更新这一节）

**上个会话停在哪**

围栏高压块带机全部跑通：充电 411 V / 4.84 ms、单发约 2000 V 尖峰 / 10 µs、
`f burst 10 1` 十发约 50 ms（要求 100 ms，2× 余量）。数据和余量见 `FENCE_BRINGUP.md`。
之后只做了文档沉淀，没动固件逻辑。

**下一步（按建议顺序）**

1. **定 `TRACK_CYCLE_SEC`** —— 现在是调试值 30 s，是续航的最大杠杆（见 `POWER_BUDGET.md` 4.2）。
   30 s → 全静止 24 天；300 s → 170 天。定这个值之前谈别的省电都没意义。
2. **心跳 N 未定** —— 现在 `do_gnss = MOTION || cold_boot`，牛一直不动就永远不上报，
   平台侧分不清"没动"和"掉线"。`TRACK_DOUBT_MAX` 到 5 之后目前什么都不触发（见第 4 节缺口）。
3. OTA 分区（2MB → 8MB，要有线全擦重烧一次）。
4. 围栏边界测试：电池串 100 / 220 mΩ 跑 `f burst 10 1`，模拟老化/低温电芯 ESR。

**悬而未决**

- 心跳周期 N：等业务侧给"最长允许多久没消息"。
- `MOTION_MEAN_DELTA_TH_MG = 20` 是台面上手晃标定的，挂到真牛身上必然要重调。

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
| `ENABLE_TRACKING_POLICY = 1`（**当前**） | 运动驱动：每 `TRACK_CYCLE_SEC` 醒一次，判定有动作才开 GNSS |
| `ENABLE_TRACKING_POLICY = 0` | 退回旧的固定周期：每 `GPS_CYCLE_SLEEP_MIN` 分钟无条件定位上报 |

注意 `GPS_CYCLE_SLEEP_MIN` **只在 `= 0` 的分支里生效**（`mqtt_module.c` 的 `#else`）。
`= 1` 时改它没有任何效果，唯一还会用到它的地方是任务启动时那行日志——容易看错。

---

## 4. 生产固件的主循环

每 **30 秒**（`TRACK_CYCLE_SEC`）深睡唤醒一次。**深睡 = 复位**，RAM 全丢，
只有 `RTC_DATA_ATTR` 存活。

```
唤醒
 ├─ gpio_init()          ← 先写电平再切方向（见第 5 节）
 ├─ 读 LIS2DH12 的 INT1_SRC 锁存位   ← 必须在任何重配置之前读
 ├─ 锁存=0 → 判「确定静止」，跳过采样，直接睡
 └─ 锁存=1 → 5 秒运动采样（100Hz ODR / 20ms 取样）
      ├─ 平均帧差 >= MOTION_MEAN_DELTA_TH_MG(20mg) → 「有动作」→ 开 GNSS 定位 + 上报
      └─ 不够 → 「存疑」→ 不开 GNSS
```

判据是**相邻帧的加速度矢量差** `‖a[n] − a[n−1]‖` 的平均值，不是 `|‖a‖ − 1g|`。
后者是旋转不变量，牛低头/甩头/走路都以旋转为主，会完全漏判（实测用力晃动
峰值只有 206 mg，活动占比 4%）。

决策逻辑全在 `tracking_policy.c` 的 `track_decide()`，是个纯函数，好测。

### 已知缺口

`do_gnss = 有动作 || 冷启动`，**没有心跳**。长时间趴着不动的牛会完全静默，
后台无法区分"牛没动"和"设备挂了"。`TRACK_DOUBT_MAX` 累计到 5 之后目前
不触发任何动作（它加的 `streak` 只改 `mode`，而 `mode` 自从去掉高低频后不再影响设备行为 —— 它和 `streak`/`doubt` 只是被打进 MQTT payload 的 `track` 字段给后台看）。

堵法：存疑累计 N 次强制定位一次（兼做心跳）。**N 待定。**

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
| `PORTING_NOTES.md` | 从 pe_board 移植的记录。引脚映射表仍然准确，可当引脚速查用 |
| `P0_BENCH.md` | P0 实验：ext1 深睡电流、GNSS 跟踪电流、热启动 TTFF、LC760Z 自带围栏、近区浅睡（分支 `p0-bench`） |
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
| `mqtt_module.c` | EG800 AT 收发、注网、MQTT 发布（最大的文件） |
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

1. **定 `TRACK_CYCLE_SEC`** —— 现在 30 s 是调试值，续航的最大杠杆
2. **心跳机制**（见第 4 节缺口），N 待定
3. 蜂窝 OTA（EG800 走 `AT+QHTTP*`，需要二进制安全的 UART 路径 + 打开硬件流控）
4. NimBLE GATT + NVS 配置层，之后是 Flutter App

> 出货前的开关检查：`FENCE_BENCH` 回 `0`、`SELFTEST` 保持 `0`、`ENABLE_TRACKING_POLICY` 保持 `1`。
> `GPS_CYCLE_SLEEP_MIN` 不用管 —— 它只在 `ENABLE_TRACKING_POLICY = 0` 时才生效。

**测试**

- 围栏在老化/低温电池下的边界（串 100/220 mΩ）
- `MOTION_MEAN_DELTA_TH_MG` 的现场标定（挂到真牛身上调）
