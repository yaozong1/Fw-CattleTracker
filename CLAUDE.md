# Fw-CattleTracker

ESP32-S3 牛羊项圈：GNSS 定位 + LTE 上报 + 电子围栏高压块。
硬件代号 `Cattle_T`，原理图在同级的 `process-engineering-cattle-tracker`（KiCad）。

> 新会话从这里开始。按需读第 6 节的文档索引，不用一次全读。
>
> **最后核对：2026-11-26 / 分支 `p1-ext1-wake`**（ext1 唤醒上板验证后更新）

---

## 1. ⚠ 当前仓库状态

**三个分支编出来的东西不一样，先看清在哪个分支：**

| 分支 | 编出来的固件 |
|---|---|
| `p1-ext1-wake`（当前开发） | `p1-protocol` + 加速度计 ext1 唤醒（`CT_WAKE_EXT1 = 1`，见第 4 节） |
| `p1-protocol` | **生产固件**，`FENCE_BENCH = 0`，按 `MQTT_PROTOCOL.md`（ct/v1）上报；每 60 s 轮询加速度计 |
| `p0-bench` | P0 实验固件（`P0_BENCH = 1`），见那个分支里的 `P0_BENCH.md` |
| `connect-fleet` | 围栏台架测试固件（`FENCE_BENCH = 1`），旧的 `fleet/<IMEI>/battery` 上报 |

### 1.1 交接（每次切会话只更新这一节）

**上个会话停在哪**（2026-11-26）

`p1-ext1-wake` 的提交（都在本地，**需要在 Windows 终端 `git push origin p1-ext1-wake`**，
会话里的 shell 连不上 GitHub SSH）：

| 提交 | 内容 |
|---|---|
| 7e68c2c | ext1 唤醒：A 长睡（ext1 开，定时到心跳）/ B 冷却（ext1 关）；`CT_WAKE_EXT1` 开关；固件 0.4.0-p1 |
| aed1b6a | B 冷却改成睡到「下次允许运动上报」（至少 `check_s`，不超过心跳） |
| c62d140 | 六个 config 参数改成白话说明（协议表、默认值注释、开机日志），字段名不变 |
| ec66433 | 唤醒原因分清【心跳定时器】/【冷却定时器】/【加速度计中断】（睡前把睡法记在 RTC） |

**2026-11-27：定位质量过滤（固件 0.4.1-p1，编译通过，未上板）**

11-26 户外实测发现跳点：#48 RMC 已是 A 但 GGA 0 颗卫星 / HDOP 99.99，位置偏 600 m；
#56、#62 只有 3 颗卫星，偏 300~400 m。原因：以前看到 RMC=A 就收工，坐标取 RMC、HDOP/卫星数取「最近一句 GGA」。
现在：坐标和质量都取同一句 GGA（`gnss_get_gga_epoch()`），每秒判断一次，
**卫星 >= 5、0 < HDOP <= 3.0、GGA 质量 >= 1 且 != 6、RMC=A 且模式不是 E/N，连续 3 秒**才接受
（`CT_FIX_MIN_SATS / CT_FIX_MAX_HDOP / CT_FIX_STABLE_S`）；超时报 fix_fail，不报差的位置。
日志里有「合格 n/3」「等稳定 —— 卫星 3 < 5」这类行。ttff_s 现在是「到接受定位」的时间，会比以前多 2 s 左右。
同一批数据确认：热启动正常（TTFF 多为 2~4 s），星历在 Backup + RESET_N 下保得住；失败基本是室内/楼边遮挡。

**已上板验证（CT-001，c62d140）**

- P1 上下行、下行配置（cfg v1 心跳 9 min）、全群总闸 v4 都正常。
- ext1：静止只在心跳醒；晃一下立刻醒（`唤醒原因：IO17`）；锁存=1 → B 冷却，锁存=0 → 回 A；
  定时器数值（547 / 60 / 311 s 这类）都是「离心跳/离可报运动还有多久」，逐条核对过。
- 晃醒后 5 s 采样常是「存疑」（醒来那 5 s 板子已经不动了）—— 测「有动作」要晃醒后继续晃 5 s。
- **ec66433（唤醒原因）还没烧。**

**踩到的坑**

- 电流表（µA/mA 档）串在电池正极时模组一注网就掉电（SIM 就绪后 `AT+CEREG?` 无任何回应、
  之后连 `AT` 都不回）。测深睡电流后一定拿掉 / 短接电流表再联网。建议电流表并一个开关。

**这几天的结论（备查）**

- P0 ①：定时 162 µA / ext1 163 µA / 强制 RTC_PERIPH ON 162 µA → ext1 不额外耗电；
  当年 +405 µA 不是 RTC_PERIPH 本身（见 5.3）。基线 162 vs 文档 112 µA 的差没查。
- 锁存语义：LIS2DH12 10 Hz、HPF、任一轴 > 96 mg → INT1_SRC.IA=1 并拉高 IO17，读 INT1_SRC 才清；
  每次醒来第一件事读并清，进睡前布防再清一次。睡 A / 睡 B 布防完全一样，区别只在 ESP32 理不理 IO17。
- 两个计时：心跳计时被**任何**上报重置；运动计时只被 motion / boot 重置（心跳、手动不重置）。
  睡 A 定时 = 距心跳；睡 B 定时 = max(check_s, 距可报运动)，不晚于心跳。
  `motion_min_s` 应 < 心跳间隔，否则心跳会穿插在运动上报之间（不报错）。
- 星历：设计上 GNSS 进 Backup 时 VCC 不断（IO11 保持高），理论上热启动；但**从没定到过位**，
  RESET_N 唤醒是否清星历未验证；`CFG-EPHSAVE` 只在诊断函数里调用，生产流程没开；
  GNSS 每次只开几秒，星历 2~4 h 过期后心跳 30 s 可能定不到。→ 室外测 ttff_s 验证。
- 流量估算（明文 MQTT，每次上报重连）：约 2.5~3 KB/次（含 3 条保留下行每次重推），
  每天 0.4~0.9 MB，每月 15~25 MB；加 TLS 约 ×2~3。可用 EG800 `AT+QGDCNT` 实测（未加）。
- USB-C（按 PCB 网络表核对）：P2 = 充电口（CC1/CC2 各 5K1 下拉 → C 对 C 充电器给 5 V；
  VBUS → F3 → D14 → CN3163，R62 2K 定充电流；插 USB 时 Q1 关掉太阳能路）；
  P1 = EG800 USB 调试口，不充电。无 PD，只有 5 V。

**下一步（建议顺序）**

1. 烧 ec66433，确认唤醒原因的新写法。
2. **室外定位**：冷启动 ttff_s → 下一次上报 ttff_s（热启动？）→ 静置 3~4 h 再看（星历过期）；
   顺便在 FleetManager GPS 页看真实轨迹。
3. 功耗：生产固件睡 A 电流；晃醒不报 / 心跳 / 运动三种醒来各耗多少 → 续航估算。
4. 24~48 h 浸泡：boot/seq 跳号、意外重启、电压曲线、在线误判。
5. 异常：拔天线 / 弱信号、离线期间改配置、非法配置拒收。
6. P0 ②~⑤（`p0-bench`）→ 定围栏方案 → 协议 v1.1 `fence`。

（以下是 2026-11-20 的记录）

- 和云端工程师分工：固件归我们，broker（TLS + 每台独立凭据）和正式后端归他。
  在他的 broker 就绪前，用 FleetManager（网络盘 Z:，分支 `p1-protocol`，说明见它根目录的 `CLAUDE.md`）当测试平台。
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
- ext1 唤醒已上板验证（见上）；持续走动时的节奏 = 每 `motion_min_s` 一次运动上报，待室外长时间验证。
- 星历能否在 Backup + RESET_N 唤醒后保住（见上）。
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

**唤醒方式（`CT_WAKE_EXT1 = 1`，分支 `p1-ext1-wake` 起）**：加速度计 INT1（IO17）做 ext1 唤醒源，两种睡法：

| 睡法 | ext1 | 定时器 | 什么时候用 |
|---|---|---|---|
| A 长睡 | 开 | 到下次心跳 | 本轮醒来锁存位 = 0（没在动） |
| B 冷却 | 关 | 到下次允许运动上报（距上次运动上报满 `motion_min_s`），至少 `check_s`，不超过距心跳 | 本轮锁存位 = 1（被晃醒 / 冷却期间还在动） |

牛不动时只有心跳会醒；一直在动时每 `motion_min_s`（默认 300 s）醒一次并上报一次 ——
报完之后那段时间醒了也不能报，锁存位会把「动没动过」记下来，所以不中途醒。
被晃醒但判定存疑（没上报）时冷却 `check_s`。冷却是必须的，
否则吃草时每几秒被叫醒一次、每次采样 5 s，比轮询还费电。睡法在**上报做完之后**才算
（`track_sleep_plan()`），按那一刻算离心跳还有多久。布防失败或 IO17 已经是高时退回 B。
`CT_WAKE_EXT1 = 0` 回到原来的每 `check_s` 轮询。

**深睡 = 复位**，RAM 全丢，只有 `RTC_DATA_ATTR` 存活。上电 / 复位那一次先开 10 秒串口窗口（`id CT-001` 写编号，`go` 跳过）。

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

### 5.3 ext0 / `gpio_hold_en()` 曾实测 +405 µA；ext1 不会

当年 `gpio_hold_en()` 和 `esp_sleep_enable_ext0_wakeup()` 实测深睡电流 112 µA → 550 µA，
当时归因于 RTC_PERIPH 整个休眠期供电。

**P0 ① 实测（2026-11-24，p0-bench）**：定时 162 µA，ext1(IO17) 163 µA，
强制 RTC_PERIPH = ON 也是 162 µA。所以 ext1 不额外耗电（已用于生产，见第 4 节），
而且那 405 µA **不是** RTC_PERIPH 本身造成的，更可能是 hold / ext0 对引脚的处理
（原因未查，反正不用 ext0 和 hold）。基线 162 vs 文档 112 µA 的差别也没查，可能是板子/测量条件。

仍然：不用 ext0，深睡期间引脚电平靠**板上电阻**保证，不靠 hold。

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

1. ~~上板验证 P1~~ → 已跑通（CT-001）
2. ~~用 ext1 中断替代 60 s 轮询~~ → 已做并上板验证（`p1-ext1-wake`），待合回主线
3. 室外定位 / 星历保持 / 功耗 / 浸泡测试（见 1.1「下一步」）
4. 视星历测试结果：生产流程打开 `CFG-EPHSAVE`、定位后多开几十秒刷星历，或 AGNSS
5. 可选：每次上报后读 `AT+QGDCNT` 打日志，实测流量
6. 蜂窝 OTA（EG800 走 `AT+QHTTP*`，需要二进制安全的 UART 路径 + 打开硬件流控）
7. NimBLE GATT + NVS 配置层，之后是 Flutter App（另一个会话在做 APK）

> 出货前的开关检查：`FENCE_BENCH = 0`、`SELFTEST = 0`、`P0_BENCH` 不存在或为 0。
> 节奏参数改 `CT_CFG_*` 默认值，或下行做好后由服务器下发。

**测试**

- 室外 ttff_s（冷 / 热 / 星历过期后）
- 功耗分解与续航
- 24~48 h 浸泡
- 围栏在老化/低温电池下的边界（串 100/220 mΩ）
- 动作门限 `motion_th_mg` 的现场标定（挂到真牛身上调）
