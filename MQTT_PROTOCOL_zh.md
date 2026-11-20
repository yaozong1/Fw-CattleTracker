# Cattle Tracker —— MQTT 协议 v1（中文版）

状态：**草案 v1.0** —— 2026-11-20（第 2 版：增加 `ts_src=rtc`；上报失败也重置计时）。供固件（Yaozong）和后端评审。
固件分支：`p1-protocol`。Topic 前缀：`ct/v1/`。

> 本文是 `MQTT_PROTOCOL.md`（英文）的中文对照版。两者内容一致；如有出入，以英文版为准（英文版是发给后端的正式契约）。

本文是项圈固件与所有后端之间的契约。这里的后端包括正式后端，也包括 FleetManager：正式 broker 就绪之前，用 FleetManager 当测试平台。双方都严格按本文实现。

---

## 1. 基本原则

### 1.1 设备几乎一直是离线的

项圈两次上报之间处于睡眠状态，**蜂窝模组是断电的**。只有在连网上报的大约 10–60 秒里才能联系到它。由此带来几条要求：

- **所有下行消息必须用 `retained` + QoS 1 发布。**
  设备用 clean session 连接；它睡眠期间发出的非 retained 消息会直接丢失。
- **下行延迟最长为一个心跳间隔**（开发阶段默认 10 分钟，量产时为几小时）。后台界面必须区分两种状态：「已下发，等待设备」和「设备已生效」，见 §5.3。
- **离线靠"沉默"判断**，不看 MQTT 会话状态。
  超过 `1.5 × heartbeat_min` 没有收到 `telemetry`，就判定设备离线。不使用遗嘱消息（Last Will）：设备每轮都会正常断开连接，真出故障时遗嘱反而不会触发。

### 1.2 服务器不能触发刺激

本协议里**没有任何一条能让设备放电的消息**。
服务器能做的只有两件事：设置一个宏观许可（`stim.enabled`，§5.2），以及将来下发围栏几何（§5.4）。是否警告、何时刺激，完全由固件里的围栏逻辑决定；相关上限写死在固件里，服务器改不了。`stim.enabled = false` 永远优先。

### 1.3 约定

| 项 | 约定 |
|---|---|
| 编码 | UTF-8 JSON，每条消息一个对象，末尾不带换行 |
| 时间 | `ts` = Unix 秒，UTC |
| 坐标 | WGS-84 十进制度，6 位小数（约 0.1 m） |
| 单位 | 不直观的写进字段名：`_s`、`_min`、`_mv`、`_mg`、`_kmh` |
| 未知 / 未测量 | 省略该字段，或按说明写 `null` |
| 大小 | 上行每条 ≤ 1 KB；下行上限待定（见 §9） |

---

## 2. 设备身份

| | 值 | 来源 |
|---|---|---|
| `id` | `CT-001`、`CT-002` …… | 通过串口控制台写入设备 NVS，一次性操作（产测或首次烧录时）。升级固件后仍然保留 |
| 备用 `id` | `IMEI-<15位数字>` | NVS 里还没有 `id` 时使用。后台应把这类设备标为**未登记** |
| `imei` | 15 位数字 | 每次开机从模组读取，在 `status` 里上报（§4.2） |

`id` 既是下文 topic 里的 `{id}` 段，也是 MQTT Client ID。

---

## 3. Topic

| Topic | 方向 | QoS | Retained | 什么时候发 |
|---|---|---|---|---|
| `ct/v1/{id}/status` | 上行 | 1 | **是** | 每次连接，最先发 |
| `ct/v1/{id}/telemetry` | 上行 | 1 | 否 | 每次上报（动作 / 心跳 / 开机 / 手动） |
| `ct/v1/{id}/event` | 上行 | 1 | 否 | 预留，给围栏事件用，v1.0 不使用 |
| `ct/v1/{id}/config` | 下行 | 1 | **是** | 后台修改设置时 |
| `ct/v1/{id}/stim` | 下行 | 1 | **是** | 刺激许可变更时 |
| `ct/v1/all/stim` | 下行 | 1 | **是** | 全群刺激总闸（一键全关） |
| `ct/v1/{id}/fence` | 下行 | 1 | **是** | 预留，格式待定（§5.4） |

### 3.1 一次连接的顺序

```
CONNECT  （clientId = id，clean session，keepalive 60 s）
SUBSCRIBE ct/v1/{id}/config, ct/v1/{id}/stim, ct/v1/all/stim     （QoS 1）
  最多等 3 s 收 retained 消息；校验；应用；存入 NVS
PUBLISH  ct/v1/{id}/status      （retained）—— 带上现在已生效的各项版本号
PUBLISH  ct/v1/{id}/telemetry
DISCONNECT
```

`status` 放在下行**应用之后**才发，所以它就是"设备现在在用什么"的回执。

---

## 4. 上行

### 4.1 `telemetry`

```json
{
  "ts": 1763625600, "ts_src": "gnss",
  "boot": 7, "seq": 1234,
  "reason": "heartbeat",
  "gps": { "lat": -1.292066, "lng": 36.821946, "alt": 1661.2,
           "spd_kmh": 0.4, "hdg": 212, "hdop": 0.9, "sats": 14, "ttff_s": 3.2 },
  "bat": { "mv": 3910, "soc": 72 },
  "act": { "event": "still", "act_mg": 0, "still_min": 41 },
  "net": { "rssi": -87 },
  "stim": false
}
```

| 字段 | 类型 | 含义 |
|---|---|---|
| `ts` | int | 本条上报的 Unix 秒（UTC）。还没有任何时间来源时为 `0` |
| `ts_src` | string | `gnss`（本轮来自 RMC 的日期 + 时间）· `net`（本轮来自模组网络时间）· `rtc`（设备时钟：之前用 `gnss`/`net` 校准过，深睡期间一直在走）· `none`（上电以来从没校准过） |
| `boot` | int | 上电计数（存 NVS）。每次冷启动 / 复位加 1，深睡唤醒不加 |
| `seq` | int | 本次上电内的消息序号，从 1 开始。去重键 =（`id`, `boot`, `seq`） |
| `reason` | string | `motion`（有动作）· `heartbeat`（心跳）· `boot`（上电后第一次上报）· `manual`（IO0 双击） |
| `gps` | object \| `null` | 本轮没拿到定位时为 `null` |
| `gps.lat` / `gps.lng` | number | 度 |
| `gps.alt` | number | 海拔（米，来自 GGA）；拿不到时省略 |
| `gps.spd_kmh` / `gps.hdg` | number | 对地速度 / 对地航向（度） |
| `gps.hdop` / `gps.sats` | number / int | 定位质量。后台应把 `hdop > 5` 视为不可靠 |
| `gps.ttff_s` | number | 本轮从唤醒 GNSS 到定位的耗时；`manual` 上报时省略（直接用最近一次定位，不等待） |
| `fix_fail` | bool | 只有在尝试了定位但超时时才出现，值为 `true` |
| `bat.mv` / `bat.soc` | int | 电池电压（mV）/ 估算电量（%） |
| `act.event` | string | 本轮运动判定：`motion` · `still` · `doubt` |
| `act.act_mg` | int | 5 秒窗口内相邻帧加速度差的平均值；`-1` = 本轮没采样 |
| `act.still_min` | int | 距上一次判定为 `motion` 过了多少分钟 |
| `net.rssi` | int | 信号强度 dBm，来自 `AT+CSQ`；未知时省略 |
| `stim` | bool | 设备上当前生效的刺激许可（§5.2） |

### 4.2 `status`（retained）

```json
{
  "ts": 1763625600,
  "id": "CT-001", "imei": "865058080796511", "iccid": "8986…",
  "fw": "0.3.0", "hw": "Cattle_T-A",
  "boot": 7, "reset": "deepsleep",
  "applied": { "cfg_ver": 3, "stim_ver": 2, "all_stim_ver": 1, "fence_ver": 0 },
  "cfg": { "heartbeat_min": 10, "motion_min_s": 300, "check_s": 60,
           "fix_timeout_s": 90, "hb_fix_timeout_s": 30, "motion_th_mg": 20 },
  "err": null
}
```

| 字段 | 含义 |
|---|---|
| `reset` | 本次启动的 ESP32 复位原因：`poweron` · `deepsleep` · `sw` · `panic` · `wdt` · `brownout` · `other` |
| `applied.*_ver` | 各类下行当前生效的版本号；`0` = 从没收到过（使用固件默认值） |
| `cfg` | **实际生效**的配置（已补全默认值、通过校验） |
| `err` | `null`，或最近一次被拒收的下行，例如 `{ "topic": "config", "ver": 4, "reason": "heartbeat_min out of range" }` |

因为是 retained，即使设备在睡觉，`status` 也始终保存着它最近一次的身份、固件版本和已生效的版本号。

### 4.3 `event` —— 预留

围栏逻辑做好后计划使用的类型：`warning`（警告）、`stimulus`（刺激）、`stim_suppressed`（刺激被抑制）、`escape`（逃逸）、`fence_fault`（围栏故障）、`low_battery`（低电量）。
具体格式和围栏设计一起定义。

---

## 5. 下行

所有下行消息：**retained，QoS 1**，都是带整数 `ver` 字段的 JSON 对象。

设备对每个下行 topic 的处理规则：

1. `ver` 等于已生效的版本 → 忽略（已经应用过）。
2. `ver` 不相同 → 校验**整条**消息。只要有一个字段不合法，就拒收整条消息，继续用之前的，并在 `status.err` 里报告。
3. 校验通过 → 从下一轮开始生效，存入 NVS，在 `status.applied` 里报告。
4. retained 消息被清空（空 payload）→ 恢复固件默认值（`config`），或 `enabled: false`（`stim`）。

判断条件用"不相同"而不是"更大"：后台数据库重置、版本号从 1 重新开始时，设备不会被卡住。

### 5.1 `config`

```json
{ "ver": 3, "heartbeat_min": 10, "motion_min_s": 300, "check_s": 60,
  "fix_timeout_s": 90, "hb_fix_timeout_s": 30, "motion_th_mg": 20 }
```

| 字段 | 默认 | 范围 | 含义 |
|---|---|---|---|
| `heartbeat_min` | 10 | 5 – 1440 | 最长沉默时间。超过这么久没上报过，就强制上报一次 |
| `motion_min_s` | 300 | 60 – 3600 | 牛持续走动时，两次 `motion` 上报之间的最小间隔 |
| `check_s` | 60 | 10 – 600 | 多久醒来检查一次加速度计。**只在轮询方式下有效**；固件改用中断唤醒（ext1）后忽略 |
| `fix_timeout_s` | 90 | 30 – 300 | `motion` / `boot` / `manual` 上报时的 GNSS 定位超时 |
| `hb_fix_timeout_s` | 30 | 0 – 120 | `heartbeat` 上报时的 GNSS 定位超时。`0` = 心跳不开 GNSS |
| `motion_th_mg` | 20 | 5 – 200 | 运动判定门限（相邻帧差平均值） |

省略的字段取默认值。不认识的字段忽略（为以后加字段留余地）。

任何一次上报都会把心跳计时清零：`motion` 上报也算"没有沉默"。

**上报失败**（没信号、连不上 broker）也会把计时清零。设备不会每 `check_s` 重试一次 —— 在没信号的地方那样会把电池耗光 —— 而是等下一个心跳或下一次动作上报再试。对后台的影响：进了信号盲区的项圈，`1.5 × heartbeat_min` 后显示离线，信号恢复后自己会回来。

### 5.2 `stim` 和 `all/stim`

```json
{ "ver": 2, "enabled": false }
```

- 实际生效的许可 = `ct/v1/{id}/stim.enabled` **且** `ct/v1/all/stim.enabled`。
- `ct/v1/{id}/stim` 从没收到过 → `false`（安全默认）。
- `ct/v1/all/stim` 从没收到过 → 不起拦截作用。它是一键全关的总闸，后台初始化时应先发布一次 `{ "ver": 1, "enabled": true }`。
- `enabled: true` 的意思是固件围栏逻辑**可以**警告和刺激，它本身永远不会引发刺激。
- v1.0 固件还没有围栏逻辑：这个值只存储、只回报（`telemetry.stim`、`status.applied`），不产生任何效果。

### 5.3 后台显示什么

| 后台状态 | 条件 |
|---|---|
| 已生效 | `status.applied.<x>_ver` == 最后发布的版本 |
| 等待设备 | 最后发布的版本 ≠ `status.applied.<x>_ver`，设备还没重新连上 |
| 被拒收 | `status.err.ver` == 最后发布的版本 |
| 离线 | 超过 `1.5 × heartbeat_min` 没有 `telemetry` |

### 5.4 `fence` —— 预留

格式取决于围栏判定的方式：用 GNSS 自带围栏，还是固件做点在多边形内的判断。台架实测正在进行。
预计是一组多边形，WGS-84 坐标，顶点数有固件规定的上限。
**后台现在就应该把围栏存在服务端**，但在 v1.1 定义好格式之前，不要往这个 topic 发布。

---

## 6. 固件默认值和时序（开发阶段）

| | 值 |
|---|---|
| 心跳 | 10 分钟 |
| 动作上报最小间隔 | 5 分钟 |
| 加速度计检查（轮询方式） | 60 s |
| GNSS 超时 —— 动作 / 心跳 | 90 s / 30 s |
| SUBSCRIBE 后等下行的时间 | 3 s |

量产值通过 `config` 下发，不需要重新烧固件。

---

## 7. 连接与安全

| | v1.0（现在，测试 broker） | 量产（后端团队） |
|---|---|---|
| 传输 | TCP 1883 | **TLS 8883**（EG800 `AT+QSSLCFG`，CA 证书存在模组里） |
| 凭据 | 共用测试账号 | **每个项圈独立**的用户名 / 密码（或客户端证书），存在设备 NVS 里 |
| Client ID | `{id}` | `{id}` |
| ACL（建议） | —— | 设备 `{id}` 只能发布 `ct/v1/{id}/status|telemetry|event`，只能订阅 `ct/v1/{id}/config|stim|fence` 和 `ct/v1/all/stim` |

---

## 8. 版本管理

- 不兼容的改动 → 换新前缀 `ct/v2/`。迁移期间设备和后台可以两套并行。
- 兼容的改动（新增可选字段）留在 v1；接收方忽略不认识的字段。

---

## 9. 待后端团队回答的问题

1. **下行大小上限**：EG800 的接收缓冲限制了 retained 消息的大小。固件会实测并定一个上限（预计 ≥ 1 KB），围栏几何必须放得下。
2. **每个项圈独立凭据的形式**：用户名 / 密码，还是 X.509 客户端证书？怎么生成？怎么交给工厂写进 NVS？
3. **`all/stim` 的范围**：一个全群总闸，还是按牧场 / 按分组（`ct/v1/group/{gid}/stim`）？
4. **时间**：设备的 `ts` 能否作为历史记录的时间依据？后台是否同时记录接收时间？（建议两个都存。）
5. **保存期限**：telemetry 历史在服务端保留多久？
