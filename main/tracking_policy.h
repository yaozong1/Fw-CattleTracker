#ifndef TRACKING_POLICY_H
#define TRACKING_POLICY_H

#include <stdint.h>
#include <stdbool.h>

/* ================= 阶段一：运动驱动的上报频率 =================
 * 这个模块只做决策，不碰任何硬件 —— 传感器读数从外面喂进来，
 * 决策结果（下一轮睡多久、本轮定位等多久）交回去执行。
 * 这样状态机可以在 PC 上用构造的输入序列直接跑测试。
 *
 * 状态存在 RTC 慢速内存里，deep sleep 不丢；上电复位时自动初始化。 */

typedef enum {
    TRACK_MODE_HIGH = 0,
    TRACK_MODE_LOW  = 1,
} track_mode_t;

/* 本轮观测到的运动证据 */
typedef struct {
    bool     latched;      /* 整个睡眠窗口内有没有超阈值事件（来自 INT1_SRC 锁存位） */
    int      activity_mg;  /* 5 秒采样的平均帧差 mg；-1 = 没采（latched 为 0，或采样失败） */
    uint32_t now_s;        /* 单调秒（ct_mono_s()），用来算心跳和动作限频 */
} track_input_t;

/* 这一轮要不要上报、为什么报（MQTT_PROTOCOL.md telemetry.reason） */
typedef enum {
    TRACK_RPT_NONE = 0,
    TRACK_RPT_BOOT,        /* 上电后第一轮 */
    TRACK_RPT_MOTION,      /* 有动作，且距上次动作上报 >= motion_min_s */
    TRACK_RPT_HEARTBEAT,   /* 距上次上报 >= heartbeat_min */
} track_report_t;

typedef enum {
    TRACK_EV_MOTION = 0,   /* 次数达标 —— 确定有动作 */
    TRACK_EV_STILL  = 1,   /* 锁存位为 0 —— 整窗口确定静止 */
    TRACK_EV_DOUBT  = 2,   /* 锁存说动过，但 5 秒快照没数够 —— 存疑 */
} track_event_t;

typedef struct {
    track_mode_t  mode;             /* 决策后的模式 */
    track_event_t event;            /* 本轮判定 */
    uint32_t      sleep_sec;        /* 下一轮睡多久 */
    uint32_t      fix_timeout_sec;  /* 本轮定位等多久 */
    uint8_t       streak;           /* 连续静止轮数 */
    uint8_t       doubt;            /* 连续存疑轮数 */
    bool          mode_changed;     /* 这一轮切换了模式 */
    bool          cold_boot;        /* 这一轮是上电冷启动 */
    bool          do_gnss;          /* 本轮要不要唤醒 GNSS（要上报且该原因的定位超时 > 0） */
    track_report_t report;          /* 本轮上报原因；NONE = 不联网 */
    int           still_min;        /* 距上次判定为「有动作」的分钟数；-1 = 开机以来没动过 */
} track_output_t;

/* 每轮唤醒调用一次。内部会更新并写回 RTC 状态。 */
track_output_t track_decide(track_input_t in);

/* 最近一次 track_decide() 的结果快照。上报时把它塞进 payload，
 * 后台就能看出这一条是在什么状态下报上来的 —— 现场调阈值时全靠它。 */
void track_last_snapshot(track_output_t *out, track_input_t *in);

/* 本轮上报尝试完之后调用（不管成功失败）。
 * 失败也算一次：没信号的地方每 check_s 重试一遍会把电池耗光，
 * 宁可等下一个心跳或下一次动作再试。 */
void track_note_report(track_report_t r, uint32_t now_s);

/* 手动上报（IO0 双击）也重置心跳计时 */
void track_note_manual_report(uint32_t now_s);

const char *track_report_str(track_report_t r);

/* 进深睡前调用（上报做完之后，now_s 用那一刻的 ct_mono_s()）：决定这一觉怎么睡。
 * latched = 本轮醒来读到的锁存位（ti.latched）。
 *   CT_WAKE_EXT1=1：latched=0 → A 长睡（ext1 开，定时到下次心跳）
 *                   latched=1 → B 冷却（ext1 关，定时 min(check_s, 距心跳)）
 *   CT_WAKE_EXT1=0：每 check_s 定时醒（轮询），不开 ext1
 * 只做计算，不碰硬件；ext1 能不能真的开（布防失败、引脚已是高）由调用方再判断。 */
typedef struct {
    uint32_t sleep_s;
    bool     arm_ext1;
    uint32_t hb_left_s;    /* 距下次心跳还有多久（日志用） */
} track_sleep_t;

track_sleep_t track_sleep_plan(bool latched, uint32_t now_s);

/* 上报成功/失败的计数，只用于日志 */
void     track_note_publish(bool ok);
uint32_t track_cycles(void);
uint32_t track_publishes(void);

const char *track_mode_str(track_mode_t m);
const char *track_event_str(track_event_t e);

#endif /* TRACKING_POLICY_H */
