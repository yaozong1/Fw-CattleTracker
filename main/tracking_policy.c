#include "tracking_policy.h"
#include "hardware_config.h"
#include "esp_attr.h"
#include "ct_config.h"

/* ---- RTC 慢速内存：deep sleep 不清零，上电复位才清 ---- */
#define TRACK_MAGIC 0x7243414Du   /* "MACr" */

RTC_DATA_ATTR static uint32_t s_magic;
RTC_DATA_ATTR static uint8_t  s_mode;
RTC_DATA_ATTR static uint8_t  s_streak;
RTC_DATA_ATTR static uint8_t  s_doubt;
RTC_DATA_ATTR static uint32_t s_cycles;
RTC_DATA_ATTR static uint32_t s_pubs;

/* 上报节奏（单调秒，见 ct_mono_s） */
RTC_DATA_ATTR static uint8_t  s_have_report;      /* 开机以来上报过 */
RTC_DATA_ATTR static uint32_t s_last_report_s;
RTC_DATA_ATTR static uint8_t  s_have_motion_rpt;
RTC_DATA_ATTR static uint32_t s_last_motion_rpt_s;
RTC_DATA_ATTR static uint8_t  s_have_motion;      /* 开机以来判定过「有动作」 */
RTC_DATA_ATTR static uint32_t s_last_motion_s;

/* 本轮的输入/输出快照。只在一次唤醒里用，不需要放 RTC。 */
static track_input_t  s_last_in;
static track_output_t s_last_out;

const char *track_mode_str(track_mode_t m)
{
    return (m == TRACK_MODE_HIGH) ? "HIGH" : "LOW";
}

const char *track_event_str(track_event_t e)
{
    switch (e) {
        case TRACK_EV_MOTION: return "有动作";
        case TRACK_EV_STILL:  return "确定静止";
        default:              return "存疑";
    }
}

track_output_t track_decide(track_input_t in)
{
    track_output_t out = (track_output_t){0};

    /* 冷启动：RTC 内存是脏的。开机先按 HIGH 走一轮，
     * 让后台尽快拿到一次位置，确认设备活着。 */
    if (s_magic != TRACK_MAGIC) {
        s_magic  = TRACK_MAGIC;
        s_mode   = (uint8_t)TRACK_MODE_HIGH;
        s_streak = 0;
        s_doubt  = 0;
        s_cycles = 0;
        s_pubs   = 0;
        s_have_report = s_have_motion_rpt = s_have_motion = 0;
        s_last_report_s = s_last_motion_rpt_s = s_last_motion_s = 0;
        out.cold_boot = true;
    }

    uint8_t mode_before = s_mode;
    s_cycles++;

    if (in.activity_mg >= (int)ct_cfg()->motion_th_mg) {
        /* 活动占比达标 —— 确定有动作，立刻回 HIGH，两个计数都清零 */
        out.event = TRACK_EV_MOTION;
        s_streak  = 0;
        s_doubt   = 0;
        s_mode    = (uint8_t)TRACK_MODE_HIGH;
    } else if (!in.latched) {
        /* 整个睡眠窗口都没超过阈值 —— 确定静止 */
        out.event = TRACK_EV_STILL;
        s_doubt   = 0;
        if (s_streak < 250) s_streak++;
    } else {
        /* 睡眠期间动过，但 5 秒快照的活动占比不够。
         * 既不清零也不累加 streak —— 这样牛断续走动时不会被误退回 LOW。
         * 但要有上限：连续存疑太多轮也当作一轮静止，否则一头每轮甩一下头的牛
         * 会永远卡在 HIGH。 */
        out.event = TRACK_EV_DOUBT;
        if (s_doubt < 250) s_doubt++;
        if (s_doubt >= TRACK_DOUBT_MAX) {
            s_doubt = 0;
            if (s_streak < 250) s_streak++;
        }
    }

    if (s_streak >= TRACK_NO_MOTION_STREAK) {
        s_mode = (uint8_t)TRACK_MODE_LOW;
    }

    out.mode         = (track_mode_t)s_mode;
    out.streak       = s_streak;
    out.doubt        = s_doubt;
    out.mode_changed = (s_mode != mode_before) && !out.cold_boot;

    /* ---- 上报决策（MQTT_PROTOCOL.md §5.1）----
     * 冷启动 → boot；有动作且离上次动作上报够久 → motion；
     * 否则离上次任何上报超过心跳间隔 → heartbeat；其余不联网。
     * mode/streak/doubt 仍然维护，只作诊断，不再影响节奏。 */
    const ct_cfg_t *c = ct_cfg();
    uint32_t now = in.now_s;

    if (out.event == TRACK_EV_MOTION) {
        s_have_motion   = 1;
        s_last_motion_s = now;
    }
    bool motion_due = (out.event == TRACK_EV_MOTION) &&
                      (!s_have_motion_rpt || (now - s_last_motion_rpt_s) >= c->motion_min_s);
    bool hb_due     = !s_have_report ||
                      (now - s_last_report_s) >= (uint32_t)c->heartbeat_min * 60U;

    if (out.cold_boot)   out.report = TRACK_RPT_BOOT;
    else if (motion_due) out.report = TRACK_RPT_MOTION;
    else if (hb_due)     out.report = TRACK_RPT_HEARTBEAT;
    else                 out.report = TRACK_RPT_NONE;

    out.sleep_sec       = c->check_s;
    out.fix_timeout_sec = (out.report == TRACK_RPT_HEARTBEAT) ? c->hb_fix_timeout_s
                                                              : c->fix_timeout_s;
    out.do_gnss   = (out.report != TRACK_RPT_NONE) && (out.fix_timeout_sec > 0);
    out.still_min = s_have_motion ? (int)((now - s_last_motion_s) / 60U) : -1;

    s_last_in  = in;
    s_last_out = out;
    return out;
}


void track_last_snapshot(track_output_t *out, track_input_t *in)
{
    if (out) *out = s_last_out;
    if (in)  *in  = s_last_in;
}

void track_note_publish(bool ok) { if (ok) s_pubs++; }
uint32_t track_cycles(void)      { return s_cycles; }
uint32_t track_publishes(void)   { return s_pubs; }

void track_note_report(track_report_t r, uint32_t now_s)
{
    if (r == TRACK_RPT_NONE) return;
    s_have_report   = 1;
    s_last_report_s = now_s;
    if (r == TRACK_RPT_MOTION || r == TRACK_RPT_BOOT) {
        s_have_motion_rpt   = 1;
        s_last_motion_rpt_s = now_s;
    }
}

void track_note_manual_report(uint32_t now_s)
{
    s_have_report   = 1;
    s_last_report_s = now_s;
}

static RTC_DATA_ATTR uint8_t  s_prev_sleep_kind;
static RTC_DATA_ATTR uint32_t s_prev_sleep_s;
static RTC_DATA_ATTR bool     s_prev_hb_due;

void track_note_sleep(track_sleep_kind_t kind, uint32_t sleep_s, bool hb_due)
{
    s_prev_sleep_kind = (uint8_t)kind;
    s_prev_sleep_s    = sleep_s;
    s_prev_hb_due     = hb_due;
}

track_sleep_kind_t track_last_sleep(uint32_t *sleep_s, bool *hb_due)
{
    if (sleep_s) *sleep_s = s_prev_sleep_s;
    if (hb_due)  *hb_due  = s_prev_hb_due;
    return (track_sleep_kind_t)s_prev_sleep_kind;
}

track_sleep_t track_sleep_plan(bool latched, uint32_t now_s)
{
    const ct_cfg_t *c = ct_cfg();
    uint32_t hb    = (uint32_t)c->heartbeat_min * 60U;
    uint32_t since = s_have_report ? (now_s - s_last_report_s) : hb;
    uint32_t left  = (since >= hb) ? 1U : (hb - since);   /* 已到期：睡 1 s 马上补心跳 */

    track_sleep_t p = { .sleep_s = c->check_s, .arm_ext1 = false, .hb_left_s = left };
#if CT_WAKE_EXT1
    if (latched) {
        /* B 冷却：睡到「下次允许运动上报」为止（至少 check_s，不超过距心跳）。
         * 报完运动后 motion_min_s 内醒来也不会再报，加速度计的锁存位会把这段时间
         * 动没动过记下来，没必要每 check_s 醒一次。 */
        uint32_t cool = c->check_s;
        if (s_have_motion_rpt) {
            uint32_t ms = now_s - s_last_motion_rpt_s;
            if (ms < c->motion_min_s && (c->motion_min_s - ms) > cool) {
                cool = c->motion_min_s - ms;
            }
        }
        p.sleep_s  = (left < cool) ? left : cool;
        p.arm_ext1 = false;
    } else {
        p.sleep_s  = left;                                      /* A 长睡 */
        p.arm_ext1 = true;
    }
#else
    (void)latched;
#endif
    return p;
}

const char *track_report_str(track_report_t r)
{
    switch (r) {
        case TRACK_RPT_BOOT:      return "boot";
        case TRACK_RPT_MOTION:    return "motion";
        case TRACK_RPT_HEARTBEAT: return "heartbeat";
        default:                  return "none";
    }
}
