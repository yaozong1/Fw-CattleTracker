#include "tracking_policy.h"
#include "hardware_config.h"
#include "esp_attr.h"

/* ---- RTC 慢速内存：deep sleep 不清零，上电复位才清 ---- */
#define TRACK_MAGIC 0x7243414Du   /* "MACr" */

RTC_DATA_ATTR static uint32_t s_magic;
RTC_DATA_ATTR static uint8_t  s_mode;
RTC_DATA_ATTR static uint8_t  s_streak;
RTC_DATA_ATTR static uint8_t  s_doubt;
RTC_DATA_ATTR static uint32_t s_cycles;
RTC_DATA_ATTR static uint32_t s_pubs;

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
        out.cold_boot = true;
    }

    uint8_t mode_before = s_mode;
    s_cycles++;

    if (in.activity_mg >= MOTION_MEAN_DELTA_TH_MG) {
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

    /* 当前阶段：固定周期，不分高低频。mode/streak/doubt 照常维护，
     * 只是不再决定睡多久 —— 以后要恢复双档，把这里改回按 mode 取值即可。 */
    out.sleep_sec       = (uint32_t)TRACK_CYCLE_SEC;
    out.fix_timeout_sec = (uint32_t)TRACK_FIX_TIMEOUT_S;

    /* 只有判定为「有动作」才值得唤醒 GNSS。
     * 冷启动例外：先无条件定一次位，让后台知道设备活着、在哪。 */
    out.do_gnss = (out.event == TRACK_EV_MOTION) || out.cold_boot;

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
