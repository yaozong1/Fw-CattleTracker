#ifndef CT_PROTO_H
#define CT_PROTO_H

#include <stdint.h>
#include <stdbool.h>
#include <time.h>

/* ================= ct/v1 协议层（MQTT_PROTOCOL.md） =================
 * 一次上报 = 一次连接：连网 → 连 broker → status(retained) → telemetry → 断开。
 * 第二步会在 status 之前插入「订阅下行 + 等 retained + 应用」。
 * ================================================================== */

typedef enum {
    CT_REASON_NONE = 0,
    CT_REASON_BOOT,
    CT_REASON_MOTION,
    CT_REASON_HEARTBEAT,
    CT_REASON_MANUAL,
} ct_reason_t;

typedef struct {
    ct_reason_t reason;
    bool        gnss_tried;   /* 本轮尝试过定位（没尝试 = 心跳配置成不开 GNSS） */
    bool        fix_ok;       /* 本轮拿到了定位；位置取自 gnss_last_location / GGA 全局量 */
    float       ttff_s;       /* 本轮从唤醒 GNSS 到定位的秒数 */
    const char *act_event;    /* "motion" / "still" / "doubt"，NULL = 不带 act */
    int         act_mg;       /* -1 = 本轮没采样 */
    int         still_min;    /* -1 = 开机以来还没判定过 motion，不带这个字段 */
    float       vbat;         /* 伏，模组上电前的空载读数 */
} ct_report_t;

/* ---- 时间 ----
 * 系统时间一旦校准（GNSS 或模组网络时间）就 settimeofday，深睡期间 RTC 继续走，
 * 之后没有定位的轮次也能给出 ts（ts_src = "rtc"）。
 * 间隔计算（心跳、动作限频）用 ct_mono_s()：校时造成的跳变已扣除，单调递增。 */
uint32_t    ct_mono_s(void);
bool        ct_time_from_gnss(void);   /* 本轮有 RMC 日期+时间时调用 */
time_t      ct_time_now(const char **src_out);

/* 只在 P1 的周期任务 / 手动上报里调用。模组必须已经开机。
 * 返回 true = status 和 telemetry 都发出去了。 */
bool        ct_report(const ct_report_t *r);

const char *ct_reason_str(ct_reason_t r);

#endif /* CT_PROTO_H */
