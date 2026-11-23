#ifndef CT_CONFIG_H
#define CT_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

/* ================= 运行配置 + 下行（MQTT_PROTOCOL.md §5） =================
 * 服务器经 retained topic 下发：
 *   ct/v1/{id}/config     运行参数（ct_cfg_t）
 *   ct/v1/{id}/stim       本机刺激许可
 *   ct/v1/all/stim        全群总闸
 * 每次连网收到后整条校验，通过才应用并存 NVS（namespace "ct"，key "dl"），
 * 版本号经 status.applied 回报；拒收的原因经 status.err 回报。
 *
 * 业务代码一律通过 ct_cfg() 取参数。新配置在下一轮生效（本轮的睡眠时长在收到之前就算好了）。
 *
 * ★ 刺激许可只是"许可"：本模块没有、也不会调用任何放电函数。
 * ====================================================================== */

typedef struct {
    uint32_t ver;              /* 已生效的下发版本，0 = 固件默认 */
    uint16_t heartbeat_min;    /* 最长沉默时间 */
    uint16_t motion_min_s;     /* 两次 motion 上报的最小间隔 */
    uint16_t check_s;          /* 轮询方式下多久醒来看一次加速度计 */
    uint16_t fix_timeout_s;    /* motion / boot / manual 的定位超时 */
    uint16_t hb_fix_timeout_s; /* heartbeat 的定位超时，0 = 心跳不开 GNSS */
    uint16_t motion_th_mg;     /* 运动判定门限 */
} ct_cfg_t;

typedef enum {
    CT_DL_CONFIG = 0,
    CT_DL_STIM,
    CT_DL_ALL_STIM,
} ct_dl_topic_t;

/* 最近一次被拒收的下行（status.err）。深睡保留，断电清空 */
typedef struct {
    bool     has;
    char     topic[12];        /* "config" / "stim" / "all/stim" */
    uint32_t ver;
    char     reason[64];
} ct_dl_err_t;

/* 开机时调用一次（NVS 已由 devid_init 初始化）：从 NVS 读出已生效的下行 */
void ct_cfg_load(void);

const ct_cfg_t *ct_cfg(void);

/* 处理一条下行。len = 0 表示 retained 被清空（恢复默认 / 许可关闭）。 */
void ct_downlink_apply(ct_dl_topic_t t, const char *payload, int len);

uint32_t ct_stim_ver(void);
uint32_t ct_all_stim_ver(void);
bool     ct_stim_enabled(void);       /* 本机开关 */
bool     ct_stim_effective(void);     /* 本机开关 AND 全群总闸（总闸没收到过 = 不拦截） */
const ct_dl_err_t *ct_last_err(void);

#endif /* CT_CONFIG_H */
