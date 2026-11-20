#ifndef CT_CONFIG_H
#define CT_CONFIG_H

#include <stdint.h>

/* ================= 运行配置（MQTT_PROTOCOL.md §5.1） =================
 * 第一步（上行）：只有编译期默认值，ver = 0（"从没收到过下发"）。
 * 第二步（下行）：服务器经 ct/v1/{id}/config 下发，校验后存 NVS，覆盖这里的值。
 * 业务代码一律通过 ct_cfg() 取值，不要直接用宏 —— 这样第二步不用再改调用方。
 * ================================================================== */

typedef struct {
    uint32_t ver;              /* 已生效的下发版本，0 = 固件默认 */
    uint16_t heartbeat_min;    /* 最长沉默时间 */
    uint16_t motion_min_s;     /* 两次 motion 上报的最小间隔 */
    uint16_t check_s;          /* 轮询方式下多久醒来看一次加速度计 */
    uint16_t fix_timeout_s;    /* motion / boot / manual 的定位超时 */
    uint16_t hb_fix_timeout_s; /* heartbeat 的定位超时，0 = 心跳不开 GNSS */
    uint16_t motion_th_mg;     /* 运动判定门限 */
} ct_cfg_t;

const ct_cfg_t *ct_cfg(void);

#endif /* CT_CONFIG_H */
