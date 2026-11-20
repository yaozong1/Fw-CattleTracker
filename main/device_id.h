#ifndef DEVICE_ID_H
#define DEVICE_ID_H

#include <stdint.h>
#include <stdbool.h>

/* ================= 设备身份（MQTT_PROTOCOL.md §2） =================
 * id    : CT-001 这类编号，存在 NVS（namespace "ct"，key "id"），升级固件不丢。
 *         没写过时退回 "IMEI-<15位>"，后台标为「未登记」。
 * boot  : 上电计数，存 NVS。冷启动 / 复位时 +1，深睡唤醒不加。
 *         和 telemetry.seq 一起组成去重键 (id, boot, seq)。
 * imei  : 每次开机从模组读（mqtt_init_network 里），不存 NVS。
 * ================================================================ */

/* 开机最早调用。初始化 NVS，读出编号；cold_boot=true 时上电计数 +1。 */
void devid_init(bool cold_boot);

/* 只在上电 / 复位那一次调用：在 UART0 上开 window_ms 毫秒的命令窗口。
 *   id             显示当前编号
 *   id CT-001      写入编号
 *   id clear       清除编号（回到 IMEI-xxx）
 *   go             立即结束窗口
 * 深睡唤醒时不要调用 —— 每轮都等 10 秒会把续航吃光。 */
void devid_console_window(uint32_t window_ms);

/* 当前用于 topic 和 Client ID 的编号：已写入的 CT-xxx，否则 IMEI-<imei> */
const char *devid_get(void);
bool        devid_is_provisioned(void);

/* IMEI：mqtt_init_network() 读到后调用 devid_set_imei() 登记 */
void        devid_set_imei(const char *imei);
const char *devid_imei(void);          /* 还没读到时返回 "" */

uint32_t    devid_boot_count(void);

#endif /* DEVICE_ID_H */
