#ifndef GNSS_MODULE_H
#define GNSS_MODULE_H

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// GNSS位置信息结构
typedef struct {
    bool   valid;     // 数据是否有效
    double lat_deg;   // 十进制度，北正南负
    double lon_deg;   // 十进制度，东正西负
    double spd_kn;    // 节
    double crs_deg;   // 航向角
    char   time_utc[16]; // hhmmss.sss（原样保存）
} gnss_location_t;

// GNSS状态变量（只读）
extern volatile bool  gnss_has_fix;
extern volatile int   gnss_fix_q;
extern volatile int   gnss_dim;
extern volatile int   gnss_sats;
extern volatile float gnss_hdop;
extern volatile bool  gnss_rmc_valid;
extern volatile bool  gnss_seen_rmc;
extern volatile bool  gnss_seen_gga;
extern volatile bool  gnss_seen_gsa;

// GNSS位置数据
extern gnss_location_t gnss_last_location;
extern gnss_location_t gnss_saved_location;

// GNSS控制函数
void gnss_enable(bool on);
void gnss_reset_assert(void);
void gnss_reset_release(void);

// GNSS状态管理
void gnss_reset_state_for_ttff(void);
void gnss_log_fix_summary_if_changed(void);
void gnss_report_ttff_if_needed(void);

// GNSS通信函数
void gnss_send_nmea(const char* sentence);
void gnss_send_ubx(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len);
void gnss_send_quectel_bin(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len);

// GNSS低功耗管理
bool gnss_enter_standby2(void);
bool gnss_wake_from_standby2(void);
bool gnss_enter_backup_via_cfg(void);
void gnss_exit_backup_via_reset(void);
bool gnss_wait_for_quiet2(uint32_t quiet_ms, uint32_t timeout_ms);
bool gnss_wait_for_activity2(uint32_t timeout_ms);

// GNSS解析函数
void gnss_parse_and_log_rmc(const char* nmea);
void gnss_parse_and_log_gga(const char* nmea);
void gnss_parse_and_log_gsa(const char* nmea);

// GNSS任务
void gnss_task(void* pv);

// GNSS初始化
void gnss_init(void);

#endif // GNSS_MODULE_H