#include "ct_proto.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <sys/time.h>

#include "esp_log.h"
#include "esp_attr.h"
#include "esp_system.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "hardware_config.h"
#include "mqtt_config.h"
#include "mqtt_module.h"
#include "gnss_module.h"
#include "battery_module.h"
#include "device_id.h"
#include "ct_config.h"

static const char *TAG = "CT";

#define CT_TOPIC_ROOT "ct/v1"

/* ======================= 时间 ======================= */

typedef enum { TS_NONE = 0, TS_GNSS = 1, TS_NET = 2 } ts_src_t;

/* RTC 慢速内存：深睡不丢。冷启动时清零 */
RTC_DATA_ATTR static int64_t  s_clock_shift;    /* 累计校时跳变量（秒），ct_mono_s 扣掉它 */
RTC_DATA_ATTR static uint8_t  s_synced;         /* 系统时间校准过 */
RTC_DATA_ATTR static uint8_t  s_last_src;       /* 最近一次校时的来源 */
RTC_DATA_ATTR static uint32_t s_last_sync_mono; /* 最近一次校时的单调时刻 */

#define CT_TIME_MIN_YEAR 2024
#define CT_TIME_MAX_YEAR 2040

/* 本条上报的时间来源判定窗口：最近这么久之内校过时，算作"本轮"来源 */
#define CT_SYNC_FRESH_S 180

static void tz_utc_once(void)
{
    static bool done = false;
    if (!done) { setenv("TZ", "UTC0", 1); tzset(); done = true; }
}

uint32_t ct_mono_s(void)
{
    return (uint32_t)((int64_t)time(NULL) - s_clock_shift);
}

static void clock_set(time_t t, ts_src_t src)
{
    time_t before = time(NULL);
    struct timeval tv = { .tv_sec = t, .tv_usec = 0 };
    settimeofday(&tv, NULL);
    s_clock_shift   += (int64_t)t - (int64_t)before;
    s_synced         = 1;
    s_last_src       = (uint8_t)src;
    s_last_sync_mono = ct_mono_s();
}

static time_t utc_from_fields(int y, int mo, int d, int h, int mi, int s)
{
    tz_utc_once();
    /* 年份只信 2024–2040：EG800 没收到 NITZ 时 AT+QLTS 会返回自己时钟的默认值
     * （实测 2070-01-01 起算），GNSS 没定到位时 RMC 也可能带默认日期。
     * 这些都必须拒绝，否则错误时钟会被 settimeofday 写进 RTC，深睡也一直带着。 */
    if (y < CT_TIME_MIN_YEAR || y > CT_TIME_MAX_YEAR) {
        ESP_LOGW(TAG, "拒绝不可信的时间 %04d-%02d-%02d %02d:%02d:%02d（年份不在 %d–%d）",
                 y, mo, d, h, mi, s, CT_TIME_MIN_YEAR, CT_TIME_MAX_YEAR);
        return 0;
    }
    if (mo < 1 || mo > 12 || d < 1 || d > 31 ||
        h < 0 || h > 23 || mi < 0 || mi > 59 || s < 0 || s > 60) {
        return 0;
    }
    struct tm tm = {0};
    tm.tm_year = y - 1900; tm.tm_mon = mo - 1; tm.tm_mday = d;
    tm.tm_hour = h; tm.tm_min = mi; tm.tm_sec = s;
    return mktime(&tm);          /* TZ=UTC，等同 timegm */
}

static int two(const char *p) { return (p[0] - '0') * 10 + (p[1] - '0'); }

bool ct_time_from_gnss(void)
{
    const char *t = gnss_last_location.time_utc;   /* hhmmss.sss */
    const char *d = gnss_last_location.date_utc;   /* ddmmyy */
    if (!gnss_last_location.valid || strlen(t) < 6 || strlen(d) != 6) return false;
    time_t u = utc_from_fields(2000 + two(d + 4), two(d + 2), two(d), two(t), two(t + 2), two(t + 4));
    if (u == 0) return false;
    clock_set(u, TS_GNSS);
    return true;
}

/* 模组网络时间：AT+QLTS=1 → +QLTS: "2026/11/20,09:30:00+32,0"（=1 时是 GMT）。
 * 运营商不下发 NITZ 时会是空的，那就算了。模组必须已注网。 */
static bool time_from_modem(void)
{
    char resp[160];
    send_at_command("AT+QLTS=1");
    int n = uart_read_response(resp, sizeof(resp), 1500);
    if (n <= 0) return false;
    const char *p = strstr(resp, "+QLTS: \"");
    if (!p) return false;
    int y, mo, d, h, mi, s;
    if (sscanf(p + 8, "%d/%d/%d,%d:%d:%d", &y, &mo, &d, &h, &mi, &s) != 6) return false;
    time_t u = utc_from_fields(y, mo, d, h, mi, s);
    if (u == 0) return false;
    clock_set(u, TS_NET);
    return true;
}

time_t ct_time_now(const char **src_out)
{
    const char *src = "none";
    time_t now = 0;
    if (s_synced) {
        now = time(NULL);
        if (ct_mono_s() - s_last_sync_mono <= CT_SYNC_FRESH_S) {
            src = (s_last_src == TS_GNSS) ? "gnss" : "net";
        } else {
            src = "rtc";
        }
    }
    if (src_out) *src_out = src;
    return now;
}

static bool time_is_fresh(void)
{
    return s_synced && (ct_mono_s() - s_last_sync_mono <= CT_SYNC_FRESH_S);
}

/* ======================= 序号 ======================= */

RTC_DATA_ATTR static uint32_t s_seq;
RTC_DATA_ATTR static uint32_t s_seq_boot;

static uint32_t next_seq(void)
{
    /* RTC 内存在软件复位时不清零，所以用 boot 计数判断"是不是新的一次上电" */
    if (s_seq_boot != devid_boot_count()) {
        s_seq_boot = devid_boot_count();
        s_seq = 0;
    }
    return ++s_seq;
}

/* ======================= 模组查询 ======================= */

/* AT+CSQ → +CSQ: <rssi>,<ber>；rssi 0..31 → -113..-51 dBm，99 = 未知 */
int modem_get_csq_dbm(int *raw_rssi)
{
    char resp[96];
    send_at_command("AT+CSQ");
    int n = uart_read_response(resp, sizeof(resp), 1000);
    if (raw_rssi) *raw_rssi = 99;
    if (n <= 0) return 0;
    const char *p = strstr(resp, "+CSQ: ");
    if (!p) return 0;
    int r = atoi(p + 6);
    if (raw_rssi) *raw_rssi = r;
    if (r < 0 || r > 31) return 0;
    return -113 + 2 * r;
}

static void modem_get_iccid(char *out, size_t out_sz)
{
    char resp[96];
    out[0] = '\0';
    send_at_command("AT+QCCID");
    int n = uart_read_response(resp, sizeof(resp), 1000);
    if (n <= 0) return;
    const char *p = strstr(resp, "+QCCID: ");
    if (!p) return;
    p += 8;
    size_t i = 0;
    while (i < out_sz - 1 && ((p[i] >= '0' && p[i] <= '9') || p[i] == 'F' || p[i] == 'f')) {
        out[i] = p[i];
        i++;
    }
    out[i] = '\0';
}

/* ======================= JSON ======================= */

const char *ct_reason_str(ct_reason_t r)
{
    switch (r) {
        case CT_REASON_BOOT:      return "boot";
        case CT_REASON_MOTION:    return "motion";
        case CT_REASON_HEARTBEAT: return "heartbeat";
        case CT_REASON_MANUAL:    return "manual";
        default:                  return "none";
    }
}

static const char *reset_str(void)
{
    switch (esp_reset_reason()) {
        case ESP_RST_POWERON:   return "poweron";
        case ESP_RST_DEEPSLEEP: return "deepsleep";
        case ESP_RST_SW:        return "sw";
        case ESP_RST_PANIC:     return "panic";
        case ESP_RST_INT_WDT:
        case ESP_RST_TASK_WDT:
        case ESP_RST_WDT:       return "wdt";
        case ESP_RST_BROWNOUT:  return "brownout";
        default:                return "other";
    }
}

/* cJSON 打印 double 时会退到 %.17g，先按精度四舍五入，输出才干净 */
static double rnd(double v, double scale) { return round(v * scale) / scale; }

static char *build_status(const char *iccid)
{
    const ct_cfg_t *c = ct_cfg();
    cJSON *o = cJSON_CreateObject();
    if (!o) return NULL;

    cJSON_AddNumberToObject(o, "ts", (double)ct_time_now(NULL));
    cJSON_AddStringToObject(o, "id", devid_get());
    cJSON_AddStringToObject(o, "imei", devid_imei());
    if (iccid && iccid[0]) cJSON_AddStringToObject(o, "iccid", iccid);
    cJSON_AddStringToObject(o, "fw", CT_FW_VERSION);
    cJSON_AddStringToObject(o, "hw", CT_HW_NAME);
    cJSON_AddNumberToObject(o, "boot", devid_boot_count());
    cJSON_AddStringToObject(o, "reset", reset_str());

    cJSON *a = cJSON_AddObjectToObject(o, "applied");
    cJSON_AddNumberToObject(a, "cfg_ver", c->ver);
    cJSON_AddNumberToObject(a, "stim_ver", ct_stim_ver());
    cJSON_AddNumberToObject(a, "all_stim_ver", ct_all_stim_ver());
    cJSON_AddNumberToObject(a, "fence_ver", 0);        /* 围栏下行在协议 v1.1 定义 */

    cJSON *g = cJSON_AddObjectToObject(o, "cfg");
    cJSON_AddNumberToObject(g, "heartbeat_min", c->heartbeat_min);
    cJSON_AddNumberToObject(g, "motion_min_s", c->motion_min_s);
    cJSON_AddNumberToObject(g, "check_s", c->check_s);
    cJSON_AddNumberToObject(g, "fix_timeout_s", c->fix_timeout_s);
    cJSON_AddNumberToObject(g, "hb_fix_timeout_s", c->hb_fix_timeout_s);
    cJSON_AddNumberToObject(g, "motion_th_mg", c->motion_th_mg);

    const ct_dl_err_t *e = ct_last_err();
    if (e) {
        cJSON *er = cJSON_AddObjectToObject(o, "err");
        cJSON_AddStringToObject(er, "topic", e->topic);
        cJSON_AddNumberToObject(er, "ver", e->ver);
        cJSON_AddStringToObject(er, "reason", e->reason);
    } else {
        cJSON_AddNullToObject(o, "err");
    }

    char *s = cJSON_PrintUnformatted(o);
    cJSON_Delete(o);
    return s;
}

static char *build_telemetry(const ct_report_t *r, int rssi_dbm, uint32_t seq)
{
    cJSON *o = cJSON_CreateObject();
    if (!o) return NULL;

    const char *src = "none";
    time_t ts = ct_time_now(&src);
    cJSON_AddNumberToObject(o, "ts", (double)ts);
    cJSON_AddStringToObject(o, "ts_src", src);
    cJSON_AddNumberToObject(o, "boot", devid_boot_count());
    cJSON_AddNumberToObject(o, "seq", seq);
    cJSON_AddStringToObject(o, "reason", ct_reason_str(r->reason));

    if (r->fix_ok && r->fix.has_pos) {
        /* 坐标、海拔、HDOP、卫星数都取被接受的那一句 GGA（同一秒、配套）；
         * 速度、航向只有 RMC 有，取最近一句 RMC */
        cJSON *g = cJSON_AddObjectToObject(o, "gps");
        cJSON_AddNumberToObject(g, "lat", rnd(r->fix.lat_deg, 1e6));
        cJSON_AddNumberToObject(g, "lng", rnd(r->fix.lon_deg, 1e6));
        if (r->fix.alt_valid) cJSON_AddNumberToObject(g, "alt", rnd(r->fix.alt_m, 10));
        cJSON_AddNumberToObject(g, "spd_kmh", rnd(gnss_last_location.spd_kn * 1.852, 10));
        cJSON_AddNumberToObject(g, "hdg", (int)(gnss_last_location.crs_deg + 0.5));
        cJSON_AddNumberToObject(g, "hdop", rnd(r->fix.hdop, 10));
        cJSON_AddNumberToObject(g, "sats", r->fix.sats);
        if (r->ttff_s >= 0) cJSON_AddNumberToObject(g, "ttff_s", rnd(r->ttff_s, 10));
    } else {
        cJSON_AddNullToObject(o, "gps");
        if (r->gnss_tried) cJSON_AddTrueToObject(o, "fix_fail");
    }

    cJSON *b = cJSON_AddObjectToObject(o, "bat");
    cJSON_AddNumberToObject(b, "mv", (int)(r->vbat * 1000.0f + 0.5f));
    cJSON_AddNumberToObject(b, "soc", estimate_soc_from_voltage(r->vbat));

    if (r->act_event) {
        cJSON *a = cJSON_AddObjectToObject(o, "act");
        cJSON_AddStringToObject(a, "event", r->act_event);
        cJSON_AddNumberToObject(a, "act_mg", r->act_mg);
        if (r->still_min >= 0) cJSON_AddNumberToObject(a, "still_min", r->still_min);
    }

    if (rssi_dbm < 0) {
        cJSON *n = cJSON_AddObjectToObject(o, "net");
        cJSON_AddNumberToObject(n, "rssi", rssi_dbm);
    }

    /* 实际生效的许可 = 本机开关 AND 全群总闸。v1.0 没有围栏逻辑，许可不会引发任何动作 */
    cJSON_AddBoolToObject(o, "stim", ct_stim_effective());

    char *s = cJSON_PrintUnformatted(o);
    cJSON_Delete(o);
    return s;
}

/* ======================= 一次上报 ======================= */

static void topic_of(char *out, size_t sz, const char *leaf)
{
    snprintf(out, sz, "%s/%s/%s", CT_TOPIC_ROOT, devid_get(), leaf);
}

#define CT_TOPIC_ALL_STIM CT_TOPIC_ROOT "/all/stim"

static char s_t_cfg[64], s_t_stim[64];

/* +QMTRECV 回调：按 topic 分发到 ct_config */
static void on_downlink(const char *topic, const char *payload, int len)
{
    if (strcmp(topic, s_t_cfg) == 0)                 ct_downlink_apply(CT_DL_CONFIG, payload, len);
    else if (strcmp(topic, s_t_stim) == 0)           ct_downlink_apply(CT_DL_STIM, payload, len);
    else if (strcmp(topic, CT_TOPIC_ALL_STIM) == 0)  ct_downlink_apply(CT_DL_ALL_STIM, payload, len);
    else ESP_LOGW(TAG, "不认识的下行 topic：%s", topic);
}

bool ct_report(const ct_report_t *r)
{
    char topic[64];
    /* 序号每次上报只取一次：重试时沿用，后台才能用 (id, boot, seq) 认出重复 */
    uint32_t seq = next_seq();

    for (int attempt = 1; attempt <= MQTT_SEND_MAX_RETRY; ++attempt) {
        ESP_LOGI(TAG, "上报 [%s] 第 %d/%d 次", ct_reason_str(r->reason), attempt, MQTT_SEND_MAX_RETRY);

        if (!mqtt_init_network()) continue;          /* 里面会读 IMEI → devid_set_imei */

        /* 本轮没有 GNSS 校时，就用模组网络时间校一次 */
        if (!time_is_fresh()) {
            ESP_LOGI(TAG, "时间：%s", time_from_modem() ? "已用模组网络时间校准" : "模组没有网络时间");
        }
        int  rssi = modem_get_csq_dbm(NULL);
        char iccid[24];
        modem_get_iccid(iccid, sizeof(iccid));

        if (!mqtt_connect_broker()) continue;        /* Client ID = devid_get() */

        /* 下行：订阅 → 收 retained → 校验应用（协议 §3.1）。必须在 status 之前，
         * status.applied 才是"现在在用什么"的回执。订阅失败不影响上报。 */
        topic_of(s_t_cfg, sizeof(s_t_cfg), "config");
        topic_of(s_t_stim, sizeof(s_t_stim), "stim");
        const char *subs[] = { s_t_cfg, s_t_stim, CT_TOPIC_ALL_STIM };
        if (!mqtt_subscribe_collect(subs, 3, CT_DOWNLINK_WAIT_MS, on_downlink)) {
            ESP_LOGW(TAG, "订阅下行失败，本轮只上报");
        }

        bool ok = false;
        char *st = build_status(iccid);
        char *tm = build_telemetry(r, rssi, seq);
        if (st && tm) {
            topic_of(topic, sizeof(topic), "status");
            ESP_LOGI(TAG, "%s  %s", topic, st);
            bool ok_st = mqtt_publish_ex(topic, st, 1, true);

            topic_of(topic, sizeof(topic), "telemetry");
            ESP_LOGI(TAG, "%s  %s", topic, tm);
            bool ok_tm = mqtt_publish_ex(topic, tm, 1, false);

            ok = ok_st && ok_tm;
            if (!ok) ESP_LOGW(TAG, "发布结果 status=%d telemetry=%d", ok_st, ok_tm);
        } else {
            ESP_LOGE(TAG, "JSON 生成失败（内存？）");
        }
        cJSON_free(st);
        cJSON_free(tm);

        mqtt_disconnect_broker();
        if (ok) return true;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    return false;
}
