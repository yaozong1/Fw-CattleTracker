/*
 * p0_bench.c —— P0 实验固件：给"围栏判定放 GNSS 还是放固件"这个架构决定拿实测数据
 *
 * 要回答的五个问题，每个对应一组命令：
 *
 *   1. 热启动 TTFF —— GNSS 在 Backup 里待 N 秒后，多久能重新定到位？
 *        p ttff <backup_s> [timeout_s]        单次
 *        p ttffseq [timeout_s]                 300 / 1800 / 7200 s 连跑（约 2.7 h）
 *
 *   2. GNSS 连续跟踪的电流
 *        p sleep track <s>                     GNSS 保持跟踪，ESP32 深睡 —— 量电池电流，
 *                                              减去 p sleep timer 的底噪就是 GNSS 跟踪电流
 *
 *   3. ext1 唤醒会不会像 ext0 / gpio_hold_en 那样多吃 405 µA
 *        p sleep timer <s>                     基线：和生产固件一样，只有定时器（应约 112 µA）
 *        p sleep ext1 <s> acc                  + ext1 挂 IO17（加速度计 INT1，高有效）
 *        p sleep ext1 <s> acc pdon             同上但强制 RTC_PERIPH 上电 —— 反向对照
 *
 *   4. LC760Z 自带围栏能不能用、GEOFENCE 引脚(IO3)什么行为
 *        p gf raw <body>                       原样发一条 $<body>*CS，打印 2 s 内所有 $P 应答
 *        p gf here <idx> <r_m>                 以当前位置为圆心设一个圆形围栏
 *        p gf circle <idx> <lat> <lon> <r_m>   指定圆心
 *        p gf read <idx> / p gf off <idx>
 *        p gf mon <s>                          监视 IO3 翻转 + 位置 + 到圆心距离
 *        p sleep track <s> gf high|low         GNSS 跟踪 + 围栏，ESP 深睡等 IO3 翻转唤醒
 *
 *   5. 近区方案里 ESP32 浅睡跟着 NMEA 醒的平均电流
 *        p ls <s>                              GNSS 跟踪，ESP 用 GPIO(IO6=GNSS_TXD) 唤醒浅睡
 *
 * 其它：p status / p gnss on|off / p stop / p help
 *
 * 深睡实验醒来后，开机第一屏会打印「上一次实验」的报告：唤醒原因、哪个脚、睡了多久。
 *
 * 这份固件不碰围栏高压：引脚安全态由 gpio_init() 保证，这里没有任何充放电命令。
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <sys/time.h>

#include "esp_console.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "hardware_config.h"
#include "gpio_module.h"
#include "gnss_module.h"
#include "battery_module.h"
#include "motion_sensor.h"
#include "p0_bench.h"

#if P0_BENCH

static const char *TAG = "P0";

/* ======================= 参数 ======================= */

/* TTFF 测试前，模组要先稳定跟踪这么久，确保星历收全（否则测出来的是"半热"启动） */
#ifndef P0_TTFF_PREWARM_S
#define P0_TTFF_PREWARM_S 60
#endif
/* 预热阶段等首次定位的上限（冷启动 / 室内时会很久） */
#ifndef P0_TTFF_FIRST_FIX_TIMEOUT_S
#define P0_TTFF_FIRST_FIX_TIMEOUT_S 300
#endif
/* p sleep track 进深睡前等定位的上限 */
#ifndef P0_TRACK_FIX_TIMEOUT_S
#define P0_TRACK_FIX_TIMEOUT_S 120
#endif
/* 浅睡实验：每次被 NMEA 唤醒后保持清醒多久，让 gnss_task 把这一串读完 */
#ifndef P0_LS_AWAKE_MS
#define P0_LS_AWAKE_MS 60
#endif

#define P0_REC_MAGIC 0x50304231u   /* "P0B1" */

/* ======================= 深睡实验记录（RTC 内存，跨深睡保留） ======================= */

typedef enum {
    SK_NONE  = 0,
    SK_TIMER = 1,   /* 只有定时器 —— 基线 */
    SK_EXT1  = 2,   /* 定时器 + ext1，GNSS 在 Backup */
    SK_TRACK = 3,   /* GNSS 保持跟踪，可选 ext1 */
} sleep_kind_t;

typedef struct {
    uint32_t magic;
    uint8_t  kind;
    uint8_t  pin_acc;      /* 1 = ext1 挂了 IO17 */
    uint8_t  pin_gf;       /* 1 = ext1 挂了 IO3  */
    uint8_t  level_high;   /* ext1 触发电平 */
    uint8_t  pd_on;        /* 1 = 强制 RTC_PERIPH 上电 */
    uint8_t  gnss_fix;     /* 进睡时 GNSS 是否有定位（track 用） */
    uint32_t req_s;
    int64_t  t_start_us;
    uint32_t seq;          /* 第几次深睡实验 */
} p0_sleep_rec_t;

RTC_DATA_ATTR static p0_sleep_rec_t s_rec;
RTC_DATA_ATTR static uint32_t       s_seq;

/* ======================= 小工具 ======================= */

static int64_t now_us(void)
{
    /* 系统时间由 RTC 定时器维持，跨深睡连续 —— 用它量"实际睡了多久" */
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (int64_t)tv.tv_sec * 1000000LL + tv.tv_usec;
}

static uint32_t ms_since(TickType_t t0)
{
    return (uint32_t)((xTaskGetTickCount() - t0) * portTICK_PERIOD_MS);
}

/* 两点距离（米），短距离用等距圆柱近似就够了 */
static double dist_m(double lat1, double lon1, double lat2, double lon2)
{
    const double R = 6371000.0, d2r = M_PI / 180.0;
    double x = (lon2 - lon1) * d2r * cos((lat1 + lat2) * 0.5 * d2r);
    double y = (lat2 - lat1) * d2r;
    return sqrt(x * x + y * y) * R;
}

/* GNSS 现在有没有在输出：看 window_ms 内行数有没有增加 */
static bool gnss_is_talking(uint32_t window_ms)
{
    uint32_t n0 = gnss_lines_seen;
    vTaskDelay(pdMS_TO_TICKS(window_ms));
    return gnss_lines_seen != n0;
}

/* 后台任务的停止标志：p stop 置位，所有长任务都轮询它 */
static volatile bool s_stop = false;
static volatile bool s_busy = false;

static bool sleep_checked(uint32_t ms)
{
    /* 可被 p stop 打断的延时。返回 false = 被打断 */
    while (ms > 0) {
        if (s_stop) return false;
        uint32_t step = ms > 200 ? 200 : ms;
        vTaskDelay(pdMS_TO_TICKS(step));
        ms -= step;
    }
    return !s_stop;
}

/* 等 RMC 有效。valid_flag 用 gnss_last_location.valid（调用方先清零）*/
static bool wait_fix(uint32_t timeout_s, uint32_t *ms_out)
{
    TickType_t t0 = xTaskGetTickCount();
    while (ms_since(t0) < timeout_s * 1000U) {
        if (gnss_last_location.valid) {
            if (ms_out) *ms_out = ms_since(t0);
            return true;
        }
        if (s_stop) return false;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    return false;
}

/* 确保 GNSS 醒着在输出；它在 Backup 里就用 RESET_N 叫醒 */
static bool gnss_make_awake(void)
{
    if (gnss_is_talking(1200)) return true;
    printf("GNSS 没在输出（应在 Backup），RESET_N 唤醒...\n");
    return gnss_wake_from_standby2();
}

/* ======================= 专有语句旁路打印 ======================= */

static volatile bool s_print_prop = false;

static void prop_hook(const char *line)
{
    if (s_print_prop) {
        printf("  <- %s\n", line);
    }
}

/* ======================= 1) 热启动 TTFF ======================= */

typedef struct {
    uint32_t backup_s;
    bool     ok;
    uint32_t wake_ms;      /* RESET_N 到第一行 NMEA */
    uint32_t ttff_ms;      /* RESET_N 到 RMC=A */
    int      sats;
    float    hdop;
    int      sats_5s;      /* 定到位 5 s 后的卫星数 / HDOP，看是不是"勉强定上" */
    float    hdop_5s;
    double   jump_m;       /* 新定位相对睡前位置的偏移 */
} ttff_result_t;

#define TTFF_MAX_RUNS 8
static ttff_result_t s_ttff_res[TTFF_MAX_RUNS];
static int           s_ttff_n;
static uint32_t      s_ttff_plan[TTFF_MAX_RUNS];
static int           s_ttff_plan_n;
static uint32_t      s_ttff_timeout_s;

static bool ttff_prewarm(void)
{
    if (!gnss_make_awake()) {
        printf("✗ GNSS 唤醒失败，放弃\n");
        return false;
    }
    if (!gnss_rmc_valid) {
        printf("预热：等首次定位（最多 %d s，冷/温启动会比较久）...\n", P0_TTFF_FIRST_FIX_TIMEOUT_S);
        gnss_last_location.valid = false;
        uint32_t ms = 0;
        if (!wait_fix(P0_TTFF_FIRST_FIX_TIMEOUT_S, &ms)) {
            printf("✗ %d s 内没有定位 —— 板子要放到能看见天的地方\n", P0_TTFF_FIRST_FIX_TIMEOUT_S);
            return false;
        }
        printf("  首次定位 %lu ms\n", (unsigned long)ms);
    }
    printf("预热：保持跟踪 %d s，让星历收全...\n", P0_TTFF_PREWARM_S);
    if (!sleep_checked(P0_TTFF_PREWARM_S * 1000U)) return false;
    printf("  预热结束：卫星 %d，HDOP %.1f\n", gnss_sats, gnss_hdop);
    return true;
}

static bool ttff_one(uint32_t backup_s, uint32_t timeout_s, ttff_result_t *r)
{
    memset(r, 0, sizeof(*r));
    r->backup_s = backup_s;

    if (!ttff_prewarm()) return false;

    double lat0 = gnss_last_location.lat_deg, lon0 = gnss_last_location.lon_deg;

    printf("\n进 Backup，停留 %lu s ...\n", (unsigned long)backup_s);
    if (!gnss_enter_standby2()) {
        printf("✗ 没进 Backup（见上面的 entry report）\n");
        return false;
    }
    uint32_t left = backup_s;
    while (left > 0) {
        uint32_t step = left > 60 ? 60 : left;
        if (!sleep_checked(step * 1000U)) return false;
        left -= step;
        if (left > 0) printf("  Backup 中，还剩 %lu s\n", (unsigned long)left);
    }

    gnss_last_location.valid = false;
    TickType_t t0 = xTaskGetTickCount();
    bool woke = gnss_wake_from_standby2();
    r->wake_ms = ms_since(t0);
    if (!woke) {
        printf("✗ 唤醒后没有 NMEA\n");
        return false;
    }

    /* TTFF 从 RESET_N 脉冲开始算，所以 wait_fix 的超时要扣掉唤醒用掉的时间 */
    uint32_t fix_ms = 0;
    r->ok = wait_fix(timeout_s, &fix_ms);
    r->ttff_ms = r->wake_ms + fix_ms;
    if (!r->ok) {
        printf("✗ %lu s 内没有重新定位\n", (unsigned long)timeout_s);
        return true;   /* 测试本身完成了，结果是失败 */
    }
    r->sats = gnss_sats;
    r->hdop = gnss_hdop;
    r->jump_m = dist_m(lat0, lon0, gnss_last_location.lat_deg, gnss_last_location.lon_deg);
    sleep_checked(5000);
    r->sats_5s = gnss_sats;
    r->hdop_5s = gnss_hdop;

    printf("✓ Backup %lu s -> TTFF %lu ms（首行 NMEA %lu ms）  卫星 %d→%d  HDOP %.1f→%.1f  偏移 %.1f m\n",
           (unsigned long)backup_s, (unsigned long)r->ttff_ms, (unsigned long)r->wake_ms,
           r->sats, r->sats_5s, r->hdop, r->hdop_5s, r->jump_m);
    return true;
}

static void ttff_print_table(void)
{
    printf("\n================ TTFF 汇总 ================\n");
    printf("  Backup(s)   结果   TTFF(ms)  首行(ms)  卫星   HDOP   偏移(m)\n");
    for (int i = 0; i < s_ttff_n; ++i) {
        ttff_result_t *r = &s_ttff_res[i];
        printf("  %8lu    %s   %8lu  %8lu  %2d→%-2d  %.1f→%.1f  %6.1f\n",
               (unsigned long)r->backup_s, r->ok ? "OK  " : "FAIL",
               (unsigned long)r->ttff_ms, (unsigned long)r->wake_ms,
               r->sats, r->sats_5s, r->hdop, r->hdop_5s, r->jump_m);
    }
    printf("============================================\n");
    printf("  判读：TTFF < 5 s = 热启动（星历还有效）；20~40 s = 温启动（星历过期）\n");
    printf("        方案 B 远区每次定位的成本 ≈ TTFF × 约 80 mA（电池侧）\n\n");
}

static void ttff_task(void *pv)
{
    s_ttff_n = 0;
    for (int i = 0; i < s_ttff_plan_n && !s_stop; ++i) {
        printf("\n######## TTFF 第 %d/%d 项：Backup %lu s ########\n",
               i + 1, s_ttff_plan_n, (unsigned long)s_ttff_plan[i]);
        if (!ttff_one(s_ttff_plan[i], s_ttff_timeout_s, &s_ttff_res[s_ttff_n])) {
            printf("（第 %d 项中止）\n", i + 1);
            break;
        }
        s_ttff_n++;
    }
    if (s_stop) printf("\n已按 p stop 中止\n");
    if (s_ttff_n > 0) ttff_print_table();
    printf("GNSS 当前保持醒着（跟踪中）。要放回 Backup 用 p gnss off\n");
    s_busy = false;
    vTaskDelete(NULL);
}

/* ======================= 2/3) 深睡实验 ======================= */

static void report_last_sleep(void)
{
    esp_sleep_wakeup_cause_t wc = esp_sleep_get_wakeup_cause();

    printf("\n==================================================\n");
    if (s_rec.magic != P0_REC_MAGIC) {
        printf(" P0 实验固件启动（上电/复位，没有待报告的深睡实验）\n");
        printf("==================================================\n\n");
        s_seq = 0;
        return;
    }

    double slept = (double)(now_us() - s_rec.t_start_us) / 1e6;
    static const char *kind_s[] = { "?", "timer 基线", "ext1", "track(GNSS 跟踪)" };

    printf(" 上一次深睡实验 #%lu 报告\n", (unsigned long)s_rec.seq);
    printf("   类型        : %s%s\n", kind_s[s_rec.kind < 4 ? s_rec.kind : 0],
           s_rec.pd_on ? "  [强制 RTC_PERIPH 上电]" : "");
    if (s_rec.pin_acc || s_rec.pin_gf) {
        printf("   ext1 引脚   : %s%s  触发电平=%s\n",
               s_rec.pin_acc ? "IO17(加速度计) " : "",
               s_rec.pin_gf  ? "IO3(GEOFENCE) "  : "",
               s_rec.level_high ? "高" : "低");
    }
    if (s_rec.kind == SK_TRACK) {
        printf("   进睡时定位  : %s\n", s_rec.gnss_fix ? "有" : "无（GNSS 在搜星，电流偏大）");
    }
    printf("   计划 / 实际 : %lu s / %.1f s\n", (unsigned long)s_rec.req_s, slept);

    switch (wc) {
        case ESP_SLEEP_WAKEUP_TIMER:
            printf("   唤醒原因    : 定时器到期\n");
            break;
        case ESP_SLEEP_WAKEUP_EXT1: {
            uint64_t m = esp_sleep_get_ext1_wakeup_status();
            printf("   唤醒原因    : ext1  %s%s(mask=0x%llx)  —— 提前 %.1f s\n",
                   (m & (1ULL << MOTION_SENSOR_INT1_PIN)) ? "IO17(动了) " : "",
                   (m & (1ULL << GNSS_GEOFENCE_PIN))      ? "IO3(围栏翻转) " : "",
                   (unsigned long long)m, (double)s_rec.req_s - slept);
            if (slept < 2.0) {
                printf("   ⚠ 不到 2 s 就醒 —— 多半是进睡时引脚已经在触发电平\n");
            }
            break;
        }
        default:
            printf("   唤醒原因    : %d（不是定时器也不是 ext1 —— 可能是复位/掉电）\n", (int)wc);
            break;
    }
    printf("==================================================\n\n");
    s_rec.magic = 0;
}

static void do_deep_sleep(sleep_kind_t kind, uint32_t sec, bool acc, bool gf,
                          bool level_high, bool pd_on)
{
    bool track = (kind == SK_TRACK);
    bool have_fix = false;

    if (track) {
        if (!gnss_make_awake()) { printf("✗ GNSS 唤醒失败\n"); return; }
        if (!gnss_rmc_valid) {
            printf("等定位（最多 %d s）—— 没定到位时 GNSS 在搜星，量到的不是跟踪电流\n",
                   P0_TRACK_FIX_TIMEOUT_S);
            gnss_last_location.valid = false;
            wait_fix(P0_TRACK_FIX_TIMEOUT_S, NULL);
        }
        have_fix = gnss_rmc_valid;
        printf("GNSS %s，保持跟踪进深睡\n", have_fix ? "已定位" : "⚠ 仍未定位");
    } else {
        /* 和生产固件一致：GNSS 在 Backup */
        gnss_ensure_standby_fast();
    }

    /* 和生产固件一致：加速度计 10 Hz 低功耗 + 锁存中断（同时把 INT1 引到 IO17） */
    if (!motion_arm_for_sleep()) {
        printf("⚠ 加速度计布防失败\n");
    }

    uint64_t mask = 0;
    if (acc) mask |= 1ULL << MOTION_SENSOR_INT1_PIN;
    if (gf)  mask |= 1ULL << GNSS_GEOFENCE_PIN;

    if (mask) {
        int trig = level_high ? 1 : 0;
        bool bad = false;
        if (acc) {
            gpio_pullup_dis(MOTION_SENSOR_INT1_PIN);
            gpio_pulldown_dis(MOTION_SENSOR_INT1_PIN);
            int lv = gpio_get_level(MOTION_SENSOR_INT1_PIN);
            printf("IO17 当前电平 %d\n", lv);
            if (lv == trig) {
                printf("✗ IO17 已经在触发电平（锁存没清干净 / 板子在动），会立刻醒。放稳再试\n");
                bad = true;
            }
        }
        if (gf) {
            gpio_pullup_dis(GNSS_GEOFENCE_PIN);
            gpio_pulldown_dis(GNSS_GEOFENCE_PIN);
            int lv = gpio_get_level(GNSS_GEOFENCE_PIN);
            printf("IO3 当前电平 %d\n", lv);
            if (lv == trig) {
                printf("✗ IO3 已经在触发电平，会立刻醒。换另一个电平参数，或先确认围栏状态\n");
                bad = true;
            }
        }
        if (bad) return;
        ESP_ERROR_CHECK(esp_sleep_enable_ext1_wakeup(mask,
            level_high ? ESP_EXT1_WAKEUP_ANY_HIGH : ESP_EXT1_WAKEUP_ANY_LOW));
    }
    if (pd_on) {
        esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
    } else {
        /* IDF 在 AUTO 遇到 GPIO/ext0 唤醒时会把全局配置直接改写成 ON 且不改回来
         * （sleep_modes.c get_power_down_flags）。p ls 用过 GPIO 唤醒后就会残留，
         * 这里显式恢复 AUTO，保证和生产固件同一条件：ext1/定时器下 RTC_PERIPH 断电。 */
        esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_AUTO);
    }

    deep_sleep_prepare_pins();
    esp_sleep_enable_timer_wakeup((uint64_t)sec * 1000000ULL);

    s_rec.magic      = P0_REC_MAGIC;
    s_rec.kind       = (uint8_t)kind;
    s_rec.pin_acc    = acc;
    s_rec.pin_gf     = gf;
    s_rec.level_high = level_high;
    s_rec.pd_on      = pd_on;
    s_rec.gnss_fix   = have_fix;
    s_rec.req_s      = sec;
    s_rec.seq        = ++s_seq;
    s_rec.t_start_us = now_us();

    printf("\n>>> 深睡 #%lu，%lu s。现在读电池侧电流（等 5~10 s 让读数稳定）。\n",
           (unsigned long)s_rec.seq, (unsigned long)sec);
    printf(">>> 醒来后开机第一屏会打印本次实验的报告。\n\n");
    fflush(stdout);
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_deep_sleep_start();
}

/* ======================= 4) GEOFENCE ======================= */

static int    s_gf_idx = -1;
static double s_gf_lat, s_gf_lon, s_gf_r;

static void gf_send(const char *body, uint32_t listen_ms)
{
    printf("  -> $%s*CS\n", body);
    s_print_prop = true;
    gnss_send_nmea_cs(body);
    vTaskDelay(pdMS_TO_TICKS(listen_ms));
    s_print_prop = false;
}

static void gf_set_circle(int idx, double lat, double lon, double r)
{
    char body[128];
    /* LC76G 系列格式：W,<Index>,<Status=1 启用>,<Res=0>,<Shape=0 圆>,<Lat0>,<Lon0>,<Radius>
     * LC760Z 是 LC76xZ 系列，不保证认这条 —— 没应答就用 p gf raw 照规格书试 */
    snprintf(body, sizeof(body), "PQTMCFGGEOFENCE,W,%d,1,0,0,%.6f,%.6f,%.1f", idx, lat, lon, r);
    gf_send(body, 1500);
    snprintf(body, sizeof(body), "PQTMCFGGEOFENCE,R,%d", idx);
    gf_send(body, 1500);
    s_gf_idx = idx; s_gf_lat = lat; s_gf_lon = lon; s_gf_r = r;
    printf("  记下圆心 (%.6f, %.6f) r=%.1f m，p gf mon 会显示到圆心的距离\n", lat, lon, r);
}

static uint32_t s_mon_s;

static void gf_mon_task(void *pv)
{
    printf("监视 IO3 %lu s（p stop 结束）。$PQTM 开头的语句也会打出来。\n", (unsigned long)s_mon_s);
    s_print_prop = true;
    int last = gpio_get_level(GNSS_GEOFENCE_PIN);
    TickType_t t0 = xTaskGetTickCount(), tlog = t0;
    printf("  t=0.0s  IO3=%d（初始）\n", last);

    while (ms_since(t0) < s_mon_s * 1000U && !s_stop) {
        int lv = gpio_get_level(GNSS_GEOFENCE_PIN);
        bool tick = ms_since(tlog) >= 5000;
        if (lv != last || tick) {
            double d = -1;
            if (s_gf_idx >= 0 && gnss_rmc_valid) {
                d = dist_m(s_gf_lat, s_gf_lon, gnss_last_location.lat_deg, gnss_last_location.lon_deg);
            }
            printf("  t=%.1fs  IO3=%d%s  fix=%s 卫星=%d HDOP=%.1f",
                   ms_since(t0) / 1000.0, lv, (lv != last) ? " ★翻转" : "",
                   gnss_rmc_valid ? "A" : "V", gnss_sats, gnss_hdop);
            if (d >= 0) printf("  距圆心 %.1f m (r=%.1f，%s)", d, s_gf_r, d > s_gf_r ? "圈外" : "圈内");
            printf("\n");
            last = lv;
            tlog = xTaskGetTickCount();
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    s_print_prop = false;
    printf("监视结束\n");
    s_busy = false;
    vTaskDelete(NULL);
}

/* ======================= 5) 浅睡 + NMEA 唤醒 ======================= */

static uint32_t s_ls_s;

static void ls_task(void *pv)
{
    if (!gnss_make_awake()) { printf("✗ GNSS 唤醒失败\n"); s_busy = false; vTaskDelete(NULL); }

    printf("浅睡实验 %lu s：GNSS 跟踪，ESP32 浅睡，IO%d(GNSS_TXD) 低电平唤醒。\n",
           (unsigned long)s_ls_s, GNSS_UART_RX);
    printf("这期间串口控制台不响应。读电池侧平均电流（万用表打到慢速/平均）。\n");
    printf("注意：唤醒那一瞬间的字节会丢，这个实验只量电流，不看数据完整性。\n\n");
    fflush(stdout);
    vTaskDelay(pdMS_TO_TICKS(100));

    gpio_wakeup_enable((gpio_num_t)GNSS_UART_RX, GPIO_INTR_LOW_LEVEL);
    esp_sleep_enable_gpio_wakeup();
    esp_sleep_enable_timer_wakeup(2000000ULL);   /* 保底，NMEA 停了也不会睡死 */

    uint32_t wakes = 0, lines0 = gnss_lines_seen;
    int64_t t0 = esp_timer_get_time(), awake_us = 0, next_rep = t0 + 10000000LL;

    while (esp_timer_get_time() - t0 < (int64_t)s_ls_s * 1000000LL && !s_stop) {
        esp_light_sleep_start();
        int64_t tw = esp_timer_get_time();
        wakes++;
        vTaskDelay(pdMS_TO_TICKS(P0_LS_AWAKE_MS));
        awake_us += esp_timer_get_time() - tw;

        if (esp_timer_get_time() >= next_rep) {
            double el = (esp_timer_get_time() - t0) / 1e6;
            printf("  %.0fs  唤醒 %lu 次（%.1f/s）  清醒占比 %.0f%%  收到 %lu 行\n",
                   el, (unsigned long)wakes, wakes / el, 100.0 * awake_us / (el * 1e6),
                   (unsigned long)(gnss_lines_seen - lines0));
            fflush(stdout);
            next_rep += 10000000LL;
        }
    }

    gpio_wakeup_disable((gpio_num_t)GNSS_UART_RX);
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_GPIO);
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
    /* GPIO 唤醒让 IDF 把 RTC_PERIPH 的全局配置改成了 ON，恢复 AUTO，免得污染之后的深睡实验 */
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_AUTO);

    double el = (esp_timer_get_time() - t0) / 1e6;
    printf("\n浅睡实验结束：%.0f s，唤醒 %lu 次，清醒占比 %.0f%%\n",
           el, (unsigned long)wakes, 100.0 * awake_us / (el * 1e6));
    printf("清醒占比接近 100%% 说明 NMEA 太密，ESP32 根本睡不下去 ——\n");
    printf("要先把模组输出减到只剩 RMC（用 p gf raw 发对应的输出速率命令）再测。\n\n");
    s_busy = false;
    vTaskDelete(NULL);
}

/* ======================= 命令 ======================= */

static void cmd_help(void)
{
    printf("\n"
"P0 实验固件  —— 所有命令以 p 开头\n"
"\n"
"  p status                         当前状态（GNSS / 引脚 / 电池）\n"
"  p gnss on|off                    GNSS 唤醒 / 进 Backup\n"
"  p stop                           中止正在跑的后台实验\n"
"\n"
"【1 热启动 TTFF】板子要能看见天\n"
"  p ttff <backup_s> [timeout_s]    预热 %ds → Backup <backup_s> → RESET_N 唤醒 → 计时到定位\n"
"  p ttffseq [timeout_s]            300 / 1800 / 7200 s 连跑，约 2.7 h，最后出汇总表\n"
"\n"
"【2/3 深睡电流】先量基线，再逐项对比。电流表串在电池正极\n"
"  p sleep timer <s>                基线（应 ≈112 µA）\n"
"  p sleep ext1 <s> acc             + ext1 挂 IO17。和基线一样 = ext1 不额外耗电\n"
"  p sleep ext1 <s> acc pdon        + 强制 RTC_PERIPH 上电（反向对照，预期 +~405 µA）\n"
"  p sleep track <s>                GNSS 保持跟踪 → 减去基线 = GNSS 跟踪电流\n"
"  p sleep track <s> gf high|low    GNSS 跟踪 + ext1 等 IO3 翻转\n"
"    ext1 实验中晃一下板子，应立刻醒，开机报告显示 IO17\n"
"\n"
"【4 GNSS 自带围栏】\n"
"  p gf raw <body>                  原样发 $<body>*CS，打印 2 s 内的 $P 应答\n"
"                                   例：p gf raw PQTMVERNO\n"
"  p gf here <idx> <r_m>            以当前位置为圆心设圆形围栏\n"
"  p gf circle <idx> <lat> <lon> <r_m>\n"
"  p gf read <idx>   p gf off <idx>\n"
"  p gf mon <s>                     监视 IO3 翻转、位置、到圆心距离\n"
"\n"
"【5 近区浅睡】\n"
"  p ls <s>                         GNSS 跟踪，ESP32 浅睡跟着 NMEA 醒，量平均电流\n"
"\n", P0_TTFF_PREWARM_S);
}

static void cmd_status(void)
{
    bool talking = gnss_is_talking(1200);
    printf("\n");
    printf("  GNSS 输出     : %s\n", talking ? "在输出（醒着）" : "无输出（Backup 或掉电）");
    printf("  RMC           : %s   卫星 %d   HDOP %.1f\n",
           gnss_rmc_valid ? "A 有效" : "V 无效", gnss_sats, gnss_hdop);
    if (gnss_last_location.valid) {
        printf("  最后位置      : %.6f, %.6f  (UTC %s)\n",
               gnss_last_location.lat_deg, gnss_last_location.lon_deg, gnss_last_location.time_utc);
    }
    printf("  IO3  GEOFENCE : %d\n", gpio_get_level(GNSS_GEOFENCE_PIN));
    printf("  IO5  3D_FIX   : %d\n", gpio_get_level(GNSS_3DFIX_PIN));
    printf("  IO17 ACC_INT  : %d\n", gpio_get_level(MOTION_SENSOR_INT1_PIN));
    if (s_gf_idx >= 0) {
        printf("  记下的围栏    : #%d  (%.6f, %.6f) r=%.1f m\n", s_gf_idx, s_gf_lat, s_gf_lon, s_gf_r);
    }
    printf("  VBAT          : %.2f V\n", read_battery_voltage());
    printf("  后台实验      : %s\n", s_busy ? "运行中（p stop 中止）" : "无");
    printf("\n");
}

static bool start_bg(TaskFunction_t fn, const char *name)
{
    if (s_busy) { printf("已有实验在跑，先 p stop\n"); return false; }
    s_stop = false;
    s_busy = true;
    if (xTaskCreate(fn, name, 6144, NULL, 5, NULL) != pdPASS) {
        s_busy = false;
        printf("✗ 建任务失败\n");
        return false;
    }
    return true;
}

static int cmd_p(int argc, char **argv)
{
    if (argc < 2 || strcmp(argv[1], "help") == 0) { cmd_help(); return 0; }
    const char *c = argv[1];

    if (strcmp(c, "status") == 0) { cmd_status(); return 0; }

    if (strcmp(c, "stop") == 0) {
        s_stop = true;
        printf(s_busy ? "已请求中止，等当前步骤结束...\n" : "没有在跑的实验\n");
        return 0;
    }

    if (s_busy && strcmp(c, "gf") != 0) {
        printf("后台实验运行中，先 p stop（p status 可以随时看）\n");
        return 0;
    }

    if (strcmp(c, "gnss") == 0) {
        if (argc < 3) { printf("用法: p gnss on|off\n"); return 0; }
        if (strcmp(argv[2], "on") == 0) gnss_make_awake();
        else                            gnss_enter_standby2();
        return 0;
    }

    if (strcmp(c, "ttff") == 0) {
        if (argc < 3) { printf("用法: p ttff <backup_s> [timeout_s]\n"); return 0; }
        s_ttff_plan[0]   = (uint32_t)strtoul(argv[2], NULL, 10);
        s_ttff_plan_n    = 1;
        s_ttff_timeout_s = (argc > 3) ? (uint32_t)strtoul(argv[3], NULL, 10) : 90;
        start_bg(ttff_task, "p0_ttff");
        return 0;
    }

    if (strcmp(c, "ttffseq") == 0) {
        static const uint32_t plan[] = { 300, 1800, 7200 };
        s_ttff_plan_n = 3;
        memcpy(s_ttff_plan, plan, sizeof(plan));
        s_ttff_timeout_s = (argc > 2) ? (uint32_t)strtoul(argv[2], NULL, 10) : 90;
        printf("TTFF 序列：300 / 1800 / 7200 s，每项超时 %lu s。预计约 2.7 小时\n",
               (unsigned long)s_ttff_timeout_s);
        start_bg(ttff_task, "p0_ttff");
        return 0;
    }

    if (strcmp(c, "sleep") == 0) {
        if (argc < 4) { printf("用法: p sleep timer|ext1|track <s> [acc] [gf high|low] [pdon]\n"); return 0; }
        sleep_kind_t k = SK_NONE;
        if      (strcmp(argv[2], "timer") == 0) k = SK_TIMER;
        else if (strcmp(argv[2], "ext1")  == 0) k = SK_EXT1;
        else if (strcmp(argv[2], "track") == 0) k = SK_TRACK;
        if (k == SK_NONE) { printf("类型只能是 timer / ext1 / track\n"); return 0; }
        uint32_t sec = (uint32_t)strtoul(argv[3], NULL, 10);
        if (sec < 5) { printf("至少 5 s\n"); return 0; }

        bool acc = false, gf = false, pdon = false, level_high = true, level_set = false;
        for (int i = 4; i < argc; ++i) {
            if      (strcmp(argv[i], "acc")  == 0) acc = true;
            else if (strcmp(argv[i], "gf")   == 0) gf = true;
            else if (strcmp(argv[i], "pdon") == 0) pdon = true;
            else if (strcmp(argv[i], "high") == 0) { level_high = true;  level_set = true; }
            else if (strcmp(argv[i], "low")  == 0) { level_high = false; level_set = true; }
            else { printf("不认识的参数 %s\n", argv[i]); return 0; }
        }
        if (k == SK_EXT1 && !acc && !gf) { printf("ext1 至少要挂一个脚：acc 或 gf\n"); return 0; }
        if (k == SK_TIMER && (acc || gf)) { printf("timer 是基线，不挂 ext1。要挂请用 ext1\n"); return 0; }
        if (gf && !level_set) { printf("gf 必须指定 high 或 low（IO3 的极性还没测过）\n"); return 0; }
        if (acc && gf && !level_high) {
            printf("ext1 所有脚共用一个触发电平；IO17 是高有效，所以 acc+gf 只能配 high\n");
            return 0;
        }
        do_deep_sleep(k, sec, acc, gf, level_high, pdon);   /* 成功的话不返回 */
        return 0;
    }

    if (strcmp(c, "gf") == 0) {
        if (argc < 3) { printf("用法见 p help\n"); return 0; }
        const char *g = argv[2];
        if (s_busy && strcmp(g, "raw") != 0) {
            printf("后台实验运行中，gf 只允许 raw\n");
            return 0;
        }
        if (strcmp(g, "raw") == 0 && argc >= 4) {
            gf_send(argv[3], 2000);
        } else if (strcmp(g, "read") == 0 && argc >= 4) {
            char b[48]; snprintf(b, sizeof(b), "PQTMCFGGEOFENCE,R,%d", atoi(argv[3]));
            gf_send(b, 1500);
        } else if (strcmp(g, "off") == 0 && argc >= 4) {
            char b[48]; snprintf(b, sizeof(b), "PQTMCFGGEOFENCE,W,%d,0", atoi(argv[3]));
            gf_send(b, 1500);
            if (s_gf_idx == atoi(argv[3])) s_gf_idx = -1;
        } else if (strcmp(g, "here") == 0 && argc >= 5) {
            if (!gnss_rmc_valid) { printf("✗ 当前没有定位\n"); return 0; }
            gf_set_circle(atoi(argv[3]), gnss_last_location.lat_deg, gnss_last_location.lon_deg,
                          atof(argv[4]));
        } else if (strcmp(g, "circle") == 0 && argc >= 7) {
            gf_set_circle(atoi(argv[3]), atof(argv[4]), atof(argv[5]), atof(argv[6]));
        } else if (strcmp(g, "mon") == 0 && argc >= 4) {
            s_mon_s = (uint32_t)strtoul(argv[3], NULL, 10);
            start_bg(gf_mon_task, "p0_gfmon");
        } else {
            printf("用法见 p help\n");
        }
        return 0;
    }

    if (strcmp(c, "ls") == 0) {
        if (argc < 3) { printf("用法: p ls <s>\n"); return 0; }
        s_ls_s = (uint32_t)strtoul(argv[2], NULL, 10);
        start_bg(ls_task, "p0_ls");
        return 0;
    }

    printf("不认识的命令：%s（p help）\n", c);
    return 0;
}

/* ======================= 入口 ======================= */

void p0_bench_start(void)
{
    report_last_sleep();

    /* 和生产固件的开机顺序一致 */
    deep_sleep_release_pins();
    gpio_init();               /* 含围栏引脚安全态 */
    battery_init();
    if (!motion_sensor_init()) {
        ESP_LOGE(TAG, "加速度计初始化失败 —— ext1 acc 实验做不了");
    }
    gnss_init();
    gnss_set_prop_hook(prop_hook);
    xTaskCreate(gnss_task, "gnss_task", 6144, NULL, 8, NULL);

    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t rc = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    rc.prompt             = "p0>";
    rc.max_cmdline_length = 160;
    esp_console_dev_uart_config_t hw = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&hw, &rc, &repl));

    const esp_console_cmd_t cmd = {
        .command = "p",
        .help    = "P0 实验（p help 看用法）",
        .hint    = NULL,
        .func    = &cmd_p,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
    ESP_ERROR_CHECK(esp_console_register_help_command());
    ESP_ERROR_CHECK(esp_console_start_repl(repl));

    ESP_LOGW(TAG, "===== P0 实验固件 =====  控制台 UART0 @115200，输入 `p help`");
    ESP_LOGW(TAG, "不进自动深睡；GNSS 已启动但状态未知（可能还在 Backup），p status 看");
}

#endif // P0_BENCH
