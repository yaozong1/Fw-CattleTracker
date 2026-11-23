#include "ct_config.h"

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stddef.h>

#include "esp_log.h"
#include "esp_attr.h"
#include "nvs.h"
#include "cJSON.h"
#include "hardware_config.h"

static const char *TAG = "CTCFG";

#define NVS_NS      "ct"
#define NVS_KEY_DL  "dl"
#define DL_MAGIC    0x43544431u     /* "CTD1"：结构体改了就换 magic，旧数据作废回默认 */

/* ---------------- 默认值与范围（必须和协议 §5.1、FleetManager collar-types.ts 一致） ---------------- */

typedef struct {
    const char *key;
    uint16_t    def;
    uint16_t    min;
    uint16_t    max;
    size_t      off;               /* 在 ct_cfg_t 里的偏移 */
} cfg_field_t;

#define F(k, d, lo, hi) { #k, d, lo, hi, offsetof(ct_cfg_t, k) }
static const cfg_field_t s_fields[] = {
    F(heartbeat_min,    CT_CFG_HEARTBEAT_MIN,    5,   1440),
    F(motion_min_s,     CT_CFG_MOTION_MIN_S,     60,  3600),
    F(check_s,          CT_CFG_CHECK_S,          10,  600),
    F(fix_timeout_s,    CT_CFG_FIX_TIMEOUT_S,    30,  300),
    F(hb_fix_timeout_s, CT_CFG_HB_FIX_TIMEOUT_S, 0,   120),
    F(motion_th_mg,     MOTION_MEAN_DELTA_TH_MG, 5,   200),
};
#undef F
#define N_FIELDS (sizeof(s_fields) / sizeof(s_fields[0]))

static uint16_t *field_ptr(ct_cfg_t *c, const cfg_field_t *f)
{
    return (uint16_t *)((uint8_t *)c + f->off);
}

static void cfg_defaults(ct_cfg_t *c)
{
    memset(c, 0, sizeof(*c));
    for (size_t i = 0; i < N_FIELDS; ++i) *field_ptr(c, &s_fields[i]) = s_fields[i].def;
    c->ver = 0;
}

/* ---------------- 状态 ---------------- */

typedef struct {
    uint32_t magic;
    ct_cfg_t cfg;
    uint32_t stim_ver;
    uint32_t astim_ver;
    uint8_t  stim_en;
    uint8_t  astim_set;            /* 全群总闸收到过 */
    uint8_t  astim_en;
    uint8_t  _pad;
} dl_state_t;

static dl_state_t s_st;
static bool       s_loaded = false;

RTC_DATA_ATTR static ct_dl_err_t s_err;   /* 深睡保留，断电清空 */

static void state_defaults(void)
{
    memset(&s_st, 0, sizeof(s_st));
    s_st.magic = DL_MAGIC;
    cfg_defaults(&s_st.cfg);
}

static void state_save(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open 失败，下行只在本次上电内有效");
        return;
    }
    if (nvs_set_blob(h, NVS_KEY_DL, &s_st, sizeof(s_st)) != ESP_OK || nvs_commit(h) != ESP_OK) {
        ESP_LOGE(TAG, "NVS 写入失败");
    }
    nvs_close(h);
}

void ct_cfg_load(void)
{
    state_defaults();
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        dl_state_t tmp;
        size_t sz = sizeof(tmp);
        if (nvs_get_blob(h, NVS_KEY_DL, &tmp, &sz) == ESP_OK && sz == sizeof(tmp) && tmp.magic == DL_MAGIC) {
            s_st = tmp;
        }
        nvs_close(h);
    }
    s_loaded = true;
    ESP_LOGI(TAG, "下行状态：cfg v%lu（心跳 %u min）  stim v%lu=%s  all/stim %s",
             (unsigned long)s_st.cfg.ver, s_st.cfg.heartbeat_min,
             (unsigned long)s_st.stim_ver, s_st.stim_en ? "ON" : "off",
             s_st.astim_set ? (s_st.astim_en ? "allowed" : "BLOCKED") : "未收到（不拦截）");
}

const ct_cfg_t *ct_cfg(void)
{
    if (!s_loaded) {          /* 防御：没调 ct_cfg_load 时也给出默认值 */
        state_defaults();
        s_loaded = true;
    }
    return &s_st.cfg;
}

uint32_t ct_stim_ver(void)        { ct_cfg(); return s_st.stim_ver; }
uint32_t ct_all_stim_ver(void)    { ct_cfg(); return s_st.astim_set ? s_st.astim_ver : 0; }
bool     ct_stim_enabled(void)    { ct_cfg(); return s_st.stim_en != 0; }
bool     ct_stim_effective(void)
{
    ct_cfg();
    return s_st.stim_en && (!s_st.astim_set || s_st.astim_en);
}
const ct_dl_err_t *ct_last_err(void) { return s_err.has ? &s_err : NULL; }

/* ---------------- 下行处理 ---------------- */

static const char *topic_name(ct_dl_topic_t t)
{
    switch (t) {
        case CT_DL_CONFIG:   return "config";
        case CT_DL_STIM:     return "stim";
        default:             return "all/stim";
    }
}

static void reject(ct_dl_topic_t t, uint32_t ver, const char *reason)
{
    s_err.has = true;
    snprintf(s_err.topic, sizeof(s_err.topic), "%s", topic_name(t));
    s_err.ver = ver;
    snprintf(s_err.reason, sizeof(s_err.reason), "%s", reason);
    ESP_LOGW(TAG, "拒收 %s v%lu：%s（继续使用之前的值）", s_err.topic, (unsigned long)ver, reason);
}

static void clear_err_if(ct_dl_topic_t t)
{
    if (s_err.has && strcmp(s_err.topic, topic_name(t)) == 0) s_err.has = false;
}

/* 取正整数。是数字、是整数、在 [lo, hi] 内才算 */
static bool get_uint(const cJSON *o, const char *key, double lo, double hi, uint32_t *out, bool *present)
{
    const cJSON *v = cJSON_GetObjectItemCaseSensitive(o, key);
    *present = (v != NULL);
    if (!v) return true;
    if (!cJSON_IsNumber(v)) return false;
    double d = v->valuedouble;
    if (d != floor(d) || d < lo || d > hi) return false;
    *out = (uint32_t)d;
    return true;
}

static void apply_config(cJSON *o, uint32_t ver)
{
    ct_cfg_t n;
    cfg_defaults(&n);             /* 省略的字段取默认值（协议 §5.1） */
    char why[64];
    for (size_t i = 0; i < N_FIELDS; ++i) {
        const cfg_field_t *f = &s_fields[i];
        uint32_t v = 0;
        bool present = false;
        if (!get_uint(o, f->key, f->min, f->max, &v, &present)) {
            snprintf(why, sizeof(why), "%s must be an integer %u-%u", f->key, f->min, f->max);
            reject(CT_DL_CONFIG, ver, why);
            return;
        }
        if (present) *field_ptr(&n, f) = (uint16_t)v;
    }
    n.ver = ver;
    s_st.cfg = n;
    state_save();
    clear_err_if(CT_DL_CONFIG);
    ESP_LOGW(TAG, "已应用 config v%lu：心跳 %u min｜动作间隔 %u s｜检查 %u s｜定位 %u/%u s｜门限 %u mg（下一轮生效）",
             (unsigned long)ver, n.heartbeat_min, n.motion_min_s, n.check_s,
             n.fix_timeout_s, n.hb_fix_timeout_s, n.motion_th_mg);
}

static void apply_switch(ct_dl_topic_t t, cJSON *o, uint32_t ver)
{
    const cJSON *en = cJSON_GetObjectItemCaseSensitive(o, "enabled");
    if (!cJSON_IsBool(en)) {
        reject(t, ver, "enabled must be true/false");
        return;
    }
    bool on = cJSON_IsTrue(en);
    if (t == CT_DL_STIM) {
        s_st.stim_ver = ver;
        s_st.stim_en  = on;
    } else {
        s_st.astim_set = 1;
        s_st.astim_ver = ver;
        s_st.astim_en  = on;
    }
    state_save();
    clear_err_if(t);
    ESP_LOGW(TAG, "已应用 %s v%lu：%s  →  实际刺激许可 %s",
             topic_name(t), (unsigned long)ver, on ? "ON" : "off",
             ct_stim_effective() ? "ON" : "off");
}

static uint32_t applied_ver(ct_dl_topic_t t)
{
    switch (t) {
        case CT_DL_CONFIG: return s_st.cfg.ver;
        case CT_DL_STIM:   return s_st.stim_ver;
        default:           return s_st.astim_set ? s_st.astim_ver : 0;
    }
}

void ct_downlink_apply(ct_dl_topic_t t, const char *payload, int len)
{
    ct_cfg();

    /* retained 被清空：恢复默认（协议 §5 第 4 条） */
    if (len <= 0) {
        if (t == CT_DL_CONFIG && s_st.cfg.ver != 0) {
            cfg_defaults(&s_st.cfg);
            ESP_LOGW(TAG, "config 被清空 → 恢复固件默认值");
        } else if (t == CT_DL_STIM && (s_st.stim_ver || s_st.stim_en)) {
            s_st.stim_ver = 0;
            s_st.stim_en  = 0;
            ESP_LOGW(TAG, "stim 被清空 → 本机许可关闭");
        } else if (t == CT_DL_ALL_STIM && s_st.astim_set) {
            s_st.astim_set = s_st.astim_en = 0;
            s_st.astim_ver = 0;
            ESP_LOGW(TAG, "all/stim 被清空 → 总闸回到未设置（不拦截）");
        } else {
            return;
        }
        clear_err_if(t);
        state_save();
        return;
    }

    cJSON *o = cJSON_ParseWithLength(payload, (size_t)len);
    if (!cJSON_IsObject(o)) {
        reject(t, 0, "not a JSON object");
        cJSON_Delete(o);
        return;
    }
    uint32_t ver = 0;
    bool present = false;
    if (!get_uint(o, "ver", 1, 4294967295.0, &ver, &present) || !present) {
        reject(t, 0, "ver missing or not a positive integer");
        cJSON_Delete(o);
        return;
    }
    if (ver == applied_ver(t)) {
        ESP_LOGI(TAG, "%s v%lu 已在使用，忽略", topic_name(t), (unsigned long)ver);
        cJSON_Delete(o);
        return;
    }

    if (t == CT_DL_CONFIG) apply_config(o, ver);
    else                   apply_switch(t, o, ver);
    cJSON_Delete(o);
}
