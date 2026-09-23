#include "gnss_module.h"
#include "hardware_config.h"
#include "mqtt_config.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "GNSS";

// GNSS 简易状态（供日志/后续逻辑参考）
volatile bool  gnss_has_fix   = false;   // 是否已定位
volatile int   gnss_fix_q     = 0;       // GGA fix质量(0无fix,1GPS,2DGPS,4RTK fix,5RTK float...)
volatile int   gnss_dim       = 0;       // GSA 维度(1无,2=2D,3=3D)
volatile int   gnss_sats      = 0;       // GGA 在用卫星数
volatile float gnss_hdop      = 0.0f;    // GGA/GSA HDOP

// 记录各句型是否已见及状态，用于综合摘要
volatile bool gnss_seen_rmc = false;
volatile bool gnss_seen_gga = false;
volatile bool gnss_seen_gsa = false;
volatile bool gnss_rmc_valid = false; // RMC: A=valid

// TTFF（首次定位用时）测量标志
static volatile bool       gnss_ttff_measuring = false;
static volatile TickType_t gnss_ttff_start     = 0;

// 待机/唤醒可视化：最近完成一行NMEA的时间与累计行数
volatile TickType_t gnss_last_sentence_tick;
volatile uint32_t   gnss_lines_seen;

// PMTK待机ACK检测标志
static volatile bool gnss_ack_pmtk161;
static volatile bool gnss_ack_pmtk225;

// 是否认为已进入待机（用于调试观测）
static volatile bool gnss_in_standby = false;

// UBX ACK/NAK 标志（仅关心 CFG-SLEEP 0x06/0x41）
static volatile bool gnss_ack_cfg_sleep = false;
static volatile bool gnss_nak_cfg_sleep = false;

// 基准测试结束后，停止 GNSS 接收
volatile bool gnss_should_stop = false;

// 位置保存结构（供 MQTT 上报/业务循环使用）
gnss_location_t gnss_last_location;  // 最近一次从RMC解析到的位置信息（有效时 valid=true）
gnss_location_t gnss_saved_location; // 每次进入Backup前的快照

// GNSS控制函数
void gnss_enable(bool on)
{
    gpio_set_level(GNSS_EN_PIN, on ? 1 : 0);
}

// RESET_N 受三极管反相：
void gnss_reset_assert(void)   
{ 
    gpio_set_level(GNSS_RST_PIN, 1); // IO2=1 -> Q6导通 -> RESET_N=Low(复位有效)
} 

void gnss_reset_release(void)  
{ 
    gpio_set_level(GNSS_RST_PIN, 0); // IO2=0 -> Q6截止 -> RESET_N=High(释放)
}

void gnss_log_fix_summary_if_changed(void)
{
    // 使用静态快照，只有有变化时才打印，避免刷屏
    static bool  last_has_fix   = false;
    static int   last_fix_q     = -1;
    static int   last_dim       = -1;
    static int   last_sats      = -1;
    static float last_hdop      = -1.0f;
    static bool  last_rmc_valid = false;
    static bool  last_seen_rmc  = false;
    static bool  last_seen_gga  = false;
    static bool  last_seen_gsa  = false;

    bool changed = false;
    if (last_has_fix != gnss_has_fix || last_fix_q != gnss_fix_q || last_dim != gnss_dim ||
        last_sats != gnss_sats || last_hdop != gnss_hdop || last_rmc_valid != gnss_rmc_valid ||
        last_seen_rmc != gnss_seen_rmc || last_seen_gga != gnss_seen_gga || last_seen_gsa != gnss_seen_gsa) {
        changed = true;
    }

    if (!changed) return;

    // 构造摘要
    char rmc_str[32], gga_str[64], gsa_str[32];
    if (gnss_seen_rmc) {
        snprintf(rmc_str, sizeof(rmc_str), "%s", gnss_rmc_valid ? "VALID" : "INVALID");
    } else {
        snprintf(rmc_str, sizeof(rmc_str), "N/A");
    }

    if (gnss_seen_gga) {
        snprintf(gga_str, sizeof(gga_str), "fixQ=%d sats=%d HDOP=%.1f", gnss_fix_q, gnss_sats, gnss_hdop);
    } else {
        snprintf(gga_str, sizeof(gga_str), "N/A");
    }

    if (gnss_seen_gsa) {
        const char* dim_str = (gnss_dim == 3) ? "3D" : (gnss_dim == 2) ? "2D" : "NO";
        snprintf(gsa_str, sizeof(gsa_str), "Dim=%s HDOP=%.1f", dim_str, gnss_hdop);
    } else {
        snprintf(gsa_str, sizeof(gsa_str), "N/A");
    }

    const char* overall = gnss_has_fix ? "FIXED" : "NO FIX";
    GNSS_PARSE_LOG("FIX SUMMARY: RMC=%s | GGA:%s | GSA:%s => %s", rmc_str, gga_str, gsa_str, overall);

    // 更新快照
    last_has_fix   = gnss_has_fix;
    last_fix_q     = gnss_fix_q;
    last_dim       = gnss_dim;
    last_sats      = gnss_sats;
    last_hdop      = gnss_hdop;
    last_rmc_valid = gnss_rmc_valid;
    last_seen_rmc  = gnss_seen_rmc;
    last_seen_gga  = gnss_seen_gga;
    last_seen_gsa  = gnss_seen_gsa;
}

void gnss_reset_state_for_ttff(void)
{
    gnss_has_fix   = false;
    gnss_fix_q     = 0;
    gnss_dim       = 0;
    gnss_sats      = 0;
    gnss_hdop      = 0.0f;

    gnss_seen_rmc  = false;
    gnss_seen_gga  = false;
    gnss_seen_gsa  = false;
    gnss_rmc_valid = false;

    gnss_ttff_measuring = true;
    gnss_ttff_start     = xTaskGetTickCount();
}

void gnss_report_ttff_if_needed(void)
{
    if (!gnss_ttff_measuring || !gnss_has_fix) return;
    if (gnss_ttff_start == 0) return;
    uint32_t ms = (uint32_t)((xTaskGetTickCount() - gnss_ttff_start) * 1000 / configTICK_RATE_HZ);
    if (ms > 0) {
        ESP_LOGI(TAG, "TTFF: %u ms (%.2f s)", (unsigned)ms, ms / 1000.0f);
        gnss_ttff_measuring = false;
    }
}

// 发送一条 NMEA 命令到 GNSS（自动追加 \r\n）
void gnss_send_nmea(const char* sentence)
{
    if (!sentence) return;
    uart_write_bytes(GNSS_UART_NUM, sentence, strlen(sentence));
    uart_write_bytes(GNSS_UART_NUM, "\r\n", 2);
}

// 发送 UBX 二进制消息（B5 62 + cls + id + len + payload + CK_A CK_B）
void gnss_send_ubx(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len)
{
    uint8_t hdr[6];
    hdr[0] = 0xB5; hdr[1] = 0x62; hdr[2] = cls; hdr[3] = id; hdr[4] = (uint8_t)(len & 0xFF); hdr[5] = (uint8_t)(len >> 8);
    uint8_t ckA = 0, ckB = 0;
    // 校验覆盖 cls/id/len/payload（不含 0xB5 0x62）
    for (int i = 2; i < 6; ++i) { ckA = (uint8_t)(ckA + hdr[i]); ckB = (uint8_t)(ckB + ckA); }
    for (uint16_t i = 0; i < len; ++i)      { ckA = (uint8_t)(ckA + payload[i]); ckB = (uint8_t)(ckB + ckA); }
    // 打印即将发送的 UBX 帧
    char hex[16 * 3]; int pos = 0;
    GNSS_UBX_LOG("TX UBX cls=0x%02X id=0x%02X len=%u", cls, id, (unsigned)len);
    // 头
    for (int i = 0; i < 6 && pos < (int)sizeof(hex) - 3; ++i) pos += snprintf(hex + pos, sizeof(hex) - pos, "%02X ", hdr[i]);
    GNSS_UBX_LOG("  hdr: %s", hex);
    // 负载
    if (len && payload) {
        pos = 0;
        for (uint16_t i = 0; i < len && pos < (int)sizeof(hex) - 3; ++i) pos += snprintf(hex + pos, sizeof(hex) - pos, "%02X ", payload[i]);
        GNSS_UBX_LOG("  payload: %s", hex);
    }
    // 校验
    GNSS_UBX_LOG("  cks: %02X %02X", ckA, ckB);

    // 发送
    uart_write_bytes(GNSS_UART_NUM, (const char*)hdr, sizeof(hdr));
    if (len && payload) uart_write_bytes(GNSS_UART_NUM, (const char*)payload, len);
    uint8_t cks[2] = { ckA, ckB };
    uart_write_bytes(GNSS_UART_NUM, (const char*)cks, 2);
}

// 发送 Quectel 专用二进制消息（F1 D9 + cls + id + len + payload + CK_A CK_B）
void gnss_send_quectel_bin(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len)
{
    uint8_t hdr[6];
    hdr[0] = 0xF1; hdr[1] = 0xD9; hdr[2] = cls; hdr[3] = id; hdr[4] = (uint8_t)(len & 0xFF); hdr[5] = (uint8_t)(len >> 8);
    uint8_t ckA = 0, ckB = 0;
    for (int i = 2; i < 6; ++i) { ckA = (uint8_t)(ckA + hdr[i]); ckB = (uint8_t)(ckB + ckA); }
    for (uint16_t i = 0; i < len; ++i)      { ckA = (uint8_t)(ckA + payload[i]); ckB = (uint8_t)(ckB + ckA); }

    // 调试打印
    GNSS_UBX_LOG("TX QBIN cls=0x%02X id=0x%02X len=%u", cls, id, (unsigned)len);
    {
        char hex[16 * 3]; int pos = 0;
        for (int i = 0; i < 6 && pos < (int)sizeof(hex) - 3; ++i) pos += snprintf(hex + pos, sizeof(hex) - pos, "%02X ", hdr[i]);
        GNSS_UBX_LOG("  hdr: %s", hex);
        if (len && payload) {
            pos = 0;
            for (uint16_t i = 0; i < len && pos < (int)sizeof(hex) - 3; ++i) pos += snprintf(hex + pos, sizeof(hex) - pos, "%02X ", payload[i]);
            GNSS_UBX_LOG("  payload: %s", hex);
        }
        GNSS_UBX_LOG("  cks: %02X %02X", ckA, ckB);
    }

    // 发送
    uart_write_bytes(GNSS_UART_NUM, (const char*)hdr, sizeof(hdr));
    if (len && payload) uart_write_bytes(GNSS_UART_NUM, (const char*)payload, len);
    uint8_t cks[2] = { ckA, ckB };
    uart_write_bytes(GNSS_UART_NUM, (const char*)cks, 2);
}

// 构造并发送 Quectel CFG-SLEEP（0x06,0x41）：payload = sleep_ms(LE,4B) + lp_mode(1B)
static inline void gnss_send_cfg_sleep_quectel(uint32_t sleep_ms, uint8_t low_power_mode)
{
    uint8_t p[5];
    p[0] = (uint8_t)(sleep_ms & 0xFF);
    p[1] = (uint8_t)((sleep_ms >> 8) & 0xFF);
    p[2] = (uint8_t)((sleep_ms >> 16) & 0xFF);
    p[3] = (uint8_t)((sleep_ms >> 24) & 0xFF);
    p[4] = low_power_mode; // 1=Standby, 3=Backup
    gnss_send_quectel_bin(0x06, 0x41, p, 5);
}

bool gnss_wait_for_quiet2(uint32_t quiet_ms, uint32_t timeout_ms)
{
    TickType_t start = xTaskGetTickCount();
    while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(timeout_ms)) {
        TickType_t since = xTaskGetTickCount() - gnss_last_sentence_tick;
        if (since >= pdMS_TO_TICKS(quiet_ms)) {
            return true;
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    return false;
}

bool gnss_wait_for_activity2(uint32_t timeout_ms)
{
    uint32_t start_lines = gnss_lines_seen;
    TickType_t start = xTaskGetTickCount();
    while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(timeout_ms)) {
        if (gnss_lines_seen != start_lines) {
            return true;
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    return false;
}

// 使用 Quectel CFG-SLEEP 进入 Standby（mode=1），sleep_ms 指定待机时间
static bool gnss_enter_standby_quectel(uint32_t sleep_ms)
{
    ESP_LOGI(TAG, "Send Quectel CFG-SLEEP Standby: sleep_ms=%u, mode=1", (unsigned)sleep_ms);
    gnss_send_cfg_sleep_quectel(sleep_ms, 1); // 1=Standby
    // 要求 ≥1.5s 静默并在随后的 1s 内保持静默
    if (gnss_wait_for_quiet2(1500, 5000)) {
        uint32_t lines_before = gnss_lines_seen;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        if (gnss_lines_seen == lines_before) {
            ESP_LOGI(TAG, "Standby entered (UART quiet and stays quiet)");
            return true;
        }
        ESP_LOGW(TAG, "Quiet detected but activity resumed -> not standby");
    } else {
        ESP_LOGW(TAG, "No quiet after Standby CFG-SLEEP within timeout");
    }
    return false;
}

bool gnss_enter_standby2(void)
{
    // 改为使用 Quectel CFG-SLEEP 的 Standby 模式（mode=1）
    gnss_in_standby = false;
    bool ok = gnss_enter_standby_quectel(GNSS_LP_OFF_SEC * 1000U);
    if (ok) gnss_in_standby = true;
    return ok;
}

bool gnss_wake_from_standby2(void)
{
    ESP_LOGI(TAG, "Wake: TX 0xFF CRLF");
    const uint8_t wake_seq[3] = {0xFF, '\r', '\n'};
    uart_write_bytes(GNSS_UART_NUM, (const char*)wake_seq, sizeof(wake_seq));
    vTaskDelay(200 / portTICK_PERIOD_MS);
    uart_flush_input(GNSS_UART_NUM);

    if (gnss_wait_for_activity2(3000)) {
        ESP_LOGI(TAG, "NMEA resumed after wake");
        gnss_in_standby = false;
        return true;
    }
    ESP_LOGW(TAG, "No NMEA after wake within 3s");
    return false;
}

bool gnss_enter_backup_via_cfg(void)
{
    ESP_LOGI(TAG, "Send Quectel CFG-SLEEP (0x06,0x41): sleep_ms=0, mode=3 (Backup)");
    gnss_ack_cfg_sleep = false;
    gnss_nak_cfg_sleep = false;
    gnss_send_cfg_sleep_quectel(0 /*forever*/, 3 /*Backup*/);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    
    if (gnss_wait_for_quiet2(1500, 5000)) {
        uint32_t lines_before = gnss_lines_seen;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        if (gnss_lines_seen == lines_before) {
            ESP_LOGI(TAG, "Backup entered (UART quiet and stays quiet)");
            gnss_in_standby = true;
            return true;
        }
        ESP_LOGW(TAG, "Quiet detected after CFG-SLEEP but activity resumed -> not backup");
    } else {
        ESP_LOGW(TAG, "No quiet after CFG-SLEEP within timeout");
    }
    return false;
}

void gnss_exit_backup_via_reset(void)
{
    ESP_LOGI(TAG, "Exit Backup via RESET_N low pulse (>=100ms), inverted driver on IO%d", GNSS_RST_PIN);
    gnss_reset_assert();
    vTaskDelay(120 / portTICK_PERIOD_MS); // 稍高于 100ms
    gnss_reset_release();
    vTaskDelay(10 / portTICK_PERIOD_MS);  // 释放后保持 ≥10ms
    uart_flush_input(GNSS_UART_NUM);
    gnss_in_standby = false;
}

// 将 NMEA 行按逗号分割为字段，保留空字段
static int nmea_split_fields(char* s, char* fields[], int max_fields)
{
    if (!s || max_fields <= 0) return 0;
    int count = 0;
    char* p = s;
    fields[count++] = p;
    for (char* q = s; *q != '\0'; ++q) {
        if (*q == '*') { // 截断校验和部分
            *q = '\0';
            break;
        }
        if (*q == ',') {
            *q = '\0';
            if (count < max_fields) fields[count++] = q + 1; // 允许空字段
        }
    }
    return count;
}

static double nmea_coord_to_deg(const char* s, char hemi, bool is_lat)
{
    if (!s || !*s) return 0.0;
    double v = atof(s);
    if (v == 0.0) return 0.0;
    // ddmm.mmmm（lat: dd；lon: ddd）
    int deg = (int)(v / 100.0);
    double minutes = v - (deg * 100.0);
    double deg_val = deg + minutes / 60.0;
    if ((is_lat && (hemi == 'S')) || (!is_lat && (hemi == 'W'))) deg_val = -deg_val;
    return deg_val;
}

void gnss_parse_and_log_rmc(const char* nmea)
{
    // 查找以 '$' 开头、任意两字母 talker、紧随其后的 "RMC," 的句子
    const char* rmc = NULL;
    for (const char* p = nmea; (p = strchr(p, '$')) != NULL; ++p) {
        if (p[1] && p[2] && p[3] && p[4] && p[5] && p[6]) {
            if (p[3] == 'R' && p[4] == 'M' && p[5] == 'C' && p[6] == ',') { rmc = p; break; }
        } else {
            break;
        }
    }
    if (!rmc) return;

    // 拷贝一份行缓冲
    char line[128];
    size_t i = 0;
    while (rmc[i] && rmc[i] != '\n' && i < sizeof(line) - 1) { line[i] = rmc[i]; i++; }
    line[i] = '\0';

    // 用自定义分割（保留空字段）
    char tmp[128];
    strncpy(tmp, line, sizeof(tmp)-1); tmp[sizeof(tmp)-1] = '\0';
    char* fields[20] = {0};
    int count = nmea_split_fields(tmp, fields, 20);
    if (count <= 8) return; // 至少需要到索引8

    const char* time_utc = fields[1];
    const char* status   = fields[2];
    const char* lat      = fields[3];
    const char* lat_hemi = fields[4];
    const char* lng      = fields[5];
    const char* lng_hemi = fields[6];
    const char* spd_kn   = fields[7];
    const char* crs_deg  = fields[8];

    GNSS_PARSE_LOG("RMC: time=%s status=%s lat=%s%s lng=%s%s spd(kn)=%s crs=%s", 
             time_utc, status, lat, lat_hemi, lng, lng_hemi, spd_kn, crs_deg);

    // RMC 状态：A=有效，V=无效
    bool rmc_fix = (status && status[0] == 'A');
    static bool last_rmc_fix = false;
    if (rmc_fix != last_rmc_fix) {
        ESP_LOGI(TAG, "RMC fix state changed: %s", rmc_fix ? "FIXED" : "NO FIX");
        last_rmc_fix = rmc_fix;
    }
    if (rmc_fix) {
        gnss_has_fix = true;
        // 解析坐标/速度/航向并保存
        if (lat && *lat && lat_hemi && *lat_hemi && lng && *lng && lng_hemi && *lng_hemi) {
            double lat_deg = nmea_coord_to_deg(lat, lat_hemi[0], true);
            double lon_deg = nmea_coord_to_deg(lng, lng_hemi[0], false);
            double spd = (spd_kn && *spd_kn) ? atof(spd_kn) : 0.0;
            double crs = (crs_deg && *crs_deg) ? atof(crs_deg) : 0.0;
            gnss_last_location.lat_deg = lat_deg;
            gnss_last_location.lon_deg = lon_deg;
            gnss_last_location.spd_kn  = spd;
            gnss_last_location.crs_deg = crs;
            strncpy(gnss_last_location.time_utc, time_utc ? time_utc : "", sizeof(gnss_last_location.time_utc)-1);
            gnss_last_location.time_utc[sizeof(gnss_last_location.time_utc)-1] = '\0';
            gnss_last_location.valid = true;
        }
    }
    gnss_seen_rmc = true;
    gnss_rmc_valid = rmc_fix;
    gnss_log_fix_summary_if_changed();
    gnss_report_ttff_if_needed();
}

void gnss_parse_and_log_gga(const char* nmea)
{
    // 匹配 $xxGGA, 任意 talker
    const char* gga = NULL;
    for (const char* p = nmea; (p = strchr(p, '$')) != NULL; ++p) {
        if (p[1] && p[2] && p[3] && p[4] && p[5] && p[6]) {
            if (p[3] == 'G' && p[4] == 'G' && p[5] == 'A' && p[6] == ',') { gga = p; break; }
        } else {
            break;
        }
    }
    if (!gga) return;

    char line[128];
    size_t i = 0;
    while (gga[i] && gga[i] != '\n' && i < sizeof(line) - 1) { line[i] = gga[i]; i++; }
    line[i] = '\0';

    char tmp[128];
    strncpy(tmp, line, sizeof(tmp)-1); tmp[sizeof(tmp)-1] = '\0';
    char* fields[20] = {0};
    int count = nmea_split_fields(tmp, fields, 20);
    if (count <= 8) return; // 需要至少到索引8

    int   fixq = fields[6] ? atoi(fields[6]) : 0;
    int   sats = fields[7] ? atoi(fields[7]) : 0;
    float hdop = fields[8] ? (float)atof(fields[8]) : 0.0f;

    bool prev_fix = gnss_has_fix;
    gnss_fix_q = fixq;
    gnss_sats  = sats;
    gnss_hdop  = hdop;
    gnss_has_fix = (fixq > 0);

    GNSS_PARSE_LOG("GGA: fixQ=%d sats=%d HDOP=%.1f -> %s",
             fixq, sats, hdop, gnss_has_fix ? "FIXED" : "NO FIX");

    if (prev_fix != gnss_has_fix) {
        ESP_LOGI(TAG, "FIX state changed by GGA: %s", gnss_has_fix ? "FIXED" : "NO FIX");
    }
    gnss_seen_gga = true;
    gnss_log_fix_summary_if_changed();
    gnss_report_ttff_if_needed();
}

void gnss_parse_and_log_gsa(const char* nmea)
{
    // 匹配 $xxGSA, 任意 talker
    const char* gsa = NULL;
    for (const char* p = nmea; (p = strchr(p, '$')) != NULL; ++p) {
        if (p[1] && p[2] && p[3] && p[4] && p[5] && p[6]) {
            if (p[3] == 'G' && p[4] == 'S' && p[5] == 'A' && p[6] == ',') { gsa = p; break; }
        } else {
            break;
        }
    }
    if (!gsa) return;

    char line[160];
    size_t i = 0;
    while (gsa[i] && gsa[i] != '\n' && i < sizeof(line) - 1) { line[i] = gsa[i]; i++; }
    line[i] = '\0';

    char tmp[160];
    strncpy(tmp, line, sizeof(tmp)-1); tmp[sizeof(tmp)-1] = '\0';
    char* fields[24] = {0};
    int count = nmea_split_fields(tmp, fields, 24);
    if (count <= 2) return;

    int dim = fields[2] ? atoi(fields[2]) : 0; // 1=无,2=2D,3=3D
    float hdop = (count > 16 && fields[16]) ? (float)atof(fields[16]) : 0.0f;

    gnss_dim = dim;
    if (hdop > 0.0f) gnss_hdop = hdop;

    const char* dim_str = (dim == 3) ? "3D" : (dim == 2) ? "2D" : "NO FIX";
    GNSS_PARSE_LOG("GSA: dim=%s (val=%d) HDOP=%.1f", dim_str, dim, gnss_hdop);

    if (dim >= 2) gnss_has_fix = true;
    gnss_seen_gsa = true;
    gnss_log_fix_summary_if_changed();
    gnss_report_ttff_if_needed();
}

void gnss_task(void* pv)
{
    // 读 GNSS NMEA 数据并打印，优先显示 RMC
    uint8_t buf[256];
    char line[256];
    size_t line_len = 0;

    // 以"上次完成一行"的时间为心跳参考（避免有乱码字节时误判为空闲）
    TickType_t last_line_tick = xTaskGetTickCount();
    size_t bytes_since_line = 0;   // 自上次换行以来累计收到的字节数
    uint8_t sample[16];            // 最近一批字节的样本（用于调试波特率不匹配）
    int sample_len = 0;

    // 初始化待机/唤醒观察变量
    gnss_last_sentence_tick = last_line_tick;
    gnss_lines_seen = 0;
    gnss_ack_pmtk161 = false;
    gnss_ack_pmtk225 = false;

    // 简易 UBX 帧解析状态机（用于捕获 ACK-ACK/ACK-NAK）
    enum {UBX_SYNC1, UBX_SYNC2, UBX_CLASS, UBX_ID, UBX_LEN1, UBX_LEN2, UBX_PAYLOAD, UBX_CKA, UBX_CKB} ubx_state = UBX_SYNC1;
    uint8_t ubx_cls=0, ubx_id=0; uint16_t ubx_len=0, ubx_pos=0; uint8_t ubx_ckA=0, ubx_ckB=0; uint8_t ubx_buf[256];

    while (1) {
        if (gnss_should_stop) {
            ESP_LOGI(TAG, "Stopping GNSS receiver after benchmark");
            vTaskDelete(NULL);
        }
        int len = uart_read_bytes(GNSS_UART_NUM, buf, sizeof(buf), 200 / portTICK_PERIOD_MS);
        if (len > 0) {
            // 更新样本与计数
            int n = (len < (int)sizeof(sample)) ? len : (int)sizeof(sample);
            memcpy(sample, buf, n);
            sample_len = n;
            bytes_since_line += len;

            for (int i = 0; i < len; ++i) {
                char c = (char)buf[i];

                // UBX 解析
                uint8_t b = (uint8_t)c;
                switch (ubx_state) {
                    case UBX_SYNC1: if (b == 0xB5) { ubx_state = UBX_SYNC2; ubx_ckA = ubx_ckB = 0; } break;
                    case UBX_SYNC2: if (b == 0x62) { ubx_state = UBX_CLASS; } else { ubx_state = UBX_SYNC1; } break;
                    case UBX_CLASS: ubx_cls = b; ubx_ckA += b; ubx_ckB += ubx_ckA; ubx_state = UBX_ID; break;
                    case UBX_ID:    ubx_id  = b; ubx_ckA += b; ubx_ckB += ubx_ckA; ubx_state = UBX_LEN1; break;
                    case UBX_LEN1:  ubx_len = b; ubx_ckA += b; ubx_ckB += ubx_ckA; ubx_state = UBX_LEN2; break;
                    case UBX_LEN2:  ubx_len |= ((uint16_t)b << 8); ubx_ckA += b; ubx_ckB += ubx_ckA;
                                     if (ubx_len > sizeof(ubx_buf)) { ubx_state = UBX_SYNC1; }
                                     else { ubx_pos = 0; ubx_state = (ubx_len ? UBX_PAYLOAD : UBX_CKA); }
                                     break;
                    case UBX_PAYLOAD:
                        ubx_buf[ubx_pos++] = b; ubx_ckA += b; ubx_ckB += ubx_ckA;
                        if (ubx_pos >= ubx_len) ubx_state = UBX_CKA;
                        break;
                    case UBX_CKA:
                        if (b == ubx_ckA) { ubx_state = UBX_CKB; }
                        else { ubx_state = UBX_SYNC1; }
                        break;
                    case UBX_CKB:
                        if (b == ubx_ckB) {
                            // 完整 UBX 帧
                            if (ubx_cls == 0x05 && ubx_id == 0x01 && ubx_len >= 2) { // ACK-ACK
                                uint8_t ack_cls = ubx_buf[0];
                                uint8_t ack_id  = ubx_buf[1];
                                GNSS_UBX_LOG("RX ACK-ACK for cls=0x%02X id=0x%02X", ack_cls, ack_id);
                                if (ack_cls == 0x06 && ack_id == 0x41) gnss_ack_cfg_sleep = true;
                            } else if (ubx_cls == 0x05 && ubx_id == 0x00 && ubx_len >= 2) { // ACK-NAK
                                uint8_t ack_cls = ubx_buf[0];
                                uint8_t ack_id  = ubx_buf[1];
                                GNSS_UBX_LOG("RX ACK-NAK for cls=0x%02X id=0x%02X", ack_cls, ack_id);
                                if (ack_cls == 0x06 && ack_id == 0x41) gnss_nak_cfg_sleep = true;
                            }
                        }
                        ubx_state = UBX_SYNC1;
                        break;
                }
                if (c == '\r' || c == '\n') {
                    line[line_len] = '\0';
                    if (line_len > 0) {
                        // 打印整句 NMEA（可由宏控制开关）
                        GNSS_RAW_LOG("%s", line);
                        // 待机命令ACK检测
                        if (strstr(line, "$PMTK001,161,3") != NULL) {
                            gnss_ack_pmtk161 = true;
                            ESP_LOGI(TAG, "ACK PMTK161");
                        } else if (strstr(line, "$PMTK001,225,3") != NULL) {
                            gnss_ack_pmtk225 = true;
                            ESP_LOGI(TAG, "ACK PMTK225");
                        }
                        // 基于句型选择性解析，避免误判
                        if (line[0] == '$' && line_len > 6 && line[6] == ',') {
                            if (line[3] == 'R' && line[4] == 'M' && line[5] == 'C') {
                                gnss_parse_and_log_rmc(line);
                            } else if (line[3] == 'G' && line[4] == 'G' && line[5] == 'A') {
                                gnss_parse_and_log_gga(line);
                            } else if (line[3] == 'G' && line[4] == 'S' && line[5] == 'A') {
                                gnss_parse_and_log_gsa(line);
                            }
                        }
                    }
                    line_len = 0;
                    // 完成一行，重置基准
                    last_line_tick = xTaskGetTickCount();
                    gnss_last_sentence_tick = last_line_tick;
                    gnss_lines_seen++;
                    bytes_since_line = 0;
                    sample_len = 0;
                } else if (line_len < sizeof(line) - 1) {
                    line[line_len++] = c;
                } else {
                    // 行过长，丢弃重来
                    line_len = 0;
                }
            }
        }

        // 超过2秒没有"完成一行"，输出心跳
        if ((xTaskGetTickCount() - last_line_tick) >= pdMS_TO_TICKS(2000)) {
            if (bytes_since_line == 0) {
                ESP_LOGI(TAG, "Waiting for NMEA on UART2 (TX=%d, RX=%d, %d)...", GNSS_UART_TX, GNSS_UART_RX, GNSS_BAUD);
            } else {
                // 有数据但无换行，多半是波特率不匹配或线路噪声
                char hex[16 * 3 + 1];
                int pos = 0;
                for (int i = 0; i < sample_len && pos < (int)sizeof(hex) - 3; ++i) {
                    pos += snprintf(hex + pos, sizeof(hex) - pos, "%02X ", sample[i]);
                }
                hex[(pos < (int)sizeof(hex)) ? pos : ((int)sizeof(hex) - 1)] = '\0';
                GNSS_RAW_LOG("RX without CR/LF for 2s, bytes=%u, sample=[%s]", (unsigned)bytes_since_line, hex);
                ESP_LOGI(TAG, "RX without CR/LF for 2s, bytes=%u", (unsigned)bytes_since_line);
            }
            // 重置基准窗口
            last_line_tick = xTaskGetTickCount();
            bytes_since_line = 0;
            sample_len = 0;
        }

        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

void gnss_init(void)
{
    // 初始化 GNSS UART2 (LC760Z)
    uart_config_t gnss_uart_cfg = {
        .baud_rate = GNSS_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    ESP_LOGI(TAG, "UART2 (GNSS) init at %d baud", GNSS_BAUD);
    ESP_ERROR_CHECK(uart_driver_install(GNSS_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(GNSS_UART_NUM, &gnss_uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(GNSS_UART_NUM, GNSS_UART_TX, GNSS_UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_LOGI(TAG, "Pins set: TX=%d -> GNSS RXD, RX=%d <- GNSS TXD", GNSS_UART_TX, GNSS_UART_RX);

    // GNSS 使能脚（IO11 -> LP5907 EN）拉高
    gpio_set_direction(GNSS_EN_PIN, GPIO_MODE_OUTPUT);
    gnss_enable(true);
    ESP_LOGI(TAG, "GNSS EN pin %d set HIGH", GNSS_EN_PIN);
    
    // GNSS 复位脚初始化
    gpio_set_direction(GNSS_RST_PIN, GPIO_MODE_OUTPUT);
    gnss_reset_release(); // 释放复位状态
}