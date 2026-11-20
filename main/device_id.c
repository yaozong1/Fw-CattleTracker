#include "device_id.h"

#include <stdio.h>
#include <string.h>
#include <ctype.h>

#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "DEVID";

#define NVS_NS        "ct"
#define NVS_KEY_ID    "id"
#define NVS_KEY_BOOT  "boot"
#define ID_MAX_LEN    20

static char     s_id[ID_MAX_LEN + 1]  = "";      /* NVS 里的编号，"" = 未写入 */
static char     s_imei[16]            = "";
static char     s_effective[32]       = "IMEI-UNKNOWN";
static uint32_t s_boot                = 0;

static void rebuild_effective(void)
{
    if (s_id[0]) {
        snprintf(s_effective, sizeof(s_effective), "%s", s_id);
    } else if (s_imei[0]) {
        snprintf(s_effective, sizeof(s_effective), "IMEI-%s", s_imei);
    } else {
        snprintf(s_effective, sizeof(s_effective), "IMEI-UNKNOWN");
    }
}

/* 编号只允许字母、数字、'-'、'_'：它要进 MQTT topic，不能有 / + # 和空格 */
static bool id_is_valid(const char *id)
{
    size_t n = strlen(id);
    if (n == 0 || n > ID_MAX_LEN) return false;
    for (size_t i = 0; i < n; ++i) {
        char c = id[i];
        if (!(isalnum((unsigned char)c) || c == '-' || c == '_')) return false;
    }
    return true;
}

static bool looks_like_ct(const char *id)
{
    if (strncmp(id, "CT-", 3) != 0 || !id[3]) return false;
    for (const char *p = id + 3; *p; ++p) {
        if (!isdigit((unsigned char)*p)) return false;
    }
    return true;
}

void devid_init(bool cold_boot)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        /* 分区格式变了才会走到这里（例如改了分区表）。擦掉会丢编号，日志里大声说 */
        ESP_LOGE(TAG, "NVS 需要擦除重建（%s）—— 设备编号会丢失，需要重新写入",
                 esp_err_to_name(err));
        nvs_flash_erase();
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS 初始化失败：%s", esp_err_to_name(err));
        rebuild_effective();
        return;
    }

    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open(%s) 失败", NVS_NS);
        rebuild_effective();
        return;
    }

    size_t len = sizeof(s_id);
    if (nvs_get_str(h, NVS_KEY_ID, s_id, &len) != ESP_OK || !id_is_valid(s_id)) {
        s_id[0] = '\0';
    }

    if (nvs_get_u32(h, NVS_KEY_BOOT, &s_boot) != ESP_OK) {
        s_boot = 0;
    }
    if (cold_boot) {
        s_boot++;
        nvs_set_u32(h, NVS_KEY_BOOT, s_boot);
        nvs_commit(h);
    }
    nvs_close(h);

    rebuild_effective();
    ESP_LOGI(TAG, "设备编号：%s%s   上电计数：%lu",
             s_id[0] ? s_id : "(未写入)",
             s_id[0] ? "" : " —— 暂用 IMEI，后台显示为未登记",
             (unsigned long)s_boot);
}

static bool save_id(const char *id)   /* id = "" 表示清除 */
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return false;
    esp_err_t e = id[0] ? nvs_set_str(h, NVS_KEY_ID, id) : nvs_erase_key(h, NVS_KEY_ID);
    if (e == ESP_ERR_NVS_NOT_FOUND) e = ESP_OK;     /* 清除一个本来就没有的 key */
    if (e == ESP_OK) e = nvs_commit(h);
    nvs_close(h);
    if (e != ESP_OK) return false;
    snprintf(s_id, sizeof(s_id), "%s", id);
    rebuild_effective();
    return true;
}

/* ---------------- 上电 10 秒串口窗口 ---------------- */

#define CON_UART UART_NUM_0

static void con_print(const char *s) { uart_write_bytes(CON_UART, s, strlen(s)); }

static bool handle_line(char *line)
{
    /* 去首尾空白 */
    while (*line == ' ' || *line == '\t') line++;
    size_t n = strlen(line);
    while (n && (line[n-1] == ' ' || line[n-1] == '\t')) line[--n] = '\0';
    if (n == 0) return false;

    char buf[96];
    if (strcmp(line, "go") == 0) {
        con_print("继续启动\r\n");
        return true;
    }
    if (strcmp(line, "id") == 0) {
        snprintf(buf, sizeof(buf), "当前编号：%s\r\n", s_id[0] ? s_id : "(未写入，使用 IMEI-xxx)");
        con_print(buf);
        return false;
    }
    if (strncmp(line, "id ", 3) == 0) {
        const char *arg = line + 3;
        while (*arg == ' ') arg++;
        if (strcmp(arg, "clear") == 0) {
            con_print(save_id("") ? "已清除编号，之后使用 IMEI-xxx\r\n" : "清除失败（NVS）\r\n");
            return false;
        }
        if (!id_is_valid(arg)) {
            snprintf(buf, sizeof(buf), "拒绝：编号只能是 1~%d 个字母/数字/-/_，例如 CT-001\r\n", ID_MAX_LEN);
            con_print(buf);
            return false;
        }
        if (save_id(arg)) {
            snprintf(buf, sizeof(buf), "已写入编号：%s%s\r\n", s_id,
                     looks_like_ct(s_id) ? "" : "（注意：不是 CT-数字 的格式）");
            con_print(buf);
        } else {
            con_print("写入失败（NVS）\r\n");
        }
        return false;
    }
    con_print("命令：id | id CT-001 | id clear | go\r\n");
    return false;
}

void devid_console_window(uint32_t window_ms)
{
    if (window_ms == 0) return;

    /* 日志走的是 ROM/VFS 的轮询输出，不依赖驱动；这里临时装一个驱动来收键盘输入，用完卸掉 */
    bool installed_here = false;
    if (!uart_is_driver_installed(CON_UART)) {
        if (uart_driver_install(CON_UART, 256, 0, 0, NULL, 0) != ESP_OK) {
            ESP_LOGW(TAG, "UART0 驱动装不上，跳过编号窗口");
            return;
        }
        installed_here = true;
    }
    uart_flush_input(CON_UART);

    char hint[160];
    snprintf(hint, sizeof(hint),
             "\r\n===== 设备编号：%s =====\r\n"
             "%lu 秒内可输入：id CT-001（写编号） / id（查看） / id clear / go（跳过）\r\n> ",
             s_id[0] ? s_id : "未写入", (unsigned long)(window_ms / 1000));
    con_print(hint);

    char line[48];
    size_t len = 0;
    TickType_t t0 = xTaskGetTickCount();
    bool done = false;

    while (!done && (xTaskGetTickCount() - t0) < pdMS_TO_TICKS(window_ms)) {
        uint8_t c;
        if (uart_read_bytes(CON_UART, &c, 1, pdMS_TO_TICKS(50)) != 1) continue;

        if (c == '\r' || c == '\n') {
            con_print("\r\n");
            line[len] = '\0';
            done = handle_line(line);
            len = 0;
            if (!done) {
                t0 = xTaskGetTickCount();     /* 有人在操作，窗口重新计时 */
                con_print("> ");
            }
        } else if (c == 0x08 || c == 0x7F) {
            if (len) { len--; con_print("\b \b"); }
        } else if (c >= 0x20 && c < 0x7F && len < sizeof(line) - 1) {
            line[len++] = (char)c;
            uart_write_bytes(CON_UART, (const char *)&c, 1);   /* 回显 */
        }
    }
    if (!done) con_print("\r\n（窗口结束，继续启动）\r\n");

    if (installed_here) uart_driver_delete(CON_UART);
    ESP_LOGI(TAG, "设备编号：%s", devid_get());
}

const char *devid_get(void)           { return s_effective; }
bool        devid_is_provisioned(void) { return s_id[0] != '\0'; }
const char *devid_imei(void)          { return s_imei; }
uint32_t    devid_boot_count(void)    { return s_boot; }

void devid_set_imei(const char *imei)
{
    if (!imei || strlen(imei) != 15) return;
    snprintf(s_imei, sizeof(s_imei), "%s", imei);
    rebuild_effective();
}
