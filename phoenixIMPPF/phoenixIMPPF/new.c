/*
 * FTM Collector – XIAO ESP32-S3
 * Phoenix SAR – NLOS/LOS Dataset Collection Firmware
 *
 * REQUIREMENTS – read before flashing:
 * ─────────────────────────────────────────────────────────────────
 * platformio.ini must use framework = espidf, not arduino.
 * FTM is not available in the Arduino Wi-Fi wrapper.
 *
 * Serial output lines:
 * BURST_TX,<seq>,<t_tx_us>,<label>
 * BURST_START,<seq>,<t_start_us>,<n_frames>,<label>
 * FTM_F,<seq>,<frame_idx>,<rtt_ps>,<t1_ps>,<t2_ps>,<t3_ps>,<t4_ps>,<rssi_dbm>,<label>
 * LABEL,<label>   (echo when label changed via serial)
 *
 * Label commands via serial (115200 baud) – send string + newline:
 * LOS_STATIC  LOS_DYNAMIC  NLOS_WALL  NLOS_CORNER  NLOS_DOOR  NLOS_DYNAMIC
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"

/* ── Configuration ────────────────────────────────────────────────────────── */
#define WIFI_SSID "FTM_AP"
#define WIFI_PASS "12345678"
#define WIFI_CHANNEL 6

#define FTM_FRMS_PER_BURST 32
#define FTM_BURST_PERIOD 0
#define MEASURE_INTERVAL_MS 300
#define MAX_RETRY 5
#define MAX_FRAME_ENTRIES 64

#define CMD_BUF_LEN 32

/* ── Event bits ───────────────────────────────────────────────────────────── */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
#define FTM_DONE_BIT BIT2

/* ── Globals ──────────────────────────────────────────────────────────────── */
static EventGroupHandle_t s_wifi_evt_group;
static EventGroupHandle_t s_ftm_evt_group;

static const char *TAG = "PHOENIX";
static int s_retry = 0;
static uint32_t s_seq = 0;
static uint8_t s_ap_bssid[6] = {0};
static uint8_t s_ap_ch = 0;

static char s_label[CMD_BUF_LEN] = "LOS_STATIC";

/* ── FTM snapshot ─────────────────────────────────────────────────────────── */
typedef struct
{
    wifi_ftm_status_t status;
    uint8_t n;
    wifi_ftm_report_entry_t e[MAX_FRAME_ENTRIES];
} ftm_snap_t;
static ftm_snap_t s_snap;

/* ── Wi-Fi event handler ─────────────────────────────────────────────────── */
static void evt_handler(void *arg, esp_event_base_t base,
                        int32_t id, void *data)
{
    if (base == WIFI_EVENT)
    {
        switch (id)
        {
        case WIFI_EVENT_STA_START:
            esp_wifi_connect();
            break;

        case WIFI_EVENT_STA_DISCONNECTED:
            if (s_retry < MAX_RETRY)
            {
                esp_wifi_connect();
                ESP_LOGW(TAG, "Retry %d/%d", ++s_retry, MAX_RETRY);
            }
            else
            {
                xEventGroupSetBits(s_wifi_evt_group, WIFI_FAIL_BIT);
            }
            break;

        case WIFI_EVENT_FTM_REPORT:
        {
            wifi_event_ftm_report_t *rep = (wifi_event_ftm_report_t *)data;
            s_snap.status = rep->status;
            /* use_get_report_api=true: event carries status only.
               Frame data is retrieved via esp_wifi_ftm_get_report(). */
            xEventGroupSetBits(s_ftm_evt_group, FTM_DONE_BIT);
            break;
        }
        default:
            break;
        }
    }
    else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP)
    {
        s_retry = 0;
        wifi_ap_record_t ap;
        if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK)
        {
            memcpy(s_ap_bssid, ap.bssid, 6);
            s_ap_ch = ap.primary;
            ESP_LOGI(TAG, "AP %02x:%02x:%02x:%02x:%02x:%02x ch=%d",
                     s_ap_bssid[0], s_ap_bssid[1], s_ap_bssid[2],
                     s_ap_bssid[3], s_ap_bssid[4], s_ap_bssid[5], s_ap_ch);
        }
        xEventGroupSetBits(s_wifi_evt_group, WIFI_CONNECTED_BIT);
    }
}

/* ── Wi-Fi init ───────────────────────────────────────────────────────────── */
static bool wifi_init_sta(void)
{
    s_wifi_evt_group = xEventGroupCreate();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, evt_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, evt_handler, NULL, NULL));

    wifi_config_t wc = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .channel = WIFI_CHANNEL,
            .scan_method = WIFI_FAST_SCAN,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "Connecting to %s ...", WIFI_SSID);

    EventBits_t b = xEventGroupWaitBits(s_wifi_evt_group,
                                        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                        pdFALSE, pdFALSE,
                                        pdMS_TO_TICKS(15000));
    if (b & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAG, "Connected.");
        return true;
    }
    ESP_LOGE(TAG, "Connection failed.");
    return false;
}

/* ── Serial command task ──────────────────────────────────────────────────── */
static void serial_cmd_task(void *pv)
{
    char buf[CMD_BUF_LEN];
    int pos = 0;
    while (true)
    {
        int c = fgetc(stdin);
        if (c == EOF)
        {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if (c == '\n' || c == '\r')
        {
            if (pos > 0)
            {
                buf[pos] = '\0';
                if (strcmp(buf, "LOS_STATIC") == 0 ||
                    strcmp(buf, "LOS_DYNAMIC") == 0 ||
                    strcmp(buf, "NLOS_WALL") == 0 ||
                    strcmp(buf, "NLOS_CORNER") == 0 ||
                    strcmp(buf, "NLOS_DOOR") == 0 ||
                    strcmp(buf, "NLOS_DYNAMIC") == 0)
                {
                    strncpy(s_label, buf, CMD_BUF_LEN - 1);
                    s_label[CMD_BUF_LEN - 1] = '\0';
                    printf("LABEL,%s\n", s_label);
                    fflush(stdout);
                }
                else
                {
                    printf("# Unknown: %s\n", buf);
                    fflush(stdout);
                }
                pos = 0;
            }
        }
        else if (pos < CMD_BUF_LEN - 1)
        {
            buf[pos++] = (char)c;
        }
    }
}

/* ── FTM measure task ────────────────────────────────────────────────────── */
static void ftm_task(void *pv)
{
    s_ftm_evt_group = xEventGroupCreate();

    printf("# SCHEMA BURST_TX: seq,t_tx_us,label\n");
    printf("# SCHEMA BURST_START: seq,t_start_us,n_frames,label\n");
    printf("# SCHEMA FTM_F: seq,frame_idx,rtt_ps,t1_ps,t2_ps,t3_ps,t4_ps,rssi_dbm,label\n");
    fflush(stdout);

    while (true)
    {
        char lbl[CMD_BUF_LEN];
        strncpy(lbl, s_label, CMD_BUF_LEN - 1);
        lbl[CMD_BUF_LEN - 1] = '\0';

        wifi_ftm_initiator_cfg_t fc = {
            .resp_mac = {0},
            .channel = s_ap_ch,
            .frm_count = FTM_FRMS_PER_BURST,
            .burst_period = FTM_BURST_PERIOD,
            .use_get_report_api = true,
        };
        memcpy(fc.resp_mac, s_ap_bssid, 6);

        int64_t t_tx_us = esp_timer_get_time();
        printf("BURST_TX,%" PRIu32 ",%" PRId64 ",%s\n", s_seq, t_tx_us, lbl);
        fflush(stdout);

        esp_err_t err = esp_wifi_ftm_initiate_session(&fc);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "FTM init: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(MEASURE_INTERVAL_MS));
            continue;
        }

        EventBits_t b = xEventGroupWaitBits(s_ftm_evt_group, FTM_DONE_BIT,
                                            pdTRUE, pdFALSE,
                                            pdMS_TO_TICKS(5000));
        if (!(b & FTM_DONE_BIT))
        {
            ESP_LOGW(TAG, "FTM timeout");
            esp_wifi_ftm_end_session();
            vTaskDelay(pdMS_TO_TICKS(MEASURE_INTERVAL_MS));
            continue;
        }
        if (s_snap.status != FTM_STATUS_SUCCESS)
        {
            ESP_LOGW(TAG, "FTM status=%d", (int)s_snap.status);
            vTaskDelay(pdMS_TO_TICKS(MEASURE_INTERVAL_MS));
            continue;
        }

        /* Retrieve frames into pre-allocated buffer, clear first so
           RTT==0 reliably marks unused slots. */
        memset(s_snap.e, 0, sizeof(s_snap.e));
        esp_wifi_ftm_get_report(s_snap.e, MAX_FRAME_ENTRIES);

        /* Count valid frames — valid entries have non-zero RTT. */
        uint8_t n_frames = 0;
        for (uint8_t i = 0; i < MAX_FRAME_ENTRIES; i++)
        {
            if (s_snap.e[i].rtt == 0)
                break;
            n_frames++;
        }

        printf("BURST_START,%" PRIu32 ",%" PRId64 ",%u,%s\n",
               s_seq, esp_timer_get_time(), n_frames, lbl);

        for (uint8_t i = 0; i < n_frames; i++)
        {
            printf("FTM_F,%" PRIu32 ",%u,%" PRIu32
                   ",%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%d,%s\n",
                   s_seq, (unsigned)i, s_snap.e[i].rtt,
                   s_snap.e[i].t1, s_snap.e[i].t2,
                   s_snap.e[i].t3, s_snap.e[i].t4,
                   (int)s_snap.e[i].rssi, lbl);
        }

        fflush(stdout);
        s_seq++;

        vTaskDelay(pdMS_TO_TICKS(MEASURE_INTERVAL_MS));
    }
}

/* ── app_main ─────────────────────────────────────────────────────────────── */
void app_main(void)
{
    esp_err_t r = nvs_flash_init();
    if (r == ESP_ERR_NVS_NO_FREE_PAGES || r == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        r = nvs_flash_init();
    }
    ESP_ERROR_CHECK(r);
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    if (!wifi_init_sta())
    {
        ESP_LOGE(TAG, "Halting.");
        return;
    }

    xTaskCreate(serial_cmd_task, "serial_cmd", 3072, NULL, 3, NULL);
    xTaskCreate(ftm_task, "ftm", 6144, NULL, 5, NULL);
}
