// Adaptive sleep/wake/upload cycle for a HaLow camera node.
//
// Each wake is one tick of the scene FSM in adaptive.c:
//  1) First cold boot: derive the node ID from the MAC (kept in RTC memory
//     across deep sleep), read config.json and any mirrored adaptive state
//     from the SD card.
//  2) Capture a cheap QVGA grayscale frame and score it against the "nothing
//     happening" reference grid held in RTC memory.
//  3) The FSM decides whether the scene is worth sending and how long to
//     sleep next. Quiet wakes skip the radio — and the SD card — entirely;
//     the radio is the dominant energy cost. Every K_RADIO-th consecutive
//     quiet wake still connects so pushed config reaches the node and cached
//     frames get flushed.
//  4) Send wakes re-init the camera for JPEG at the configured resolution
//     (config was applied on connect, so a pushed resolution lands in this
//     wake's photo), stamp EXIF with the diff score + state, upload over
//     MQTT, and cache to SD when unacknowledged.
//  5) The reference blends toward quiet frames, freezes during activity, and
//     is mirrored to SD on wakes that already mounted the card.
//  6) Deep sleep for the FSM-chosen interval.
//
// The MQTT protocol is identical to the Arduino node, so base_station/
// receive_images.py and send_config.py work against this firmware unchanged.

#include <dirent.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>

#include "driver/gpio.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_netif_sntp.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "adaptive.h"
#include "camera.h"
#include "exif.h"
#include "mmhalow.h"
#include "mqtt_node.h"
#include "storage.h"

#define WIFI_SSID CONFIG_WIFI_SSID
#define WIFI_PSK  CONFIG_WIFI_PSK

// HaLow association is slow — the Arduino node allowed 30 s and needs it.
#define CONNECT_TIMEOUT_MS 30000
#define MQTT_CONNECT_MS    15000
#define NTP_TIMEOUT_MS     5000
#define MQTT_FLUSH_MS      3000    // window for a pushed config to land
#define ACK_TIMEOUT_MS     10000

static const char *TAG = "halow_camera";

// Connect the radio on every Kth consecutive quiet wake even though the scene
// hasn't asked for it, so pushed config still reaches the node and cached
// frames get flushed (adaptive plan §7/§19).
#define K_RADIO 6

// Battery sensing (HT-HC33 datasheet §4.1). VBAT is read through a switched 2:1
// resistor divider (R17/R28 = 100K/100K), so VBAT = ADC_mV × 2. The divider node
// ADC_IN is GPIO1 = ADC1_CH0 (ADC1 sidesteps the ADC2/radio conflict); ADC_Ctrl
// on GPIO20 gates the P-FET — driven high to connect the divider for the read,
// low afterward so it doesn't leak ~VBAT/200K. In deep sleep the pin floats and
// R20 holds the P-FET off, so nothing drains and no RTC hold is needed.
#define BATT_CTRL_GPIO   GPIO_NUM_20
#define BATT_ADC_UNIT    ADC_UNIT_1
#define BATT_ADC_CHANNEL ADC_CHANNEL_0    // GPIO1
#define BATT_ADC_ATTEN   ADC_ATTEN_DB_12  // ADC_IN ≤ VBAT/2 ≈ 2.1 V, within range
#define BATT_DIVIDER     2                 // 100K/100K → ×2
#define BATT_ADC_SAMPLES 32                // average — the 50 kΩ divider is noisy
#define BATT_SETTLE_MS   3

// Survives deep sleep.
RTC_DATA_ATTR static bool     g_first_boot = true;
RTC_DATA_ATTR static char     g_node_id[13];

// Node config: thresholds, cadences, resolutions, and the base station's lease.
// The initializer runs on cold boot only — a warm wake keeps what RTC memory
// held, which is how a pushed config survives deep sleep without touching SD.
RTC_DATA_ATTR static node_config_t g_cfg = {
    .frame_size      = FRAMESIZE_VGA,
    .frame_size_high = FRAMESIZE_HD,
    .config_ts       = 0,
    .params          = ADAPT_PARAMS_DEFAULTS,
    .override        = { .mode = ADAPT_OVR_NONE },
};

// Adaptive scene state (~1 KB of the S3's 8 KB RTC_SLOW): FSM + reference
// grid live here so quiet wakes touch neither the SD card nor the radio.
// g_adapt_valid is false only until the state is first seeded (SD restore or
// provisional reference) after a power loss.
RTC_DATA_ATTR static adaptive_state_t g_adapt;
RTC_DATA_ATTR static bool     g_adapt_valid = false;
RTC_DATA_ATTR static uint32_t g_wakes_since_radio = 0;

// Decision log (plan §15), buffered in RTC until a wake that has the card
// mounted anyway. K_RADIO bounds the quiet wakes between mounts at 6, so 32 is
// generous headroom for a run of failed mounts; overflow drops the oldest and
// says so at flush rather than losing records quietly.
#define DECISION_RING_N 32
RTC_DATA_ATTR static decision_rec_t g_dlog[DECISION_RING_N];   // 512 B
RTC_DATA_ATTR static uint16_t g_dlog_count = 0;
RTC_DATA_ATTR static uint16_t g_dlog_head = 0;      // oldest record
RTC_DATA_ATTR static uint16_t g_dlog_dropped = 0;

// This wake's downscaled probe. Static: 1 KB doesn't fit the 3.5 KB main stack.
static uint8_t s_cur_grid[ADAPT_CELLS];

// A config push lands on the MQTT task, mid-wake. Rather than let it write
// g_cfg underneath the main task — which could read a new override's mode
// alongside the old one's deadline — the callback stages the parsed result here
// and the main task commits it at a point of its choosing. s_config_pending is
// written last and read first, so it can't advertise a struct that isn't there.
static node_config_t s_pending_cfg;
static volatile bool s_config_pending = false;

// Unix seconds, or 0 when the clock has never synced. 0 is what tells
// adaptive_apply_override that leases can't be evaluated this wake.
// Read the battery voltage (millivolts) through the switched divider. Returns 0
// on any failure — battery_update treats 0 as "no reading". Must be called at a
// consistent point every wake (radio down) so the systematic load offset cancels
// in the charge-rate trend (plan §11 voltage-sag trap).
static uint16_t battery_sample_mv(void)
{
    // Connect the divider and let the 50 kΩ node settle before sampling.
    gpio_reset_pin(BATT_CTRL_GPIO);
    gpio_set_direction(BATT_CTRL_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(BATT_CTRL_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(BATT_SETTLE_MS));

    uint16_t vbat_mv = 0;
    adc_oneshot_unit_handle_t adc = NULL;
    adc_oneshot_unit_init_cfg_t ucfg = { .unit_id = BATT_ADC_UNIT };
    if (adc_oneshot_new_unit(&ucfg, &adc) == ESP_OK) {
        adc_oneshot_chan_cfg_t ccfg = {
            .atten    = BATT_ADC_ATTEN,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        adc_oneshot_config_channel(adc, BATT_ADC_CHANNEL, &ccfg);

        adc_cali_handle_t cali = NULL;
        adc_cali_curve_fitting_config_t calcfg = {
            .unit_id  = BATT_ADC_UNIT,
            .chan     = BATT_ADC_CHANNEL,
            .atten    = BATT_ADC_ATTEN,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        bool have_cali =
            (adc_cali_create_scheme_curve_fitting(&calcfg, &cali) == ESP_OK);

        int acc = 0, got = 0;
        for (int i = 0; i < BATT_ADC_SAMPLES; i++) {
            int raw = 0;
            if (adc_oneshot_read(adc, BATT_ADC_CHANNEL, &raw) == ESP_OK) {
                acc += raw;
                got++;
            }
        }
        if (got > 0) {
            int raw_avg = acc / got;
            int mv = raw_avg;
            if (have_cali) {
                adc_cali_raw_to_voltage(cali, raw_avg, &mv);
            } else {
                // Coarse fallback if eFuse calibration is unavailable: 12-bit at
                // 12 dB atten ≈ 3100 mV full-scale. The bands tolerate the slop.
                mv = raw_avg * 3100 / 4095;
            }
            vbat_mv = (uint16_t)(mv * BATT_DIVIDER);
        }
        if (have_cali) {
            adc_cali_delete_scheme_curve_fitting(cali);
        }
        adc_oneshot_del_unit(adc);
    }

    // Disconnect the divider so it stops drawing between reads.
    gpio_set_level(BATT_CTRL_GPIO, 0);
    return vbat_mv;
}

static uint32_t wall_now(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (tv.tv_sec > 1000000LL) ? (uint32_t)tv.tv_sec : 0;
}

static void dlog_append(const decision_rec_t *r)
{
    uint16_t slot = (g_dlog_head + g_dlog_count) % DECISION_RING_N;
    g_dlog[slot] = *r;
    if (g_dlog_count < DECISION_RING_N) {
        g_dlog_count++;
    } else {
        g_dlog_head = (g_dlog_head + 1) % DECISION_RING_N;   // overwrote the oldest
        g_dlog_dropped++;
    }
}

// Drain the ring oldest-first, retiring records only once they're actually on
// the card, so a full or flaky card costs a retry next mount rather than the
// batch. The ring wraps and a copy would need 512 B the main task's stack
// doesn't have, so a wrapped ring flushes as two runs straight out of its own
// storage — each retired on its own, or the second one failing would make the
// retry write the first one's records twice.
static void dlog_flush(void)
{
    if (g_dlog_count == 0 || !storage_available()) {
        return;
    }
    if (g_dlog_dropped) {
        ESP_LOGW(TAG, "Decision log overflowed: %u record(s) dropped before flush",
                 (unsigned)g_dlog_dropped);
        g_dlog_dropped = 0;
    }

    while (g_dlog_count > 0) {
        uint16_t run = DECISION_RING_N - g_dlog_head;   // to the end of the array
        if (run > g_dlog_count) {
            run = g_dlog_count;
        }
        if (!storage_flush_decision_log(&g_dlog[g_dlog_head], run)) {
            return;   // keep what's left; the next mounted wake tries again
        }
        g_dlog_head = (g_dlog_head + run) % DECISION_RING_N;
        g_dlog_count -= run;
    }
}

static SemaphoreHandle_t s_halow_connected_sem = NULL;
static SemaphoreHandle_t s_got_ip_sem = NULL;

// ============================================================================
// Config
// ============================================================================

// Called from the MQTT task when the base station pushes a config. Parses and
// persists it, then stages the result for the main task rather than writing
// g_cfg directly (see s_pending_cfg). Acknowledges the update on
// camera/config_ack/<node> so the operator pushing the change knows it actually
// reached and was applied by the node — a bare publish to the broker proves
// nothing if the node was offline or the JSON didn't parse.
//
// storage_save_config rejects the push outright if its "ts" isn't newer than
// the config already applied — this is what stops a stale retained message on
// the broadcast or per-node topic from silently re-winning after a fresher
// command was already applied, which otherwise looks like the resolution
// "snapping back" a moment after it changes.
static void on_config_pushed(const char *json, size_t len)
{
    node_config_t cfg = g_cfg;
    if (storage_save_config(json, len, &cfg)) {
        s_pending_cfg    = cfg;
        s_config_pending = true;   // last: the flag is what publishes the struct
        mqtt_node_publish_config_ack(cfg.params.sleep_baseline_s,
                                     storage_frame_size_to_string(cfg.frame_size),
                                     cfg.config_ts);
    }
}

// ============================================================================
// Capture time
// ============================================================================

// ISO-8601 UTC for the moment a photo was taken. False means the node has never
// synced its clock, so the time is genuinely unknown rather than a bogus 1970
// value — the base station flags the image accordingly.
static bool format_capture_time(char *buf, size_t len)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    if (tv.tv_sec > 1000000LL) {
        struct tm t;
        gmtime_r(&tv.tv_sec, &t);
        snprintf(buf, len, "%04d-%02d-%02dT%02d:%02d:%02dZ",
                 t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
                 t.tm_hour, t.tm_min, t.tm_sec);
        return true;
    }
    snprintf(buf, len, "unknown");
    return false;
}

// Recovers the capture time of a cached frame from the name storage_build_cache_path
// gave it. "YYYYMMDD_HHMMSS_<node>.jpg" was taken with a synced clock;
// "UNSYNCED_..." was not.
static bool capture_time_from_name(const char *name, char *out, size_t len)
{
    int y, mo, d, h, mi, s;
    if (sscanf(name, "%4d%2d%2d_%2d%2d%2d", &y, &mo, &d, &h, &mi, &s) == 6) {
        snprintf(out, len, "%04d-%02d-%02dT%02d:%02d:%02dZ", y, mo, d, h, mi, s);
        return true;
    }
    snprintf(out, len, "unknown");
    return false;
}

// ============================================================================
// HaLow
// ============================================================================

static void ip_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&e->ip_info.ip));
        if (s_got_ip_sem) {
            xSemaphoreGive(s_got_ip_sem);
        }
    }
}

static void halow_status_cb(enum mmwlan_sta_state state)
{
    switch (state) {
    case MMWLAN_STA_DISABLED:
        ESP_LOGI(TAG, "WLAN STA disabled");
        break;
    case MMWLAN_STA_CONNECTING:
        ESP_LOGI(TAG, "WLAN STA connecting");
        break;
    case MMWLAN_STA_CONNECTED:
        ESP_LOGI(TAG, "WLAN STA connected");
        if (s_halow_connected_sem) {
            xSemaphoreGive(s_halow_connected_sem);
        }
        break;
    }
}

static bool halow_connect(void)
{
    ESP_LOGI(TAG, "Initialising Wi-Fi HaLow interface...");
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                               &ip_event_handler, NULL));

    if (mmhalow_init(NULL) != 0) {
        ESP_LOGE(TAG, "mmhalow_init failed — transport is broken");
        return false;
    }
    mmhalow_print_version_info();

    mmhalow_wifi_config_t conf = { .sta = MMWLAN_STA_ARGS_INIT };
    memcpy(conf.sta.ssid, WIFI_SSID, strlen(WIFI_SSID));
    conf.sta.ssid_len = strlen(WIFI_SSID);
    memcpy(conf.sta.passphrase, WIFI_PSK, strlen(WIFI_PSK));
    conf.sta.passphrase_len = strlen(WIFI_PSK);
    conf.sta.security_type = MMWLAN_SAE;
    mmhalow_set_config(WIFI_IF_STA, &conf);

    s_halow_connected_sem = xSemaphoreCreateBinary();
    s_got_ip_sem = xSemaphoreCreateBinary();

    ESP_LOGI(TAG, "Connecting to SSID: %s (up to %d s)", WIFI_SSID, CONNECT_TIMEOUT_MS / 1000);
    mmhalow_connect(halow_status_cb);

    if (xSemaphoreTake(s_halow_connected_sem, pdMS_TO_TICKS(CONNECT_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "HaLow association timed out");
        return false;
    }
    if (xSemaphoreTake(s_got_ip_sem, pdMS_TO_TICKS(CONNECT_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "Timed out waiting for DHCP");
        return false;
    }
    return true;
}

static void sync_time(void)
{
    esp_sntp_config_t cfg = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
    esp_netif_sntp_init(&cfg);
    if (esp_netif_sntp_sync_wait(pdMS_TO_TICKS(NTP_TIMEOUT_MS)) == ESP_OK) {
        char now[24];
        format_capture_time(now, sizeof(now));
        ESP_LOGI(TAG, "NTP synced: %s", now);
    } else {
        ESP_LOGW(TAG, "NTP sync timed out — capture times stay as they were");
    }
    esp_netif_sntp_deinit();
}

// ============================================================================
// Upload
// ============================================================================

// Publishes the sidecar, then the JPEG, then waits for the base station's ACK.
//
// A successful publish only proves the bytes reached the broker — the broker
// stays up even when receive_images.py is down. Only the ACK proves someone
// wrote the image to disk. An ACK of "incomplete" still counts as delivered:
// the source bytes can't change, so resending would reproduce the same
// truncated file. Returns false only when no ACK comes back at all.
static bool upload_image(const uint8_t *buf, size_t len,
                         const char *capture_time, bool time_synced)
{
    mqtt_node_reset_ack();

    if (!mqtt_node_publish_meta(capture_time, time_synced)) {
        return false;
    }
    if (!mqtt_node_publish_image(buf, len)) {
        ESP_LOGE(TAG, "Image publish failed");
        return false;
    }

    bool complete = false;
    if (!mqtt_node_wait_ack(ACK_TIMEOUT_MS, &complete)) {
        ESP_LOGW(TAG, "No ACK from base station (mqtt connected=%d)", mqtt_node_connected());
        return false;
    }
    if (complete) {
        ESP_LOGI(TAG, "Base station ACKed the image");
    } else {
        ESP_LOGW(TAG, "Base station ACKed the image as incomplete "
                      "(accepted, no retry — the bytes can't change)");
    }
    return true;
}

// Flushes every frame cached by an earlier wake, deleting each only once the
// base station has acknowledged it.
static void upload_cache(void)
{
    DIR *dir = opendir(SD_CACHE_DIR);
    if (!dir) {
        ESP_LOGW(TAG, "Cache dir missing — nothing to flush");
        return;
    }

    int uploaded = 0;
    struct dirent *ent;
    while ((ent = readdir(dir)) != NULL) {
        if (ent->d_type == DT_DIR) {
            continue;
        }

        char path[300];
        snprintf(path, sizeof(path), "%s/%s", SD_CACHE_DIR, ent->d_name);

        size_t len = 0;
        uint8_t *buf = storage_read_file(path, &len);
        if (!buf) {
            ESP_LOGW(TAG, "Could not read %s — skipping", path);
            continue;
        }

        char capture_time[24];
        bool time_known = capture_time_from_name(ent->d_name, capture_time, sizeof(capture_time));
        ESP_LOGI(TAG, "Uploading cached %s (%u bytes)%s", path, (unsigned)len,
                 len > UPLOAD_SINGLE_MAX ? " [chunked]" : "");

        bool ok = upload_image(buf, len, capture_time, time_known);
        free(buf);

        if (!ok) {
            // The base station ACKs everything or nothing. If this frame wasn't
            // acknowledged the rest won't be either — keep them all for the
            // next wake rather than burning link time.
            ESP_LOGW(TAG, "No ACK — keeping %s and the rest of the cache", path);
            break;
        }

        unlink(path);
        uploaded++;
        ESP_LOGI(TAG, "Uploaded and removed %s", path);
    }
    closedir(dir);

    if (uploaded > 0) {
        ESP_LOGI(TAG, "Flushed %d cached frame(s)", uploaded);
    }
}

// ============================================================================
#if CONFIG_NODE_STREAM_SERVER
static void start_stream_server(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;
    config.lru_purge_enable = true;

    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        return;
    }
    if (camera_register_handlers(server) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register camera handlers");
        return;
    }
    ESP_LOGI(TAG, "HTTP server started — stream at /stream, still at /capture");
}
#endif

static void deep_sleep(uint32_t seconds)
{
    storage_unmount();
    ESP_LOGI(TAG, "Sleeping %u s...", (unsigned)seconds);
    esp_sleep_enable_timer_wakeup((uint64_t)seconds * 1000000ULL);
    esp_deep_sleep_start();   // never returns; the next wake re-enters app_main
}

// First cold boot: derive the node ID from the eFuse MAC (no radio needed)
// and read config.json from the SD card. Warm wakes get both from RTC memory.
static void ensure_node_identity(bool sd_ok)
{
    if (!g_first_boot) {
        ESP_LOGI(TAG, "Node ID: %s (from RTC)", g_node_id);
        return;
    }

    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(g_node_id, sizeof(g_node_id), "%02X%02X%02X%02X%02X%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    ESP_LOGI(TAG, "Node ID: %s", g_node_id);

    if (sd_ok) {
        storage_load_config(&g_cfg);   // leaves g_cfg's defaults where the file is silent
    }
    g_first_boot = false;
}

// HaLow association, NTP, MQTT connect. Any config the base station has
// waiting (retained on the broker, or pushed while we're connected) is applied
// here, before the camera is touched — so a resolution change lands in *this*
// wake's photo instead of only taking effect a whole sleep cycle later.
static bool radio_bring_up(void)
{
    bool mqtt_ready = false;
    if (halow_connect()) {
        sync_time();
        mqtt_ready = mqtt_node_start(g_node_id, on_config_pushed, MQTT_CONNECT_MS);
    }
    return mqtt_ready;
}

// One full JPEG round: capture at the resolution in effect right now, stamp
// EXIF (node ID, capture time, optional scene note), upload, flush the cache
// of earlier frames, and cache this one on SD if it wasn't delivered.
// The camera must already be initialised for JPEG. Returns whether the frame
// reached the base station.
static bool capture_and_send(bool mqtt_ready, bool sd_ok, const char *note)
{
    camera_fb_t *fb = camera_capture();

    // Stamp the time at the moment of capture. If HaLow connected this wake,
    // NTP already synced, so this is normally fresh; otherwise it's the
    // clock carried across deep sleep from an earlier sync.
    char capture_time[24];
    bool time_known = format_capture_time(capture_time, sizeof(capture_time));

    // Embed the node ID (MAC-derived) and capture time into the JPEG itself as
    // an EXIF APP1 segment, so the image is self-describing on the SD cache and
    // after upload alike — it doesn't depend on the base station's out-of-band
    // metadata sidecar to know who took it or when. Falls back to the raw frame
    // buffer if the insert fails for any reason (e.g. OOM).
    uint8_t *img_buf = NULL;
    size_t   img_len = 0;
    if (fb) {
        img_buf = exif_embed_jpeg(fb->buf, fb->len, g_node_id, note, capture_time, time_known, &img_len);
        if (!img_buf) {
            img_buf = fb->buf;
            img_len = fb->len;
        }
    }

    if (!fb) {
        ESP_LOGE(TAG, "Camera capture failed");
    } else {
        ESP_LOGI(TAG, "Captured %ux%u JPEG, %u bytes (%u with EXIF), EOI=%s, taken %s%s",
                 (unsigned)fb->width, (unsigned)fb->height, (unsigned)fb->len, (unsigned)img_len,
                 camera_frame_complete(fb) ? "yes" : "NO",
                 capture_time, time_known ? "" : " (clock not set)");
    }

    // --- Upload ---
    bool frame_sent = false;
    if (mqtt_ready) {
        // Send this frame straight from PSRAM — never via the SD card, which
        // truncates large sustained writes on this board.
        if (fb) {
            ESP_LOGI(TAG, "Uploading current frame (%u bytes)%s",
                     (unsigned)img_len, img_len > UPLOAD_SINGLE_MAX ? " [chunked]" : "");
            frame_sent = upload_image(img_buf, img_len, capture_time, time_known);
        }

        // Only flush the cache if the base station is actually answering.
        // If this frame got no ACK, the station is down and every cached
        // upload would time out too.
        if (sd_ok && (frame_sent || !fb)) {
            upload_cache();
        }
    }

    // --- Anything unacknowledged gets cached and retried on a later wake.
    // Covers every failure: no HaLow link, broker unreachable, or the base
    // station script not running (the publish reaches the broker but no ACK
    // ever comes back).
    if (fb && !frame_sent) {
        if (sd_ok) {
            char path[300];
            storage_build_cache_path(path, sizeof(path), g_node_id);
            if (storage_write_photo(path, img_buf, img_len)) {
                ESP_LOGI(TAG, "Frame cached — will upload when the base station is reachable");
            }
        } else {
            ESP_LOGE(TAG, "SD unavailable and frame not sent — frame lost");
        }
    }

    if (img_buf && img_buf != fb->buf) {
        free(img_buf);
    }
    if (fb) {
        esp_camera_fb_return(fb);
    }
    return frame_sent;
}

void app_main(void)
{
    ESP_LOGI(TAG, "--- HaLow Node Wake (built %s %s) ---", __DATE__, __TIME__);
    ESP_LOGI(TAG, "PSRAM: %u bytes free of %u",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
             (unsigned)heap_caps_get_total_size(MALLOC_CAP_SPIRAM));

#if CONFIG_NODE_STREAM_SERVER
    // Dev mode: the legacy always-on flow, no scene FSM, no deep sleep.
    bool sd_ok = storage_mount();
    ensure_node_identity(sd_ok);
    bool mqtt_ready = radio_bring_up();
    if (camera_init(g_cfg.frame_size) == ESP_OK) {
        capture_and_send(mqtt_ready, sd_ok, NULL);
    }
    if (mqtt_ready) {
        mqtt_node_wait_for_config(MQTT_FLUSH_MS);
        mqtt_node_stop();
    }
    ESP_LOGI(TAG, "Stream mode: staying awake instead of deep sleeping");
    start_stream_server();
    return;
#else
    // --- Identity + cold-boot restore. Warm wakes have everything in RTC
    // memory already, so they mount nothing and touch no radio here.
    bool cold  = g_first_boot;
    bool sd_ok = false;
    if (cold) {
        // Cold boot mounts unconditionally: config.json and the mirrored
        // adaptive state (both lost from RTC on power loss) live on the card.
        sd_ok = storage_mount();
    }
    ensure_node_identity(sd_ok);
    if (cold && sd_ok && storage_load_adaptive(&g_adapt)) {
        g_adapt_valid = true;
    }

    // --- Cheap grayscale probe: the FSM's input. No JPEG is encoded and no
    // radio powered unless the scene turns out to deserve it.
    bool probe_ok = false;
    if (camera_init_grayscale() == ESP_OK) {
        camera_fb_t *fb = camera_capture();
        if (fb) {
            adaptive_downscale(fb->buf, fb->width, fb->height, s_cur_grid);
            esp_camera_fb_return(fb);
            probe_ok = true;
        } else {
            ESP_LOGE(TAG, "Grayscale probe capture failed");
        }
        camera_deinit();   // frees the driver for a JPEG re-init if we send
    }

    // --- Battery: read at this fixed point every wake, with the radio still down,
    // so a systematic load offset cancels in the trend (plan §11 voltage-sag trap).
    // Folds into g_adapt.batt, which is a valid default on cold boot (zeroed RTC)
    // and real state on a warm wake, so this is safe before the FSM state is
    // (re)seeded below. The band it refreshes is the governor's ceiling.
    uint16_t vbat_mv = battery_sample_mv();
    battery_update(&g_adapt.batt, vbat_mv, wall_now());
    ESP_LOGI(TAG, "battery: %u mV soc=%u%% band=%s rate=%d mV/min",
             (unsigned)vbat_mv, (unsigned)battery_soc_from_mv(vbat_mv),
             battery_band_name(g_adapt.batt.band), (int)g_adapt.batt.charge_rate);

    if (!probe_ok) {
        // Camera dead this wake: no score, so the FSM doesn't tick. Still
        // honor the radio duty cycle — a camera-dead node must stay reachable
        // for config pushes instead of going dark until someone walks out.
        ESP_LOGE(TAG, "Scene probe failed — fallback sleep %u s",
                 (unsigned)g_cfg.params.sleep_baseline_s);
        bool radio_up = false;
        if (++g_wakes_since_radio >= K_RADIO) {
            g_wakes_since_radio = 0;
            if (!sd_ok) {
                sd_ok = storage_mount();
            }
            if (radio_bring_up()) {
                mqtt_node_wait_for_config(MQTT_FLUSH_MS);
                mqtt_node_stop();
            }
            // A camera-dead node is exactly where a pushed fix is aimed, so
            // take the staged config even though this wake decides nothing.
            if (s_config_pending) {
                g_cfg = s_pending_cfg;
                s_config_pending = false;
            }
            radio_up = true;
        }

        // Log the dead wake too — a gap in the log otherwise reads as a node
        // that missed its wake entirely, which is a very different problem.
        uint32_t now = wall_now();
        decision_rec_t rec = {
            .ts            = now,
            .score         = 0,
            .next_sleep_s  = (uint16_t)g_cfg.params.sleep_baseline_s,
            .t_low         = g_cfg.params.t_low,
            .t_high        = g_cfg.params.t_high,
            .state         = g_adapt.fsm_state,
            .resolution    = (uint8_t)g_cfg.frame_size,
            .override_mode = ADAPT_OVR_NONE,
            .band          = g_adapt.batt.band,
            .soc           = battery_soc_from_mv(vbat_mv),
            .charge_rate   = g_adapt.batt.charge_rate,
            .flags         = DLOG_F_PROBE_FAILED
                           | (radio_up ? DLOG_F_RADIO_UP : 0)
                           | (now ? DLOG_F_TIME_SYNCED : 0),
        };
        dlog_append(&rec);
        dlog_flush();
        deep_sleep(g_cfg.params.sleep_baseline_s);
    }

    if (!g_adapt_valid) {
        // Power loss with no usable SD mirror: this first frame becomes the
        // provisional reference, so this wake scores quiet and starts from
        // BASELINE (plan §5).
        adaptive_init_default(&g_adapt, &g_cfg.params, s_cur_grid);
        g_adapt_valid = true;
    }

    // --- Score the scene and tick the FSM. This is the scene's proposal, from
    // the scene alone: d_base is kept so the override can be re-applied later
    // against a clean decision rather than compounding onto its own output.
    bool lighting = false;
    int  score = adaptive_score(s_cur_grid, g_adapt.ref_grid, &g_cfg.params, &lighting);
    adaptive_decision_t d_base = adaptive_fsm_step(&g_adapt, &g_cfg.params, score, lighting);
    ESP_LOGI(TAG, "adaptive: score=%d lighting=%d state=%s send=%d next_sleep=%u s",
             score, (int)lighting, adaptive_fsm_state_name(d_base.next_state),
             (int)d_base.send, (unsigned)d_base.next_sleep_s);

    // --- Pass 1: the lease already in RTC memory, applied before the radio gate
    // so a live override can ask for a send, and an expired one can lapse, on a
    // wake where the radio never comes up at all. That local expiry is the whole
    // point: the base station cannot be relied on to end its own lease.
    adaptive_decision_t d = d_base;
    adaptive_apply_override(&d, &g_cfg.params, &g_cfg.override, wall_now());
    if (d.override_active) {
        ESP_LOGI(TAG, "override: %s active until %u",
                 adaptive_override_mode_name(d.override_mode),
                 (unsigned)g_cfg.override.valid_until);
    }

    // Energy governor last (plan §1): cap the scene+override decision by the SoC
    // band before the radio gate reads d.send, so a battery-suppressed send never
    // powers up the radio. The Kth-quiet-wake duty cycle below still runs, so the
    // node stays reachable for a "resume" config push even in CRITICAL.
    adaptive_govern(&d, &g_adapt.batt, &g_cfg.params, score);
    if (g_adapt.batt.band != BATT_HEALTHY) {
        ESP_LOGI(TAG, "governor: band=%s capped send=%d next_sleep=%u s",
                 battery_band_name(g_adapt.batt.band), (int)d.send,
                 (unsigned)d.next_sleep_s);
    }

    // --- Radio only when the scene warrants it, or on the Kth quiet wake so
    // config pushes and cache flushes still happen. This skip is the node's
    // main energy win: a quiet wake costs camera + CPU, no HaLow session.
    bool mqtt_ready = false;
    if (d.send || g_wakes_since_radio + 1 >= K_RADIO) {
        // Reset on the attempt, not on success — bounds the connect duty
        // cycle to one attempt per K quiet wakes even with the base station
        // down, instead of burning a 30 s association timeout every wake.
        g_wakes_since_radio = 0;
        if (!sd_ok) {
            sd_ok = storage_mount();   // pushed-config save + cache need it
        }
        mqtt_ready = radio_bring_up();
        if (mqtt_ready) {
            // Hold the connection open for the broker's retained config (and any
            // live push) to land *before* pass 2 commits it and before the camera
            // captures. radio_bring_up() returns the instant MQTT connects — about
            // half a second ahead of the retained PUBLISH — so without this wait
            // pass 2 consumes an empty s_config_pending, the config arrives too
            // late, and deep sleep wipes the non-RTC staging buffer. That's what
            // otherwise makes retuning and the override lease silently never take
            // hold on a warm wake. Mirrors the camera-dead path above.
            mqtt_node_wait_for_config(MQTT_FLUSH_MS);
        }
    } else {
        g_wakes_since_radio++;
        ESP_LOGI(TAG, "Quiet wake — radio skipped (%u/%u)",
                 (unsigned)g_wakes_since_radio, (unsigned)K_RADIO);
    }

    // --- Pass 2: re-apply against the freshest lease, on every radio-up wake
    // rather than only when a config landed. sync_time() inside radio_bring_up()
    // may have just given the node a real clock for the first time since a power
    // loss, and pass 1 drops every lease while the clock is unknown — so this is
    // the only pass that can honour a live one. Recomputing from d_base keeps
    // this from stacking on pass 1's result.
    if (mqtt_ready) {
        if (s_config_pending) {
            g_cfg = s_pending_cfg;
            s_config_pending = false;
        }
        d = d_base;
        adaptive_apply_override(&d, &g_cfg.params, &g_cfg.override, wall_now());
        // Governor stays the final cap, recomputed from d_base like the override
        // (a pushed config may have just retuned the band caps). Energy still wins.
        adaptive_govern(&d, &g_adapt.batt, &g_cfg.params, score);
    }

    // --- Act on the decision.
    bool        frame_sent = false;
    framesize_t used_fs    = g_cfg.frame_size;
    if (d.send) {
        char note[64];
        snprintf(note, sizeof(note), "adaptive score=%d state=%s ovr=%s",
                 score, adaptive_fsm_state_name(d.next_state),
                 adaptive_override_mode_name(d.override_mode));
        used_fs = d.send_highres ? g_cfg.frame_size_high : g_cfg.frame_size;
        if (camera_init(used_fs) == ESP_OK) {
            frame_sent = capture_and_send(mqtt_ready, sd_ok, note);
            camera_deinit();
            if (d.send_highres && !g_cfg.override.fired) {
                // Spend the one-shot. Persisted immediately so a cold boot
                // doesn't read the still-live lease off the card and ship a
                // second detail frame nobody asked for.
                g_cfg.override.fired = true;
                if (storage_available()) {
                    storage_write_config(&g_cfg);
                }
            }
        } else {
            // The FSM decision stands: state and sleep are already set, and
            // an ACTIVE scene retries in seconds on the next wake.
            ESP_LOGE(TAG, "JPEG re-init failed — frame skipped this wake");
        }
    } else if (mqtt_ready && sd_ok) {
        upload_cache();   // radio is up for the config pull anyway
    }
    if (mqtt_ready) {
        // Heartbeat last, so `sent` reports what actually happened rather than
        // what was intended, and the override reflects pass 2.
        mqtt_node_state_t hb = {
            .ts                   = wall_now(),
            .state                = adaptive_fsm_state_name(d.next_state),
            .score                = score,
            .t_low                = g_cfg.params.t_low,
            .t_high               = g_cfg.params.t_high,
            .sent                 = frame_sent,
            .resolution           = storage_frame_size_to_string(used_fs),
            .next_sleep_s         = d.next_sleep_s,
            .override_mode        = adaptive_override_mode_name(d.override_mode),
            .override_active      = d.override_active,
            .override_valid_until = d.override_active ? g_cfg.override.valid_until : 0,
            .config_ts            = g_cfg.config_ts,
            .time_synced          = (wall_now() != 0),
            .band                 = battery_band_name(g_adapt.batt.band),
            .soc                  = battery_soc_from_mv(vbat_mv),
            .charge_rate          = g_adapt.batt.charge_rate,
        };
        mqtt_node_publish_state(&hb);

        // A config pushed after the one applied on connect (a second, late
        // message racing in) can still land here — it affects the next wake.
        mqtt_node_wait_for_config(MQTT_FLUSH_MS);
        mqtt_node_stop();
    }

    // --- Reference update + persistence. The RTC copy is g_adapt itself;
    // the SD mirror rides along only when the card is already mounted.
    if (d.rebaseline) {
        // Hybrid re-baseline (plan §5 option a): adopt the settled scene as the
        // new reference in one step. A slow blend would take many wakes to stop
        // the parked object from scoring, leaving the node ACTIVE in the meantime.
        memcpy(g_adapt.ref_grid, s_cur_grid, ADAPT_CELLS);
        ESP_LOGI(TAG, "Re-baselined: parked scene adopted as reference, back to BASELINE");
    } else {
        adaptive_update_ref(g_adapt.ref_grid, s_cur_grid, /*quiet=*/!d.freeze_ref);
    }
    if (storage_available()) {
        storage_save_adaptive(&g_adapt);
    }

    // --- Record why this wake did what it did. Buffered in RTC; dlog_flush is
    // a no-op unless the card happens to be mounted already, so a quiet wake
    // still touches nothing.
    uint32_t now = wall_now();
    decision_rec_t rec = {
        .ts            = now,
        .score         = (uint16_t)(score < 0 ? 0 : score),
        .next_sleep_s  = (uint16_t)d.next_sleep_s,
        .t_low         = g_cfg.params.t_low,
        .t_high        = g_cfg.params.t_high,
        .state         = (uint8_t)d.next_state,
        .resolution    = (uint8_t)used_fs,
        .override_mode = d.override_mode,
        .band          = g_adapt.batt.band,
        .soc           = battery_soc_from_mv(vbat_mv),
        .charge_rate   = g_adapt.batt.charge_rate,
        .flags         = (frame_sent ? DLOG_F_SENT : 0)
                       | (d.override_active ? DLOG_F_OVERRIDE_ACTIVE : 0)
                       | (lighting ? DLOG_F_LIGHTING : 0)
                       | (mqtt_ready ? DLOG_F_RADIO_UP : 0)
                       | (now ? DLOG_F_TIME_SYNCED : 0),
    };
    dlog_append(&rec);
    dlog_flush();

    deep_sleep(d.next_sleep_s);
#endif
}
