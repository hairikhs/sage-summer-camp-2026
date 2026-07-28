#include "storage.h"

#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/time.h>

#include "cJSON.h"
#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "driver/sdspi_host.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

static const char *TAG = "storage";

// The HaLow radio claims SPI2 in mmhalow_init(), so the card goes on SPI3.
// These pins are the ones left free by the camera's DVP bus (12-21, 38-48) and
// the radio (2-9).
#define SD_SCK_PIN   15
#define SD_MISO_PIN  16
#define SD_MOSI_PIN  11
#define SD_CS_PIN    10
#define SD_SPI_HOST  SPI3_HOST

static sdmmc_card_t *s_card = NULL;
static bool s_mounted = false;

bool storage_available(void) { return s_mounted; }

bool storage_mount(void)
{
    if (s_mounted) {
        return true;
    }

    spi_bus_config_t bus = {
        .mosi_io_num = SD_MOSI_PIN,
        .miso_io_num = SD_MISO_PIN,
        .sclk_io_num = SD_SCK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };
    esp_err_t err = spi_bus_initialize(SD_SPI_HOST, &bus, SPI_DMA_CH_AUTO);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {   // INVALID_STATE = already up
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(err));
        return false;
    }

    // Deliberately NOT enabling the ESP32's internal pull-ups on MISO/MOSI/CS:
    // they are weak (~45k) and this module does not enumerate with them on. It
    // supplies its own pull-ups.
    esp_vfs_fat_sdmmc_mount_config_t mount_cfg = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024,
    };

    sdspi_device_config_t slot = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot.gpio_cs = SD_CS_PIN;
    slot.host_id = SD_SPI_HOST;

    // 500 kHz. The card on this build won't run the bus faster; note the driver
    // always *probes* at SDMMC_FREQ_PROBING (400 kHz) regardless, so this is the
    // rate used once the card is up, not during init.
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SD_SPI_HOST;
    host.max_freq_khz = 500;

    // Cards can be slow to come up after power-on; a couple of retries costs
    // nothing and saves a wake cycle's worth of photos.
    for (int attempt = 1; attempt <= 3; attempt++) {
        err = esp_vfs_fat_sdspi_mount(SD_MOUNT_POINT, &host, &slot, &mount_cfg, &s_card);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "SD mounted at %d kHz", host.max_freq_khz);
            sdmmc_card_print_info(stdout, s_card);
            s_mounted = true;
            mkdir(SD_CACHE_DIR, 0777);
            return true;
        }
        ESP_LOGW(TAG, "SD mount attempt %d failed: %s", attempt, esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    ESP_LOGE(TAG, "SD init failed — photos will not be cached");
    spi_bus_free(SD_SPI_HOST);
    return false;
}

void storage_unmount(void)
{
    if (!s_mounted) {
        return;
    }
    esp_vfs_fat_sdcard_unmount(SD_MOUNT_POINT, s_card);
    spi_bus_free(SD_SPI_HOST);
    s_card = NULL;
    s_mounted = false;
}

framesize_t storage_parse_frame_size(const char *s, framesize_t fallback)
{
    if (!s)                    return fallback;
    if (!strcmp(s, "QXGA"))    return FRAMESIZE_QXGA;
    if (!strcmp(s, "FHD"))     return FRAMESIZE_FHD;
    if (!strcmp(s, "HD"))      return FRAMESIZE_HD;
    if (!strcmp(s, "XGA"))     return FRAMESIZE_XGA;
    if (!strcmp(s, "VGA"))     return FRAMESIZE_VGA;
    if (!strcmp(s, "QVGA"))    return FRAMESIZE_QVGA;
    ESP_LOGW(TAG, "Unknown resolution '%s', keeping current", s);
    return fallback;
}

const char *storage_frame_size_to_string(framesize_t fs)
{
    switch (fs) {
    case FRAMESIZE_QXGA: return "QXGA";
    case FRAMESIZE_FHD:  return "FHD";
    case FRAMESIZE_HD:   return "HD";
    case FRAMESIZE_XGA:  return "XGA";
    case FRAMESIZE_VGA:  return "VGA";
    case FRAMESIZE_QVGA: return "QVGA";
    default:             return "UNKNOWN";
    }
}

// Unix seconds, or 0 when the clock has never been synced — the same "is this a
// real time?" test format_capture_time uses. A lease derived against 0 is dead
// on arrival, which is the intended conservative outcome.
static uint32_t wall_now(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (tv.tv_sec > 1000000LL) ? (uint32_t)tv.tv_sec : 0;
}

// Reads an optional uint16 field, leaving *dst alone when the document omits it.
static void get_u16_field(const cJSON *doc, const char *key, uint16_t *dst)
{
    const cJSON *it = cJSON_GetObjectItemCaseSensitive(doc, key);
    if (cJSON_IsNumber(it) && it->valuedouble >= 0) {
        *dst = (uint16_t)it->valuedouble;
    }
}

static void get_u32_field(const cJSON *doc, const char *key, uint32_t *dst)
{
    const cJSON *it = cJSON_GetObjectItemCaseSensitive(doc, key);
    if (cJSON_IsNumber(it) && it->valuedouble >= 0) {
        *dst = (uint32_t)it->valuedouble;
    }
}

// Parses the "override" block (plan §8). Three distinct cases, and the
// difference between the first two matters: an *absent* override keeps whatever
// lease is live, because `send_config.py --sleep 60` sends no override block and
// must not cancel a lease as a side effect. Cancelling is explicit.
static void apply_override_json(const cJSON *doc, node_config_t *cfg)
{
    const cJSON *ov = cJSON_GetObjectItemCaseSensitive(doc, "override");
    if (!ov) {
        return;                       // absent: keep the current lease
    }
    if (cJSON_IsNull(ov)) {
        ESP_LOGI(TAG, "Override cleared");
        cfg->override = (adaptive_override_t){ .mode = ADAPT_OVR_NONE };
        return;
    }
    if (!cJSON_IsObject(ov)) {
        ESP_LOGW(TAG, "Override is not an object — keeping current");
        return;
    }

    const cJSON *mode_item = cJSON_GetObjectItemCaseSensitive(ov, "mode");
    uint8_t mode = ADAPT_OVR_NONE;
    if (!cJSON_IsString(mode_item) ||
        !adaptive_override_mode_parse(mode_item->valuestring, &mode)) {
        ESP_LOGW(TAG, "Override has no usable 'mode' — keeping current");
        return;
    }

    uint32_t now = wall_now();
    adaptive_override_t n = { .mode = mode, .fired = false, .cadence_s = 0 };
    get_u32_field(ov, "cadence_s", &n.cadence_s);

    // Absent deadline: give it the default lease rather than treating it as
    // "forever". A retained message with no expiry is how a node ends up pinned
    // hot long after the animal has gone.
    n.valid_until = now + ADAPT_OVERRIDE_TTL_S;
    get_u32_field(ov, "valid_until", &n.valid_until);

    if (now == 0) {
        ESP_LOGW(TAG, "Override '%s' arrived before the clock synced — it will be "
                      "ignored until NTP lands", adaptive_override_mode_name(mode));
    } else if (n.valid_until > now + ADAPT_OVERRIDE_MAX_LEASE_S) {
        ESP_LOGW(TAG, "Override lease capped: valid_until=%u is beyond the %u s max",
                 (unsigned)n.valid_until, (unsigned)ADAPT_OVERRIDE_MAX_LEASE_S);
        n.valid_until = now + ADAPT_OVERRIDE_MAX_LEASE_S;
    }

    cfg->override = n;
    ESP_LOGI(TAG, "Override applied: mode=%s cadence=%us valid_until=%u (now=%u)",
             adaptive_override_mode_name(mode), (unsigned)n.cadence_s,
             (unsigned)n.valid_until, (unsigned)now);
}

// Pulls the config schema (plan §8) out of a document. Fields the document omits
// keep their existing value, so a partial push only changes what it names. When
// `is_push` is set (a live MQTT config, as opposed to the config file read at
// boot), a document whose "ts" is not newer than cfg->config_ts is rejected
// outright — this is what stops a stale retained MQTT message (an old broadcast
// or per-node override still sitting on the broker) from winning a replay race
// against a fresher command and silently reverting a setting the operator just
// changed.
static bool apply_config_json(const char *json, size_t len, node_config_t *cfg, bool is_push)
{
    cJSON *doc = cJSON_ParseWithLength(json, len);
    if (!doc) {
        ESP_LOGW(TAG, "Config parse failed");
        return false;
    }

    uint32_t ts = 0;
    cJSON *ts_item = cJSON_GetObjectItemCaseSensitive(doc, "ts");
    if (cJSON_IsNumber(ts_item)) {
        ts = (uint32_t)ts_item->valuedouble;
    }

    if (is_push && cfg->config_ts != 0 && ts <= cfg->config_ts) {
        ESP_LOGW(TAG, "Ignoring stale config push (ts=%u <= last applied ts=%u)",
                 (unsigned)ts, (unsigned)cfg->config_ts);
        cJSON_Delete(doc);
        return false;
    }

    // Params land in a local copy so a document that names several of them is
    // validated as a set — t_high has to be checked against the t_low arriving
    // alongside it, not the one being replaced.
    adaptive_params_t p = cfg->params;

    // "sleep_seconds" is the original wire name and still works: the base
    // station's send_config.py speaks it, and there's no reason to break that
    // over a rename. "sleep_baseline_s" wins if a document somehow has both.
    get_u32_field(doc, "sleep_seconds", &p.sleep_baseline_s);
    get_u32_field(doc, "sleep_baseline_s", &p.sleep_baseline_s);
    get_u32_field(doc, "sleep_aroused_s", &p.sleep_aroused_s);
    get_u32_field(doc, "sleep_active_s", &p.sleep_active_s);
    get_u16_field(doc, "t_low", &p.t_low);
    get_u16_field(doc, "t_high", &p.t_high);
    get_u16_field(doc, "pixel_delta", &p.pixel_delta);
    get_u16_field(doc, "n_confirm", &p.n_confirm);
    get_u16_field(doc, "m_cooldown", &p.m_cooldown);
    get_u32_field(doc, "cap_balanced_s", &p.cap_balanced_s);
    get_u32_field(doc, "cap_low_s", &p.cap_low_s);
    get_u32_field(doc, "cap_critical_s", &p.cap_critical_s);

    if (!adaptive_params_validate(&p)) {
        // Clamped, not rejected: the operator gets the closest survivable thing
        // to what they asked for plus a loud log, rather than a push that
        // vanishes without explanation.
        ESP_LOGW(TAG, "Config values out of range — clamped to "
                      "t_low=%u t_high=%u delta=%u sleep=%u/%u/%u",
                 (unsigned)p.t_low, (unsigned)p.t_high, (unsigned)p.pixel_delta,
                 (unsigned)p.sleep_active_s, (unsigned)p.sleep_aroused_s,
                 (unsigned)p.sleep_baseline_s);
    }
    cfg->params = p;

    cJSON *res = cJSON_GetObjectItemCaseSensitive(doc, "resolution");
    if (cJSON_IsString(res) && res->valuestring) {
        cfg->frame_size = storage_parse_frame_size(res->valuestring, cfg->frame_size);
    }

    cJSON *res_hi = cJSON_GetObjectItemCaseSensitive(doc, "resolution_high");
    if (cJSON_IsString(res_hi) && res_hi->valuestring) {
        cfg->frame_size_high = storage_parse_frame_size(res_hi->valuestring,
                                                        cfg->frame_size_high);
    }

    apply_override_json(doc, cfg);

    if (ts > cfg->config_ts) {
        cfg->config_ts = ts;
    }

    cJSON_Delete(doc);
    ESP_LOGI(TAG, "Config applied: sleep=%us resolution=%d t_low=%u t_high=%u ts=%u",
             (unsigned)cfg->params.sleep_baseline_s, (int)cfg->frame_size,
             (unsigned)cfg->params.t_low, (unsigned)cfg->params.t_high,
             (unsigned)cfg->config_ts);
    return true;
}

// Serializes the full merged config (not just whatever fields the triggering
// push happened to include) so a partial update — e.g. a --resolution-only
// push — can't clobber a previously-set sleep interval on disk.
bool storage_write_config(const node_config_t *cfg)
{
    if (!s_mounted) {
        return false;
    }
    cJSON *doc = cJSON_CreateObject();
    // "sleep_seconds" is written as well as read under its original name, so a
    // card moved between an old and a new node still reads the same.
    cJSON_AddNumberToObject(doc, "sleep_seconds", cfg->params.sleep_baseline_s);
    cJSON_AddNumberToObject(doc, "sleep_baseline_s", cfg->params.sleep_baseline_s);
    cJSON_AddNumberToObject(doc, "sleep_aroused_s", cfg->params.sleep_aroused_s);
    cJSON_AddNumberToObject(doc, "sleep_active_s", cfg->params.sleep_active_s);
    cJSON_AddNumberToObject(doc, "t_low", cfg->params.t_low);
    cJSON_AddNumberToObject(doc, "t_high", cfg->params.t_high);
    cJSON_AddNumberToObject(doc, "pixel_delta", cfg->params.pixel_delta);
    cJSON_AddNumberToObject(doc, "n_confirm", cfg->params.n_confirm);
    cJSON_AddNumberToObject(doc, "m_cooldown", cfg->params.m_cooldown);
    cJSON_AddNumberToObject(doc, "cap_balanced_s", cfg->params.cap_balanced_s);
    cJSON_AddNumberToObject(doc, "cap_low_s", cfg->params.cap_low_s);
    cJSON_AddNumberToObject(doc, "cap_critical_s", cfg->params.cap_critical_s);
    cJSON_AddStringToObject(doc, "resolution", storage_frame_size_to_string(cfg->frame_size));
    cJSON_AddStringToObject(doc, "resolution_high",
                            storage_frame_size_to_string(cfg->frame_size_high));
    cJSON_AddNumberToObject(doc, "ts", cfg->config_ts);

    if (cfg->override.mode != ADAPT_OVR_NONE) {
        cJSON *ov = cJSON_AddObjectToObject(doc, "override");
        cJSON_AddStringToObject(ov, "mode", adaptive_override_mode_name(cfg->override.mode));
        cJSON_AddNumberToObject(ov, "cadence_s", cfg->override.cadence_s);
        cJSON_AddNumberToObject(ov, "valid_until", cfg->override.valid_until);
        // The spent latch rides along so a cold boot doesn't re-fire a
        // send_highres the node already answered.
        cJSON_AddBoolToObject(ov, "fired", cfg->override.fired);
    }

    bool ok = false;
    char *out = cJSON_PrintUnformatted(doc);
    if (out) {
        FILE *f = fopen(SD_CONFIG_FILE, "w");
        if (f) {
            size_t len = strlen(out);
            ok = (fwrite(out, 1, len, f) == len);
            fclose(f);
        } else {
            ESP_LOGW(TAG, "Could not persist config to %s", SD_CONFIG_FILE);
        }
        cJSON_free(out);
    }
    cJSON_Delete(doc);
    return ok;
}

// The document doesn't fit the main task's 3.5 KB stack, and it's only ever read
// here — on the main task at cold boot — so a file-static buffer is safe.
static char s_cfg_buf[CONFIG_JSON_MAX];

bool storage_load_config(node_config_t *cfg)
{
    if (!s_mounted) {
        return false;
    }
    FILE *f = fopen(SD_CONFIG_FILE, "r");
    if (!f) {
        ESP_LOGI(TAG, "No config at %s, using defaults", SD_CONFIG_FILE);
        return false;
    }

    size_t n = fread(s_cfg_buf, 1, sizeof(s_cfg_buf) - 1, f);
    fclose(f);
    s_cfg_buf[n] = '\0';

    if (n == sizeof(s_cfg_buf) - 1) {
        // Say so rather than parsing a truncated document: a silent fallback to
        // defaults here is indistinguishable from the file not being read.
        ESP_LOGE(TAG, "Config at %s is larger than %u bytes — ignoring it",
                 SD_CONFIG_FILE, (unsigned)sizeof(s_cfg_buf) - 1);
        return false;
    }

    // Boot-time load establishes the baseline — nothing to compare against yet.
    return apply_config_json(s_cfg_buf, n, cfg, false);
}

// ============================================================================
// Decision log (plan §15)
// ============================================================================

// The header names every column in plan §15, including the ones nothing fills
// in yet: band/soc/charge_rate arrive with the energy governor, and
// tier1_verdict/model_version with the local classifier. They cost ~5 bytes a
// row as empty fields, and emitting them now means those chunks start writing
// data into a log the analysis side already parses, rather than forcing a
// second, incompatible CSV generation into the same deployment's logs.
static const char DLOG_HEADER[] =
    "ts,state,score,t_low,t_high,band,soc,charge_rate,sent,resolution,"
    "next_sleep_s,tier1_verdict,override_mode,override_active,model_version\n";

// ~80 bytes a row, ~16 wakes an hour: ~11 MB a year. One generation of history
// is kept — enough to survive the roll, without letting the card fill.
#define DLOG_MAX_BYTES (1024u * 1024u)

static void roll_decision_log_if_full(void)
{
    struct stat st;
    if (stat(SD_DECISION_LOG, &st) == 0 && st.st_size > (off_t)DLOG_MAX_BYTES) {
        remove(SD_DECISION_LOG_PREV);
        if (rename(SD_DECISION_LOG, SD_DECISION_LOG_PREV) != 0) {
            ESP_LOGW(TAG, "Could not roll %s", SD_DECISION_LOG);
        }
    }
}

bool storage_flush_decision_log(const decision_rec_t *recs, size_t n)
{
    if (!s_mounted || !recs || n == 0) {
        return false;
    }
    roll_decision_log_if_full();

    struct stat st;
    bool need_header = (stat(SD_DECISION_LOG, &st) != 0 || st.st_size == 0);

    FILE *f = fopen(SD_DECISION_LOG, "a");
    if (!f) {
        ESP_LOGW(TAG, "Could not open %s", SD_DECISION_LOG);
        return false;
    }
    if (need_header) {
        fputs(DLOG_HEADER, f);
    }

    for (size_t i = 0; i < n; i++) {
        const decision_rec_t *r = &recs[i];
        fprintf(f, "%u,%s,%u,%u,%u,%s,%u,%d,%u,%s,%u,,%s,%u,\n",
                (unsigned)r->ts,
                adaptive_fsm_state_name(r->state),
                (unsigned)r->score,
                (unsigned)r->t_low,
                (unsigned)r->t_high,
                battery_band_name(r->band),
                (unsigned)r->soc,
                (int)r->charge_rate,
                (r->flags & DLOG_F_SENT) ? 1u : 0u,
                storage_frame_size_to_string((framesize_t)r->resolution),
                (unsigned)r->next_sleep_s,
                adaptive_override_mode_name(r->override_mode),
                (r->flags & DLOG_F_OVERRIDE_ACTIVE) ? 1u : 0u);
    }

    bool ok = (fflush(f) == 0);
    fclose(f);
    if (!ok) {
        ESP_LOGW(TAG, "Decision log write failed — keeping %u record(s) buffered",
                 (unsigned)n);
    }
    return ok;
}

bool storage_save_config(const char *json, size_t len, node_config_t *cfg)
{
    // Apply it even if the card is missing — the setting still takes effect for
    // the next wake via RTC memory, it just won't survive a power cycle.
    bool applied = apply_config_json(json, len, cfg, true);
    if (applied) {
        storage_write_config(cfg);
    }
    return applied;
}

void storage_build_cache_path(char *out, size_t len, const char *node_id)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);

    if (tv.tv_sec > 1000000LL) {           // clock is real (NTP synced at some point)
        struct tm t;
        gmtime_r(&tv.tv_sec, &t);
        snprintf(out, len, "%s/%04d%02d%02d_%02d%02d%02d_%s.jpg",
                 SD_CACHE_DIR,
                 t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
                 t.tm_hour, t.tm_min, t.tm_sec, node_id);
    } else {
        // Never synced: uptime is the only thing that makes the name unique.
        snprintf(out, len, "%s/UNSYNCED_%llu_%s.jpg",
                 SD_CACHE_DIR,
                 (unsigned long long)(esp_timer_get_time() / 1000), node_id);
    }
}

bool storage_write_photo(const char *path, const uint8_t *buf, size_t len)
{
    if (!s_mounted) {
        return false;
    }
    FILE *f = fopen(path, "wb");
    if (!f) {
        ESP_LOGE(TAG, "Cannot open %s for write", path);
        return false;
    }

    // A single large write truncates on this card/SPI setup, so go in blocks
    // and check that every one was accepted.
    const size_t BLK = 512;
    size_t written = 0;
    while (written < len) {
        size_t want = len - written;
        if (want > BLK) want = BLK;
        size_t w = fwrite(buf + written, 1, want, f);
        written += w;
        if (w < want) {
            ESP_LOGE(TAG, "SD stopped accepting data");
            break;
        }
    }
    fflush(f);
    fclose(f);

    if (written != len) {
        ESP_LOGE(TAG, "SD write SHORT on %s: wrote %u of %u bytes",
                 path, (unsigned)written, (unsigned)len);
        return false;
    }
    ESP_LOGI(TAG, "Saved %s (%u bytes)", path, (unsigned)len);
    return true;
}

// One shared buffer for the serialized blob: at 1046 bytes it would not be
// safe on the 3.5 KB main-task stack.
static uint8_t s_adapt_buf[ADAPT_SERIAL_SIZE];

bool storage_save_adaptive(const adaptive_state_t *st)
{
    if (!s_mounted) {
        return false;
    }
    size_t n = adaptive_serialize(st, s_adapt_buf, sizeof(s_adapt_buf));
    if (n == 0) {
        return false;
    }
    return storage_write_photo(SD_ADAPTIVE_FILE, s_adapt_buf, n);
}

bool storage_load_adaptive(adaptive_state_t *st)
{
    if (!s_mounted) {
        return false;
    }
    FILE *f = fopen(SD_ADAPTIVE_FILE, "rb");
    if (!f) {
        ESP_LOGI(TAG, "No adaptive state at %s", SD_ADAPTIVE_FILE);
        return false;
    }
    size_t n = fread(s_adapt_buf, 1, sizeof(s_adapt_buf), f);
    // A trailing read distinguishes "exactly ADAPT_SERIAL_SIZE" from a longer
    // (corrupt or future-format) file, which deserialize must reject.
    if (n == sizeof(s_adapt_buf) && fgetc(f) != EOF) {
        n++;
    }
    fclose(f);

    if (!adaptive_deserialize(st, s_adapt_buf, n)) {
        ESP_LOGW(TAG, "Adaptive state at %s is invalid — ignoring", SD_ADAPTIVE_FILE);
        return false;
    }
    ESP_LOGI(TAG, "Adaptive state restored from %s", SD_ADAPTIVE_FILE);
    return true;
}

uint8_t *storage_read_file(const char *path, size_t *out_len)
{
    FILE *f = fopen(path, "rb");
    if (!f) {
        return NULL;
    }
    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (size <= 0) {
        fclose(f);
        return NULL;
    }

    uint8_t *buf = heap_caps_malloc((size_t)size, MALLOC_CAP_SPIRAM);
    if (!buf) {
        ESP_LOGE(TAG, "Out of PSRAM reading %s (%ld bytes)", path, size);
        fclose(f);
        return NULL;
    }

    size_t n = fread(buf, 1, (size_t)size, f);
    fclose(f);
    if (n != (size_t)size) {
        ESP_LOGE(TAG, "Short read on %s: %u of %ld", path, (unsigned)n, size);
        free(buf);
        return NULL;
    }

    *out_len = n;
    return buf;
}
