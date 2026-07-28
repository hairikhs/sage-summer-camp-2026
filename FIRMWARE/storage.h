#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_camera.h"

#include "adaptive.h"

#define SD_MOUNT_POINT   "/sdcard"
#define SD_CACHE_DIR     SD_MOUNT_POINT "/cache"
#define SD_CONFIG_FILE   SD_MOUNT_POINT "/config.json"
#define SD_ADAPTIVE_FILE SD_MOUNT_POINT "/adaptive.bin"
#define SD_DECISION_LOG  SD_MOUNT_POINT "/decisions.csv"
#define SD_DECISION_LOG_PREV SD_MOUNT_POINT "/decisions.1.csv"

// Node configuration, mirrored on the SD card and pushed by the base station.
// This is the node's *config* state, as opposed to the scene state the camera
// accumulates in adaptive_state_t (adaptive.h) — keeping the two apart is what
// stops a value having two homes that can disagree.
typedef struct {
    framesize_t         frame_size;       // "resolution" — the normal send
    framesize_t         frame_size_high;  // "resolution_high" — send_highres detail
    // Unix timestamp of the config currently applied. A pushed config is only
    // accepted if its own "ts" is newer than this — otherwise a stale retained
    // MQTT message (e.g. an old broadcast or per-node override still sitting
    // on the broker) can silently win a replay race against a fresher command
    // and "lock in" an old resolution/sleep value forever. 0 means no config
    // has ever been applied yet, so the first push is always accepted.
    uint32_t            config_ts;
    adaptive_params_t   params;           // thresholds + cadences (plan §9)
    adaptive_override_t override;         // the base station's lease (plan §8)
} node_config_t;

// Largest config document read from the card. The schema outgrew the old 256 B
// stack buffer, and an oversized read is worse than it looks: it truncates to
// invalid JSON, the parse fails, and the node silently falls back to defaults —
// which looks exactly like "the config file isn't being read at all".
#define CONFIG_JSON_MAX 768

// Mounts the microSD card on SPI3. SPI2 belongs to the HaLow radio, so the two
// buses never collide. Safe to call when no card is present — returns false and
// the caller carries on without a cache.
bool storage_mount(void);
void storage_unmount(void);
bool storage_available(void);

// Reads SD_CONFIG_FILE into `cfg`, leaving any field the file doesn't specify
// untouched. Returns false if there is no config file (defaults stand).
bool storage_load_config(node_config_t *cfg);

// Persists a base-station config payload verbatim and applies it to `cfg`.
bool storage_save_config(const char *json, size_t len, node_config_t *cfg);

// Writes the full merged config to SD_CONFIG_FILE. Callers use this to persist
// a change the node itself made rather than one the base station pushed — the
// spent send_highres latch, so a cold boot doesn't re-fire it. No-op returning
// false when the card isn't mounted.
bool storage_write_config(const node_config_t *cfg);

// Maps the base station's resolution strings ("QXGA", "VGA", ...) to the
// sensor's framesize_t. Unknown strings leave `fallback` in place.
framesize_t storage_parse_frame_size(const char *s, framesize_t fallback);

// Inverse of storage_parse_frame_size — used to report the resolution
// actually in effect back to the base station in the config ACK.
const char *storage_frame_size_to_string(framesize_t fs);

// Builds the cache path for a frame captured now: "<cache>/YYYYMMDD_HHMMSS_<node>.jpg"
// when the clock is set, "<cache>/UNSYNCED_<ms>_<node>.jpg" when it never synced.
// receive_images.py reads the capture time back out of this name.
void storage_build_cache_path(char *out, size_t len, const char *node_id);

// Streams a frame to the cache. Writes in small blocks and verifies the byte
// count: this card truncates large single writes.
bool storage_write_photo(const char *path, const uint8_t *buf, size_t len);

// Reads a cached file into a freshly malloc'd PSRAM buffer. Caller frees.
uint8_t *storage_read_file(const char *path, size_t *out_len);

// One wake's decision, compact enough to sit in RTC memory until a wake that
// has the card mounted anyway can flush a batch of them (plan §15). Logging
// every wake directly would mean mounting the SD on quiet wakes, which is most
// of what chunk B's radio-skip was buying back.
typedef struct {
    uint32_t ts;             // wall clock, 0 if the node has never synced
    uint16_t score;
    uint16_t next_sleep_s;
    uint16_t t_low;
    uint16_t t_high;
    int16_t  charge_rate;    // battery charge rate, mV/min (signed); + charging
    uint8_t  state;          // adaptive_fsm_state_t
    uint8_t  resolution;     // framesize_t actually used
    uint8_t  override_mode;  // adaptive_override_mode_t
    uint8_t  flags;          // DLOG_F_*
    uint8_t  band;           // battery_band_t (energy governor)
    uint8_t  soc;            // battery state-of-charge, percent
} decision_rec_t;

#define DLOG_F_SENT            (1u << 0)
#define DLOG_F_OVERRIDE_ACTIVE (1u << 1)
#define DLOG_F_LIGHTING        (1u << 2)
#define DLOG_F_RADIO_UP        (1u << 3)
#define DLOG_F_TIME_SYNCED     (1u << 4)
#define DLOG_F_PROBE_FAILED    (1u << 5)

// Appends `n` records to SD_DECISION_LOG as CSV, writing the header first if
// the file is new. Never mounts the card — returns false when it isn't mounted,
// so the caller keeps the batch and retries on the next mounted wake rather
// than losing it.
bool storage_flush_decision_log(const decision_rec_t *recs, size_t n);

// Mirrors the adaptive scene state (reference grid + FSM) to SD_ADAPTIVE_FILE
// so it survives a battery pull; the working copy lives in RTC memory. Never
// mounts the card itself — a no-op returning false when it isn't mounted.
bool storage_save_adaptive(const adaptive_state_t *st);

// Restores the adaptive state from SD on a cold boot. Returns false — with
// *st untouched — when the card is unmounted, the file is missing, or the
// blob fails validation (size/magic/version/CRC); the caller then starts
// from adaptive_init_default with a provisional reference.
bool storage_load_adaptive(adaptive_state_t *st);
