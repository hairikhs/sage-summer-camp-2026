#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Frames larger than this go out as numbered chunks instead of one message.
// receive_images.py reassembles them; the threshold and chunk size must match
// what it expects.
#define UPLOAD_SINGLE_MAX  (60u * 1024u)
#define UPLOAD_CHUNK_BYTES 8192u

// Invoked from the MQTT task when the base station pushes a config document on
// "config" (broadcast) or "node/<id>/config" (targeted).
typedef void (*mqtt_config_cb_t)(const char *json, size_t len);

// Connects to the broker and subscribes to the config and ACK topics. Blocks up
// to `timeout_ms` for the connection to come up.
bool mqtt_node_start(const char *node_id, mqtt_config_cb_t on_config, uint32_t timeout_ms);
void mqtt_node_stop(void);
bool mqtt_node_connected(void);

// Clear the ACK latch before starting an upload — the ACK can race in while the
// last chunks are still going out.
void mqtt_node_reset_ack(void);

// Waits for the base station's camera/ack/<node> message. Returns false only on
// a real timeout or a dropped connection, i.e. nobody answered. On true,
// *complete reports whether the ACK said the JPEG arrived whole ("1") or
// truncated ("0") — both mean the bytes are on disk at the base station.
bool mqtt_node_wait_ack(uint32_t timeout_ms, bool *complete);

// Sidecar announcing who took the next image and when:
//   camera/meta/<node>  {"node":..,"time":..,"time_synced":..}
bool mqtt_node_publish_meta(const char *capture_time, bool time_synced);

// Confirms a config push was received and applied, mirroring the image ACK
// protocol so send_config.py can tell the operator it actually landed instead
// of publishing into the void:
//   camera/config_ack/<node>  {"sleep_seconds":N,"resolution":"HD","ts":T}
// `ts` echoes back the timestamp of the config now in effect, so the operator
// can confirm their own push (and not some other, older one) is what won.
bool mqtt_node_publish_config_ack(uint32_t sleep_seconds, const char *resolution, uint32_t ts);

// Sends a JPEG, picking the single-message or chunked protocol by size.
bool mqtt_node_publish_image(const uint8_t *buf, size_t len);

// A wake's decision, rolled up for the operator and the base station's detector
// (plan §15). Same fields as the SD decision log — this is the copy you can see
// without walking out to the node and pulling the card.
typedef struct {
    uint32_t    ts;                    // 0 when the node has never synced
    const char *state;                 // "BASELINE" | "AROUSED" | ...
    int         score;
    uint16_t    t_low;
    uint16_t    t_high;
    bool        sent;
    const char *resolution;
    uint32_t    next_sleep_s;
    const char *override_mode;
    bool        override_active;
    uint32_t    override_valid_until;
    uint32_t    config_ts;
    bool        time_synced;
    const char *band;                  // battery SoC band (energy governor)
    uint8_t     soc;                   // battery state-of-charge, percent
    int16_t     charge_rate;           // battery charge rate, mV/min (signed)
} mqtt_node_state_t;

// Publishes to camera/state/<node>. Radio-up wakes only — a quiet wake has no
// radio to say it with. Not retained: a heartbeat outlives its truth within one
// sleep interval, and the durable copy is the CSV on the card.
bool mqtt_node_publish_state(const mqtt_node_state_t *s);

// Pumps the config-flush window: lets a pushed config land before we sleep.
void mqtt_node_wait_for_config(uint32_t ms);
