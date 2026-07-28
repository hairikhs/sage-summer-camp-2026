#include "mqtt_node.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "mqtt_client.h"

static const char *TAG = "mqtt_node";

#define BIT_CONNECTED  BIT0
#define BIT_ACK        BIT1

static esp_mqtt_client_handle_t s_client = NULL;
static EventGroupHandle_t       s_events = NULL;
static mqtt_config_cb_t         s_on_config = NULL;
static bool                     s_ack_complete = false;
static char                     s_node_id[13];

// Topics we listen on, built once at start.
static char s_topic_ack[64];
static char s_topic_node_config[64];

bool mqtt_node_connected(void)
{
    return s_client && (xEventGroupGetBits(s_events) & BIT_CONNECTED);
}

static void handle_data(esp_mqtt_event_handle_t e)
{
    // Incoming messages here are small (an ACK byte or a config document), so
    // they fit one event; the config path checks rather than assumes.
    if (e->topic_len == (int)strlen(s_topic_ack) &&
        strncmp(e->topic, s_topic_ack, e->topic_len) == 0) {
        s_ack_complete = (e->data_len > 0 && e->data[0] == '1');
        if (!s_ack_complete) {
            ESP_LOGW(TAG, "  base station ACK: image accepted as incomplete");
        }
        xEventGroupSetBits(s_events, BIT_ACK);
        return;
    }

    bool is_broadcast = (e->topic_len == 6 && strncmp(e->topic, "config", 6) == 0);
    bool is_targeted  = (e->topic_len == (int)strlen(s_topic_node_config) &&
                         strncmp(e->topic, s_topic_node_config, e->topic_len) == 0);

    if ((is_broadcast || is_targeted) && s_on_config) {
        // A config document still fits one event, but it's no longer 60 bytes
        // and the schema keeps growing. Handing a fragment to the parser would
        // surface as an unexplained "config parse failed" rather than as this.
        if (e->total_data_len != e->data_len) {
            ESP_LOGW(TAG, "Config arrived fragmented (%d of %d bytes) — ignoring",
                     e->data_len, e->total_data_len);
            return;
        }
        ESP_LOGI(TAG, "Config pushed from base station (%d bytes)", e->data_len);
        s_on_config(e->data, (size_t)e->data_len);
    }
}

static void mqtt_event_handler(void *args, esp_event_base_t base, int32_t id, void *data)
{
    esp_mqtt_event_handle_t e = (esp_mqtt_event_handle_t)data;

    switch ((esp_mqtt_event_id_t)id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT connected");
        // Re-subscribe on every connect: a reconnect starts a fresh session.
        esp_mqtt_client_subscribe(s_client, "config", 0);
        esp_mqtt_client_subscribe(s_client, s_topic_node_config, 0);
        esp_mqtt_client_subscribe(s_client, s_topic_ack, 0);
        xEventGroupSetBits(s_events, BIT_CONNECTED);
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "MQTT disconnected");
        xEventGroupClearBits(s_events, BIT_CONNECTED);
        break;

    case MQTT_EVENT_DATA:
        handle_data(e);
        break;

    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "MQTT error");
        break;

    default:
        break;
    }
}

bool mqtt_node_start(const char *node_id, mqtt_config_cb_t on_config, uint32_t timeout_ms)
{
    snprintf(s_node_id, sizeof(s_node_id), "%s", node_id);
    snprintf(s_topic_ack, sizeof(s_topic_ack), "camera/ack/%s", node_id);
    snprintf(s_topic_node_config, sizeof(s_topic_node_config), "node/%s/config", node_id);
    s_on_config = on_config;

    char client_id[32];
    snprintf(client_id, sizeof(client_id), "HC33-%s", node_id);

    s_events = xEventGroupCreate();

    esp_mqtt_client_config_t cfg = {
        .broker.address.uri = CONFIG_MQTT_BROKER_URI,
        .credentials.client_id = client_id,
        .session.keepalive = 60,
        .network.timeout_ms = 30000,
        // The HaLow link is slow; don't let the client give up and tear down
        // the socket mid-upload.
        .network.reconnect_timeout_ms = 5000,
        .buffer.size = 4096,
        .buffer.out_size = 16384,
    };

    s_client = esp_mqtt_client_init(&cfg);
    if (!s_client) {
        ESP_LOGE(TAG, "esp_mqtt_client_init failed");
        return false;
    }
    esp_mqtt_client_register_event(s_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);

    ESP_LOGI(TAG, "Connecting to MQTT as %s (%s)", client_id, CONFIG_MQTT_BROKER_URI);
    if (esp_mqtt_client_start(s_client) != ESP_OK) {
        ESP_LOGE(TAG, "esp_mqtt_client_start failed");
        return false;
    }

    EventBits_t bits = xEventGroupWaitBits(s_events, BIT_CONNECTED, pdFALSE, pdTRUE,
                                           pdMS_TO_TICKS(timeout_ms));
    if (!(bits & BIT_CONNECTED)) {
        ESP_LOGE(TAG, "MQTT connect timed out");
        return false;
    }

    // Give the broker a moment to process the subscriptions, otherwise an ACK
    // published immediately after our first image can be dropped before the
    // subscription is live.
    vTaskDelay(pdMS_TO_TICKS(200));
    return true;
}

void mqtt_node_stop(void)
{
    if (!s_client) {
        return;
    }
    esp_mqtt_client_disconnect(s_client);
    esp_mqtt_client_stop(s_client);
    esp_mqtt_client_destroy(s_client);
    s_client = NULL;
}

void mqtt_node_reset_ack(void)
{
    s_ack_complete = false;
    if (s_events) {
        xEventGroupClearBits(s_events, BIT_ACK);
    }
}

bool mqtt_node_wait_ack(uint32_t timeout_ms, bool *complete)
{
    EventBits_t bits = xEventGroupWaitBits(s_events, BIT_ACK, pdFALSE, pdTRUE,
                                           pdMS_TO_TICKS(timeout_ms));
    if (!(bits & BIT_ACK)) {
        return false;
    }
    if (complete) {
        *complete = s_ack_complete;
    }
    return true;
}

bool mqtt_node_publish_meta(const char *capture_time, bool time_synced)
{
    char topic[64];
    char payload[128];
    snprintf(topic, sizeof(topic), "camera/meta/%s", s_node_id);
    snprintf(payload, sizeof(payload),
             "{\"node\":\"%s\",\"time\":\"%s\",\"time_synced\":%s}",
             s_node_id, capture_time, time_synced ? "true" : "false");

    if (esp_mqtt_client_publish(s_client, topic, payload, 0, 0, 0) < 0) {
        ESP_LOGE(TAG, "  image meta publish failed");
        return false;
    }
    return true;
}

bool mqtt_node_publish_state(const mqtt_node_state_t *s)
{
    if (!mqtt_node_connected()) {
        return false;
    }
    char topic[64];
    char payload[384];
    snprintf(topic, sizeof(topic), "camera/state/%s", s_node_id);
    snprintf(payload, sizeof(payload),
             "{\"node\":\"%s\",\"ts\":%u,\"time_synced\":%s,\"state\":\"%s\","
             "\"score\":%d,\"t_low\":%u,\"t_high\":%u,\"sent\":%s,"
             "\"resolution\":\"%s\",\"next_sleep_s\":%u,\"override_mode\":\"%s\","
             "\"override_active\":%s,\"override_valid_until\":%u,\"config_ts\":%u,"
             "\"band\":\"%s\",\"soc\":%u,\"charge_rate\":%d}",
             s_node_id, (unsigned)s->ts, s->time_synced ? "true" : "false",
             s->state, s->score, (unsigned)s->t_low, (unsigned)s->t_high,
             s->sent ? "true" : "false", s->resolution, (unsigned)s->next_sleep_s,
             s->override_mode, s->override_active ? "true" : "false",
             (unsigned)s->override_valid_until, (unsigned)s->config_ts,
             s->band, (unsigned)s->soc, (int)s->charge_rate);

    if (esp_mqtt_client_publish(s_client, topic, payload, 0, 0, 0) < 0) {
        ESP_LOGE(TAG, "  state heartbeat publish failed");
        return false;
    }
    ESP_LOGI(TAG, "State published: %s", payload);
    return true;
}

bool mqtt_node_publish_config_ack(uint32_t sleep_seconds, const char *resolution, uint32_t ts)
{
    if (!mqtt_node_connected()) {
        return false;
    }
    char topic[64];
    char payload[128];
    snprintf(topic, sizeof(topic), "camera/config_ack/%s", s_node_id);
    snprintf(payload, sizeof(payload),
             "{\"sleep_seconds\":%u,\"resolution\":\"%s\",\"ts\":%u}",
             (unsigned)sleep_seconds, resolution, (unsigned)ts);

    if (esp_mqtt_client_publish(s_client, topic, payload, 0, 0, 0) < 0) {
        ESP_LOGE(TAG, "  config ack publish failed");
        return false;
    }
    ESP_LOGI(TAG, "Config ACK sent: %s", payload);
    return true;
}

static bool publish_single(const uint8_t *buf, size_t len)
{
    char topic[64];
    snprintf(topic, sizeof(topic), "camera/image/%s", s_node_id);
    return esp_mqtt_client_publish(s_client, topic, (const char *)buf, (int)len, 0, 0) >= 0;
}

// Large frames: announce the chunk count, then send each chunk on its own
// topic. Mirrors the reassembly in receive_images.py:
//   camera/chunk/<node>/meta  -> {"chunks":N,"total":B}
//   camera/chunk/<node>/<i>   -> raw bytes of chunk i
static bool publish_chunked(const uint8_t *buf, size_t len)
{
    uint32_t n_chunks = (len + UPLOAD_CHUNK_BYTES - 1) / UPLOAD_CHUNK_BYTES;

    char topic[64];
    char meta[64];
    snprintf(topic, sizeof(topic), "camera/chunk/%s/meta", s_node_id);
    snprintf(meta, sizeof(meta), "{\"chunks\":%u,\"total\":%u}",
             (unsigned)n_chunks, (unsigned)len);
    if (esp_mqtt_client_publish(s_client, topic, meta, 0, 0, 0) < 0) {
        ESP_LOGE(TAG, "  chunk meta publish failed");
        return false;
    }

    for (uint32_t i = 0; i < n_chunks; i++) {
        size_t off = (size_t)i * UPLOAD_CHUNK_BYTES;
        size_t this_len = len - off;
        if (this_len > UPLOAD_CHUNK_BYTES) {
            this_len = UPLOAD_CHUNK_BYTES;
        }

        snprintf(topic, sizeof(topic), "camera/chunk/%s/%u", s_node_id, (unsigned)i);
        if (esp_mqtt_client_publish(s_client, topic, (const char *)(buf + off),
                                    (int)this_len, 0, 0) < 0) {
            ESP_LOGE(TAG, "  chunk %u publish failed", (unsigned)i);
            return false;
        }
        ESP_LOGI(TAG, "  chunk %u/%u sent (%u bytes)",
                 (unsigned)(i + 1), (unsigned)n_chunks, (unsigned)this_len);
        vTaskDelay(pdMS_TO_TICKS(5));   // let the HaLow link breathe
    }
    return true;
}

bool mqtt_node_publish_image(const uint8_t *buf, size_t len)
{
    if (!mqtt_node_connected()) {
        return false;
    }
    return (len <= UPLOAD_SINGLE_MAX) ? publish_single(buf, len)
                                      : publish_chunked(buf, len);
}

void mqtt_node_wait_for_config(uint32_t ms)
{
    // esp-mqtt runs its own task, so there is no loop() to pump — just hold the
    // connection open long enough for a retained/queued config to arrive.
    vTaskDelay(pdMS_TO_TICKS(ms));
}
