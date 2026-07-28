#include "camera.h"

#include <stdio.h>
#include <string.h>

#include "esp_camera.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "camera";

// DVP pinout for the Heltec HT-HC33, mirroring
// variants/HT-HC33/pins_arduino.h. The OV3660 has no dedicated reset line on
// this board (RESET is -1); it is held in/out of reset via PWDN alone.
#define PWDN_GPIO_NUM   20
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM   47
#define SIOD_GPIO_NUM   45
#define SIOC_GPIO_NUM   42
#define Y9_GPIO_NUM     38
#define Y8_GPIO_NUM     48
#define Y7_GPIO_NUM     46
#define Y6_GPIO_NUM     18
#define Y5_GPIO_NUM     14
#define Y4_GPIO_NUM     12
#define Y3_GPIO_NUM     13
#define Y2_GPIO_NUM     17
#define VSYNC_GPIO_NUM  40
#define HREF_GPIO_NUM   39
#define PCLK_GPIO_NUM   21

#define STREAM_CONTENT_TYPE "multipart/x-mixed-replace;boundary=" STREAM_BOUNDARY
#define STREAM_BOUNDARY     "halowframe"
#define STREAM_PART_HDR     "\r\n--" STREAM_BOUNDARY \
                            "\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n"

static esp_err_t init_with(pixformat_t fmt, framesize_t frame_size,
                           int fb_count, camera_grab_mode_t grab)
{
    camera_config_t cfg = {
        .ledc_channel = LEDC_CHANNEL_0,
        .ledc_timer   = LEDC_TIMER_0,

        .pin_d0 = Y2_GPIO_NUM, .pin_d1 = Y3_GPIO_NUM,
        .pin_d2 = Y4_GPIO_NUM, .pin_d3 = Y5_GPIO_NUM,
        .pin_d4 = Y6_GPIO_NUM, .pin_d5 = Y7_GPIO_NUM,
        .pin_d6 = Y8_GPIO_NUM, .pin_d7 = Y9_GPIO_NUM,

        .pin_xclk  = XCLK_GPIO_NUM,  .pin_pclk  = PCLK_GPIO_NUM,
        .pin_vsync = VSYNC_GPIO_NUM, .pin_href  = HREF_GPIO_NUM,
        .pin_sccb_sda = SIOD_GPIO_NUM, .pin_sccb_scl = SIOC_GPIO_NUM,
        .pin_pwdn  = PWDN_GPIO_NUM,  .pin_reset = RESET_GPIO_NUM,

        // 20 MHz overruns the DVP->DMA->PSRAM path on large frames and hands
        // back JPEGs truncated at a DMA-buffer boundary. 10 MHz is slower but
        // gives complete frames.
        .xclk_freq_hz = 10000000,
        .pixel_format = fmt,
        .frame_size   = frame_size,
        .jpeg_quality = 12,
        .fb_count     = fb_count,
        .fb_location  = CAMERA_FB_IN_PSRAM,
        .grab_mode    = grab,
    };

    esp_err_t err = esp_camera_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_camera_init failed: 0x%x", err);
        return err;
    }

    sensor_t *s = esp_camera_sensor_get();
    ESP_LOGI(TAG, "Sensor PID 0x%04x", s->id.PID);
    // Same orientation in both formats so the adaptive reference grid built
    // from grayscale probes matches what the JPEGs show.
    s->set_vflip(s, 1);

    return ESP_OK;
}

esp_err_t camera_init(framesize_t frame_size)
{
    // Double-buffer: DMA headroom for big frames.
    return init_with(PIXFORMAT_JPEG, frame_size, 2, CAMERA_GRAB_LATEST);
}

esp_err_t camera_init_grayscale(void)
{
    // Single buffer is plenty for a 76 KB Y8 frame; WHEN_EMPTY avoids the
    // continuous re-grab of LATEST since the probe takes exactly one frame.
    return init_with(PIXFORMAT_GRAYSCALE, FRAMESIZE_QVGA, 1, CAMERA_GRAB_WHEN_EMPTY);
}

esp_err_t camera_deinit(void)
{
    esp_err_t err = esp_camera_deinit();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_camera_deinit failed: 0x%x", err);
    }
    return err;
}

camera_fb_t *camera_capture(void)
{
    for (int i = 0; i < 2; i++) {
        camera_fb_t *warm = esp_camera_fb_get();
        if (warm) {
            esp_camera_fb_return(warm);
        }
    }
    return esp_camera_fb_get();
}

bool camera_frame_complete(const camera_fb_t *fb)
{
    return fb && fb->len >= 2 &&
           fb->buf[fb->len - 2] == 0xFF && fb->buf[fb->len - 1] == 0xD9;
}

static esp_err_t capture_get_handler(httpd_req_t *req)
{
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "capture failed");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    esp_err_t res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    return res;
}

static esp_err_t stream_get_handler(httpd_req_t *req)
{
    esp_err_t res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
    if (res != ESP_OK) {
        return res;
    }
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    int64_t window_start = esp_timer_get_time();
    uint32_t window_frames = 0;
    uint32_t window_bytes = 0;
    char part_hdr[64];

    while (true) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "stream: capture failed");
            res = ESP_FAIL;
            break;
        }

        size_t hdr_len = snprintf(part_hdr, sizeof(part_hdr), STREAM_PART_HDR,
                                  (unsigned)fb->len);

        res = httpd_resp_send_chunk(req, part_hdr, hdr_len);
        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
        }
        window_frames++;
        window_bytes += fb->len;
        esp_camera_fb_return(fb);

        if (res != ESP_OK) {
            // Client went away (browser closed the tab, HaLow link dropped).
            ESP_LOGI(TAG, "stream: client disconnected");
            break;
        }

        // Per-frame logging floods the console at streaming rates; report an
        // average roughly every 5 s instead.
        int64_t elapsed_us = esp_timer_get_time() - window_start;
        if (elapsed_us >= 5000000) {
            ESP_LOGI(TAG, "MJPEG: %.1f fps, %.0f KB/s (avg frame %uKB)",
                     window_frames * 1000000.0 / (double)elapsed_us,
                     window_bytes * (1000000.0 / 1024.0) / (double)elapsed_us,
                     (unsigned)(window_bytes / window_frames / 1024));
            window_start = esp_timer_get_time();
            window_frames = 0;
            window_bytes = 0;
        }
    }

    return res;
}

static esp_err_t index_get_handler(httpd_req_t *req)
{
    static const char page[] =
        "<!doctype html><html><head><meta charset=utf-8>"
        "<title>HaLow Camera</title>"
        "<style>body{margin:0;background:#111;color:#eee;font-family:sans-serif;"
        "text-align:center}img{max-width:100%;height:auto}</style></head>"
        "<body><h2>HaLow Camera</h2>"
        "<img src=\"/stream\">"
        "<p><a style=\"color:#6cf\" href=\"/capture\">single frame</a></p>"
        "</body></html>";

    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, page, HTTPD_RESP_USE_STRLEN);
}

esp_err_t camera_register_handlers(httpd_handle_t server)
{
    httpd_uri_t index_uri   = { .uri = "/",        .method = HTTP_GET, .handler = index_get_handler };
    httpd_uri_t capture_uri = { .uri = "/capture", .method = HTTP_GET, .handler = capture_get_handler };
    httpd_uri_t stream_uri  = { .uri = "/stream",  .method = HTTP_GET, .handler = stream_get_handler };

    esp_err_t res = httpd_register_uri_handler(server, &index_uri);
    if (res == ESP_OK) res = httpd_register_uri_handler(server, &capture_uri);
    if (res == ESP_OK) res = httpd_register_uri_handler(server, &stream_uri);
    return res;
}
