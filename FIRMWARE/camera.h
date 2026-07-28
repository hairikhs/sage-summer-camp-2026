#pragma once

#include "esp_camera.h"
#include "esp_err.h"
#include "esp_http_server.h"

// Brings up the OV3660 on the HT-HC33's DVP bus at the given resolution.
esp_err_t camera_init(framesize_t frame_size);

// Brings up the sensor as a QVGA (320x240) native-Y8 grayscale probe for the
// adaptive frame diff: fb->buf is width*height bytes of luma, no JPEG encoded.
// The driver can't switch pixel format after init, so pair with camera_deinit()
// before re-initialising for a JPEG capture in the same wake.
esp_err_t camera_init_grayscale(void);

// Tears the driver down so the same wake can re-init in a different pixformat.
esp_err_t camera_deinit(void);

// Grabs a frame, discarding the sensor's first few (exposure/gain are still
// settling and the DMA pipeline hasn't primed, so they come back partial).
// Return the buffer with esp_camera_fb_return().
camera_fb_t *camera_capture(void);

// True when the JPEG ends in the EOI marker (FF D9), i.e. the sensor handed us
// a whole frame. Tells a camera-side truncation apart from one introduced later
// by the SD write or the upload.
bool camera_frame_complete(const camera_fb_t *fb);

// Registers /capture (still) and /stream (multipart MJPEG) on `server`. Only
// used when CONFIG_NODE_STREAM_SERVER is on.
esp_err_t camera_register_handlers(httpd_handle_t server);
