#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Inserts a JPEG APP1 (EXIF) segment right after the SOI marker, carrying the
// node ID (ImageDescription, tag 0x010E), an optional scene note (Software,
// tag 0x0131) and, when the clock has been synced, the capture time
// (DateTime, tag 0x0132). This bakes the metadata into the JPEG bytes
// themselves so an image is self-describing wherever it ends up — on the SD
// cache, or after a network upload — instead of depending on the base station
// to attach it out of band.
//
// `note` (NULL to omit) records why the frame was taken, e.g.
// "adaptive score=12 state=ACTIVE"; truncated past 63 chars.
// `capture_time_iso` is "YYYY-MM-DDTHH:MM:SSZ"; ignored when time_synced is
// false. Returns a freshly heap_caps_malloc'd (PSRAM) buffer with the new
// length in *out_len, or NULL if `jpeg` doesn't start with a valid SOI marker
// or allocation fails — callers should fall back to the original buffer.
// Caller owns the returned buffer and must free() it.
uint8_t *exif_embed_jpeg(const uint8_t *jpeg, size_t jpeg_len,
                         const char *node_id, const char *note,
                         const char *capture_time_iso,
                         bool time_synced, size_t *out_len);
