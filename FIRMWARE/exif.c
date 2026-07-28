#include "exif.h"

#include <string.h>
#include <stdio.h>

#include "esp_heap_caps.h"

// --- little-endian helpers for the EXIF/TIFF structure (EXIF itself is
// big-endian in the JPEG marker length, but the TIFF body is written "II" =
// little-endian, matching the reference node firmware) ---
static inline void put16le(uint8_t *p, uint16_t v) { p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF; }
static inline void put32le(uint8_t *p, uint32_t v)
{
    p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF; p[2] = (v >> 16) & 0xFF; p[3] = (v >> 24) & 0xFF;
}

// "2026-07-14T15:53:15Z" -> "2026:07:14 15:53:15" (EXIF DateTime format).
// Buffer must be at least 20 bytes (19 chars + NUL).
static void iso8601_to_exif_datetime(const char *iso, char *out)
{
    // Defensive: only touch bytes that exist in a well-formed ISO string.
    snprintf(out, 20, "%.4s:%.2s:%.2s %.2s:%.2s:%.2s",
             iso, iso + 5, iso + 8, iso + 11, iso + 14, iso + 17);
}

// Builds the APP1 (EXIF) marker segment. Returns bytes written, 0 on failure.
static size_t build_exif_app1(uint8_t *out, size_t cap, const char *node_id,
                              const char *note, const char *capture_time_iso,
                              bool time_synced)
{
    char node_str[64];
    size_t node_len = snprintf(node_str, sizeof(node_str), "%s", node_id) + 1;

    char note_str[64];
    size_t note_len = 0;
    if (note) {
        note_len = strlcpy(note_str, note, sizeof(note_str)) + 1;
        if (note_len > sizeof(note_str)) {
            note_len = sizeof(note_str);   // truncated
        }
    }

    char dt_str[20];
    const uint32_t dt_len = 20;
    if (time_synced) {
        iso8601_to_exif_datetime(capture_time_iso, dt_str);
    }

    uint32_t entry_count = 1 + (note ? 1 : 0) + (time_synced ? 1 : 0);

    uint8_t tiff[256];
    size_t p = 0;
    tiff[p++] = 'I'; tiff[p++] = 'I';
    put16le(tiff + p, 0x002A); p += 2;   // TIFF magic
    put32le(tiff + p, 8);      p += 4;   // offset to IFD0

    // IFD0 @ offset 8: entry count (2B), entries (12B each), next-IFD (4B),
    // then the data area for any string too long to inline.
    const uint32_t data_off = 10 + entry_count * 12 + 4;
    const uint32_t node_off = data_off;
    const uint32_t note_off = data_off + (uint32_t)node_len;
    const uint32_t dt_off   = note_off + (uint32_t)note_len;

    put16le(tiff + p, (uint16_t)entry_count); p += 2;

    // Entry: ImageDescription (0x010E), ASCII — tags must be ascending.
    put16le(tiff + p, 0x010E); p += 2;
    put16le(tiff + p, 2);      p += 2;              // type = ASCII
    put32le(tiff + p, (uint32_t)node_len); p += 4;
    if (node_len <= 4) {
        memset(tiff + p, 0, 4);
        memcpy(tiff + p, node_str, node_len);
    } else {
        put32le(tiff + p, node_off);
    }
    p += 4;

    if (note) {
        // Entry: Software (0x0131), ASCII — carries the scene note; slots
        // between ImageDescription and DateTime so tags stay ascending.
        put16le(tiff + p, 0x0131); p += 2;
        put16le(tiff + p, 2);      p += 2;
        put32le(tiff + p, (uint32_t)note_len); p += 4;
        if (note_len <= 4) {
            memset(tiff + p, 0, 4);
            memcpy(tiff + p, note_str, note_len);
        } else {
            put32le(tiff + p, note_off);
        }
        p += 4;
    }

    if (time_synced) {
        // Entry: DateTime (0x0132), ASCII
        put16le(tiff + p, 0x0132); p += 2;
        put16le(tiff + p, 2);      p += 2;
        put32le(tiff + p, dt_len); p += 4;
        put32le(tiff + p, dt_off); p += 4;
    }

    put32le(tiff + p, 0); p += 4;   // no next IFD

    // Data area (p == data_off here). Strings inlined above still reserve
    // their slot here so the precomputed offsets stay valid.
    if (node_len > 4) {
        memcpy(tiff + p, node_str, node_len);
    }
    p += node_len;
    if (note) {
        if (note_len > 4) {
            memcpy(tiff + p, note_str, note_len);
        }
        p += note_len;
    }
    if (time_synced) {
        memcpy(tiff + p, dt_str, 19);
        tiff[p + 19] = 0;
        p += dt_len;
    }
    size_t tiff_len = p;

    size_t seg_len = 2 /*length field*/ + 6 /*"Exif\0\0"*/ + tiff_len;
    size_t need = 2 + seg_len;   // + FFE1 marker
    if (need > cap) {
        return 0;
    }

    size_t o = 0;
    out[o++] = 0xFF; out[o++] = 0xE1;
    out[o++] = (seg_len >> 8) & 0xFF;
    out[o++] = seg_len & 0xFF;
    memcpy(out + o, "Exif\0\0", 6); o += 6;
    memcpy(out + o, tiff, tiff_len); o += tiff_len;
    return o;
}

uint8_t *exif_embed_jpeg(const uint8_t *jpeg, size_t jpeg_len,
                         const char *node_id, const char *note,
                         const char *capture_time_iso,
                         bool time_synced, size_t *out_len)
{
    if (jpeg_len < 2 || jpeg[0] != 0xFF || jpeg[1] != 0xD8) {
        return NULL;   // not a valid SOI — nothing to insert into
    }

    uint8_t exif[288];
    size_t exif_len = build_exif_app1(exif, sizeof(exif), node_id, note,
                                      capture_time_iso, time_synced);
    if (exif_len == 0) {
        return NULL;
    }

    size_t total_len = 2 + exif_len + (jpeg_len - 2);
    uint8_t *out = heap_caps_malloc(total_len, MALLOC_CAP_SPIRAM);
    if (!out) {
        out = heap_caps_malloc(total_len, MALLOC_CAP_8BIT);
    }
    if (!out) {
        return NULL;
    }

    memcpy(out, jpeg, 2);                            // SOI
    memcpy(out + 2, exif, exif_len);                 // EXIF APP1
    memcpy(out + 2 + exif_len, jpeg + 2, jpeg_len - 2);  // rest of JPEG

    *out_len = total_len;
    return out;
}
