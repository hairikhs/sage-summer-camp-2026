#!/usr/bin/env python3
"""
Subscribes to camera images from HaLow nodes via MQTT and saves them to images/.

Each image is preceded by a metadata sidecar message carrying the node ID and
the time the photo was taken:

    Topic:  camera/meta/<node_id>
    Payload: {"node": "<id>", "time": "<ISO8601 UTC>", "time_synced": true|false}

  time_synced=false means the node's clock was never NTP-synced, so "time" is
  "unknown" and the capture time is genuinely unavailable. The metadata is stored
  per node and applied to the next image saved for that node (filename, log line,
  and EXIF tags embedded directly in the saved .jpg — see save_jpeg()/build_exif_bytes()).

Two upload protocols are handled transparently:

  Single-message (all resolutions except QXGA):
    Topic:  camera/image/<node_id>
    Payload: raw JPEG bytes

  Chunked (QXGA — file > 60 KB):
    Topic:  camera/chunk/<node_id>/meta   payload: {"chunks": N, "total": B}
    Topic:  camera/chunk/<node_id>/0      payload: raw bytes (chunk 0)
    Topic:  camera/chunk/<node_id>/1      payload: raw bytes (chunk 1)
    ...
    Reassembled automatically; saved as a single JPEG once all chunks arrive.

After a JPEG is saved (either protocol, complete or not), an acknowledgement
is published on:

    Topic:  camera/ack/<node_id>   payload: "1" (complete) or "0" (incomplete)

The node waits for this before deleting its SD-cached copy. Incomplete
images are ACKed too ("0") rather than left unacknowledged: the source file
is already truncated on the node's SD card, so retrying would just reproduce
the same incomplete bytes — the ACK just lets the node's log reflect that it
was incomplete. If no ACK arrives at all (base station offline, or the
message never reached the broker) the node keeps the image cached and
retries on a later wake instead of losing it.

Usage:
    pip install paho-mqtt piexif
    python3 receive_images.py
"""

import os
import io
import re
import json
import datetime
import paho.mqtt.client as mqtt
import piexif
from piexif import helper as exif_helper

BROKER_HOST = "43.154.113.203"
BROKER_PORT = 1883
OUTPUT_DIR  = "images"

os.makedirs(OUTPUT_DIR, exist_ok=True)

# Tracks in-progress chunked transfers keyed by node_id.
# { node_id: {"total": N, "chunks": [bytes|None, ...]} }
pending = {}

# Latest image metadata received per node (from camera/meta/<node_id>), applied
# to the next image saved for that node.
# { node_id: {"node": str, "time": str, "time_synced": bool} }
node_meta = {}


def compact_capture_time(iso):
    """Turns '2026-07-02T13:45:09Z' into '20260702_134509' for use as a filename
    prefix. Returns None if the string has fewer than 14 digits (e.g. 'unknown')."""
    digits = re.sub(r"\D", "", iso or "")
    if len(digits) < 14:
        return None
    return f"{digits[:8]}_{digits[8:14]}"


def humanize_iso(iso):
    """Displays '2026-07-02T13:45:09Z' as '2026-07-02 13:45:09 UTC' for logs
    and stored metadata. Returns the input unchanged if it isn't in that exact
    shape (e.g. the literal 'unknown' sent when a node's clock was never
    synced)."""
    if iso and re.match(r"^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}Z$", iso):
        return f"{iso[:10]} {iso[11:19]} UTC"
    return iso


def is_complete_jpeg(data, expected_bytes=None):
    """A whole JPEG starts with SOI (FF D8) and ends with EOI (FF D9). For
    chunked transfers we also know the exact byte count from the meta message,
    so verify that too — a dropped chunk over QoS-0 MQTT would otherwise leave a
    short-but-EOI-less blob that looks 'large image, small file'."""
    if len(data) < 4:
        return False
    if data[:2] != b"\xff\xd8" or data[-2:] != b"\xff\xd9":
        return False
    if expected_bytes is not None and len(data) != expected_bytes:
        return False
    return True


def build_exif_bytes(node_id, capture_time, time_synced, received_at_str, complete, num_bytes):
    """Packs everything that used to live in the .json sidecar into an EXIF
    blob, so the saved .jpg is the single, self-describing record. Node ID and
    capture time go into the standard EXIF tags (matching the tags the camera
    firmware itself would set); the fields with no standard EXIF tag — sync
    state, receive time, byte count, completeness — go into UserComment as
    JSON."""
    zeroth = {piexif.ImageIFD.ImageDescription: node_id.encode()}
    exif_ifd = {}
    if time_synced and capture_time:
        # "2026-07-02T13:45:09Z" -> "2026:07:02 13:45:09" (EXIF DateTime format)
        exif_dt = capture_time.replace("-", ":").rstrip("Z").replace("T", " ")
        zeroth[piexif.ImageIFD.DateTime] = exif_dt.encode()
        exif_ifd[piexif.ExifIFD.DateTimeOriginal] = exif_dt.encode()

    comment = json.dumps({
        "capture_time": humanize_iso(capture_time) if time_synced else None,
        "time_synced": time_synced,
        "received_at": received_at_str,
        "bytes": num_bytes,
        "complete": complete,
    })
    exif_ifd[piexif.ExifIFD.UserComment] = exif_helper.UserComment.dump(comment, encoding="unicode")

    return piexif.dump({"0th": zeroth, "Exif": exif_ifd})


def save_jpeg(client, node_id, data, note="", expected_bytes=None):
    complete = is_complete_jpeg(data, expected_bytes)
    received_at = datetime.datetime.utcnow()
    received_at_str = received_at.strftime("%Y-%m-%d %H:%M:%S UTC")

    # Apply the node's latest metadata sidecar, if one arrived before the image.
    meta = node_meta.pop(node_id, None)
    capture_time = meta.get("time") if meta else None
    time_synced = bool(meta.get("time_synced")) if meta else False

    # Name the file by the capture time when the node's clock was set; otherwise
    # fall back to our receive time and flag that the capture time is unknown.
    prefix = compact_capture_time(capture_time) if time_synced else None
    if prefix is None:
        prefix = received_at.strftime("%Y%m%d_%H%M%S")
    unsynced = "_CLOCKUNSET" if (meta and not time_synced) else ""
    suffix = "" if complete else "_PARTIAL"
    base = f"{prefix}_{node_id}{unsynced}{suffix}"
    path = os.path.join(OUTPUT_DIR, f"{base}.jpg")

    # Embed the metadata directly into the JPEG bytes rather than writing a
    # separate .json — a truncated/incomplete capture may not parse as a valid
    # JPEG for EXIF purposes, so fall back to the raw bytes rather than losing
    # the image over a metadata-embedding failure.
    exif_bytes = build_exif_bytes(node_id, capture_time, time_synced, received_at_str, complete, len(data))
    try:
        out_buf = io.BytesIO()
        piexif.insert(exif_bytes, data, out_buf)
        data_to_write = out_buf.getvalue()
    except Exception as e:
        print(f"[{node_id}] EXIF embed failed ({e}) — saving raw bytes")
        data_to_write = data

    with open(path, "wb") as f:
        f.write(data_to_write)

    size_note = f"{len(data)/1024:.1f} KB"
    if expected_bytes is not None and len(data) != expected_bytes:
        size_note += f", expected {expected_bytes/1024:.1f} KB"

    if meta and time_synced:
        when = f"taken {humanize_iso(capture_time)}"
    elif meta:
        when = "capture time UNKNOWN (node clock not set)"
    else:
        when = "no capture metadata received"
    note = f" [node {node_id}, {when}]{note}"

    if complete:
        print(f"Saved {path} ({size_note}){note}")
    else:
        print(f"INCOMPLETE image {path} ({size_note}){note} — "
              f"saving as-is")

    # Acknowledge receipt so the node can drop its SD-cached copy. Without this
    # the node can't tell a delivered image from one published into the void
    # (QoS 0), and would keep re-uploading it forever. We ACK incomplete images
    # too: the source file is already truncated on the node's SD card, so
    # re-sending it again would just reproduce the same incomplete bytes —
    # retrying buys nothing. The payload flags which case it was ("1" complete,
    # "0" incomplete) purely so the node's own log reflects it.
    client.publish(f"camera/ack/{node_id}", "1" if complete else "0")
    print(f"[{node_id}] ACK sent" + ("" if complete else " (incomplete — accepted, no retry)"))


def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print(f"Connected to broker. Listening for images...")
        client.subscribe("camera/meta/#")    # per-image node ID + capture time
        client.subscribe("camera/image/#")   # single-message uploads
        client.subscribe("camera/chunk/#")   # chunked QXGA uploads
        client.subscribe("camera/state/#")   # per-wake FSM/override heartbeat
    else:
        print(f"Connection failed, rc={rc}")


def on_message(client, userdata, msg):
    parts = msg.topic.split("/")

    # --- image metadata sidecar (node ID + capture time) ---
    if parts[1] == "meta":
        node_id = parts[2] if len(parts) >= 3 else "unknown"
        try:
            info = json.loads(msg.payload)
        except (ValueError, json.JSONDecodeError):
            print(f"[{node_id}] bad meta payload — ignoring")
            return
        node_meta[node_id] = info
        if info.get("time_synced"):
            print(f"[{node_id}] meta: taken {humanize_iso(info.get('time'))}")
        else:
            print(f"[{node_id}] meta: capture time unknown (node clock not set)")
        return

    # --- FSM / override heartbeat (published once per radio-up wake) ---
    if parts[1] == "state":
        node_id = parts[2] if len(parts) >= 3 else "unknown"
        try:
            hb = json.loads(msg.payload)
        except (ValueError, json.JSONDecodeError):
            print(f"[{node_id}] bad state payload — ignoring")
            return
        if hb.get("override_active"):
            ovr = f"{hb.get('override_mode')}(until {hb.get('override_valid_until')})"
        else:
            ovr = "none"
        print(f"[{node_id}] state={hb.get('state')} score={hb.get('score')} "
              f"sent={hb.get('sent')} res={hb.get('resolution')} "
              f"next_sleep={hb.get('next_sleep_s')}s override={ovr} "
              f"ts={hb.get('ts')} tsync={hb.get('time_synced')}", flush=True)
        return

    # --- single-message upload ---
    if parts[1] == "image":
        node_id = parts[2] if len(parts) >= 3 else "unknown"
        save_jpeg(client, node_id, msg.payload)
        return

    # --- chunked upload ---
    if parts[1] == "chunk" and len(parts) >= 4:
        node_id = parts[2]
        kind    = parts[3]

        if kind == "meta":
            info = json.loads(msg.payload)
            n = info["chunks"]
            pending[node_id] = {"total": n, "bytes": info.get("total"),
                                "chunks": [None] * n}
            print(f"[{node_id}] chunked transfer: {n} chunks, {info['total']} bytes total")

        elif kind.isdigit():
            idx = int(kind)
            if node_id not in pending:
                print(f"[{node_id}] chunk {idx} arrived before meta — ignoring")
                return
            state = pending[node_id]
            state["chunks"][idx] = bytes(msg.payload)
            received = sum(1 for c in state["chunks"] if c is not None)
            print(f"[{node_id}] chunk {idx + 1}/{state['total']} received", flush=True)
            if received == state["total"]:
                st = pending.pop(node_id)
                data = b"".join(st["chunks"])
                save_jpeg(client, node_id, data, " [chunked/QXGA]",
                          expected_bytes=st.get("bytes"))


client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(BROKER_HOST, BROKER_PORT, keepalive=60)
client.loop_forever()
