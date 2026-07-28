#!/usr/bin/env python3
"""
Pushes a JSON config to one node or all nodes via MQTT.

Usage:
    python3 send_config.py                               # broadcast defaults
    python3 send_config.py <node_id>                    # target a single node
    python3 send_config.py --sleep 60 --resolution HD   # custom fields
    python3 send_config.py <node_id> --sleep 60 --resolution VGA
    python3 send_config.py <node_id> --sleep 60 --no-wait-ack
    python3 send_config.py <node_id> --t-low 10 --t-high 40      # retune thresholds
    python3 send_config.py <node_id> --override stay_hot --lease 120   # hold hot 120s
    python3 send_config.py <node_id> --override stay_hot --forever     # keepalive
    python3 send_config.py <node_id> --override set_cadence --cadence 5 --lease 60
    python3 send_config.py <node_id> --override none            # cancel the lease

Resolution options (matches camera sensor rated fps):
    QXGA  2048x1536  15 fps
    FHD   1920x1080  20 fps
    HD    1280x720   45 fps
    XGA   1024x768   45 fps
    VGA   640x480    60 fps   (default)
    QVGA  320x240   120 fps

Config fields the ESP32 reads:
    sleep_seconds  -- deep-sleep duration between captures
    resolution     -- one of the strings above
    ts             -- unix timestamp of this push, added automatically below

Every push is stamped with the wall-clock time it was sent (ts). A node only
accepts a pushed config if its ts is newer than the one it already has
applied — this is what stops a stale *retained* MQTT message (e.g. an old
broadcast or an old per-node override still sitting on the broker from a
previous test) from replaying on a later wake and silently overriding a
command you sent more recently. Without this, whichever of the broadcast and
per-node retained messages happens to be delivered second on a given wake
wins, which looks like the setting "snapping back" right after you change it.

After publishing, this script listens for the node's confirmation that the
config was actually received and applied:

    Topic:  camera/config_ack/<node_id>
    Payload: {"sleep_seconds": N, "resolution": "HD", "ts": T}

A successful publish only proves the JSON reached the broker — it says
nothing about whether any node was listening. If a node is asleep when the
config is published, MQTT's retain flag holds it at the broker and the node
will pick it up (and ACK it) on its next wake, which can be well past the
wait window below; a missing ACK within the window doesn't mean the push
failed, only that no node has confirmed it yet.
"""

import argparse
import json
import time
import paho.mqtt.client as mqtt

BROKER_HOST = "43.154.113.203"
BROKER_PORT = 1883

RESOLUTIONS = ["QXGA", "FHD", "HD", "XGA", "VGA", "QVGA"]

parser = argparse.ArgumentParser(
    description="Push config to HaLow camera node(s).",
    formatter_class=argparse.RawDescriptionHelpFormatter,
    epilog=(
        "Resolution options:\n"
        "  QXGA  2048x1536  15 fps\n"
        "  FHD   1920x1080  20 fps\n"
        "  HD    1280x720   45 fps\n"
        "  XGA   1024x768   45 fps\n"
        "  VGA   640x480    60 fps\n"
        "  QVGA  320x240   120 fps\n"
    ),
)
parser.add_argument("node_id", nargs="?", default=None,
                    help="Target node ID (hex MAC, e.g. A1B2C3D4E5F6). Omit to broadcast.")
parser.add_argument("--sleep", type=int, default=None,
                    help="Sleep interval in seconds")
parser.add_argument("--resolution", choices=RESOLUTIONS, metavar="RES",
                    help=f"Camera resolution: {', '.join(RESOLUTIONS)}")
parser.add_argument("--forever", action="store_true",
                    help="Keep republishing every --interval seconds until Ctrl+C")
parser.add_argument("--interval", type=int, default=5,
                    help="Republish interval in seconds when --forever is set (default 5)")
parser.add_argument("--ack-timeout", type=int, default=20,
                    help="Seconds to wait for a config ACK before giving up (default 20)")
parser.add_argument("--no-wait-ack", action="store_true",
                    help="Publish and exit immediately without waiting for an ACK")

# --- Scene FSM tunables (plan §9). All optional: a field the push omits keeps
# its current value on the node. These share the config schema with sleep/res.
parser.add_argument("--t-low", type=int, help="Arousal threshold (cells changed)")
parser.add_argument("--t-high", type=int, help="Immediate-active threshold (cells changed)")
parser.add_argument("--pixel-delta", type=int, help="Per-cell changed threshold (of 255)")
parser.add_argument("--n-confirm", type=int, help="Consecutive arousals before ACTIVE")
parser.add_argument("--m-cooldown", type=int, help="Alert wakes held after activity")
parser.add_argument("--sleep-active", type=int, help="Hot-cadence sleep, seconds")
parser.add_argument("--sleep-aroused", type=int, help="Suspect-cadence sleep, seconds")
parser.add_argument("--sleep-baseline", type=int,
                    help="Idle-cadence sleep, seconds (alias of --sleep)")
parser.add_argument("--resolution-high", choices=RESOLUTIONS, metavar="RES",
                    help="Resolution for send_highres detail frames")

# --- Override lease (plan §8). Thor biases the node's behaviour for a bounded
# window; the node enforces valid_until against its own clock and reverts on its
# own when the lease lapses. 'none' cancels an active lease explicitly (an
# *omitted* override, by contrast, leaves any live lease untouched).
parser.add_argument("--override", metavar="MODE",
                    choices=["stay_hot", "relax", "set_cadence", "send_highres", "none"],
                    help="Override mode: stay_hot | relax | set_cadence | send_highres | "
                         "none (cancel the live lease)")
parser.add_argument("--lease", type=int, default=120,
                    help="Override lease length in seconds; valid_until = now + lease "
                         "(default 120). Refreshed each round under --forever, so that "
                         "acts as a keepalive that keeps extending the lease.")
parser.add_argument("--cadence", type=int, default=None,
                    help="Cadence in seconds for --override set_cadence")
args = parser.parse_args()

# Override-specific argument checks.
if args.override == "set_cadence" and args.cadence is None:
    parser.error("--override set_cadence requires --cadence SECONDS.")
if args.cadence is not None and args.override != "set_cadence":
    parser.error("--cadence only applies to --override set_cadence.")

# Static (non-time) config fields. A field left as None is omitted from the push
# so the node keeps whatever it already has; "sleep_seconds" is the wire name the
# firmware has always read, kept as the primary --sleep spelling.
static_fields = {
    "sleep_seconds":    args.sleep,
    "sleep_baseline_s": args.sleep_baseline,
    "sleep_aroused_s":  args.sleep_aroused,
    "sleep_active_s":   args.sleep_active,
    "t_low":            args.t_low,
    "t_high":           args.t_high,
    "pixel_delta":      args.pixel_delta,
    "n_confirm":        args.n_confirm,
    "m_cooldown":       args.m_cooldown,
    "resolution":       args.resolution,
    "resolution_high":  args.resolution_high,
}
static_config = {k: v for k, v in static_fields.items() if v is not None}

if not static_config and args.override is None:
    parser.error("Specify at least one field to push "
                 "(e.g. --sleep, a threshold like --t-low, or --override).")


def build_payload(now):
    """Assemble the config JSON for this send, stamping a fresh ts — and, for an
    override, a fresh valid_until — so a repeat isn't rejected as stale against
    itself and a --forever keepalive keeps extending the lease."""
    cfg = dict(static_config)
    cfg["ts"] = now
    if args.override == "none":
        cfg["override"] = None          # explicit cancel: node clears the lease
    elif args.override is not None:
        ov = {"mode": args.override, "valid_until": now + args.lease}
        if args.override == "set_cadence":
            ov["cadence_s"] = args.cadence
        cfg["override"] = ov
    return cfg


payload = json.dumps(build_payload(int(time.time())))
topic = f"node/{args.node_id}/config" if args.node_id else "config"
ack_topic = f"camera/config_ack/{args.node_id}" if args.node_id else "camera/config_ack/#"

acked_nodes = {}   # node_id -> parsed ack payload (or raw bytes if not JSON)


def on_ack(client, userdata, msg):
    node = msg.topic.split("/")[-1]
    if node in acked_nodes:
        return   # only the first ACK per node matters for confirmation
    try:
        info = json.loads(msg.payload)
        acked_nodes[node] = info
        print(f"  ACK from {node}: sleep={info.get('sleep_seconds')}s "
              f"resolution={info.get('resolution')} (ts={info.get('ts')})", flush=True)
    except (ValueError, json.JSONDecodeError):
        acked_nodes[node] = msg.payload
        print(f"  ACK from {node}: {msg.payload!r}", flush=True)


client = mqtt.Client()
client.on_message = on_ack
client.connect(BROKER_HOST, BROKER_PORT)
client.subscribe(ack_topic)
client.loop_start()

if args.forever:
    print(f"Publishing to [{topic}] every {args.interval}s — Ctrl+C to stop")
    print(f"Payload: {payload}")
    try:
        while True:
            # Rebuild each round so ts is strictly newer than the last (the node
            # requires that) and, for an override, valid_until advances too —
            # that's the lease keepalive that holds the node hot across a long
            # event without ever pinning it forever.
            payload = json.dumps(build_payload(int(time.time())))
            result = client.publish(topic, payload, retain=True)
            print(f"  sent (rc={result.rc})", flush=True)
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\nStopped.")
else:
    result = client.publish(topic, payload, retain=True)
    print(f"Sent to [{topic}]: {payload}  (rc={result.rc})")

    if not args.no_wait_ack:
        print(f"Waiting up to {args.ack_timeout}s for confirmation...")
        deadline = time.time() + args.ack_timeout
        while time.time() < deadline:
            if args.node_id and args.node_id in acked_nodes:
                break
            time.sleep(0.2)

        if args.node_id:
            if args.node_id not in acked_nodes:
                print(f"No ACK from {args.node_id} within {args.ack_timeout}s — "
                      "it may be asleep or offline; it will apply the config "
                      "(and ACK it) on its next wake.")
        elif not acked_nodes:
            print(f"No ACKs within {args.ack_timeout}s — nodes may be asleep "
                  "or offline; they will apply the config (and ACK it) on "
                  "their next wake.")
        else:
            print(f"{len(acked_nodes)} node(s) confirmed: {', '.join(sorted(acked_nodes))}")

client.loop_stop()
client.disconnect()
