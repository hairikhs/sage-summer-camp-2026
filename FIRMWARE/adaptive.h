// Adaptive-sampling core: frame-diff, scene FSM, and state (de)serialization.
//
// This module is deliberately free of any ESP-IDF, radio, or camera
// dependency — it operates on plain grayscale buffers and POD state, so it
// compiles and unit-tests on the host (see test/test_adaptive). Chunk B wires
// it into main.c: an RTC_DATA_ATTR adaptive_state_t, an SD mirror of the
// serialized blob, and the cheap grayscale capture that feeds the diff.
//
// See adaptive_sampling_plan.md sections 1, 4, 5, 6, 14, 19.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "battery.h"   // battery_state_t / battery_band_t for the energy governor

// --- Tunables (plan §19). The operator-tunable ones are pushed by the base
// station and live in adaptive_params_t below; these are their defaults. The
// rest stay compile-time: alpha and the backoff ratio shape the reference and
// the cadence curve, GRID sizes the reference grid itself (changing it would
// invalidate every stored reference), and the lighting percentages are a
// property of the diff metric rather than of a deployment site.
#define ADAPT_GRID          32                 // downscale grid (GRID×GRID cells)
#define ADAPT_CELLS         (ADAPT_GRID * ADAPT_GRID)
#define ADAPT_PIXEL_DELTA   25                 // per-cell "changed" threshold (of 255)
#define ADAPT_T_LOW         6                  // arousal threshold (cells changed)
#define ADAPT_T_HIGH        25                 // immediate-active threshold
#define ADAPT_N_CONFIRM     2                  // consecutive arousals → ACTIVE
#define ADAPT_M_COOLDOWN    5                  // alert wakes held after activity
#define ADAPT_SUSTAIN_MAX   40                 // active wakes before reduced cadence

// Hybrid re-baseline (plan §5, option a — the alternative to the default
// sustained-watch). Once a scene has been ACTIVE past the sustained-activity
// ceiling, a subject that has gone *static* — frame-to-frame score barely
// moving — is a parked object, not a live animal. Rather than watch it forever
// (option b), fold it into the reference and stand back down to BASELINE, so the
// node stays responsive to the *next* change. A still-moving subject keeps the
// score changing wake-to-wake, so the stable run never accumulates and option
// (b)'s stretched watch continues. Both are field-tunable.
#define ADAPT_REBASELINE_STABLE_DELTA 15       // max |score - last_score| to count a wake "static"
#define ADAPT_REBASELINE_STABLE_N     3        // consecutive static wakes before re-baselining

#define ADAPT_SLEEP_BASELINE_S  60             // idle interval (= current fixed interval)
#define ADAPT_SLEEP_AROUSED_S   30             // suspect / cooldown medium interval
#define ADAPT_SLEEP_ACTIVE_S    8              // hot interval

// Energy-governor interval caps per SoC band (plan §10): the minimum sleep the
// node is allowed to take in each band, applied as a floor on the scene's chosen
// interval so the governor can only ever *lengthen* it. HEALTHY has no cap (the
// scene's own cadence stands). These are operator-pushable (plan §9) so Thor can
// field-tune how hard each battery band throttles.
#define ADAPT_CAP_BALANCED_S    30             // 30-70% SoC
#define ADAPT_CAP_LOW_S         120            // 10-30% SoC
#define ADAPT_CAP_CRITICAL_S    600            // <10% SoC

// Backoff multiplier 1.8× as an integer ratio, so the sleep math stays float-free.
#define ADAPT_BACKOFF_NUM   9
#define ADAPT_BACKOFF_DEN   5
// Reference blend weight α = 0.9 as an integer ratio: ref = (9·ref + 1·cur)/10.
#define ADAPT_ALPHA_NUM     9
#define ADAPT_ALPHA_DEN     10

// Lighting-rejection gate (plan §4.6): a change counts as illumination — not an
// object — when this fraction of cells changed *and* the raw deltas mostly share
// sign (a uniform shift, not a localized cluster).
#define ADAPT_LIGHTING_CELL_PCT  75            // % of cells changed to suspect lighting
#define ADAPT_LIGHTING_SIGN_PCT  80            // % sign agreement to confirm it

// Any interval the node can be told to sleep for. The ceiling is what stops a
// fat-fingered push (sleep_baseline_s: 3000000) from sleeping the node for a
// month — there would be no radio wake left in which to push a correction, so
// recovering it means walking out to the site.
#define ADAPT_SLEEP_MAX_S   3600

// The operator-tunable subset, pushed by the base station (plan §9): the ESP
// applies these in real time, but Thor retunes them remotely — nudge t_low down
// if animals are missed, up if wind keeps tripping the camera.
typedef struct {
    uint16_t t_low;              // arousal threshold (cells changed)
    uint16_t t_high;             // immediate-active threshold
    uint16_t pixel_delta;        // per-cell "changed" threshold (of 255)
    uint16_t n_confirm;          // consecutive arousals → ACTIVE
    uint16_t m_cooldown;         // alert wakes held after activity
    uint32_t sleep_baseline_s;   // idle interval
    uint32_t sleep_aroused_s;    // suspect / cooldown medium interval
    uint32_t sleep_active_s;     // hot interval
    uint32_t cap_balanced_s;     // energy-governor interval floor, 30-70% band
    uint32_t cap_low_s;          // energy-governor interval floor, 10-30% band
    uint32_t cap_critical_s;     // energy-governor interval floor, <10% band
} adaptive_params_t;

#define ADAPT_PARAMS_DEFAULTS {                    \
    .t_low            = ADAPT_T_LOW,               \
    .t_high           = ADAPT_T_HIGH,              \
    .pixel_delta      = ADAPT_PIXEL_DELTA,         \
    .n_confirm        = ADAPT_N_CONFIRM,           \
    .m_cooldown       = ADAPT_M_COOLDOWN,          \
    .sleep_baseline_s = ADAPT_SLEEP_BASELINE_S,    \
    .sleep_aroused_s  = ADAPT_SLEEP_AROUSED_S,     \
    .sleep_active_s   = ADAPT_SLEEP_ACTIVE_S,      \
    .cap_balanced_s   = ADAPT_CAP_BALANCED_S,      \
    .cap_low_s        = ADAPT_CAP_LOW_S,           \
    .cap_critical_s   = ADAPT_CAP_CRITICAL_S,      \
}

void adaptive_params_defaults(adaptive_params_t *p);

// Clamps `p` in place to a survivable range and returns false if it had to
// change anything. This is the guard between a typo in a pushed config and a
// node that has to be recovered on foot: t_low=0 arouses on every frame and
// pins the node hot until the battery is flat, and an absurd sleep_baseline_s
// takes the node off the air for as long as it says. A push that needs
// clamping is still applied — clamped and logged — because silently dropping
// it just moves the confusion somewhere harder to see.
bool adaptive_params_validate(adaptive_params_t *p);

// --- The override lease (plan §8). The base station biases the node's behavior
// but never seizes it: every override carries a deadline the node enforces
// against its own clock, so if the base station goes silent — animal gone, Thor
// crashed, link down — the lease lapses and the node reverts to its own FSM.
// Holding a node hot through a long event means *re-publishing* with a fresh
// deadline. Every failure path decays toward idle, not toward hot-forever.
typedef enum {
    ADAPT_OVR_NONE = 0,
    ADAPT_OVR_STAY_HOT,       // hold the hot cadence and keep sending
    ADAPT_OVR_RELAX,          // it's a branch, not an animal — back off
    ADAPT_OVR_SET_CADENCE,    // sample at an explicit interval
    ADAPT_OVR_SEND_HIGHRES,   // one detail frame, then done (one-shot)
} adaptive_override_mode_t;

typedef struct {
    uint8_t  mode;         // adaptive_override_mode_t
    bool     fired;        // one-shot latch, SEND_HIGHRES only
    uint32_t cadence_s;    // SET_CADENCE
    uint32_t valid_until;  // absolute unix seconds — the lease deadline
} adaptive_override_t;

#define ADAPT_OVERRIDE_TTL_S       120   // lease length when Thor omits valid_until
// A lease can't outlive this no matter what the base station asks for. Without
// it, one bad valid_until (a Thor bug, a units mix-up) pins the node hot for
// years — the exact failure the lease exists to prevent.
#define ADAPT_OVERRIDE_MAX_LEASE_S 600

const char *adaptive_override_mode_name(uint8_t mode);
bool        adaptive_override_mode_parse(const char *s, uint8_t *out);

typedef enum {
    ADAPT_BASELINE = 0,   // idle; long sleep, reference blends
    ADAPT_AROUSED,        // suspect; one T_LOW crossing, awaiting confirmation
    ADAPT_ACTIVE,         // hot; confirmed change, send + freeze reference
    ADAPT_COOLDOWN,       // alert; activity stopped, stay watchful M wakes
} adaptive_fsm_state_t;

// Persisted *scene* state — what the camera saw. Kept a POD so it drops straight
// into RTC_DATA_ATTR. Config lives in node_config_t (storage.h), mirrored to
// config.json and ts-guarded there; keeping it out of here is what stops the
// same value having two homes that can disagree. Chunk E's sensed battery state
// does belong here, and the versioned serial format makes that additive without
// invalidating older blobs' magic check.
typedef struct {
    uint8_t  fsm_state;                 // adaptive_fsm_state_t
    uint32_t sleep_s;                   // current sleep interval (governs backoff)
    uint16_t consec_quiet;              // consecutive quiet wakes
    uint16_t consec_arousal;           // consecutive arousals (drives N_CONFIRM)
    uint16_t cooldown_left;            // alert wakes remaining in COOLDOWN
    uint16_t sustain_count;           // consecutive active wakes (drives SUSTAIN_MAX)
    uint16_t last_score;              // previous wake's score (frame-to-frame stability)
    uint16_t stable_count;            // consecutive static wakes past the ceiling (re-baseline)
    uint8_t  ref_grid[ADAPT_CELLS];   // the "nothing happening" reference (1 KB)
    battery_state_t batt;             // energy signal (plan §10–11); serial v3+
} adaptive_state_t;

// What one FSM tick decides. The caller applies it: send the frame if `send`,
// call adaptive_update_ref with quiet=!freeze_ref, then deep-sleep next_sleep_s.
typedef struct {
    adaptive_fsm_state_t next_state;
    bool     send;          // upload a frame this wake
    bool     freeze_ref;    // true → do NOT blend the reference (something's happening)
    uint32_t next_sleep_s;  // governed sleep interval
    bool     send_highres;  // capture at the detail resolution, not the normal one
    bool     override_active;  // an override shaped this decision (log/heartbeat §15)
    uint8_t  override_mode;    // which one; ADAPT_OVR_NONE if none applied
    bool     rebaseline;       // adopt cur as the new reference this wake (plan §5 option a)
} adaptive_decision_t;

// Block-average an arbitrary w×h grayscale frame down to the GRID×GRID grid.
void adaptive_downscale(const uint8_t *gray, int w, int h, uint8_t out[ADAPT_CELLS]);

// Score `cur` against `ref`: mean-normalize both (cancels uniform illumination),
// count cells whose normalized |Δ| exceeds p->pixel_delta. Separately flags a raw,
// near-uniform whole-frame shift via *is_lighting (may be NULL). The FSM treats a
// lighting-flagged frame as quiet regardless of the returned score.
int adaptive_score(const uint8_t cur[ADAPT_CELLS], const uint8_t ref[ADAPT_CELLS],
                   const adaptive_params_t *p, bool *is_lighting);

// Blend the reference toward `cur` when quiet (tracks slow legitimate change);
// freeze it (no-op) when not, so a moving subject is never absorbed (plan §5).
void adaptive_update_ref(uint8_t ref[ADAPT_CELLS], const uint8_t cur[ADAPT_CELLS], bool quiet);

// Pure scene-FSM transition (plan §3, §6). Mutates counters/state in `st` and
// returns the decision. A lighting-flagged frame is treated as quiet. This is
// the "what is happening?" half of plan §1: it proposes from the scene alone and
// knows nothing of overrides — adaptive_apply_override adjusts what it returns.
adaptive_decision_t adaptive_fsm_step(adaptive_state_t *st, const adaptive_params_t *p,
                                      int score, bool is_lighting);

// Adjust an already-computed decision by the base station's override, if one is
// live (plan §1: the scene proposes, the modifiers adjust). Separate from
// adaptive_fsm_step so the FSM's counters tick exactly once per wake, from the
// scene alone, while this can be re-run when a fresher lease lands mid-wake —
// it mutates `d`, never `st`, so applying it twice is the same as once.
//
// `now` is unix seconds, or 0 when the clock has never been synced. 0 drops
// every override: an unsynced node reads tv_sec≈5, so `now < valid_until` would
// be true forever and a retained stay_hot would pin it hot until the battery
// died — precisely the failure the lease is meant to prevent, arriving through
// the lease itself. No clock means no lease, and the node falls back to its own
// FSM, which is always the safe default.
//
// Never touches freeze_ref: reference-blend policy stays scene-driven. A lease
// that froze the reference over a quiet scene would let the sun move the frame
// out from under it, and the moment the lease lapsed the node would see a huge
// diff and storm into ACTIVE. An override buys cadence and transmits, not
// reference semantics.
void adaptive_apply_override(adaptive_decision_t *d, const adaptive_params_t *p,
                             const adaptive_override_t *ov, uint32_t now);

// The energy governor (plan §1 step 2, §10–11): cap an already-shaped decision
// by the battery's SoC band, then optionally promote *within* that allowance on
// a sustained rising charge. It is applied last — after the scene FSM and the
// override — because energy is the hard physical limit: you cannot power a lease
// you can't afford, so even a stay_hot override is capped here.
//
// It only ever makes the node LESS active than its input: it lengthens the
// interval to the band's floor (never shortens below the scene) and restricts
// transmits (LOW sends only strong positives, score ≥ t_high; CRITICAL sends
// nothing) — it never sets `send` where the scene cleared it. The one exception
// is the bounded charge-rate promotion (plan §11): a sustained rising trend eases
// the interval floor toward the next-healthier band's, but never re-enables a
// transmit the band suppressed (the hard SoC ceiling) and never promotes out of
// CRITICAL ("never splurge at <10%"). Never touches freeze_ref. Idempotent, so
// it composes cleanly when re-run against a fresh lease on a later pass.
void adaptive_govern(adaptive_decision_t *d, const battery_state_t *b,
                     const adaptive_params_t *p, int score);

// Cold-boot init: BASELINE, baseline sleep, reference seeded from a provisional
// grid (pass NULL to zero the reference).
void adaptive_init_default(adaptive_state_t *st, const adaptive_params_t *p,
                           const uint8_t *provisional_grid);

// "BASELINE" | "AROUSED" | "ACTIVE" | "COOLDOWN" | "UNKNOWN". Shared by the
// decision log, the heartbeat, and the EXIF note so they can't drift apart.
const char *adaptive_fsm_state_name(uint8_t state);

// Serialized blob layout: magic | version | scene payload | [battery payload] | crc32.
//
// Version 3 appends the battery payload (plan §14). The format is additive: a v2
// blob written by a pre-chunk-E build still deserializes — its battery state is
// defaulted — so a node updated in the field reads its old SD mirror once, then
// rewrites v3. The version byte selects the expected length and CRC coverage.
#define ADAPT_SERIAL_VERSION  3
// Battery payload: last_mv(2) + last_ts(4) + charge_rate(2) + band(1) + trend(1)
// + rise_run(1).
#define ADAPT_BATT_PAYLOAD    (2 + 4 + 2 + 1 + 1 + 1)
// The pre-battery layout, still accepted on read.
#define ADAPT_SERIAL_SIZE_V2  (4 + 1 + 1 + 4 + 2 + 2 + 2 + 2 + 2 + 2 + ADAPT_CELLS + 4)
// Current (v3) layout: the v2 body plus the battery payload, before the crc.
#define ADAPT_SERIAL_SIZE     (ADAPT_SERIAL_SIZE_V2 + ADAPT_BATT_PAYLOAD)

// Serialize `st` into `buf`. Returns bytes written (ADAPT_SERIAL_SIZE), or 0 if
// `cap` is too small.
size_t adaptive_serialize(const adaptive_state_t *st, uint8_t *buf, size_t cap);

// Deserialize into `st`. Returns false on wrong length, bad magic, unknown
// version, or CRC mismatch — the caller then falls back to adaptive_init_default
// (plan §5: power loss → provisional reference + BASELINE).
bool adaptive_deserialize(adaptive_state_t *st, const uint8_t *buf, size_t len);
