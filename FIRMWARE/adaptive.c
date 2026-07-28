#include "adaptive.h"

#include <stdlib.h>   // abs
#include <string.h>   // memcpy, memset

// ============================================================================
// Tunable parameters (plan §9, §19)
// ============================================================================

void adaptive_params_defaults(adaptive_params_t *p)
{
    adaptive_params_t d = ADAPT_PARAMS_DEFAULTS;
    *p = d;
}

static bool clamp_u16(uint16_t *v, uint16_t lo, uint16_t hi)
{
    uint16_t was = *v;
    if (*v < lo) *v = lo;
    if (*v > hi) *v = hi;
    return *v != was;
}

static bool clamp_u32(uint32_t *v, uint32_t lo, uint32_t hi)
{
    uint32_t was = *v;
    if (*v < lo) *v = lo;
    if (*v > hi) *v = hi;
    return *v != was;
}

bool adaptive_params_validate(adaptive_params_t *p)
{
    bool ok = true;

    // t_low = 0 would make every frame aroused: the node pins ACTIVE, wakes at
    // the hot cadence forever, and the battery is gone in a day.
    ok &= !clamp_u16(&p->t_low, 1, ADAPT_CELLS);
    ok &= !clamp_u16(&p->t_high, p->t_low, ADAPT_CELLS);
    ok &= !clamp_u16(&p->pixel_delta, 1, 254);
    ok &= !clamp_u16(&p->n_confirm, 1, 100);
    ok &= !clamp_u16(&p->m_cooldown, 0, 1000);

    ok &= !clamp_u32(&p->sleep_baseline_s, 1, ADAPT_SLEEP_MAX_S);
    ok &= !clamp_u32(&p->sleep_aroused_s, 1, ADAPT_SLEEP_MAX_S);
    ok &= !clamp_u32(&p->sleep_active_s, 1, ADAPT_SLEEP_MAX_S);

    // The three cadences are a ladder: hot ≤ suspect ≤ idle. Out of order, an
    // aroused node would sample *less* often than an idle one — looking away
    // precisely when it has reason to look.
    //
    // Compress the ladder down under sleep_baseline_s rather than raising
    // sleep_baseline_s to fit the others. It's the value the operator actually
    // pushes (send_config.py --sleep speaks it, under its old name), so it's the
    // one to honour: `--sleep 20` against the stock aroused=30 has to mean a 20 s
    // idle cadence, not silently become 30 because a default it didn't mention
    // outranked it.
    if (p->sleep_aroused_s > p->sleep_baseline_s) {
        p->sleep_aroused_s = p->sleep_baseline_s;
        ok = false;
    }
    if (p->sleep_active_s > p->sleep_aroused_s) {
        p->sleep_active_s = p->sleep_aroused_s;
        ok = false;
    }

    // Energy-governor band caps (plan §10). Each is an interval *floor* the
    // governor imposes as the battery drops, so a zero would defeat the throttle.
    ok &= !clamp_u32(&p->cap_balanced_s, 1, ADAPT_SLEEP_MAX_S);
    ok &= !clamp_u32(&p->cap_low_s,      1, ADAPT_SLEEP_MAX_S);
    ok &= !clamp_u32(&p->cap_critical_s, 1, ADAPT_SLEEP_MAX_S);

    // The caps are a ladder in the opposite direction to the cadences: a *lower*
    // battery band must never be allowed to sample more often than a higher one,
    // or draining the battery would speed the node up. Enforce
    // cap_balanced ≤ cap_low ≤ cap_critical, pushing a stray value up under the
    // next rung rather than down, so the more-conserving band always wins.
    if (p->cap_low_s < p->cap_balanced_s) {
        p->cap_low_s = p->cap_balanced_s;
        ok = false;
    }
    if (p->cap_critical_s < p->cap_low_s) {
        p->cap_critical_s = p->cap_low_s;
        ok = false;
    }

    return ok;
}

const char *adaptive_fsm_state_name(uint8_t state)
{
    switch ((adaptive_fsm_state_t)state) {
    case ADAPT_BASELINE: return "BASELINE";
    case ADAPT_AROUSED:  return "AROUSED";
    case ADAPT_ACTIVE:   return "ACTIVE";
    case ADAPT_COOLDOWN: return "COOLDOWN";
    default:             return "UNKNOWN";
    }
}

// ============================================================================
// Frame-difference metric (plan §4)
// ============================================================================

void adaptive_downscale(const uint8_t *gray, int w, int h, uint8_t out[ADAPT_CELLS])
{
    if (!gray || w <= 0 || h <= 0) {
        memset(out, 0, ADAPT_CELLS);
        return;
    }

    for (int gy = 0; gy < ADAPT_GRID; gy++) {
        int y0 = gy * h / ADAPT_GRID;
        int y1 = (gy + 1) * h / ADAPT_GRID;
        if (y1 <= y0) y1 = y0 + 1;
        if (y1 > h)   y1 = h;

        for (int gx = 0; gx < ADAPT_GRID; gx++) {
            int x0 = gx * w / ADAPT_GRID;
            int x1 = (gx + 1) * w / ADAPT_GRID;
            if (x1 <= x0) x1 = x0 + 1;
            if (x1 > w)   x1 = w;

            uint32_t sum = 0;
            int cnt = 0;
            for (int y = y0; y < y1; y++) {
                const uint8_t *row = &gray[(size_t)y * w];
                for (int x = x0; x < x1; x++) {
                    sum += row[x];
                    cnt++;
                }
            }
            out[gy * ADAPT_GRID + gx] = cnt ? (uint8_t)(sum / cnt) : 0;
        }
    }
}

int adaptive_score(const uint8_t cur[ADAPT_CELLS], const uint8_t ref[ADAPT_CELLS],
                   const adaptive_params_t *p, bool *is_lighting)
{
    // Grid means, for illumination normalization (plan §4.3).
    long sum_cur = 0, sum_ref = 0;
    for (int i = 0; i < ADAPT_CELLS; i++) {
        sum_cur += cur[i];
        sum_ref += ref[i];
    }
    int mean_diff = (int)(sum_cur / ADAPT_CELLS) - (int)(sum_ref / ADAPT_CELLS);

    int score = 0;          // normalized changed cells → the FSM's motion score
    int raw_changed = 0;    // raw changed cells → lighting heuristic
    long sign_sum = 0;      // net sign of the raw deltas → uniformity of the shift

    for (int i = 0; i < ADAPT_CELLS; i++) {
        int raw = (int)cur[i] - (int)ref[i];
        if (abs(raw) > (int)p->pixel_delta) {
            raw_changed++;
            sign_sum += (raw > 0) ? 1 : -1;
        }
        // Subtract the global brightness shift so a uniform dawn/dusk change,
        // which moves every cell the same way, cancels here.
        int norm = raw - mean_diff;
        if (abs(norm) > (int)p->pixel_delta) {
            score++;
        }
    }

    if (is_lighting) {
        // Lighting = a large fraction of cells changed in the raw diff AND those
        // changes mostly share sign (a whole-frame shift, not a localized blob).
        bool wide = (long)raw_changed * 100 >= (long)ADAPT_CELLS * ADAPT_LIGHTING_CELL_PCT;
        bool uniform = raw_changed > 0 &&
                       labs(sign_sum) * 100 >= (long)raw_changed * ADAPT_LIGHTING_SIGN_PCT;
        *is_lighting = wide && uniform;
    }
    return score;
}

void adaptive_update_ref(uint8_t ref[ADAPT_CELLS], const uint8_t cur[ADAPT_CELLS], bool quiet)
{
    if (!quiet) {
        return;   // freeze — never absorb a moving subject into the reference
    }
    for (int i = 0; i < ADAPT_CELLS; i++) {
        // ref = (α·ref + (1−α)·cur), rounded. +DEN/2 rounds to nearest.
        uint32_t blended = (uint32_t)ADAPT_ALPHA_NUM * ref[i]
                         + (uint32_t)(ADAPT_ALPHA_DEN - ADAPT_ALPHA_NUM) * cur[i]
                         + ADAPT_ALPHA_DEN / 2;
        ref[i] = (uint8_t)(blended / ADAPT_ALPHA_DEN);
    }
}

// ============================================================================
// Scene FSM (plan §3, §6)
// ============================================================================

// Ease the sleep interval up toward baseline by the backoff factor, always
// growing by at least one second so it can't stall below the cap.
static uint32_t backoff(uint32_t s, const adaptive_params_t *p)
{
    uint64_t n = (uint64_t)s * ADAPT_BACKOFF_NUM / ADAPT_BACKOFF_DEN;
    if (n <= s) {
        n = (uint64_t)s + 1;
    }
    if (n > p->sleep_baseline_s) {
        n = p->sleep_baseline_s;
    }
    return (uint32_t)n;
}

adaptive_decision_t adaptive_fsm_step(adaptive_state_t *st, const adaptive_params_t *p,
                                      int score, bool is_lighting)
{
    // A lighting-flagged frame is quiet no matter its score, so the whole-frame
    // shift updates the reference instead of tripping the FSM.
    bool aroused = !is_lighting && score >= (int)p->t_low;
    bool high    = !is_lighting && score >= (int)p->t_high;
    bool quiet   = !aroused;

    adaptive_fsm_state_t next = (adaptive_fsm_state_t)st->fsm_state;

    switch ((adaptive_fsm_state_t)st->fsm_state) {
    case ADAPT_BASELINE:
        if (high) {
            next = ADAPT_ACTIVE;
        } else if (aroused) {
            next = ADAPT_AROUSED;
            st->consec_arousal = 1;
        } else {
            next = ADAPT_BASELINE;
        }
        break;

    case ADAPT_AROUSED:
        if (high) {
            next = ADAPT_ACTIVE;
        } else if (aroused) {
            st->consec_arousal++;
            next = (st->consec_arousal >= p->n_confirm) ? ADAPT_ACTIVE : ADAPT_AROUSED;
        } else {
            // A single-frame blip (bird, cloud, gust) that didn't confirm —
            // ease back toward baseline.
            next = ADAPT_BASELINE;
            st->consec_arousal = 0;
        }
        break;

    case ADAPT_ACTIVE:
        if (quiet) {
            next = ADAPT_COOLDOWN;
            st->cooldown_left = p->m_cooldown;
            st->sustain_count = 0;
        } else {
            next = ADAPT_ACTIVE;   // still hot
        }
        break;

    case ADAPT_COOLDOWN:
        if (high || aroused) {
            next = ADAPT_ACTIVE;   // re-entry before the alert window elapsed
        } else {
            if (st->cooldown_left > 0) {
                st->cooldown_left--;
            }
            // Hold the alert cadence until the window is exhausted, then begin
            // backing off toward baseline.
            next = (st->cooldown_left == 0) ? ADAPT_BASELINE : ADAPT_COOLDOWN;
        }
        break;

    default:
        next = ADAPT_BASELINE;
        break;
    }

    // Counters that track quietness independent of the transition.
    if (quiet) {
        st->consec_quiet++;
    } else {
        st->consec_quiet = 0;
    }

    // Reference freezes whenever something is happening (plan §5).
    bool freeze_ref = !quiet;

    // Sleep interval and send policy per resulting state.
    uint32_t sleep_s;
    bool send       = false;
    bool rebaseline = false;
    switch (next) {
    case ADAPT_ACTIVE:
        st->sustain_count++;
        // Hybrid re-baseline (plan §5 option a). Past the sustained-activity
        // ceiling, a scene whose score has stopped moving wake-to-wake is a
        // parked object, not a live subject: after ADAPT_REBASELINE_STABLE_N such
        // static wakes, adopt it as the new reference and stand down to BASELINE
        // instead of watching it forever. A still-moving subject keeps the score
        // changing, so the run resets and option (b)'s stretched watch holds.
        if (st->sustain_count >= ADAPT_SUSTAIN_MAX &&
            abs(score - (int)st->last_score) <= ADAPT_REBASELINE_STABLE_DELTA) {
            st->stable_count++;
        } else {
            st->stable_count = 0;
        }
        if (st->stable_count >= ADAPT_REBASELINE_STABLE_N) {
            next             = ADAPT_BASELINE;  // stand down; counters reset below
            rebaseline       = true;            // caller hard-adopts cur as the reference
            freeze_ref       = false;
            st->stable_count = 0;
            sleep_s          = p->sleep_baseline_s;
            send             = false;
            break;
        }
        // A parked subject (grazing livestock) keeps scoring high. After
        // SUSTAIN_MAX active wakes, stretch the interval so we keep watching
        // without hammering — reduced sustained cadence, not a return to baseline.
        sleep_s = (st->sustain_count >= ADAPT_SUSTAIN_MAX)
                      ? backoff(st->sleep_s, p)
                      : p->sleep_active_s;
        send = true;                 // commit: send one "something's here" frame
        break;
    case ADAPT_AROUSED:
        sleep_s = p->sleep_aroused_s;
        break;                       // still confirming — do not send yet
    case ADAPT_COOLDOWN:
        sleep_s = p->sleep_aroused_s;      // medium cadence while alert
        break;
    case ADAPT_BASELINE:
    default:
        sleep_s = backoff(st->sleep_s, p); // ease up toward baseline
        break;
    }

    if (next != ADAPT_ACTIVE) {
        st->sustain_count = 0;
        st->stable_count  = 0;
    }

    st->fsm_state  = (uint8_t)next;
    st->sleep_s    = sleep_s;
    st->last_score = (uint16_t)(score < 0 ? 0 : score);

    adaptive_decision_t d = {
        .next_state   = next,
        .send         = send,
        .freeze_ref   = freeze_ref,
        .next_sleep_s = sleep_s,
        .rebaseline   = rebaseline,
    };
    return d;
}

// ============================================================================
// Override lease (plan §8)
// ============================================================================

const char *adaptive_override_mode_name(uint8_t mode)
{
    switch ((adaptive_override_mode_t)mode) {
    case ADAPT_OVR_NONE:         return "none";
    case ADAPT_OVR_STAY_HOT:     return "stay_hot";
    case ADAPT_OVR_RELAX:        return "relax";
    case ADAPT_OVR_SET_CADENCE:  return "set_cadence";
    case ADAPT_OVR_SEND_HIGHRES: return "send_highres";
    default:                     return "unknown";
    }
}

bool adaptive_override_mode_parse(const char *s, uint8_t *out)
{
    if (!s) {
        return false;
    }
    static const uint8_t modes[] = {
        ADAPT_OVR_NONE, ADAPT_OVR_STAY_HOT, ADAPT_OVR_RELAX,
        ADAPT_OVR_SET_CADENCE, ADAPT_OVR_SEND_HIGHRES,
    };
    for (size_t i = 0; i < sizeof(modes) / sizeof(modes[0]); i++) {
        if (strcmp(s, adaptive_override_mode_name(modes[i])) == 0) {
            *out = modes[i];
            return true;
        }
    }
    return false;
}

void adaptive_apply_override(adaptive_decision_t *d, const adaptive_params_t *p,
                             const adaptive_override_t *ov, uint32_t now)
{
    if (!ov || ov->mode == ADAPT_OVR_NONE) {
        return;
    }
    // No clock, no lease — see the header. Fails toward the autonomous FSM.
    if (now == 0) {
        return;
    }
    // Expiry is checked here, against the node's own clock, rather than on
    // receipt: that's what lets a lease lapse on a wake where the radio never
    // came up, and what makes a stale retained message on the broker harmless.
    if (now >= ov->valid_until) {
        return;
    }
    // Spent one-shot: the lease is used up even though the deadline hasn't
    // passed, so it isn't "active" any more either.
    if (ov->mode == ADAPT_OVR_SEND_HIGHRES && ov->fired) {
        return;
    }

    d->override_active = true;
    d->override_mode   = ov->mode;

    switch ((adaptive_override_mode_t)ov->mode) {
    case ADAPT_OVR_STAY_HOT:
        // Beats the sustained-activity cadence stretch on purpose: the base
        // station has seen the frame and is making an explicit call about what
        // the node is looking at. The deadline bounds what that can cost.
        d->send         = true;
        d->next_sleep_s = p->sleep_active_s;
        break;

    case ADAPT_OVR_RELAX:
        d->send         = false;
        d->send_highres = false;
        // max, not assign: an override may never make the node *more* active
        // than the scene asked for (plan §1). Relaxing a node that is already
        // sleeping longer than baseline must not shorten its sleep.
        if (d->next_sleep_s < p->sleep_baseline_s) {
            d->next_sleep_s = p->sleep_baseline_s;
        }
        break;

    case ADAPT_OVR_SET_CADENCE: {
        uint32_t c = ov->cadence_s;
        if (c < 1)                 c = 1;
        if (c > ADAPT_SLEEP_MAX_S) c = ADAPT_SLEEP_MAX_S;
        d->next_sleep_s = c;
        break;
    }

    case ADAPT_OVR_SEND_HIGHRES:
        // One detail frame. The caller latches `fired` once it has acted, so
        // this doesn't re-fire on every wake until the deadline.
        d->send         = true;
        d->send_highres = true;
        break;

    default:
        break;
    }
}

// ============================================================================
// Energy governor (plan §1 step 2, §10–11)
// ============================================================================

void adaptive_govern(adaptive_decision_t *d, const battery_state_t *b,
                     const adaptive_params_t *p, int score)
{
    // Interval floor per SoC band, indexed by battery_band_t. HEALTHY has no
    // floor — the scene's own cadence stands.
    const uint32_t floor_of[4] = {
        [BATT_CRITICAL] = p->cap_critical_s,
        [BATT_LOW]      = p->cap_low_s,
        [BATT_BALANCED] = p->cap_balanced_s,
        [BATT_HEALTHY]  = 0,
    };

    int band = b->band;
    if (band < BATT_CRITICAL) band = BATT_CRITICAL;
    if (band > BATT_HEALTHY)  band = BATT_HEALTHY;

    // Charge-rate promotion (plan §11): a sustained rising trend eases the floor
    // to the next-healthier band's — a within-allowance promotion. Disabled in
    // CRITICAL (never splurge at <10%) and, being one step, never below HEALTHY.
    uint32_t floor = floor_of[band];
    if (b->trend == BATT_TREND_RISING &&
        band > BATT_CRITICAL && band < BATT_HEALTHY) {
        floor = floor_of[band + 1];
    }

    // Cap: the governor only ever *lengthens* the interval (plan §1).
    if (d->next_sleep_s < floor) {
        d->next_sleep_s = floor;
    }

    // Transmit policy — the hard SoC ceiling. Promotion above never touches this:
    // a rising charge buys cadence, not transmits it can't yet afford.
    switch ((battery_band_t)band) {
    case BATT_CRITICAL:
        // Frame-diff only; the radio stays down and frames wait for recharge.
        d->send         = false;
        d->send_highres = false;
        break;
    case BATT_LOW:
        // ESP is the authority: send only strong positives (a score that reached
        // the immediate-active bar). This overrides even a stay_hot lease —
        // energy survival shifts authority to the node (plan §10).
        if (d->send && score < (int)p->t_high) {
            d->send         = false;
            d->send_highres = false;
        }
        break;
    case BATT_BALANCED:
    case BATT_HEALTHY:
    default:
        break;   // send as the scene / override decided
    }
}

void adaptive_init_default(adaptive_state_t *st, const adaptive_params_t *p,
                           const uint8_t *provisional_grid)
{
    memset(st, 0, sizeof(*st));
    st->fsm_state = ADAPT_BASELINE;
    st->sleep_s   = p->sleep_baseline_s;
    battery_init(&st->batt);
    if (provisional_grid) {
        memcpy(st->ref_grid, provisional_grid, ADAPT_CELLS);
    }
}

// ============================================================================
// Persistence (plan §14) — versioned, CRC-guarded byte blob for the SD mirror
// ============================================================================

static const uint8_t ADAPT_MAGIC[4] = { 'A', 'D', 'P', '1' };

// Standard CRC-32 (IEEE 802.3, reflected, poly 0xEDB88820). Self-contained so
// the module stays host-portable — no esp_crc dependency.
static uint32_t crc32(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            uint32_t mask = -(crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88820u & mask);
        }
    }
    return ~crc;
}

static void put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
}

static void put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)((v >> 24) & 0xFF);
}

static uint16_t get_u16(const uint8_t *p)
{
    return (uint16_t)(p[0] | (p[1] << 8));
}

static uint32_t get_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

size_t adaptive_serialize(const adaptive_state_t *st, uint8_t *buf, size_t cap)
{
    if (cap < ADAPT_SERIAL_SIZE) {
        return 0;
    }
    uint8_t *p = buf;
    memcpy(p, ADAPT_MAGIC, 4);           p += 4;
    *p++ = ADAPT_SERIAL_VERSION;
    *p++ = st->fsm_state;
    put_u32(p, st->sleep_s);             p += 4;
    put_u16(p, st->consec_quiet);        p += 2;
    put_u16(p, st->consec_arousal);      p += 2;
    put_u16(p, st->cooldown_left);       p += 2;
    put_u16(p, st->sustain_count);       p += 2;
    put_u16(p, st->last_score);          p += 2;
    put_u16(p, st->stable_count);        p += 2;
    memcpy(p, st->ref_grid, ADAPT_CELLS); p += ADAPT_CELLS;

    // Battery payload (v3). charge_rate is signed; two's-complement round-trips
    // cleanly through the u16 helpers.
    put_u16(p, st->batt.last_mv);              p += 2;
    put_u32(p, st->batt.last_ts);              p += 4;
    put_u16(p, (uint16_t)st->batt.charge_rate); p += 2;
    *p++ = st->batt.band;
    *p++ = st->batt.trend;
    *p++ = st->batt.rise_run;

    uint32_t crc = crc32(buf, (size_t)(p - buf));
    put_u32(p, crc);                     p += 4;

    return (size_t)(p - buf);
}

bool adaptive_deserialize(adaptive_state_t *st, const uint8_t *buf, size_t len)
{
    if (len < 5 || memcmp(buf, ADAPT_MAGIC, 4) != 0) {
        return false;
    }

    // The version byte fixes the expected length and CRC coverage. v2 (pre-chunk-E)
    // blobs are still accepted so a field-updated node reads its old SD mirror;
    // their battery state is defaulted below. Any other version is rejected — the
    // caller falls back to adaptive_init_default.
    uint8_t  version = buf[4];
    size_t   expect;
    bool     has_batt;
    switch (version) {
    case 2: expect = ADAPT_SERIAL_SIZE_V2; has_batt = false; break;
    case 3: expect = ADAPT_SERIAL_SIZE;    has_batt = true;  break;
    default: return false;
    }
    if (len != expect) {
        return false;
    }
    uint32_t stored = get_u32(buf + expect - 4);
    if (stored != crc32(buf, expect - 4)) {
        return false;
    }

    const uint8_t *p = buf + 5;   // past magic + version
    st->fsm_state      = *p++;
    st->sleep_s        = get_u32(p); p += 4;
    st->consec_quiet   = get_u16(p); p += 2;
    st->consec_arousal = get_u16(p); p += 2;
    st->cooldown_left  = get_u16(p); p += 2;
    st->sustain_count  = get_u16(p); p += 2;
    st->last_score     = get_u16(p); p += 2;
    st->stable_count   = get_u16(p); p += 2;
    memcpy(st->ref_grid, p, ADAPT_CELLS); p += ADAPT_CELLS;

    if (has_batt) {
        st->batt.last_mv     = get_u16(p);          p += 2;
        st->batt.last_ts     = get_u32(p);          p += 4;
        st->batt.charge_rate = (int16_t)get_u16(p); p += 2;
        st->batt.band        = *p++;
        st->batt.trend       = *p++;
        st->batt.rise_run    = *p++;
    } else {
        battery_init(&st->batt);   // older blob predates the battery signal
    }
    return true;
}
