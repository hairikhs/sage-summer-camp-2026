#include "battery.h"

#include <stddef.h>   // NULL
#include <string.h>   // memset

// ============================================================================
// State-of-charge from voltage (plan §11 — disciplined voltage sampling)
// ============================================================================

// Single-cell LiPo resting-voltage → SoC breakpoints. Monotonic in both columns;
// the band boundaries (10/30/70%) land on sensible voltages. Interpolated
// linearly between points, clamped past the ends (3.40 V empty, 4.20 V full —
// the HT-HC33's stated 3.4–4.2 V battery range).
static const uint16_t SOC_MV[]  = { 3400, 3600, 3700, 3750, 3800, 3900, 3950, 4000, 4100, 4200 };
static const uint8_t  SOC_PCT[] = {    0,   10,   20,   30,   40,   60,   70,   80,   90,  100 };
#define SOC_POINTS ((int)(sizeof(SOC_MV) / sizeof(SOC_MV[0])))

uint8_t battery_soc_from_mv(uint16_t mv)
{
    if (mv <= SOC_MV[0]) {
        return SOC_PCT[0];
    }
    if (mv >= SOC_MV[SOC_POINTS - 1]) {
        return SOC_PCT[SOC_POINTS - 1];
    }
    for (int i = 1; i < SOC_POINTS; i++) {
        if (mv <= SOC_MV[i]) {
            int dv   = SOC_MV[i] - SOC_MV[i - 1];
            int dpct = SOC_PCT[i] - SOC_PCT[i - 1];
            int frac = (int)mv - (int)SOC_MV[i - 1];
            return (uint8_t)(SOC_PCT[i - 1] + (dpct * frac + dv / 2) / dv);
        }
    }
    return SOC_PCT[SOC_POINTS - 1];   // unreachable
}

// ============================================================================
// Band selection (plan §10) with directional hysteresis
// ============================================================================

// Upper SoC edge (percent) of band 0, 1, 2 — i.e. the promote thresholds.
static const int BAND_EDGE[3] = {
    BATT_SOC_LOW_PCT,       // CRITICAL -> LOW
    BATT_SOC_BALANCED_PCT,  // LOW      -> BALANCED
    BATT_SOC_HEALTHY_PCT,   // BALANCED -> HEALTHY
};

battery_band_t battery_band_from_soc(uint8_t soc, battery_band_t prev)
{
    int b = (int)prev;
    if (b < 0) b = 0;
    if (b > 3) b = 3;

    // Promote only when clearly above the next edge (slow, margin H above).
    while (b < 3 && (int)soc > BAND_EDGE[b] + BATT_BAND_HYST_PCT) {
        b++;
    }
    // Demote as soon as at/below the lower edge minus the margin (eager).
    while (b > 0 && (int)soc <= BAND_EDGE[b - 1] - BATT_BAND_HYST_PCT) {
        b--;
    }
    return (battery_band_t)b;
}

// ============================================================================
// Charge-rate trend (plan §11) — normalized by the variable wake interval
// ============================================================================

void battery_init(battery_state_t *b)
{
    memset(b, 0, sizeof(*b));
    b->band = BATT_CRITICAL;   // unknown battery is treated as empty (fail-safe)
}

static int16_t clamp_i16(int32_t v)
{
    if (v >  32767) return  32767;
    if (v < -32768) return -32768;
    return (int16_t)v;
}

void battery_update(battery_state_t *b, uint16_t mv, uint32_t now_s)
{
    // The band is the governor's ceiling and must be as current as the level
    // allows — refresh it on any real reading, clock or no clock.
    if (mv > 0) {
        uint8_t soc = battery_soc_from_mv(mv);
        b->band = (uint8_t)battery_band_from_soc(soc, (battery_band_t)b->band);
    }

    // The rate is a derivative, so it needs a prior timestamped reading and a
    // forward-moving clock. Normalize by elapsed minutes, not raw delta, because
    // the wake interval varies (plan §11). Sampling at a consistent point every
    // wake means a systematic load offset cancels in the delta, so radio load
    // can't fake a "draining" signal.
    if (mv > 0 && now_s != 0 &&
        b->last_mv != 0 && b->last_ts != 0 && now_s > b->last_ts) {
        int32_t dmv  = (int32_t)mv - (int32_t)b->last_mv;
        uint32_t dt  = now_s - b->last_ts;
        int32_t rate = clamp_i16(dmv * 60 / (int32_t)dt);       // mV/min

        // charge_rate is the reported signal: EMA-smoothed to damp ADC noise
        // (3·old + new)/4, first blend starting from 0.
        b->charge_rate = clamp_i16(((int32_t)b->charge_rate * 3 + rate) / 4);

        // Trend is asymmetric — quick to conserve, slow to splurge — so it keys
        // off the *instantaneous* rate, not the smoothed one: a single negative
        // step flips to FALLING at once, while RISING needs a sustained run of
        // positive steps (the smoothed value would lag the drop and delay
        // conserving, exactly the wrong direction to be slow in).
        if (rate <= -(int32_t)BATT_RATE_FALL_MV_MIN) {
            b->trend    = BATT_TREND_FALLING;
            b->rise_run = 0;
        } else if (rate >= (int32_t)BATT_RATE_RISE_MV_MIN) {
            if (b->rise_run < 255) {
                b->rise_run++;
            }
            b->trend = (b->rise_run >= BATT_RISE_CONFIRM_N)
                           ? BATT_TREND_RISING   // sustained rise confirmed
                           : BATT_TREND_FLAT;    // still earning the promotion
        } else {
            b->trend    = BATT_TREND_FLAT;
            b->rise_run = 0;
        }
    }

    // Latch this reading as the next baseline only when it is usable, so a failed
    // read or an unsynced clock doesn't poison the next rate computation.
    if (mv > 0 && now_s != 0) {
        b->last_mv = mv;
        b->last_ts = now_s;
    }
}

const char *battery_band_name(uint8_t band)
{
    switch ((battery_band_t)band) {
    case BATT_CRITICAL: return "CRITICAL";
    case BATT_LOW:      return "LOW";
    case BATT_BALANCED: return "BALANCED";
    case BATT_HEALTHY:  return "HEALTHY";
    default:            return "UNKNOWN";
    }
}
