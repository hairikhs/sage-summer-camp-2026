// Energy signal: state-of-charge, SoC band, and charge-rate trend (plan §10, §11).
//
// Like adaptive.c, this module is deliberately free of any ESP-IDF, ADC, radio,
// or camera dependency — it operates on plain millivolt readings and POD state,
// so it compiles and unit-tests on the host (see test/test_adaptive). main.c
// owns the actual ADC read (the HT-HC33 senses VBAT through a switched 2:1
// divider on GPIO1/ADC1_CH0, gated by GPIO20) and feeds the millivolts in here.
//
// This is only the *signal*. The energy governor that caps the FSM decision with
// it lives next to the other decision-shapers in adaptive.c (adaptive_govern),
// because it shapes an adaptive_decision_t.
//
// See adaptive_sampling_plan.md sections 1, 10, 11, 14.
#pragma once

#include <stdbool.h>
#include <stdint.h>

// --- SoC bands (plan §10). Ordered so a *higher* value means *more* energy: the
// governor uses `band` as a ceiling and only ever moves the node toward a lower
// band's (less active) behavior on its own.
typedef enum {
    BATT_CRITICAL = 0,   // <10%   frame-diff only, no transmit, wait for recharge
    BATT_LOW,            // 10-30% ESP authority, send only strong positives
    BATT_BALANCED,       // 30-70% ESP gates, Thor confirms hits
    BATT_HEALTHY,        // >70%   send generously
} battery_band_t;

// Energy trend, from the sign of the (hysteretic) charge rate.
typedef enum {
    BATT_TREND_FLAT = 0,
    BATT_TREND_RISING,
    BATT_TREND_FALLING,
} battery_trend_t;

// SoC band thresholds in percent (plan §10) and the hysteresis margin that keeps
// the whole energy *policy* from flapping when SoC hovers on a boundary.
#define BATT_SOC_HEALTHY_PCT   70
#define BATT_SOC_BALANCED_PCT  30
#define BATT_SOC_LOW_PCT       10
#define BATT_BAND_HYST_PCT     3

// Charge-rate hysteresis (plan §11): quick to conserve, slow to splurge. A single
// sufficiently-negative rate flips to FALLING; RISING needs a *sustained* run of
// positive rates. Units are mV/min. Field-tunable.
#define BATT_RATE_RISE_MV_MIN  2
#define BATT_RATE_FALL_MV_MIN  2
#define BATT_RISE_CONFIRM_N    3

// Persisted battery state. Embedded in adaptive_state_t (adaptive.h) so it rides
// the one RTC blob + SD mirror + CRC, added by the versioned serial format
// without invalidating older blobs.
typedef struct {
    uint16_t last_mv;      // last VBAT reading (mV); 0 = none taken yet
    uint32_t last_ts;      // wall-clock secs of that reading; 0 = clock unknown
    int16_t  charge_rate;  // EMA-smoothed mV/min (signed); + charging, - draining
    uint8_t  band;         // battery_band_t held in RTC (the governor's ceiling)
    uint8_t  trend;        // battery_trend_t
    uint8_t  rise_run;     // consecutive rising updates (asymmetric hysteresis)
} battery_state_t;

// Zero the state: no reading yet, CRITICAL band until the first read refreshes it
// (fail-safe — an unknown battery is treated as empty, never as full).
void battery_init(battery_state_t *b);

// Single-cell LiPo open-circuit voltage → state-of-charge percent (0..100),
// clamped. Monotonic piecewise-linear table; the absolute value is coarse (the
// on-board divider is high-impedance), which is all the bands need.
uint8_t battery_soc_from_mv(uint16_t mv);

// SoC → band with directional hysteresis around each boundary (uses the previous
// band). Demoting toward CRITICAL is eager; promoting toward HEALTHY needs to
// clear the boundary by the hysteresis margin.
battery_band_t battery_band_from_soc(uint8_t soc, battery_band_t prev);

// Fold a fresh reading `mv` taken at wall-clock `now_s` into the state: refresh
// the band from the level (so the governor always has a current ceiling), and —
// when a prior timestamped reading exists — compute the time-normalized charge
// rate (mV/min, so the variable wake interval doesn't skew it), EMA-smooth it,
// and update the trend under asymmetric hysteresis. `mv==0` (read failed) or
// `now_s==0` (no clock) leaves the rate/trend inert but still refreshes the band
// when a level is available; the reading is only latched as the next baseline
// when it is usable (mv>0 and clock known).
void battery_update(battery_state_t *b, uint16_t mv, uint32_t now_s);

// "CRITICAL" | "LOW" | "BALANCED" | "HEALTHY" | "UNKNOWN". Shared by the decision
// log and the heartbeat so they can't drift apart.
const char *battery_band_name(uint8_t band);
