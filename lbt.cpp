#include <Arduino.h>
#include <stdlib.h>             /* qsort() */
#include "radio_driver.h"       /* SUBGRF_GetRssiInst() */
#include "lbt.h"

/* Own record, next to the mesh config at 0x0100.  The flash driver patches
 * whole pages read-modify-write, so both records can share a page. */
#define LBT_FLASH_OFFSET   0x0110
#define LBT_FLASH_MAGIC    0x5A
#define LBT_FLASH_VERSION  0x02

/* Only used when the boot sampling yields nothing usable. */
#define LBT_NOISE_FALLBACK_DBM  (-100)

struct __attribute__((packed)) lbt_flash_config_t {
    uint8_t magic;
    uint8_t version;
    int8_t  limit_dbm;
    uint8_t delta_min_db;       /* added in version 2; v1 records keep default */
};

static int8_t  s_pin;           /* dBm; 0 = adaptive */
static int16_t s_delta_min;     /* dB above the noise floor, at minimum */
static int16_t s_noise;         /* boot noise-floor estimate, dBm */
static int16_t s_threshold;     /* adaptive threshold, dBm */

static uint16_t s_decisions;    /* LBT decisions in the current window */
static uint16_t s_busy_count;
static uint16_t s_retx_count;

static int16_t s_rx_rssi;       /* last received frame, for the quality gate */
static int8_t  s_rx_snr;
static bool    s_rx_valid;

static int cmp_i16(const void *a, const void *b)
{
    return (*(const int16_t *)a > *(const int16_t *)b) -
           (*(const int16_t *)a < *(const int16_t *)b);
}

/* Trimmed mean of the quietest (100 - LBT_NOISE_TRIM_PCT)% of the samples.
 * Dropping the loudest samples rejects transient interferers; dropping the
 * -128 reads rejects "no measurement" from a radio that was not listening. */
static int16_t lbt_estimate_noise_floor(void)
{
    int16_t samples[LBT_NOISE_SAMPLES];
    uint8_t used = 0;
    uint8_t keep;
    int32_t sum = 0;

    delay(LBT_NOISE_SETTLE_MS);

    for (uint8_t i = 0; i < LBT_NOISE_SAMPLES; i++) {
        int8_t rssi = SUBGRF_GetRssiInst();
        if (rssi > INT8_MIN) {
            samples[used++] = rssi;
        }
        delay(LBT_NOISE_SAMPLE_MS);
    }
    if (used == 0) {
        return LBT_NOISE_FALLBACK_DBM;
    }

    qsort(samples, used, sizeof(samples[0]), cmp_i16);
    keep = (uint8_t)(used - (uint16_t)used * LBT_NOISE_TRIM_PCT / 100);
    if (keep == 0) {
        keep = 1;
    }
    for (uint8_t i = 0; i < keep; i++) {
        sum += samples[i];
    }
    return (int16_t)(sum / keep);
}

static void lbt_reset_window(void)
{
    /* No busy, no retx, so the step function only clamps the start value. */
    s_threshold  = lbt_adapt_threshold((int16_t)(s_noise + LBT_DELTA_START_DB),
                                       s_noise, s_delta_min, 0, 0, true);
    s_decisions  = 0;
    s_busy_count = 0;
    s_retx_count = 0;
}

static void lbt_persist(void)
{
    lbt_flash_config_t cfg;

    cfg.magic        = LBT_FLASH_MAGIC;
    cfg.version      = LBT_FLASH_VERSION;
    cfg.limit_dbm    = s_pin;
    cfg.delta_min_db = (uint8_t)s_delta_min;
    api.system.flash.set(LBT_FLASH_OFFSET, (uint8_t *)&cfg, sizeof(cfg));
}

void lbt_init(void)
{
    lbt_flash_config_t cfg;

    s_pin       = 0;
    s_delta_min = LBT_DELTA_MIN_DEFAULT_DB;
    if (api.system.flash.get(LBT_FLASH_OFFSET, (uint8_t *)&cfg, sizeof(cfg)) &&
        cfg.magic == LBT_FLASH_MAGIC && cfg.limit_dbm <= 0) {
        s_pin = cfg.limit_dbm;
        /* A version 1 record holds the pin only; its trailing byte is still
         * whatever the page held, so only version 2 carries a delta. */
        if (cfg.version == 0x02 &&
            cfg.delta_min_db >= 1 && cfg.delta_min_db <= LBT_DELTA_MAX_DB) {
            s_delta_min = cfg.delta_min_db;
        }
    }

    s_noise = lbt_estimate_noise_floor();
    s_rx_valid = false;
    lbt_reset_window();
}

void lbt_set_rssi_limit(int16_t limit_dbm)
{
    if (limit_dbm > 0 || limit_dbm < INT8_MIN) {
        return;                 /* out of range: leave the threshold unchanged */
    }
    if (s_pin == (int8_t)limit_dbm) {
        return;                 /* already there: keep the window and the flash */
    }
    s_pin = (int8_t)limit_dbm;
    if (s_pin == 0) {
        lbt_reset_window();     /* back to the adaptive threshold */
    }
    lbt_persist();
}

void lbt_set_delta_min(int16_t delta_min_db)
{
    if (delta_min_db < 1 || delta_min_db > LBT_DELTA_MAX_DB) {
        return;
    }
    if (s_delta_min == delta_min_db) {
        return;
    }
    s_delta_min = delta_min_db;
    if (s_pin == 0) {
        lbt_reset_window();     /* re-clamp the adaptive threshold */
    }
    lbt_persist();
}

int16_t lbt_get_rssi_limit(void)
{
    return s_pin;
}

int16_t lbt_get_threshold(void)
{
    return s_pin ? s_pin : s_threshold;
}

int16_t lbt_get_noise_floor(void)
{
    return s_noise;
}

int16_t lbt_get_delta_min(void)
{
    return s_delta_min;
}

/* Average a few back-to-back reads: one instantaneous sample is noisy enough
 * that a single marginal interferer flips the decision. */
static int16_t lbt_sample_rssi(void)
{
    int16_t sum = 0;

    for (uint8_t i = 0; i < LBT_CHECK_SAMPLES; i++) {
        sum = (int16_t)(sum + SUBGRF_GetRssiInst());
    }
    return (int16_t)(sum / LBT_CHECK_SAMPLES);
}

static void lbt_adapt(void)
{
    bool link_good;
    int busy_pct;
    int retx_pct;

    if (s_decisions < LBT_ADAPT_WINDOW) {
        return;
    }
    busy_pct  = (int)((uint32_t)s_busy_count * 100 / s_decisions);
    retx_pct  = (int)((uint32_t)s_retx_count * 100 / s_decisions);
    link_good = s_rx_valid &&
                s_rx_rssi >= LBT_LINK_RSSI_MIN_DBM &&
                s_rx_snr  >= LBT_LINK_SNR_MIN_DB;

    s_threshold  = lbt_adapt_threshold(s_threshold, s_noise, s_delta_min,
                                       busy_pct, retx_pct, link_good);
    s_decisions  = 0;
    s_busy_count = 0;
    s_retx_count = 0;
}

bool lbt_channel_busy(void)
{
    bool busy = lbt_sample_rssi() > lbt_get_threshold();

    if (s_pin == 0) {
        /* A pinned threshold is a calibration, not a control loop: keep the
         * estimator out of it. */
        s_decisions++;
        if (busy) {
            s_busy_count++;
        }
        lbt_adapt();
    }
    return busy;
}

void lbt_note_rx(int16_t rssi, int8_t snr)
{
    s_rx_rssi  = rssi;
    s_rx_snr   = snr;
    s_rx_valid = true;
}

void lbt_note_retx(void)
{
    if (s_retx_count < 0xFFFF) {
        s_retx_count++;
    }
}

int8_t lbt_get_rssi(void)
{
    return SUBGRF_GetRssiInst();
}
