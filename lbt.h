/*
 * lbt.h
 *
 *  Listen before talk, as an instantaneous RSSI check (a CAD would block
 *  inside psend() and blow the host's reply window).
 *
 *  The threshold is measured, not configured per site: at boot the noise floor
 *  is estimated from LBT_NOISE_SAMPLES trimmed RSSI samples and the threshold
 *  starts at noise + LBT_DELTA_START_DB.  A slow controller then moves it
 *  inside [noise + delta_min, LBT_THRESHOLD_MAX_DBM], once per
 *  LBT_ADAPT_WINDOW decisions, from the busy rate and the retransmission rate
 *  (lbt_adapt_threshold()).  ATC+LBT=<dBm> pins the threshold; ATC+LBT=0 and
 *  every ATC+P2P / ATC+P2PM return to adaptive.
 */

#ifndef INC_LBT_H_
#define INC_LBT_H_

#include <stdint.h>
#include <stdbool.h>

/* --- Tuning ------------------------------------------------------------- *
 * Distinct per deployment, so they are macros rather than persisted config. */
#define LBT_NOISE_SAMPLES        32     /* boot samples for the noise floor   */
#define LBT_NOISE_TRIM_PCT       25     /* discard the loudest N% of samples  */
#define LBT_NOISE_SAMPLE_MS       5     /* spacing between boot samples       */
#define LBT_NOISE_SETTLE_MS     200     /* RX settle time before sampling     */

#define LBT_CHECK_SAMPLES         3     /* RSSI reads averaged per decision   */
#define LBT_DELTA_START_DB        8     /* initial margin over the noise      */
#define LBT_DELTA_MIN_DEFAULT_DB  3     /* closest approach to the noise floor;
                                         * runtime-settable, ATC+LBT=<dBm>:<dB> */
#define LBT_DELTA_MAX_DB         40     /* accepted range of that setting     */
#define LBT_THRESHOLD_MAX_DBM  (-70)    /* ... nor higher than this           */

#define LBT_ADAPT_WINDOW         32     /* decisions between adjustment steps */
#define LBT_ADAPT_STEP_DB         1
#define LBT_BUSY_HIGH_PCT        50     /* deferred this often -> raise       */
#define LBT_REXMIT_HIGH_PCT      25     /* retried this often -> maybe lower  */
#define LBT_LINK_RSSI_MIN_DBM (-105)    /* downlink quality gate              */
#define LBT_LINK_SNR_MIN_DB      (-6)

/* One adaptation step.  Pure, so the controller can be tested on the host.
 * `busy_pct` / `retx_pct` are percentages over the window, `link_good` is the
 * downlink quality gate.  Returns the next threshold, always clamped. */
static inline int16_t lbt_adapt_threshold(int16_t threshold, int16_t noise_floor,
                                          int16_t delta_min_db,
                                          int busy_pct, int retx_pct, bool link_good)
{
    int16_t lo = (int16_t)(noise_floor + delta_min_db);
    int16_t hi = LBT_THRESHOLD_MAX_DBM;

    if (hi < lo) {
        hi = lo;            /* site louder than the cap: keep the margin */
    }
    if (retx_pct >= LBT_REXMIT_HIGH_PCT && !link_good) {
        /* Retries with a weak downlink point at link margin, not at LBT:
         * moving the threshold either way makes it worse, so freeze. */
    } else if (busy_pct >= LBT_BUSY_HIGH_PCT && retx_pct < LBT_REXMIT_HIGH_PCT) {
        threshold = (int16_t)(threshold + LBT_ADAPT_STEP_DB);   /* fewer deferrals */
    } else if (busy_pct < LBT_BUSY_HIGH_PCT && retx_pct >= LBT_REXMIT_HIGH_PCT) {
        threshold = (int16_t)(threshold - LBT_ADAPT_STEP_DB);   /* defer more */
    }
    if (threshold < lo) {
        threshold = lo;
    }
    if (threshold > hi) {
        threshold = hi;
    }
    return threshold;
}

/* Load the pin, estimate the noise floor, set the starting threshold.
 * The radio must already be in receive mode.  Call once at boot. */
void    lbt_init(void);

/* Pin the threshold in dBm (negative); 0 returns to adaptive.  Persists. */
void    lbt_set_rssi_limit(int16_t limit_dbm);
int16_t lbt_get_rssi_limit(void);       /* the pin; 0 = adaptive  */

int16_t lbt_get_threshold(void);        /* effective threshold, dBm */
int16_t lbt_get_noise_floor(void);      /* boot estimate, dBm       */

/* Closest the threshold may come to the noise floor, in dB (default 3). */
void    lbt_set_delta_min(int16_t delta_min_db);   /* persists */
int16_t lbt_get_delta_min(void);

/* True when the sampled RSSI reads above the threshold. */
bool    lbt_channel_busy(void);

/* Current instantaneous RSSI (dBm), for calibrating / diagnostics. */
int8_t  lbt_get_rssi(void);

/* Adaptation inputs: the last received frame's quality, and one call per MAC
 * retransmission / ACK timeout. */
void    lbt_note_rx(int16_t rssi, int8_t snr);
void    lbt_note_retx(void);

#endif /* INC_LBT_H_ */
