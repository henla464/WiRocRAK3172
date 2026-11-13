/*
 * lbt.h
 *
 *  Listen before talk, as an instantaneous RSSI check (a CAD would block
 *  inside psend() and blow the host's reply window).
 *
 *  Every transmit path compares the instantaneous RSSI against the persisted
 *  limit before sending; the channel counts as busy when the reading is above
 *  it.  0 disables the check, which is the default: the right threshold
 *  depends on the site's noise floor and radio settings.
 */

#ifndef INC_LBT_H_
#define INC_LBT_H_

#include <stdint.h>
#include <stdbool.h>

/* Load the persisted limit (0 = disabled).  Call once at boot. */
void    lbt_init(void);

void    lbt_set_rssi_limit(int16_t limit_dbm);  /* persists it */
int16_t lbt_get_rssi_limit(void);

/* Set the limit that suits a RUI bandwidth index (7 = 31.25 kHz ->
 * -115 dBm, 4 = 10.42 kHz -> -120 dBm, anything else -> disabled). */
void    lbt_set_default_for_bandwidth(uint32_t bandwidth);

/* True when the limit is set and the instantaneous RSSI reads above it. */
bool    lbt_channel_busy(void);

/* Current instantaneous RSSI (dBm), for calibrating the limit. */
int8_t  lbt_get_rssi(void);

#endif /* INC_LBT_H_ */
