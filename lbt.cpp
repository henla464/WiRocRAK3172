#include <Arduino.h>
#include "radio_driver.h"       /* SUBGRF_GetRssiInst() */
#include "lbt.h"

/* Own record, next to the mesh config at 0x0100.  The flash driver patches
 * whole pages read-modify-write, so both records can share a page. */
#define LBT_FLASH_OFFSET   0x0110
#define LBT_FLASH_MAGIC    0x5A
#define LBT_FLASH_VERSION  0x01

struct __attribute__((packed)) lbt_flash_config_t {
    uint8_t magic;
    uint8_t version;
    int8_t  limit_dbm;
};

static int8_t s_limit;          /* dBm; 0 = disabled */

void lbt_init(void)
{
    lbt_flash_config_t cfg;

    s_limit = 0;
    if (!api.system.flash.get(LBT_FLASH_OFFSET, (uint8_t *)&cfg, sizeof(cfg))) {
        return;
    }
    if (cfg.magic != LBT_FLASH_MAGIC || cfg.version != LBT_FLASH_VERSION) {
        return;
    }
    if (cfg.limit_dbm > 0) {
        return;                 /* corrupted: positive dBm would read busy always */
    }
    s_limit = cfg.limit_dbm;
}

void lbt_set_rssi_limit(int16_t limit_dbm)
{
    lbt_flash_config_t cfg;

    if (limit_dbm > 0 || limit_dbm < INT8_MIN) {
        return;                 /* out of range: leave the limit unchanged */
    }
    s_limit = (int8_t)limit_dbm;
    cfg.magic = LBT_FLASH_MAGIC;
    cfg.version = LBT_FLASH_VERSION;
    cfg.limit_dbm = s_limit;
    api.system.flash.set(LBT_FLASH_OFFSET, (uint8_t *)&cfg, sizeof(cfg));
}

int16_t lbt_get_rssi_limit(void)
{
    return s_limit;
}

void lbt_set_default_for_bandwidth(uint32_t bandwidth)
{
    switch (bandwidth) {
    case 7:                     /* 31.25 kHz */
        lbt_set_rssi_limit(-115);
        break;
    case 4:                     /* 10.42 kHz */
        lbt_set_rssi_limit(-120);
        break;
    default:
        lbt_set_rssi_limit(0);  /* no calibrated default at other bandwidths */
        break;
    }
}

bool lbt_channel_busy(void)
{
    if (s_limit == 0) {
        return false;           /* LBT disabled */
    }
    return lbt_get_rssi() > s_limit;
}

int8_t lbt_get_rssi(void)
{
    return SUBGRF_GetRssiInst();
}
