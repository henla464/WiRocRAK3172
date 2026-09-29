/*
 * mesh.cpp
 *
 *  LoRa mesh mode for WiRocRAK3172 - M0 scaffolding.
 *
 *  Persistent mesh configuration (enabled / role / own address) held in the
 *  RUI user flash partition. Mesh is disabled by default so existing P2P
 *  behaviour is unchanged until it is explicitly enabled.
 */

#include <Arduino.h>
#include "mesh.h"

#define MESH_FLAG_ENABLED       0x01
#define MESH_FLAG_MASTER        0x02

/* Persistent layout (4 bytes). */
struct __attribute__((packed)) mesh_flash_config_t {
    uint8_t magic;
    uint8_t version;
    uint8_t flags;      /* MESH_FLAG_* */
    uint8_t address;    /* own 5-bit address, MESH_ADDR_NONE when unassigned */
};

static mesh_flash_config_t s_cfg;

static void mesh_config_defaults(void)
{
    s_cfg.magic   = MESH_FLASH_MAGIC;
    s_cfg.version = MESH_FLASH_VERSION;
    s_cfg.flags   = 0;                  /* meshing disabled, not master */
    s_cfg.address = MESH_ADDR_NONE;
}

static bool mesh_config_load(void)
{
    mesh_flash_config_t tmp;

    if (!api.system.flash.get(MESH_FLASH_OFFSET, (uint8_t *)&tmp, sizeof(tmp))) {
        return false;
    }
    if (tmp.magic != MESH_FLASH_MAGIC || tmp.version != MESH_FLASH_VERSION) {
        return false;
    }
    if (tmp.address > MESH_ADDR_MAX) {
        return false;
    }
    s_cfg = tmp;
    return true;
}

bool mesh_config_save(void)
{
    s_cfg.magic   = MESH_FLASH_MAGIC;
    s_cfg.version = MESH_FLASH_VERSION;
    return api.system.flash.set(MESH_FLASH_OFFSET, (uint8_t *)&s_cfg, sizeof(s_cfg));
}

bool mesh_init(void)
{
    if (!mesh_config_load()) {
        /* Virgin device (or incompatible version): use defaults in RAM only.
         * We don't force a flash write here; it happens on the first change. */
        mesh_config_defaults();
    }
    return true;
}

bool mesh_is_enabled(void)
{
    return (s_cfg.flags & MESH_FLAG_ENABLED) != 0;
}

void mesh_set_enabled(bool enabled)
{
    if (enabled) {
        s_cfg.flags |= MESH_FLAG_ENABLED;
    } else {
        s_cfg.flags &= (uint8_t)~MESH_FLAG_ENABLED;
    }
}

bool mesh_is_master(void)
{
    return (s_cfg.flags & MESH_FLAG_MASTER) != 0;
}

void mesh_set_master(bool master)
{
    if (master) {
        s_cfg.flags |= MESH_FLAG_MASTER;
    } else {
        s_cfg.flags &= (uint8_t)~MESH_FLAG_MASTER;
    }
}

uint8_t mesh_get_address(void)
{
    return (uint8_t)(s_cfg.address & MESH_ADDR_MASK);
}

void mesh_set_address(uint8_t address)
{
    s_cfg.address = (uint8_t)(address & MESH_ADDR_MASK);
}
