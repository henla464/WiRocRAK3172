/*
 * mesh_alloc.cpp - master-side slave address allocator (see mesh_alloc.h).
 */

#include <string.h>
#include "mesh_alloc.h"

typedef struct {
    uint8_t token[MESH_TOKEN_LEN];
    uint8_t addr;                       /* MESH_ADDR_NONE == free slot */
} mesh_alloc_entry_t;

static mesh_alloc_entry_t s_entries[MESH_MAX_SLAVES];

static int mesh_alloc_find(const uint8_t token[MESH_TOKEN_LEN])
{
    for (uint8_t i = 0; i < MESH_MAX_SLAVES; i++) {
        if (s_entries[i].addr != MESH_ADDR_NONE &&
            memcmp(s_entries[i].token, token, MESH_TOKEN_LEN) == 0) {
            return (int)i;
        }
    }
    return -1;
}

static bool mesh_alloc_addr_used(uint8_t addr)
{
    for (uint8_t i = 0; i < MESH_MAX_SLAVES; i++) {
        if (s_entries[i].addr == addr) {
            return true;
        }
    }
    return false;
}

void mesh_alloc_reset(void)
{
    memset(s_entries, 0, sizeof(s_entries));
}

uint8_t mesh_alloc_count(void)
{
    uint8_t n = 0;
    for (uint8_t i = 0; i < MESH_MAX_SLAVES; i++) {
        if (s_entries[i].addr != MESH_ADDR_NONE) {
            n++;
        }
    }
    return n;
}

uint8_t mesh_alloc_lookup(const uint8_t token[MESH_TOKEN_LEN])
{
    int i = mesh_alloc_find(token);
    return (i < 0) ? MESH_ADDR_NONE : s_entries[i].addr;
}

bool mesh_alloc_occupies(uint8_t addr)
{
    if (addr == MESH_ADDR_NONE) {
        return false;
    }
    return mesh_alloc_addr_used(addr);
}

uint8_t mesh_alloc_assign(const uint8_t token[MESH_TOKEN_LEN])
{
    /* Idempotent: an already-known node keeps its address. */
    int i = mesh_alloc_find(token);
    if (i >= 0) {
        return s_entries[i].addr;
    }
    /* Lowest free address in the slave range. */
    for (uint8_t addr = MESH_FIRST_SLAVE_ADDR; addr <= MESH_ADDR_MAX; addr++) {
        if (mesh_alloc_addr_used(addr)) {
            continue;
        }
        for (uint8_t k = 0; k < MESH_MAX_SLAVES; k++) {
            if (s_entries[k].addr == MESH_ADDR_NONE) {
                memcpy(s_entries[k].token, token, MESH_TOKEN_LEN);
                s_entries[k].addr = addr;
                return addr;
            }
        }
        break;                          /* no free slot */
    }
    return MESH_ADDR_NONE;              /* pool exhausted */
}

void mesh_alloc_free(uint8_t addr)
{
    for (uint8_t i = 0; i < MESH_MAX_SLAVES; i++) {
        if (s_entries[i].addr == addr) {
            memset(&s_entries[i], 0, sizeof(s_entries[i]));
            return;
        }
    }
}

void mesh_alloc_bitmap(uint8_t out[4])
{
    memset(out, 0, 4);
    out[MESH_MASTER_ADDR >> 3] |= (uint8_t)(1u << (MESH_MASTER_ADDR & 7));
    for (uint8_t i = 0; i < MESH_MAX_SLAVES; i++) {
        uint8_t addr = s_entries[i].addr;
        if (addr != MESH_ADDR_NONE) {
            out[addr >> 3] |= (uint8_t)(1u << (addr & 7));
        }
    }
}
