/*
 * mesh_route.cpp - link-quality metric + parent selection (see mesh_route.h).
 */

#include <stddef.h>
#include "mesh_route.h"

/* LoRa demodulation floor (0.1 dB units) per spreading factor.  These are the
 * SNRs at which the PHY still decodes; the bandwidth does not change them. */
static int16_t mesh_route_snr_floor_x10(uint8_t sf)
{
    switch (sf) {
    case 5:  return -25;    /* -2.5 dB  */
    case 6:  return -50;    /* -5.0 dB  */
    case 7:  return -75;    /* -7.5 dB  */
    case 8:  return -100;   /* -10.0 dB */
    default: return -75;    /* outside the narrowband set: treat as SF7 */
    }
}

/* Cost = f(margin over the SF's demod floor).  Thresholds 12.5/7.5/2.5 dB map
 * to the classic >= +5 / 0 / -5 dB at SF7 and shift by +/- the floor elsewhere. */
uint8_t mesh_route_link_cost(int16_t snr_x10, uint8_t sf)
{
    int32_t margin = (int32_t)snr_x10 - (int32_t)mesh_route_snr_floor_x10(sf);

    if (margin >= 125) return 1;        /* >= 12.5 dB margin : good  */
    if (margin >= 75)  return 2;        /* >=  7.5 dB margin : fair  */
    if (margin >= 25)  return 3;        /* >=  2.5 dB margin : poor  */
    return 6;                           /* below floor       : avoid */
}

static void mesh_route_clear(mesh_route_t *r)
{
    r->parent_addr = MESH_ADDR_NONE;
    r->parent_cost = 0;
    r->self_cost   = 0;
    r->self_hops   = 0;
}

void mesh_route_select(const mesh_route_neighbor_t *n, uint8_t count,
                       const mesh_route_t *cur, uint8_t self_addr,
                       uint8_t hysteresis, mesh_route_t *out)
{
    const mesh_route_neighbor_t *best = NULL;
    uint8_t best_total = 0xFF;
    uint8_t cur_total = 0xFF;
    uint8_t i;

    for (i = 0; i < count; i++) {
        const mesh_route_neighbor_t *c = &n[i];
        uint8_t total;

        if (c->addr == MESH_ADDR_NONE || c->addr == self_addr) {
            continue;
        }
        total = (uint8_t)(c->link_cost + c->cost);
        if (best == NULL ||
            total < best_total ||
            (total == best_total && c->hops < best->hops) ||
            (total == best_total && c->hops == best->hops && c->snr_x10 > best->snr_x10)) {
            best = c;
            best_total = total;
        }
    }

    if (best == NULL) {
        mesh_route_clear(out);
        return;
    }

    if (cur->parent_addr != MESH_ADDR_NONE) {
        for (i = 0; i < count; i++) {
            if (n[i].addr == cur->parent_addr) {
                cur_total = (uint8_t)(n[i].link_cost + n[i].cost);
                break;
            }
        }
    }
    /* Keep the current parent unless the new best beats it by the margin. */
    if (cur->parent_addr != MESH_ADDR_NONE && cur_total != 0xFF &&
        (uint8_t)(best_total + hysteresis) > cur_total) {
        *out = *cur;
        return;
    }

    out->parent_addr = best->addr;
    out->parent_cost = best->cost;
    out->self_cost   = (uint8_t)(best->link_cost + best->cost);
    out->self_hops   = (uint8_t)(best->hops + 1);
    if (out->self_hops > MESH_DEFAULT_TTL) {
        out->self_hops = MESH_DEFAULT_TTL;
    }
}
