/*
 * mesh_route.cpp - link-quality metric + parent selection (see mesh_route.h).
 */

#include <stddef.h>
#include "mesh_route.h"

uint8_t mesh_route_link_cost(int16_t snr_x10)
{
    if (snr_x10 >= 50)  return 1;       /* >= 5 dB  : good   */
    if (snr_x10 >= 0)   return 2;       /* 0..5 dB  : fair   */
    if (snr_x10 >= -50) return 3;       /* -5..0 dB : poor   */
    return 6;                           /* < -5 dB  : avoid  */
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
