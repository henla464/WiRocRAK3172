/*
 * mesh_route.h
 *
 *  Link-quality metric and parent selection for the convergecast tree.
 *
 *  Pure code (stdint/stdbool only, no Arduino/RUI dependency) so the routing
 *  decision can be unit-tested on the host: a node picks the neighbour that
 *  minimises (link cost + neighbour's advertised path cost), so an extra
 *  reliable hop is preferred over one weak link.  Switching parent requires a
 *  hysteresis margin to avoid flapping.
 *
 *  The link cost is a margin over the LoRa demodulation floor, which depends on
 *  the spreading factor (not the bandwidth): SF5 ~ -2.5 dB .. SF8 ~ -10 dB.
 *  Working in margin keeps the metric meaningful when the datarate is retuned.
 */

#ifndef INC_MESH_ROUTE_H_
#define INC_MESH_ROUTE_H_

#include <stdint.h>
#include "mesh.h"

/* One tracked neighbour (as maintained from its beacons). */
typedef struct {
    uint8_t  addr;          /* neighbour address (0 == free slot)             */
    int16_t  snr_x10;       /* smoothed SNR in 0.1 dB units                   */
    uint8_t  cost;          /* neighbour's advertised path cost               */
    uint8_t  hops;          /* neighbour's advertised hop-count to the master */
    uint8_t  link_cost;     /* cost of our link to this neighbour             */
    uint32_t last_ms;       /* time of the last beacon from this neighbour    */
} mesh_route_neighbor_t;

/* Our route to the master. */
typedef struct {
    uint8_t parent_addr;    /* next hop (0 = none)                            */
    uint8_t parent_cost;    /* parent's advertised cost                       */
    uint8_t self_cost;      /* our cost to the master                         */
    uint8_t self_hops;      /* our hop-count to the master                    */
} mesh_route_t;

/* Map a smoothed SNR (0.1 dB units) to a small integer link cost, given the
 * active spreading factor `sf` (its demodulation floor sets the reference).
 * At SF7 this reproduces the fixed >= +5 / 0 / -5 dB thresholds. */
uint8_t mesh_route_link_cost(int16_t snr_x10, uint8_t sf);

/* Pick the parent from `count` neighbours, honouring `hysteresis` against the
 * current route `cur` (which may have `parent_addr == 0`).  `self_addr` is
 * never chosen.  The result is written to `out`. */
void mesh_route_select(const mesh_route_neighbor_t *n, uint8_t count,
                       const mesh_route_t *cur, uint8_t self_addr,
                       uint8_t hysteresis, mesh_route_t *out);

#endif /* INC_MESH_ROUTE_H_ */
