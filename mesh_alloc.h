/*
 * mesh_alloc.h
 *
 *  Master-side slave address allocator for LoRa mesh mode.
 *
 *  Maps a node identity token (MESH_TOKEN_LEN bytes) to a 4-bit slave address
 *  in the range [MESH_FIRST_SLAVE_ADDR .. MESH_ADDR_MAX].  The table is
 *  RAM-only (lost on reboot) per the design: nodes are the source of truth for
 *  their own address via flash and re-announce it when the master restarts.
 *
 *  Pure code (stdint/stdbool only, no Arduino/RUI dependency) so it can be
 *  unit-tested on the host.
 */

#ifndef INC_MESH_ALLOC_H_
#define INC_MESH_ALLOC_H_

#include <stdint.h>
#include <stdbool.h>
#include "mesh.h"

/* Drop every assignment (master reboot / mesh re-init). */
void    mesh_alloc_reset(void);

/* Number of slave addresses currently allocated. */
uint8_t mesh_alloc_count(void);

/* Address assigned to `token`, or MESH_ADDR_NONE when unknown. */
uint8_t mesh_alloc_lookup(const uint8_t token[MESH_TOKEN_LEN]);

/* Return the existing address for `token`, or allocate the lowest free slave
 * address.  Returns MESH_ADDR_NONE when the pool is exhausted. */
uint8_t mesh_alloc_assign(const uint8_t token[MESH_TOKEN_LEN]);

/* Bind `token` to a specific, currently-free slave address (used when a node
 * re-claims its flash-stored address after a master restart).  Idempotent when
 * the token already maps to `addr`.  Returns false when `addr` is out of range,
 * already held by another token, or the table is full. */
bool    mesh_alloc_claim(const uint8_t token[MESH_TOKEN_LEN], uint8_t addr);

/* Release the address held by `addr` (used for eviction / recycling). */
void    mesh_alloc_free(uint8_t addr);

/* True when `addr` is currently allocated to a slave. */
bool    mesh_alloc_occupies(uint8_t addr);

/* Pack the 16-bit occupied bitmap (master + allocated slaves) into out[2]. */
void    mesh_alloc_bitmap(uint8_t out[2]);

#endif /* INC_MESH_ALLOC_H_ */
