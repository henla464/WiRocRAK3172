/*
 * mesh_alloc.h
 *
 *  Master-side slave address allocator for LoRa mesh mode.
 *
 *  Maps a node device id (MESH_NODE_DEVICE_ID_LEN bytes) to a 4-bit slave address
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

/* Address assigned to `device_id`, or MESH_ADDR_NONE when unknown. */
uint8_t mesh_alloc_lookup(const uint8_t device_id[MESH_NODE_DEVICE_ID_LEN]);

/* Reverse lookup: copy the device id bound to `addr` into `out` and return true,
 * or return false (leaving `out` untouched) when `addr` is free / out of range. */
bool    mesh_alloc_device_id(uint8_t addr, uint8_t out[MESH_NODE_DEVICE_ID_LEN]);

/* Return the existing address for `device_id`, or allocate the lowest free slave
 * address.  Returns MESH_ADDR_NONE when the pool is exhausted. */
uint8_t mesh_alloc_assign(const uint8_t device_id[MESH_NODE_DEVICE_ID_LEN]);

/* Bind `device_id` to a specific, currently-free slave address (used when a node
 * re-claims its flash-stored address after a master restart).  Idempotent when
 * the device_id already maps to `addr`.  Returns false when `addr` is out of range,
 * already held by another device_id, or the table is full. */
bool    mesh_alloc_claim(const uint8_t device_id[MESH_NODE_DEVICE_ID_LEN], uint8_t addr);

/* Release the address held by `addr` (used for eviction / recycling). */
void    mesh_alloc_free(uint8_t addr);

/* Monotonic change counter: bumped on every *real* mutation of the table (a new
 * binding in assign/claim, an actual release in free, or a reset).  Idempotent
 * assign/claim calls do not move it.  The master uses it to flood ADDR_TABLE
 * when -- and only when -- the occupied set has actually changed. */
uint16_t mesh_alloc_version(void);

/* True when `addr` is currently allocated to a slave. */
bool    mesh_alloc_occupies(uint8_t addr);

/* Pack the 16-bit occupied bitmap (master + allocated slaves) into out[2]. */
void    mesh_alloc_bitmap(uint8_t out[2]);

#endif /* INC_MESH_ALLOC_H_ */
