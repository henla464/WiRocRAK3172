/*
 * mesh_wire.h
 *
 *  Bit-packed mesh frame header. Pure code (stdint only, no Arduino/RUI
 *  dependency) so it can be round-trip unit-tested on the host.
 *
 *  Fixed header: 3 bytes (24 bits), written MSB-first.
 *
 *   bits  field    meaning
 *   ----  -------  --------------------------------------------------
 *     4   type     MESH_TYPE_* (see below)
 *     4   version  protocol version
 *     4   src      source 4-bit address
 *     4   dst      destination 4-bit address
 *     3   hops     beacon: hop-count to master; data: TTL
 *     5   seq      dedup key (src,seq)
 *
 *   byte0 = type<<4 | version
 *   byte1 = src<<4  | dst
 *   byte2 = hops<<5 | seq
 */

#ifndef INC_MESH_WIRE_H_
#define INC_MESH_WIRE_H_

#include <stdint.h>

#define MESH_WIRE_VERSION   4
#define MESH_HEADER_SIZE    3

/* Message type (4 bits). Includes the count so it doubles as "invalid" bound. */
typedef enum {
    MESH_TYPE_BEACON = 0,
    MESH_TYPE_JOIN_REQ,
    MESH_TYPE_ADDR_ASSIGN,
    MESH_TYPE_ADDR_TABLE,
    MESH_TYPE_DATA_UPLINK,
    MESH_TYPE_DATA_DOWNLINK,
    MESH_TYPE_LINK_ACK,
    MESH_TYPE_ADDR_CLAIM,       /* rootward: { devid, parent }, bind addr<->devid */
    MESH_TYPE_ALIVE,            /* single hop: { subtree_bitmap }, liveness      */
    MESH_TYPE_TOPOLOGY,         /* rootward: { parent, {addr,cost}* }, map      */
    MESH_TYPE_COUNT
} mesh_type_t;

typedef struct {
    uint8_t type;       /* 4 bits */
    uint8_t version;    /* 4 bits */
    uint8_t src;        /* 4 bits */
    uint8_t dst;        /* 4 bits */
    uint8_t hops;       /* 3 bits */
    uint8_t seq;        /* 5 bits */
} mesh_header_t;

/* Encode a header into MESH_HEADER_SIZE bytes. */
void mesh_wire_encode(uint8_t out[MESH_HEADER_SIZE], const mesh_header_t *h);

/* Decode MESH_HEADER_SIZE bytes into a header. */
void mesh_wire_decode(const uint8_t in[MESH_HEADER_SIZE], mesh_header_t *h);

/* Bytes required to pack an `n`-address (4-bit) source route: ceil(4n/8) = ceil(n/2). */
uint8_t mesh_wire_path_bytes(uint8_t n);

#endif /* INC_MESH_WIRE_H_ */
