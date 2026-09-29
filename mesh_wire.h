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
 *     3   type     MESH_TYPE_* (see below)
 *     2   flags    MESH_FLAG_*
 *     5   src      source 5-bit address
 *     5   dst      destination 5-bit address
 *     3   hops     beacon: hop-count to master; data: TTL
 *     4   seq      dedup key (src,seq)
 *     2   version  protocol version
 *
 *   byte0 = type<<5 | flags<<3 | src>>2
 *   byte1 = (src&3)<<6 | dst<<1 | hops>>2
 *   byte2 = (hops&3)<<6 | seq<<2 | version
 */

#ifndef INC_MESH_WIRE_H_
#define INC_MESH_WIRE_H_

#include <stdint.h>

#define MESH_WIRE_VERSION   0
#define MESH_HEADER_SIZE    3

/* Message type (3 bits). Includes the count so it doubles as "invalid" bound. */
typedef enum {
    MESH_TYPE_BEACON = 0,
    MESH_TYPE_JOIN_REQ,
    MESH_TYPE_ADDR_ASSIGN,
    MESH_TYPE_ADDR_TABLE,
    MESH_TYPE_DATA_UPLINK,
    MESH_TYPE_DATA_DOWNLINK,
    MESH_TYPE_LINK_ACK,
    MESH_TYPE_ADDR_CLAIM,
    MESH_TYPE_COUNT
} mesh_type_t;

/* Header flags (2 bits). */
#define MESH_FLAG_ACK_REQ   0x01
#define MESH_FLAG_HAS_PATH  0x02

typedef struct {
    uint8_t version;    /* 2 bits */
    uint8_t type;       /* 3 bits */
    uint8_t flags;      /* 2 bits */
    uint8_t src;        /* 5 bits */
    uint8_t dst;        /* 5 bits */
    uint8_t hops;       /* 3 bits */
    uint8_t seq;        /* 4 bits */
} mesh_header_t;

/* Encode a header into MESH_HEADER_SIZE bytes. */
void mesh_wire_encode(uint8_t out[MESH_HEADER_SIZE], const mesh_header_t *h);

/* Decode MESH_HEADER_SIZE bytes into a header. */
void mesh_wire_decode(const uint8_t in[MESH_HEADER_SIZE], mesh_header_t *h);

/* Bytes required to pack an `n`-address (5-bit) source route: ceil(5n/8). */
uint8_t mesh_wire_path_bytes(uint8_t n);

#endif /* INC_MESH_WIRE_H_ */
