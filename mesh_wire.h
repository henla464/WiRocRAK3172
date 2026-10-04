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
#include <stdbool.h>

#define MESH_WIRE_VERSION   1
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
    MESH_TYPE_ADDR_ALIVE,       /* single hop: { subtree_bitmap }, liveness      */
    MESH_TYPE_TOPOLOGY,         /* rootward: { parent, {addr,cost}* }, map      */
    MESH_TYPE_MASTER_QUERY,     /* broadcast: { devid }, "who is master?", see M8 */
    MESH_TYPE_MASTER_ANNOUNCE,  /* broadcast: { devid }, a master states its id  */
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

/* The frame's hop-level acknowledgement, if it carries one: write the (origin,
 * seq) it confirms to `ack_src`/`ack_seq` and return true, else return false.
 *
 * A data frame is confirmed by the next hop carrying it on, which is heard as
 * the same origin and seq addressed somewhere else; our own copy (addressed to
 * us) is not a forward.  A LINK_ACK is confirmed by whoever the path ends at:
 * it names the origin of the frame it acks in the header dst and carries that
 * frame's seq as its only payload byte, so it counts even when addressed to us.
 */
bool mesh_wire_ack_key(const mesh_header_t *h, const uint8_t *payload, uint8_t plen,
                       uint8_t self_addr, uint8_t *ack_src, uint8_t *ack_seq);

#endif /* INC_MESH_WIRE_H_ */
