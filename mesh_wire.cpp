/*
 * mesh_wire.cpp - bit-packed mesh header codec (see mesh_wire.h).
 */

#include "mesh_wire.h"

void mesh_wire_encode(uint8_t out[MESH_HEADER_SIZE], const mesh_header_t *h)
{
    out[0] = (uint8_t)(((h->type & 0x0F) << 4)
                     | (h->version & 0x0F));
    out[1] = (uint8_t)(((h->src & 0x0F) << 4)
                     | (h->dst & 0x0F));
    out[2] = (uint8_t)(((h->hops & 0x07) << 5)
                     | (h->seq & 0x1F));
}

void mesh_wire_decode(const uint8_t in[MESH_HEADER_SIZE], mesh_header_t *h)
{
    h->type    = (uint8_t)((in[0] >> 4) & 0x0F);
    h->version = (uint8_t)(in[0] & 0x0F);
    h->src     = (uint8_t)((in[1] >> 4) & 0x0F);
    h->dst     = (uint8_t)(in[1] & 0x0F);
    h->hops    = (uint8_t)((in[2] >> 5) & 0x07);
    h->seq     = (uint8_t)(in[2] & 0x1F);
}

uint8_t mesh_wire_path_bytes(uint8_t n)
{
    return (uint8_t)(((uint16_t)n * 4 + 7) / 8);
}

bool mesh_wire_ack_key(const mesh_header_t *h, const uint8_t *payload, uint8_t plen,
                       uint8_t self_addr, uint8_t *ack_src, uint8_t *ack_seq)
{
    if (h->type == MESH_TYPE_DATA_UPLINK || h->type == MESH_TYPE_DATA_DOWNLINK) {
        if (h->dst == self_addr) {
            return false;               /* our own copy, not the next hop's forward */
        }
        *ack_src = h->src;              /* a carried-on data frame keeps its origin */
        *ack_seq = h->seq;
        return true;
    }
    if (h->type == MESH_TYPE_LINK_ACK) {
        if (plen < 1) {
            return false;
        }
        *ack_src = h->dst;              /* acked origin rides in the header dst */
        *ack_seq = payload[0];
        return true;
    }
    return false;
}
