/*
 * mesh_wire.cpp - bit-packed mesh header codec (see mesh_wire.h).
 */

#include "mesh_wire.h"

void mesh_wire_encode(uint8_t out[MESH_HEADER_SIZE], const mesh_header_t *h)
{
    out[0] = (uint8_t)(((h->type & 0x07) << 5)
                     | ((h->flags & 0x03) << 3)
                     | ((h->src & 0x1F) >> 2));
    out[1] = (uint8_t)(((h->src & 0x03) << 6)
                     | ((h->dst & 0x1F) << 1)
                     | ((h->hops & 0x07) >> 2));
    out[2] = (uint8_t)(((h->hops & 0x03) << 6)
                     | ((h->seq & 0x0F) << 2)
                     | (h->version & 0x03));
}

void mesh_wire_decode(const uint8_t in[MESH_HEADER_SIZE], mesh_header_t *h)
{
    h->type    = (uint8_t)((in[0] >> 5) & 0x07);
    h->flags   = (uint8_t)((in[0] >> 3) & 0x03);
    h->src     = (uint8_t)(((in[0] & 0x07) << 2) | ((in[1] >> 6) & 0x03));
    h->dst     = (uint8_t)((in[1] >> 1) & 0x1F);
    h->hops    = (uint8_t)(((in[1] & 0x01) << 2) | ((in[2] >> 6) & 0x03));
    h->seq     = (uint8_t)((in[2] >> 2) & 0x0F);
    h->version = (uint8_t)(in[2] & 0x03);
}

uint8_t mesh_wire_path_bytes(uint8_t n)
{
    return (uint8_t)(((uint16_t)n * 5 + 7) / 8);
}
