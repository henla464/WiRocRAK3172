/*
 * mesh_wire.cpp - bit-packed mesh header codec (see mesh_wire.h).
 */

#include "mesh_wire.h"

void mesh_wire_encode(uint8_t out[MESH_HEADER_SIZE], const mesh_header_t *h)
{
    out[0] = (uint8_t)(((h->type & 0x0F) << 4)
                     | ((h->flags & 0x01) << 3)
                     | ((h->src & 0x0F) >> 1));
    out[1] = (uint8_t)(((h->src & 0x01) << 7)
                     | ((h->dst & 0x0F) << 3)
                     | (h->hops & 0x07));
    out[2] = (uint8_t)(((h->seq & 0x1F) << 3)
                     | (h->version & 0x07));
}

void mesh_wire_decode(const uint8_t in[MESH_HEADER_SIZE], mesh_header_t *h)
{
    h->type    = (uint8_t)((in[0] >> 4) & 0x0F);
    h->flags   = (uint8_t)((in[0] >> 3) & 0x01);
    h->src     = (uint8_t)(((in[0] & 0x07) << 1) | ((in[1] >> 7) & 0x01));
    h->dst     = (uint8_t)((in[1] >> 3) & 0x0F);
    h->hops    = (uint8_t)(in[1] & 0x07);
    h->seq     = (uint8_t)((in[2] >> 3) & 0x1F);
    h->version = (uint8_t)(in[2] & 0x07);
}

uint8_t mesh_wire_path_bytes(uint8_t n)
{
    return (uint8_t)(((uint16_t)n * 4 + 7) / 8);
}
