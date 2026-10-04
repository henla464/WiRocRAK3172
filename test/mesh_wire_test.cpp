/*
 * Host-side round-trip test for the bit-packed mesh header codec.
 * Build:  g++ -std=c++17 -I. test/mesh_wire_test.cpp mesh_wire.cpp -o /tmp/mesh_wire_test && /tmp/mesh_wire_test
 */

#include <cstdio>
#include <cstdint>
#include <cassert>
#include "mesh_wire.h"

int main()
{
    /* --- Exhaustive round-trip over every field range --- */
    for (unsigned type = 0; type < MESH_TYPE_COUNT; ++type)
    for (unsigned ver = 0; ver < 16; ++ver)
    for (unsigned src = 0; src < 16; ++src)
    for (unsigned dst = 0; dst < 16; ++dst)
    for (unsigned hops = 0; hops < 8; ++hops)
    for (unsigned seq = 0; seq < 32; ++seq) {
        mesh_header_t in { (uint8_t)type, (uint8_t)ver,
                           (uint8_t)src, (uint8_t)dst, (uint8_t)hops, (uint8_t)seq };
        uint8_t buf[MESH_HEADER_SIZE];
        mesh_wire_encode(buf, &in);
        mesh_header_t out;
        mesh_wire_decode(buf, &out);
        if (out.version != in.version || out.type != in.type ||
            out.src != in.src || out.dst != in.dst || out.hops != in.hops || out.seq != in.seq) {
            std::printf("FAIL rt type=%u ver=%u src=%u dst=%u hops=%u seq=%u\n",
                        type, ver, src, dst, hops, seq);
            return 1;
        }
    }
    std::printf("round-trip: OK (%u combinations)\n",
                (unsigned)MESH_TYPE_COUNT * 16u * 16u * 16u * 8u * 32u);

    /* --- Known bit vector (guards the exact bit layout) --- */
    mesh_header_t h { MESH_TYPE_DATA_UPLINK, MESH_WIRE_VERSION,
                      5, 1, 2, 3 };
    uint8_t b[MESH_HEADER_SIZE];
    mesh_wire_encode(b, &h);
    /* byte0 carries type in the high nibble and version in the low one; derived
     * from the macro so a version bump does not have to touch this vector. */
    assert(b[0] == ((MESH_TYPE_DATA_UPLINK << 4) | MESH_WIRE_VERSION) &&
           b[1] == 0x51 && b[2] == 0x43);
    std::printf("known vector: OK (%02X %02X %02X)\n", b[0], b[1], b[2]);

    /* --- Hop-level ACK keying: which received frame clears which pending --- */
    {
        uint8_t ack_src, ack_seq;
        uint8_t pl[1] = { 0 };
        mesh_header_t f;

        /* A relay at 3 holding an in-flight uplink from origin 5 clears it when
         * it hears the next hop carry the frame on -- same origin, addressed
         * onward (to 2), not back to us. */
        f = mesh_header_t{ MESH_TYPE_DATA_UPLINK, MESH_WIRE_VERSION, 5, 2, 2, 11 };
        assert(mesh_wire_ack_key(&f, pl, 1, 3, &ack_src, &ack_seq) &&
               ack_src == 5 && ack_seq == 11);

        /* Our own copy of that uplink (addressed to us) confirms nothing. */
        f.dst = 3;
        assert(!mesh_wire_ack_key(&f, pl, 1, 3, &ack_src, &ack_seq));

        /* A downlink is acked the same way, and its origin is the root (1),
         * never a non-root node -- so the two directions cannot clear each other. */
        f = mesh_header_t{ MESH_TYPE_DATA_DOWNLINK, MESH_WIRE_VERSION, 1, 9, 3, 4 };
        assert(mesh_wire_ack_key(&f, pl, 1, 7, &ack_src, &ack_seq) &&
               ack_src == 1 && ack_seq == 4);

        /* A LINK_ACK acks the origin named in its header dst, read from its
         * payload byte -- and counts even when we are the one addressed (an
         * uplink origin hears the root's ACK for its own frame directly). */
        f = mesh_header_t{ MESH_TYPE_LINK_ACK, MESH_WIRE_VERSION, 1, 2, 0, 0 };
        pl[0] = 19;
        assert(mesh_wire_ack_key(&f, pl, 1, 2, &ack_src, &ack_seq) &&
               ack_src == 2 && ack_seq == 19);

        /* A LINK_ACK missing its payload byte acks nothing. */
        assert(!mesh_wire_ack_key(&f, pl, 0, 2, &ack_src, &ack_seq));

        /* Control frames carry no hop ACK at all. */
        f = mesh_header_t{ MESH_TYPE_BEACON, MESH_WIRE_VERSION, 5, 0, 1, 2 };
        assert(!mesh_wire_ack_key(&f, pl, 1, 3, &ack_src, &ack_seq));

        std::printf("ack key: OK\n");
    }

    std::printf("ALL TESTS PASSED\n");
    return 0;
}
