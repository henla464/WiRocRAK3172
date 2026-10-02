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
    for (unsigned type = 0; type < 8; ++type)
    for (unsigned flags = 0; flags < 4; ++flags)
    for (unsigned src = 0; src < 16; ++src)
    for (unsigned dst = 0; dst < 16; ++dst)
    for (unsigned hops = 0; hops < 8; ++hops)
    for (unsigned seq = 0; seq < 32; ++seq)
    for (unsigned ver = 0; ver < 8; ++ver) {
        mesh_header_t in { (uint8_t)ver, (uint8_t)type, (uint8_t)flags,
                           (uint8_t)src, (uint8_t)dst, (uint8_t)hops, (uint8_t)seq };
        uint8_t buf[MESH_HEADER_SIZE];
        mesh_wire_encode(buf, &in);
        mesh_header_t out;
        mesh_wire_decode(buf, &out);
        if (out.version != in.version || out.type != in.type || out.flags != in.flags ||
            out.src != in.src || out.dst != in.dst || out.hops != in.hops || out.seq != in.seq) {
            std::printf("FAIL rt type=%u flags=%u src=%u dst=%u hops=%u seq=%u ver=%u\n",
                        type, flags, src, dst, hops, seq, ver);
            return 1;
        }
    }
    std::printf("round-trip: OK (16,777,216 combinations)\n");

    /* --- Known bit vector (guards the exact bit layout) --- */
    mesh_header_t h { MESH_WIRE_VERSION, MESH_TYPE_DATA_UPLINK,
                      0, 5, 1, 2, 3 };
    uint8_t b[MESH_HEADER_SIZE];
    mesh_wire_encode(b, &h);
    assert(b[0] == 0x82 && b[1] == 0x8A && b[2] == 0x1A);
    std::printf("known vector: OK (%02X %02X %02X)\n", b[0], b[1], b[2]);

    /* --- Path length packing (4-bit addresses: ceil(n/2) bytes) --- */
    assert(mesh_wire_path_bytes(0) == 0);
    assert(mesh_wire_path_bytes(1) == 1);
    assert(mesh_wire_path_bytes(2) == 1);
    assert(mesh_wire_path_bytes(3) == 2);
    assert(mesh_wire_path_bytes(4) == 2);
    assert(mesh_wire_path_bytes(8) == 4);
    assert(mesh_wire_path_bytes(16) == 8);
    std::printf("path bytes: OK\n");

    std::printf("ALL TESTS PASSED\n");
    return 0;
}
