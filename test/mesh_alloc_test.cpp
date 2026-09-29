/*
 * Host unit test for the master-side mesh address allocator (mesh_alloc.cpp).
 *
 * Build:
 *   g++ -std=c++17 -Wall -Wextra -I. test/mesh_alloc_test.cpp mesh_alloc.cpp -o /tmp/mesh_alloc_test
 */

#include <cstdio>
#include <cstring>
#include "mesh_alloc.h"

static int g_failures = 0;

#define CHECK(cond)                                                        \
    do {                                                                   \
        if (!(cond)) {                                                     \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);    \
            g_failures++;                                                  \
        }                                                                  \
    } while (0)

static void make_token(uint8_t token[MESH_TOKEN_LEN], uint8_t id)
{
    std::memset(token, 0, MESH_TOKEN_LEN);
    token[0] = id;
    token[MESH_TOKEN_LEN - 1] = (uint8_t)(id ^ 0xFF);
}

int main(void)
{
    uint8_t tA[MESH_TOKEN_LEN], tB[MESH_TOKEN_LEN], tC[MESH_TOKEN_LEN];
    uint8_t unknown[MESH_TOKEN_LEN];
    uint8_t bm[4];

    make_token(tA, 0x11);
    make_token(tB, 0x22);
    make_token(tC, 0x33);
    make_token(unknown, 0xEE);

    mesh_alloc_reset();
    CHECK(mesh_alloc_count() == 0);
    CHECK(mesh_alloc_lookup(tA) == MESH_ADDR_NONE);

    /* Lowest free addresses are handed out in order. */
    CHECK(mesh_alloc_assign(tA) == 2);
    CHECK(mesh_alloc_assign(tB) == 3);
    CHECK(mesh_alloc_assign(tC) == 4);
    CHECK(mesh_alloc_count() == 3);

    /* Idempotent re-assignment. */
    CHECK(mesh_alloc_assign(tA) == 2);
    CHECK(mesh_alloc_assign(tB) == 3);
    CHECK(mesh_alloc_count() == 3);

    /* Lookup. */
    CHECK(mesh_alloc_lookup(tA) == 2);
    CHECK(mesh_alloc_lookup(tC) == 4);
    CHECK(mesh_alloc_lookup(unknown) == MESH_ADDR_NONE);

    /* Occupancy. */
    CHECK(mesh_alloc_occupies(2));
    CHECK(mesh_alloc_occupies(4));
    CHECK(!mesh_alloc_occupies(5));
    CHECK(!mesh_alloc_occupies(MESH_ADDR_NONE));

    /* Bitmap: master bit + allocated slaves set, everything else clear. */
    mesh_alloc_bitmap(bm);
    CHECK(bm[0] & (1u << MESH_MASTER_ADDR));
    CHECK(bm[0] & (1u << 2));
    CHECK(bm[0] & (1u << 3));
    CHECK(bm[0] & (1u << 4));
    CHECK(!(bm[0] & (1u << 5)));
    CHECK(bm[1] == 0 && bm[2] == 0 && bm[3] == 0);

    /* Exhaust the pool (30 slaves total; A/B/C already hold 3) -> next fails. */
    for (uint16_t id = 0x40; id < 0x40 + (MESH_MAX_SLAVES - 3); id++) {
        uint8_t t[MESH_TOKEN_LEN];
        make_token(t, (uint8_t)id);
        CHECK(mesh_alloc_assign(t) != MESH_ADDR_NONE);
    }
    CHECK(mesh_alloc_count() == MESH_MAX_SLAVES);
    CHECK(mesh_alloc_assign(unknown) == MESH_ADDR_NONE);

    /* Freeing an address recycles the lowest free slot. */
    mesh_alloc_free(3);
    CHECK(mesh_alloc_count() == MESH_MAX_SLAVES - 1);
    CHECK(!mesh_alloc_occupies(3));
    CHECK(mesh_alloc_assign(unknown) == 3);

    /* Freeing an unassigned address is a no-op (0 is never handed out). */
    mesh_alloc_free(MESH_ADDR_NONE);
    CHECK(mesh_alloc_count() == MESH_MAX_SLAVES);

    /* Reset clears everything. */
    mesh_alloc_reset();
    CHECK(mesh_alloc_count() == 0);
    CHECK(!mesh_alloc_occupies(2));

    if (g_failures == 0) {
        std::printf("ALL TESTS PASSED\n");
        return 0;
    }
    std::printf("%d FAILURE(S)\n", g_failures);
    return 1;
}
