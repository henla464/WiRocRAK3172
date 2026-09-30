/*
 * Host unit test for the mesh routing decision (mesh_route.cpp): link-cost
 * mapping, "prefer an extra reliable hop over a weak link", hysteresis and
 * tie-breaking.
 *
 * Build:
 *   g++ -std=c++17 -Wall -Wextra -I. test/mesh_route_test.cpp mesh_route.cpp -o /tmp/mesh_route_test
 */

#include <cstdio>
#include <cstring>
#include "mesh_route.h"

static int g_failures = 0;

#define CHECK(cond)                                                        \
    do {                                                                   \
        if (!(cond)) {                                                     \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);    \
            g_failures++;                                                  \
        }                                                                  \
    } while (0)

#define NEIGH_MAX MESH_NEIGHBOR_MAX

static void put(mesh_route_neighbor_t n[NEIGH_MAX], int i, uint8_t addr,
                int16_t snr_x10, uint8_t cost, uint8_t hops)
{
    n[i].addr      = addr;
    n[i].snr_x10   = snr_x10;
    n[i].cost      = cost;
    n[i].hops      = hops;
    n[i].link_cost = mesh_route_link_cost(snr_x10, 7);
    n[i].last_ms   = 0;
}

int main(void)
{
    mesh_route_neighbor_t n[NEIGH_MAX];
    mesh_route_t cur, out;

    /* --- link-cost mapping: SF7 reproduces the classic +5/0/-5 dB ------ */
    CHECK(mesh_route_link_cost(90, 7)  == 1);
    CHECK(mesh_route_link_cost(50, 7)  == 1);
    CHECK(mesh_route_link_cost(49, 7)  == 2);
    CHECK(mesh_route_link_cost(0, 7)   == 2);
    CHECK(mesh_route_link_cost(-1, 7)  == 3);
    CHECK(mesh_route_link_cost(-50, 7) == 3);
    CHECK(mesh_route_link_cost(-51, 7) == 6);
    CHECK(mesh_route_link_cost(-200, 7)== 6);

    /* --- thresholds shift with the SF's demodulation floor -------------- */
    /* SF5 floor -2.5 dB:  cost1 >= +10.0, cost2 >= +5.0, cost3 >= +0.0 */
    CHECK(mesh_route_link_cost(100, 5) == 1);
    CHECK(mesh_route_link_cost(99, 5)  == 2);
    CHECK(mesh_route_link_cost(50, 5)  == 2);
    CHECK(mesh_route_link_cost(49, 5)  == 3);
    CHECK(mesh_route_link_cost(0, 5)   == 3);
    CHECK(mesh_route_link_cost(-1, 5)  == 6);
    /* SF8 floor -10.0 dB: cost1 >= +2.5, cost2 >= -2.5, cost3 >= -7.5 */
    CHECK(mesh_route_link_cost(25, 8)  == 1);
    CHECK(mesh_route_link_cost(24, 8)  == 2);
    CHECK(mesh_route_link_cost(-25, 8) == 2);
    CHECK(mesh_route_link_cost(-26, 8) == 3);
    CHECK(mesh_route_link_cost(-75, 8) == 3);
    CHECK(mesh_route_link_cost(-76, 8) == 6);

    /* --- prefer a good 2-hop path over a weak direct link to the master --- */
    std::memset(n, 0, sizeof(n));
    put(n, 0, MESH_MASTER_ADDR, -100, 0, 0);   /* master, weak link: 6+0 = 6  */
    put(n, 1, 5, 80, 1, 1);                     /* relay, good, cost 1: 1+1=2  */
    std::memset(&cur, 0, sizeof(cur));
    cur.parent_addr = MESH_ADDR_NONE;
    mesh_route_select(n, NEIGH_MAX, &cur, 9, MESH_PARENT_HYSTERESIS, &out);
    CHECK(out.parent_addr == 5);
    CHECK(out.self_cost == 2);
    CHECK(out.self_hops == 2);

    /* --- no neighbours clears the route --------------------------------- */
    std::memset(n, 0, sizeof(n));
    mesh_route_select(n, NEIGH_MAX, &cur, 9, MESH_PARENT_HYSTERESIS, &out);
    CHECK(out.parent_addr == MESH_ADDR_NONE);
    CHECK(out.self_cost == 0);

    /* --- self is never selected ----------------------------------------- */
    std::memset(n, 0, sizeof(n));
    put(n, 0, 9, 100, 0, 0);
    mesh_route_select(n, NEIGH_MAX, &cur, 9, MESH_PARENT_HYSTERESIS, &out);
    CHECK(out.parent_addr == MESH_ADDR_NONE);

    /* --- hysteresis: a tie does not move the parent --------------------- */
    std::memset(n, 0, sizeof(n));
    put(n, 0, 3, 80, 1, 1);    /* 1 + 1 = 2 */
    put(n, 1, 4, 80, 1, 1);    /* 1 + 1 = 2 */
    cur.parent_addr = 3;
    cur.self_cost   = 2;
    cur.self_hops   = 2;
    mesh_route_select(n, NEIGH_MAX, &cur, 9, MESH_PARENT_HYSTERESIS, &out);
    CHECK(out.parent_addr == 3);            /* kept: no strict improvement */

    /* --- a strictly better path (by >= margin) does move the parent ----- */
    std::memset(n, 0, sizeof(n));
    put(n, 0, 3, 80, 3, 2);    /* 1 + 3 = 4 (current) */
    put(n, 1, 4, 80, 1, 1);    /* 1 + 1 = 2 (better)  */
    cur.parent_addr = 3;
    cur.self_cost   = 4;
    cur.self_hops   = 3;
    mesh_route_select(n, NEIGH_MAX, &cur, 9, MESH_PARENT_HYSTERESIS, &out);
    CHECK(out.parent_addr == 4);
    CHECK(out.self_cost == 2);
    CHECK(out.self_hops == 2);

    /* --- hysteresis boundary: 1 better is enough, 0 (tie) is not -------- */
    std::memset(n, 0, sizeof(n));
    put(n, 0, 3, 80, 1, 1);    /* 1 + 1 = 2 (current) */
    put(n, 1, 4, 80, 0, 1);    /* 1 + 0 = 1 (one better) */
    cur.parent_addr = 3;
    cur.self_cost   = 2;
    cur.self_hops   = 2;
    mesh_route_select(n, NEIGH_MAX, &cur, 9, MESH_PARENT_HYSTERESIS, &out);
    CHECK(out.parent_addr == 4);

    /* --- tie-break: equal cost -> fewer hops wins ----------------------- */
    std::memset(n, 0, sizeof(n));
    put(n, 0, 3, 80, 2, 3);    /* 1 + 2 = 3, hops 3 */
    put(n, 1, 4, 80, 2, 1);    /* 1 + 2 = 3, hops 1 */
    cur.parent_addr = MESH_ADDR_NONE;
    mesh_route_select(n, NEIGH_MAX, &cur, 9, MESH_PARENT_HYSTERESIS, &out);
    CHECK(out.parent_addr == 4);

    /* --- tie-break: equal cost & hops -> higher SNR wins ---------------- */
    std::memset(n, 0, sizeof(n));
    put(n, 0, 3, 20, 2, 1);    /* 2 + 2 = 4, snr 20 */
    put(n, 1, 4, 80, 2, 1);    /* 2 + 2 = 4, snr 80 */
    cur.parent_addr = MESH_ADDR_NONE;
    mesh_route_select(n, NEIGH_MAX, &cur, 9, MESH_PARENT_HYSTERESIS, &out);
    CHECK(out.parent_addr == 4);

    if (g_failures == 0) {
        std::printf("ALL TESTS PASSED\n");
        return 0;
    }
    std::printf("%d FAILURE(S)\n", g_failures);
    return 1;
}
