/*
 * Host unit test for the LBT threshold controller (lbt_adapt_threshold() in
 * lbt.h): step direction from the busy / retransmission rates, the
 * link-quality freeze, and the clamps around the measured noise floor and the
 * configurable minimum delta.
 *
 * Build:
 *   g++ -std=c++17 -Wall -Wextra -I. test/lbt_adapt_test.cpp -o /tmp/lbt_adapt_test
 */

#include <cstdio>
#include "lbt.h"

static int g_failures = 0;

#define CHECK(cond)                                                        \
    do {                                                                   \
        if (!(cond)) {                                                     \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);    \
            g_failures++;                                                  \
        }                                                                  \
    } while (0)

int main()
{
    const int16_t noise = -100;
    const int16_t d3 = LBT_DELTA_MIN_DEFAULT_DB;

    /* Quiet site, nothing retried: the threshold sits still. */
    CHECK(lbt_adapt_threshold(-92, noise, d3, 10, 0, true) == -92);

    /* Deferred too often, retries low -> raise: false-positive deferrals. */
    CHECK(lbt_adapt_threshold(-92, noise, d3, 60, 5, true) == -91);
    CHECK(lbt_adapt_threshold(-92, noise, d3, 60, 0, false) == -91);

    /* Rarely deferred but retrying, downlink decoded fine -> lower. */
    CHECK(lbt_adapt_threshold(-92, noise, d3, 10, 30, true) == -93);

    /* Retrying and the downlink looks bad -> link margin, not LBT: freeze. */
    CHECK(lbt_adapt_threshold(-92, noise, d3, 10, 30, false) == -92);

    /* Retrying hard and busy: the rules disagree, so hold. */
    CHECK(lbt_adapt_threshold(-92, noise, d3, 60, 30, true) == -92);

    /* Clamps: never above the cap, never closer than delta_min to the noise. */
    CHECK(lbt_adapt_threshold(-70, noise, d3, 90, 0, true) == -70);
    CHECK(lbt_adapt_threshold(-97, noise, d3, 0, 90, true) == -97);

    /* A configured minimum delta moves the lower clamp. */
    CHECK(lbt_adapt_threshold(-94, noise, 6, 0, 90, true) == -94);
    CHECK(lbt_adapt_threshold(-93, noise, 6, 0, 90, true) == -94);
    CHECK(lbt_adapt_threshold(-75, noise, 20, 0, 90, true) == -76);
    CHECK(lbt_adapt_threshold(-80, noise, 20, 0, 90, true) == -80);

    /* A site louder than the cap keeps the noise-relative margin. */
    CHECK(lbt_adapt_threshold(-50, -60, d3, 0, 0, true) == -57);
    CHECK(lbt_adapt_threshold(-50, -60, d3, 0, 90, true) == -57);
    CHECK(lbt_adapt_threshold(-50, -100, LBT_DELTA_MAX_DB, 0, 0, true) == -60);

    if (g_failures) {
        std::printf("%d check(s) failed\n", g_failures);
        return 1;
    }
    std::printf("lbt_adapt_test: all checks passed\n");
    return 0;
}
