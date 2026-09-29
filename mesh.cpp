/*
 * mesh.cpp
 *
 *  LoRa mesh mode for WiRocRAK3172.
 *
 *  Persistent mesh configuration (enabled / role / own address) held in the
 *  RUI user flash partition. Mesh is disabled by default so existing P2P
 *  behaviour is unchanged until it is explicitly enabled.
 *
 *  M0: configuration + AT accessors.
 *  M1: bit-packed frame RX (validate + dedup) and a timer-drained TX queue.
 *  M2: join / automatic address assignment (JOIN_REQ / ADDR_ASSIGN / ADDR_TABLE).
 */

#include <Arduino.h>
#include <string.h>
#include "stm32wle5xx.h"        /* UID_BASE: STM32 96-bit unique device id */
#include "mesh.h"
#include "mesh_alloc.h"
#include "mesh_route.h"
#include "MessageQueue.h"

#define MESH_FLAG_ENABLED       0x01
#define MESH_FLAG_MASTER        0x02

/* ======================================================================= */
/*  Persistent configuration                                              */
/* ======================================================================= */

/* Persistent layout (6 bytes). */
struct __attribute__((packed)) mesh_flash_config_t {
    uint8_t  magic;
    uint8_t  version;
    uint8_t  flags;      /* MESH_FLAG_* */
    uint8_t  address;    /* own 5-bit address, MESH_ADDR_NONE when unassigned */
    uint16_t boot_count; /* master boot counter: bumped each boot, used as the
                          * beacon epoch so nodes detect a master restart */
};

static mesh_flash_config_t s_cfg;

static void mesh_config_defaults(void)
{
    s_cfg.magic      = MESH_FLASH_MAGIC;
    s_cfg.version    = MESH_FLASH_VERSION;
    s_cfg.flags      = 0;               /* meshing disabled, not master */
    s_cfg.address    = MESH_ADDR_NONE;
    s_cfg.boot_count = 0;
}

static bool mesh_config_load(void)
{
    mesh_flash_config_t tmp;

    if (!api.system.flash.get(MESH_FLASH_OFFSET, (uint8_t *)&tmp, sizeof(tmp))) {
        return false;
    }
    if (tmp.magic != MESH_FLASH_MAGIC || tmp.version != MESH_FLASH_VERSION) {
        return false;
    }
    if (tmp.address > MESH_ADDR_MAX) {
        return false;
    }
    s_cfg = tmp;
    return true;
}

bool mesh_config_save(void)
{
    s_cfg.magic   = MESH_FLASH_MAGIC;
    s_cfg.version = MESH_FLASH_VERSION;
    return api.system.flash.set(MESH_FLASH_OFFSET, (uint8_t *)&s_cfg, sizeof(s_cfg));
}

/* ======================================================================= */
/*  TX queue (drained by the mesh timer)                                  */
/* ======================================================================= */

typedef struct {
    uint8_t len;
    uint8_t buf[MESH_MAX_FRAME];
} mesh_frame_t;

static mesh_frame_t s_txq[MESH_TX_QUEUE_SIZE];
static uint8_t s_txq_head;
static uint8_t s_txq_tail;
static uint8_t s_txq_count;

/* ======================================================================= */
/*  (src,seq) duplicate cache                                             */
/* ======================================================================= */

typedef struct {
    uint8_t src;
    uint8_t seq;
    bool    valid;
} mesh_dedup_entry_t;

static mesh_dedup_entry_t s_dedup[MESH_DEDUP_SIZE];
static uint8_t s_dedup_idx;

/* ======================================================================= */
/*  Join / address-assignment state                                       */
/* ======================================================================= */

static mesh_state_t s_state;                    /* this node's join state     */
static uint8_t  s_token[MESH_TOKEN_LEN];        /* device identity token      */
static uint8_t  s_bcast_seq;                    /* rotating broadcast seq     */
static uint32_t s_join_next_ms;                 /* next JOIN_REQ tx time      */
static uint32_t s_join_interval_ms;             /* JOIN_REQ backoff interval  */
static uint32_t s_table_next_ms;                /* next ADDR_TABLE flood (mst)*/
static uint8_t  s_table_miss;                   /* consecutive table misses   */

/* ======================================================================= */
/*  M3: neighbour table + convergecast routing                            */
/* ======================================================================= */

static mesh_route_neighbor_t s_neigh[MESH_NEIGHBOR_MAX];
static mesh_route_t s_route;                    /* our route to the master    */
static uint32_t s_beacon_next_ms;               /* next beacon transmission   */
static uint8_t  s_data_seq;                     /* rotating seq for uplink    */
static int16_t  s_rx_rssi;                      /* last received radio metrics*/
static int8_t   s_rx_snr;

/* Last unicast frame awaiting an implicit hop ACK (M4). */
typedef struct {
    bool     active;
    uint8_t  frame[MESH_MAX_FRAME];
    uint8_t  len;
    uint8_t  src;                   /* origin address of the awaited frame    */
    uint8_t  seq;                   /* seq of the awaited frame               */
    uint8_t  retries;
    uint32_t next_ms;
} mesh_pending_t;
static mesh_pending_t s_pending;

/* --- M5: recovery / robustness ----------------------------------------- */
static uint16_t s_boot_epoch;       /* master: this boot's epoch              */
static uint16_t s_master_epoch;     /* node: last epoch heard (0 = unknown)   */
static bool     s_epoch_valid;      /* node: s_master_epoch is meaningful      */
static uint32_t s_claim_next_ms;    /* node: next periodic ADDR_CLAIM         */
static bool     s_had_parent;       /* node: had a parent on the previous tick*/
static uint32_t s_boot_ms;          /* master: uptime reference for RECOVER   */
static uint32_t s_heard_ms[MESH_ADDR_MAX + 1]; /* master: last frame per addr  */

/* Defined in the M3 section below; used by the timer. */
static void mesh_emit_beacon(void);
static void mesh_check_parent_staleness(uint32_t now);
static void mesh_deliver_to_host(const uint8_t *payload, uint8_t len, uint8_t src);
static void mesh_pending_tick(uint32_t now);

/* Defined in the M2 section below; used by the timer. */
static bool mesh_send_broadcast(uint8_t type, uint8_t hops,
                                const uint8_t *payload, uint8_t plen);

/* Defined in the M5 section below; used by the timer. */
static void mesh_send_claim(void);

/* Recompute the join FSM state from the current role / address. */
static void mesh_update_state(void)
{
    if (!mesh_is_enabled()) {
        s_state = MESH_STATE_UNASSIGNED;
    } else if (mesh_is_master() || mesh_get_address() != MESH_ADDR_NONE) {
        s_state = MESH_STATE_JOINED;
    } else {
        s_state = MESH_STATE_JOINING;
    }
}

static void mesh_state_clear(void)
{
    s_txq_head = s_txq_tail = s_txq_count = 0;
    memset(s_dedup, 0, sizeof(s_dedup));
    s_dedup_idx = 0;

    s_bcast_seq        = 0;
    s_join_next_ms     = 0;
    s_join_interval_ms = MESH_JOIN_INTERVAL_MS;
    s_table_next_ms    = 0;
    s_table_miss       = 0;

    memset(s_neigh, 0, sizeof(s_neigh));
    memset(&s_route, 0, sizeof(s_route));
    s_route.parent_addr = MESH_ADDR_NONE;
    s_beacon_next_ms = 0;
    s_data_seq       = 0;
    s_rx_rssi        = 0;
    s_rx_snr         = 0;
    memset(&s_pending, 0, sizeof(s_pending));

    /* M5 recovery state.  s_boot_epoch is *not* reset here: it is owned by
     * mesh_apply_role() and must survive an enable/disable cycle. */
    s_master_epoch   = 0;
    s_epoch_valid    = false;
    s_claim_next_ms  = 0;
    s_had_parent     = false;
    s_boot_ms        = millis();
    memset(s_heard_ms, 0, sizeof(s_heard_ms));

    mesh_alloc_reset();
    mesh_update_state();
}

/* Returns true when (src,seq) was already seen (i.e. drop as duplicate). */
static bool mesh_dedup_check(uint8_t src, uint8_t seq)
{
    for (uint8_t i = 0; i < MESH_DEDUP_SIZE; i++) {
        if (s_dedup[i].valid && s_dedup[i].src == src && s_dedup[i].seq == seq) {
            return true;
        }
    }
    s_dedup[s_dedup_idx].src   = src;
    s_dedup[s_dedup_idx].seq   = seq;
    s_dedup[s_dedup_idx].valid = true;
    s_dedup_idx = (uint8_t)((s_dedup_idx + 1) % MESH_DEDUP_SIZE);
    return false;
}

/* ======================================================================= */
/*  Timer-driven housekeeping                                             */
/* ======================================================================= */

static void mesh_tx_drain(void)
{
    if (s_txq_count == 0) {
        return;
    }
    /* Retry next tick if the radio is busy (psend returns false). */
    if (api.lora.psend(s_txq[s_txq_head].len, s_txq[s_txq_head].buf)) {
        s_txq_head = (uint8_t)((s_txq_head + 1) % MESH_TX_QUEUE_SIZE);
        s_txq_count--;
    }
}

static void mesh_timer_cb(void *)
{
    uint32_t now;

    if (!mesh_is_enabled()) {
        return;
    }
    mesh_tx_drain();

    now = millis();

    /* Beacons: the master and every routable node advertise periodically so
     * neighbours can pick parents and detect a node going down. */
    if ((int32_t)(now - s_beacon_next_ms) >= 0) {
        uint32_t interval = MESH_BEACON_INTERVAL_MS;
        if (mesh_is_master() && (uint32_t)(now - s_boot_ms) < MESH_RECOVER_MS) {
            interval = MESH_BEACON_FAST_MS; /* beacon fast while recovering */
        }
        mesh_emit_beacon();
        s_beacon_next_ms = now + interval;
    }
    mesh_check_parent_staleness(now);
    mesh_pending_tick(now);

    if (mesh_is_master()) {
        /* Recycle the address of a node we have not heard from for a long
         * time (its ADDR_CLAIM also refreshes s_heard_ms, so a live-but-idle
         * node is never evicted). */
        for (uint8_t a = MESH_FIRST_SLAVE_ADDR; a <= MESH_ADDR_MAX; a++) {
            if (mesh_alloc_occupies(a) &&
                (uint32_t)(now - s_heard_ms[a]) > MESH_EVICT_MS) {
                mesh_alloc_free(a);
            }
        }
        /* Periodically flood the occupied bitmap so nodes can reconcile. */
        if ((int32_t)(now - s_table_next_ms) >= 0) {
            uint8_t bm[4];
            mesh_alloc_bitmap(bm);
            mesh_send_broadcast(MESH_TYPE_ADDR_TABLE, MESH_DEFAULT_TTL, bm, 4);
            s_table_next_ms = now + MESH_TABLE_INTERVAL_MS;
        }
    } else {
        if (s_state == MESH_STATE_JOINING) {
            /* Ask the master for an address, backing off between attempts. */
            if ((int32_t)(now - s_join_next_ms) >= 0) {
                mesh_send_broadcast(MESH_TYPE_JOIN_REQ, MESH_DEFAULT_TTL,
                                    s_token, MESH_TOKEN_LEN);
                if (s_join_interval_ms < MESH_JOIN_INTERVAL_MAX_MS) {
                    s_join_interval_ms += MESH_JOIN_INTERVAL_MS;
                    if (s_join_interval_ms > MESH_JOIN_INTERVAL_MAX_MS) {
                        s_join_interval_ms = MESH_JOIN_INTERVAL_MAX_MS;
                    }
                }
                s_join_next_ms = now + s_join_interval_ms;
            }
        } else if (s_state == MESH_STATE_JOINED &&
                   mesh_get_address() != MESH_ADDR_NONE &&
                   (int32_t)(now - s_claim_next_ms) >= 0) {
            /* Safety net: re-announce our address so a restarted master can
             * rebuild its RAM-only table (and refresh our liveness). */
            mesh_send_claim();
        }
    }
}

static void mesh_timer_start(void)
{
    api.system.timer.create(RAK_TIMER_2, mesh_timer_cb, RAK_TIMER_PERIODIC);
    api.system.timer.start(RAK_TIMER_2, MESH_TIMER_PERIOD_MS, NULL);
}

static void mesh_timer_stop(void)
{
    api.system.timer.stop(RAK_TIMER_2);
}

/* ======================================================================= */
/*  Lifecycle + accessors                                                 */
/* ======================================================================= */

/* Device identity token for address assignment (stable across reboots).
 * Uses the STM32 96-bit unique device id (UID) read straight from the device
 * registers: it is guaranteed unique per die, so no hashing is needed.
 * (api.system.chipId.get() is NOT usable here - in this RUI3 build it returns
 * the compile-time constant chip_id "stm32wle5xx", identical on every board.) */
static void mesh_token_load(void)
{
    const volatile uint32_t *uid = (const volatile uint32_t *)UID_BASE;

    for (uint8_t i = 0; i < MESH_TOKEN_LEN; i++) {
        s_token[i] = (uint8_t)(uid[i >> 2] >> ((i & 3) * 8));
    }
}

/* The master always owns MESH_MASTER_ADDR; a demoted master must rejoin. */
static void mesh_apply_role(void)
{
    if (mesh_is_master()) {
        if (mesh_get_address() != MESH_MASTER_ADDR) {
            mesh_set_address(MESH_MASTER_ADDR);
            mesh_config_save();
        }
        /* Give this master run a fresh epoch so nodes detect the restart and
         * re-announce their flash-stored addresses. */
        if (s_boot_epoch == 0) {
            s_cfg.boot_count = (uint16_t)(s_cfg.boot_count + 1);
            if (s_cfg.boot_count == 0) {
                s_cfg.boot_count = 1;   /* wrap past the "unknown" value */
            }
            s_boot_epoch = s_cfg.boot_count;
            mesh_config_save();
        }
    } else if (mesh_get_address() == MESH_MASTER_ADDR) {
        mesh_set_address(MESH_ADDR_NONE);
        mesh_config_save();
    }
}

bool mesh_init(void)
{
    if (!mesh_config_load()) {
        /* Virgin device (or incompatible version): use defaults in RAM only.
         * We don't force a flash write here; it happens on the first change. */
        mesh_config_defaults();
    }
    mesh_token_load();
    mesh_state_clear();
    mesh_apply_role();
    mesh_update_state();
    if (mesh_is_enabled()) {
        mesh_timer_start();
    }
    return true;
}

bool mesh_is_enabled(void)
{
    return (s_cfg.flags & MESH_FLAG_ENABLED) != 0;
}

void mesh_set_enabled(bool enabled)
{
    bool was = mesh_is_enabled();

    if (enabled) {
        s_cfg.flags |= MESH_FLAG_ENABLED;
    } else {
        s_cfg.flags &= (uint8_t)~MESH_FLAG_ENABLED;
    }

    if (enabled && !was) {
        mesh_apply_role();
        mesh_state_clear();
        mesh_timer_start();
    } else if (!enabled && was) {
        mesh_timer_stop();
        mesh_state_clear();
    }
}

bool mesh_is_master(void)
{
    return (s_cfg.flags & MESH_FLAG_MASTER) != 0;
}

void mesh_set_master(bool master)
{
    if (master) {
        s_cfg.flags |= MESH_FLAG_MASTER;
    } else {
        s_cfg.flags &= (uint8_t)~MESH_FLAG_MASTER;
    }
    mesh_apply_role();
    mesh_update_state();
}

uint8_t mesh_get_address(void)
{
    return (uint8_t)(s_cfg.address & MESH_ADDR_MASK);
}

void mesh_set_address(uint8_t address)
{
    s_cfg.address = (uint8_t)(address & MESH_ADDR_MASK);
}

/* ======================================================================= */
/*  RX / TX plumbing                                                      */
/* ======================================================================= */

bool mesh_send_frame(const uint8_t *frame, uint8_t len)
{
    if (!mesh_is_enabled() || len == 0 || len > MESH_MAX_FRAME) {
        return false;
    }
    if (s_txq_count >= MESH_TX_QUEUE_SIZE) {
        return false;
    }
    s_txq[s_txq_tail].len = len;
    memcpy(s_txq[s_txq_tail].buf, frame, len);
    s_txq_tail = (uint8_t)((s_txq_tail + 1) % MESH_TX_QUEUE_SIZE);
    s_txq_count++;
    return true;
}

uint8_t mesh_tx_queue_count(void)
{
    return s_txq_count;
}

mesh_state_t mesh_get_state(void)
{
    return s_state;
}

uint8_t mesh_master_alloc_count(void)
{
    return mesh_alloc_count();
}

/* ======================================================================= */
/*  M2: join / address assignment                                         */
/* ======================================================================= */

/* Build a frame (header + optional control payload) into out; return length. */
static uint8_t mesh_build_frame(uint8_t *out, uint8_t type, uint8_t flags,
                                uint8_t src, uint8_t dst, uint8_t hops, uint8_t seq,
                                const uint8_t *payload, uint8_t plen)
{
    mesh_header_t h;

    h.version = MESH_WIRE_VERSION;
    h.type    = type;
    h.flags   = flags;
    h.src     = src;
    h.dst     = dst;
    h.hops    = hops;
    h.seq     = seq;

    mesh_wire_encode(out, &h);
    if (plen > 0 && payload != NULL) {
        memcpy(out + MESH_HEADER_SIZE, payload, plen);
    }
    return (uint8_t)(MESH_HEADER_SIZE + plen);
}

/* Broadcast a control frame (dst = NONE) with an auto-incremented seq.
 * `hops` is the beacon hop-count, or the TTL for flooded control frames. */
static uint8_t mesh_token_hash8(const uint8_t token[MESH_TOKEN_LEN]);
static void mesh_mark_seen(uint8_t type, uint8_t src, uint8_t seq, const uint8_t *payload);

static bool mesh_send_broadcast(uint8_t type, uint8_t hops,
                                const uint8_t *payload, uint8_t plen)
{
    uint8_t frame[MESH_MAX_FRAME];
    uint8_t seq = s_bcast_seq;
    uint8_t len;

    s_bcast_seq = (uint8_t)((s_bcast_seq + 1) & 0x0F);
    len = mesh_build_frame(frame, type, 0, mesh_get_address(),
                           MESH_ADDR_NONE, hops, seq, payload, plen);
    mesh_mark_seen(type, mesh_get_address(), seq, payload);
    return mesh_send_frame(frame, len);
}

/* Re-broadcast a flooded control frame once, decrementing the TTL. */
static void mesh_relay_flood(const mesh_header_t *h, const uint8_t *payload, uint8_t plen)
{
    uint8_t frame[MESH_MAX_FRAME];
    uint8_t flen;

    if (h->hops <= 1) {
        return;                         /* TTL exhausted */
    }
    flen = mesh_build_frame(frame, h->type, h->flags, h->src, h->dst,
                            (uint8_t)(h->hops - 1), h->seq, payload, plen);
    mesh_mark_seen(h->type, h->src, h->seq, payload);
    mesh_send_frame(frame, flen);
}

/* 8-bit hash of a node token, used to dedup JOIN_REQ (whose src is 0). */
static uint8_t mesh_token_hash8(const uint8_t token[MESH_TOKEN_LEN])
{
    uint32_t h = 2166136261u;           /* FNV-1a 32-bit offset basis */
    for (uint8_t i = 0; i < MESH_TOKEN_LEN; i++) {
        h ^= token[i];
        h *= 16777619u;
    }
    return (uint8_t)(h & 0xFF);
}

/* Record a frame we are about to transmit as "seen", so copies relayed back
 * to us (floods echo) are ignored and cannot loop. */
static void mesh_mark_seen(uint8_t type, uint8_t src, uint8_t seq, const uint8_t *payload)
{
    if (type == MESH_TYPE_JOIN_REQ) {
        if (payload != NULL) {
            (void)mesh_dedup_check(mesh_token_hash8(payload), seq);
        }
    } else if (type != MESH_TYPE_BEACON) {
        (void)mesh_dedup_check(src, seq);
    }
}

/* Flood ADDR_ASSIGN{ token, addr } so the requesting node learns its address. */
static void mesh_send_addr_assign(const uint8_t token[MESH_TOKEN_LEN], uint8_t addr)
{
    uint8_t out[MESH_TOKEN_LEN + 1];

    memcpy(out, token, MESH_TOKEN_LEN);
    out[MESH_TOKEN_LEN] = (uint8_t)(addr & MESH_ADDR_MASK);
    mesh_send_broadcast(MESH_TYPE_ADDR_ASSIGN, MESH_DEFAULT_TTL,
                        out, (uint8_t)(MESH_TOKEN_LEN + 1));
}

/* A joining node asks for an address (payload = token, flooded).  The master
 * allocates and floods ADDR_ASSIGN; every node relays the request onward. */
static void mesh_rx_join_req(const mesh_header_t *h, const uint8_t *payload, uint8_t plen)
{
    if (plen < MESH_TOKEN_LEN) {
        return;
    }
    if (mesh_is_master()) {
        /* After a restart the table is empty; give nodes a moment to re-claim
         * their flash addresses before handing addresses to fresh joiners. */
        if ((uint32_t)(millis() - s_boot_ms) >= MESH_RECOVER_MS) {
            uint8_t addr = mesh_alloc_assign(payload);
            if (addr != MESH_ADDR_NONE) {
                mesh_send_addr_assign(payload, addr);
            }
        }
    }
    mesh_relay_flood(h, payload, plen);
}

/* Node: adopt an address the master assigned to our token (flooded). */
static void mesh_rx_addr_assign(const mesh_header_t *h, const uint8_t *payload, uint8_t plen)
{
    uint8_t addr;

    if (plen < MESH_TOKEN_LEN + 1) {
        return;
    }
    addr = (uint8_t)(payload[MESH_TOKEN_LEN] & MESH_ADDR_MASK);
    if (!mesh_is_master()) {
        if (memcmp(payload, s_token, MESH_TOKEN_LEN) == 0) {
            if (addr >= MESH_FIRST_SLAVE_ADDR) {
                if (mesh_get_address() != addr) {
                    mesh_set_address(addr);
                    mesh_config_save();
                }
                mesh_update_state();
            }
        } else if (addr >= MESH_FIRST_SLAVE_ADDR && addr == mesh_get_address()) {
            /* Our address was handed to a different node: relinquish it and
             * re-join so no duplicate address can persist (safety net). */
            mesh_set_address(MESH_ADDR_NONE);
            mesh_config_save();
            mesh_update_state();
        }
    }
    mesh_relay_flood(h, payload, plen);
}

/* Node: reconcile our address with the master's occupied bitmap (flooded). */
static void mesh_rx_addr_table(const mesh_header_t *h, const uint8_t *payload, uint8_t plen)
{
    uint8_t addr = mesh_get_address();

    if (plen < 4) {
        return;
    }
    if (!mesh_is_master() && addr != MESH_ADDR_NONE) {
        if (payload[addr >> 3] & (uint8_t)(1u << (addr & 7))) {
            s_table_miss = 0;           /* master still knows us */
        } else if (++s_table_miss >= MESH_TABLE_MISS_LIMIT) {
            /* Our address vanished from the master's table.  A single lost
             * flood is normal, so only relinquish after several misses
             * (M5 will refine this with an explicit grace policy). */
            s_table_miss = 0;
            mesh_set_address(MESH_ADDR_NONE);
            mesh_config_save();
            mesh_update_state();
        }
    }
    mesh_relay_flood(h, payload, plen);
}

/* ======================================================================= */
/*  M3: neighbour table, parent selection and convergecast uplink         */
/* ======================================================================= */

/* Map a smoothed SNR (0.1 dB units) to a small integer link cost.  Prefers
 * an extra reliable hop over a weak direct link (see the design notes). */
static uint8_t mesh_link_cost(int16_t snr_x10)
{
    return mesh_route_link_cost(snr_x10);
}

static mesh_route_neighbor_t *mesh_neighbor_find(uint8_t addr)
{
    for (uint8_t i = 0; i < MESH_NEIGHBOR_MAX; i++) {
        if (s_neigh[i].addr == addr) {
            return &s_neigh[i];
        }
    }
    return NULL;
}

/* Find a slot for `addr`, evicting the oldest neighbour when full. */
static mesh_route_neighbor_t *mesh_neighbor_obtain(uint8_t addr)
{
    mesh_route_neighbor_t *n = mesh_neighbor_find(addr);
    mesh_route_neighbor_t *oldest = &s_neigh[0];
    uint8_t i;

    if (n != NULL) {
        return n;
    }
    for (i = 0; i < MESH_NEIGHBOR_MAX; i++) {
        if (s_neigh[i].addr == MESH_ADDR_NONE) {
            return &s_neigh[i];
        }
    }
    for (i = 1; i < MESH_NEIGHBOR_MAX; i++) {
        if ((int32_t)(s_neigh[i].last_ms - oldest->last_ms) < 0) {
            oldest = &s_neigh[i];
        }
    }
    memset(oldest, 0, sizeof(*oldest));
    return oldest;
}

uint8_t mesh_get_neighbor_count(void)
{
    uint8_t n = 0;
    for (uint8_t i = 0; i < MESH_NEIGHBOR_MAX; i++) {
        if (s_neigh[i].addr != MESH_ADDR_NONE) {
            n++;
        }
    }
    return n;
}

/* Recompute the best parent from the neighbour table (with hysteresis). */
static void mesh_select_parent(void)
{
    mesh_route_t out;

    mesh_route_select(s_neigh, MESH_NEIGHBOR_MAX, &s_route, mesh_get_address(),
                      MESH_PARENT_HYSTERESIS, &out);
    s_route = out;
}

static void mesh_check_parent_staleness(uint32_t now)
{
    uint8_t i;

    /* Drop neighbours that have gone quiet. */
    for (i = 0; i < MESH_NEIGHBOR_MAX; i++) {
        if (s_neigh[i].addr != MESH_ADDR_NONE &&
            (uint32_t)(now - s_neigh[i].last_ms) > MESH_BEACON_STALE_MS) {
            memset(&s_neigh[i], 0, sizeof(s_neigh[i]));
        }
    }
    if (s_route.parent_addr != MESH_ADDR_NONE &&
        mesh_neighbor_find(s_route.parent_addr) == NULL) {
        memset(&s_route, 0, sizeof(s_route));   /* parent lost: re-attach */
        s_route.parent_addr = MESH_ADDR_NONE;
        s_had_parent = false;                   /* claim again when we re-attach */
        mesh_select_parent();
    }
}

/* Emit this node's beacon: only routable nodes (master, or a node with a
 * parent) advertise, so advertised costs are always meaningful. */
static void mesh_emit_beacon(void)
{
    uint8_t payload[MESH_BEACON_LEN];
    uint16_t epoch;

    if (mesh_is_master()) {
        epoch = s_boot_epoch;
        payload[0] = (uint8_t)(epoch & 0xFF);
        payload[1] = (uint8_t)(epoch >> 8);
        payload[2] = 0;                 /* master: cost 0, hops 0 */
        mesh_send_broadcast(MESH_TYPE_BEACON, 0, payload, MESH_BEACON_LEN);
        return;
    }
    if (s_route.parent_addr != MESH_ADDR_NONE) {
        epoch = s_master_epoch;
        payload[0] = (uint8_t)(epoch & 0xFF);
        payload[1] = (uint8_t)(epoch >> 8);
        payload[2] = s_route.self_cost;
        mesh_send_broadcast(MESH_TYPE_BEACON, s_route.self_hops, payload, MESH_BEACON_LEN);
    }
}

/* Track the master's boot epoch.  A change (or the first one seen while we
 * already hold an address) means the master restarted, so schedule a claim. */
static void mesh_note_epoch(uint16_t epoch)
{
    bool announce = false;

    if (mesh_is_master()) {
        return;
    }
    if (!s_epoch_valid) {
        s_epoch_valid  = true;
        s_master_epoch = epoch;
        announce = (mesh_get_address() != MESH_ADDR_NONE);
    } else if (epoch != s_master_epoch) {
        s_master_epoch = epoch;
        announce = true;
    }
    if (announce && mesh_get_address() != MESH_ADDR_NONE) {
        s_claim_next_ms = 0;            /* re-announce on the next timer tick */
    }
}

static void mesh_rx_beacon(const mesh_header_t *h, const uint8_t *payload, uint8_t plen)
{
    mesh_route_neighbor_t *n;
    int16_t sample;
    uint32_t now = millis();
    uint16_t epoch;

    if (h->src == mesh_get_address() || plen < MESH_BEACON_LEN) {
        return;
    }
    epoch = (uint16_t)(payload[0] | ((uint16_t)payload[1] << 8));
    n = mesh_neighbor_obtain(h->src);
    if (n == NULL) {
        return;
    }
    if (n->addr == MESH_ADDR_NONE) {
        n->addr = h->src;
        n->snr_x10 = s_rx_snr;          /* first sample */
    } else {
        sample = (int16_t)s_rx_snr * 10;
        n->snr_x10 = (int16_t)((n->snr_x10 * 3 + sample) / 4);  /* EWMA */
    }
    n->cost      = payload[2];
    n->hops      = h->hops;
    n->link_cost = mesh_link_cost(n->snr_x10);
    n->last_ms   = now;

    mesh_check_parent_staleness(now);
    mesh_select_parent();

    /* If we re-attached after losing our parent, re-announce our address. */
    if (!mesh_is_master() && s_route.parent_addr != MESH_ADDR_NONE) {
        if (!s_had_parent && mesh_get_address() != MESH_ADDR_NONE) {
            s_claim_next_ms = 0;
        }
        s_had_parent = true;
    }
    mesh_note_epoch(epoch);
}

/* --- M4: implicit link ACK + bounded retries ---------------------------- */

/* Remember a unicast frame so the timer can retransmit it if the next hop
 * does not visibly forward it (implicit ACK) within the timeout. */
static void mesh_set_pending(const uint8_t *frame, uint8_t len, uint8_t src, uint8_t seq)
{
    memcpy(s_pending.frame, frame, len);
    s_pending.len     = len;
    s_pending.src     = src;
    s_pending.seq     = seq;
    s_pending.retries = 0;
    s_pending.active  = true;
    s_pending.next_ms = millis() + MESH_LINK_ACK_TIMEOUT_MS;
}

static void mesh_pending_tick(uint32_t now)
{
    if (!s_pending.active || (int32_t)(now - s_pending.next_ms) < 0) {
        return;
    }
    if (s_pending.retries >= MESH_LINK_RETRIES) {
        s_pending.active = false;       /* give up; the app-layer ACK covers it */
        return;
    }
    mesh_send_frame(s_pending.frame, s_pending.len);
    s_pending.retries++;
    s_pending.next_ms = now + MESH_LINK_ACK_TIMEOUT_MS;
}

bool mesh_send_uplink(const uint8_t *payload, uint8_t len)
{
    uint8_t frame[MESH_MAX_FRAME];
    uint8_t flen;
    uint8_t seq;
    bool ok;

    if (len > MESH_MAX_FRAME - MESH_HEADER_SIZE) {
        return false;
    }
    if (mesh_is_master()) {
        /* The master's own message is delivered straight to its host. */
        mesh_deliver_to_host(payload, len, MESH_MASTER_ADDR);
        return true;
    }
    if (s_state != MESH_STATE_JOINED || s_route.parent_addr == MESH_ADDR_NONE) {
        return false;                   /* no route yet */
    }
    seq = s_data_seq;
    s_data_seq = (uint8_t)((s_data_seq + 1) & 0x0F);
    flen = mesh_build_frame(frame, MESH_TYPE_DATA_UPLINK, MESH_FLAG_ACK_REQ,
                            mesh_get_address(), s_route.parent_addr,
                            MESH_DEFAULT_TTL, seq, payload, len);
    ok = mesh_send_frame(frame, flen);
    if (ok) {
        mesh_set_pending(frame, flen, mesh_get_address(), seq);
    }
    return ok;
}

static void mesh_deliver_to_host(const uint8_t *payload, uint8_t len, uint8_t src)
{
    LoraMeessage_t msg;

    if (len > sizeof(LoraMeessage_t::Buffer) || MessageQueue_isFull(&incomingMessageQueue)) {
        return;
    }
    msg.BufferSize = len;
    memcpy(msg.Buffer, payload, len);
    msg.Rssi       = s_rx_rssi;
    msg.Snr        = s_rx_snr;
    msg.Status     = 0;
    msg.SourceAddr = src;
    MessageQueue_enQueue(&incomingMessageQueue, &msg);
}

/* Explicit link ACK: the master has no next hop to forward to, so it acks the
 * uplink it just delivered by (origin,seq); relays clear on the same key. */
static void mesh_send_link_ack(uint8_t acked_src, uint8_t acked_seq)
{
    uint8_t frame[MESH_MAX_FRAME];
    uint8_t payload[2];
    uint8_t flen;

    payload[0] = acked_src;
    payload[1] = acked_seq;
    flen = mesh_build_frame(frame, MESH_TYPE_LINK_ACK, 0, mesh_get_address(),
                            MESH_ADDR_NONE, 0, 0, payload, 2);
    mesh_send_frame(frame, flen);
}

static void mesh_rx_data_uplink(const mesh_header_t *h, const uint8_t *payload, uint8_t plen)
{
    uint8_t frame[MESH_MAX_FRAME];
    uint8_t flen;

    if (mesh_is_master()) {
        mesh_deliver_to_host(payload, plen, h->src);
        if (h->flags & MESH_FLAG_ACK_REQ) {
            mesh_send_link_ack(h->src, h->seq);
        }
        return;
    }
    if (h->dst != mesh_get_address()) {
        return;                         /* not addressed to us */
    }
    if (h->hops <= 1 || s_route.parent_addr == MESH_ADDR_NONE) {
        return;                         /* TTL exhausted / no route */
    }
    flen = mesh_build_frame(frame, MESH_TYPE_DATA_UPLINK, h->flags, h->src,
                            s_route.parent_addr, (uint8_t)(h->hops - 1), h->seq,
                            payload, plen);
    if (mesh_send_frame(frame, flen)) {
        /* Wait for our parent to forward it (implicit ACK); retry otherwise. */
        mesh_set_pending(frame, flen, h->src, h->seq);
    }
}

/* Master: flood a payload down to a specific node. */
bool mesh_send_downlink(uint8_t dst, const uint8_t *payload, uint8_t len)
{
    uint8_t frame[MESH_MAX_FRAME];
    uint8_t flen;
    uint8_t seq;

    if (!mesh_is_master() || dst < MESH_FIRST_SLAVE_ADDR) {
        return false;
    }
    if (len > MESH_MAX_FRAME - MESH_HEADER_SIZE) {
        return false;
    }
    seq = s_data_seq;
    s_data_seq = (uint8_t)((s_data_seq + 1) & 0x0F);
    flen = mesh_build_frame(frame, MESH_TYPE_DATA_DOWNLINK, 0, mesh_get_address(),
                            dst, MESH_DEFAULT_TTL, seq, payload, len);
    mesh_mark_seen(MESH_TYPE_DATA_DOWNLINK, mesh_get_address(), seq, payload);
    return mesh_send_frame(frame, flen);
}

/* Node: a downlink is flooded; the target delivers it to its host, everyone
 * else relays it one hop further (bounded by TTL and dedup). */
static void mesh_rx_data_downlink(const mesh_header_t *h, const uint8_t *payload, uint8_t plen)
{
    if (h->dst == mesh_get_address()) {
        mesh_deliver_to_host(payload, plen, h->src);
        return;
    }
    if (mesh_is_master()) {
        return;                         /* the master originates downlinks */
    }
    mesh_relay_flood(h, payload, plen);
}

/* ======================================================================= */
/*  M5: recovery / robustness                                             */
/* ======================================================================= */

/* Node: re-announce our flash-stored address so a restarted master can
 * rebuild its RAM-only table (flooded, best-effort). */
static void mesh_send_claim(void)
{
    uint8_t payload[MESH_TOKEN_LEN + 1];

    if (mesh_is_master() || mesh_get_address() == MESH_ADDR_NONE) {
        return;
    }
    memcpy(payload, s_token, MESH_TOKEN_LEN);
    payload[MESH_TOKEN_LEN] = mesh_get_address();
    mesh_send_broadcast(MESH_TYPE_ADDR_CLAIM, MESH_DEFAULT_TTL,
                        payload, (uint8_t)(MESH_TOKEN_LEN + 1));
    s_claim_next_ms = millis() + MESH_CLAIM_INTERVAL_MS;
}

/* Master: rebuild the table from a node's claim.  Adopt the claimed address
 * when it is free, re-assert our existing assignment when the token is known,
 * and fall back to a fresh allocation when the address is already taken. */
static void mesh_rx_addr_claim(const mesh_header_t *h, const uint8_t *payload, uint8_t plen)
{
    uint8_t want;
    uint8_t have;

    if (plen < MESH_TOKEN_LEN + 1) {
        return;
    }
    if (mesh_is_master()) {
        want = (uint8_t)(payload[MESH_TOKEN_LEN] & MESH_ADDR_MASK);
        if (want >= MESH_FIRST_SLAVE_ADDR) {
            have = mesh_alloc_lookup(payload);
            if (have == want) {
                /* already known — nothing to do */
            } else if (have != MESH_ADDR_NONE) {
                /* token known under another address: re-assert ours */
                mesh_send_addr_assign(payload, have);
            } else if (!mesh_alloc_claim(payload, want)) {
                /* claimed address taken by another node: assign a fresh one */
                uint8_t na = mesh_alloc_assign(payload);
                if (na != MESH_ADDR_NONE) {
                    mesh_send_addr_assign(payload, na);
                }
            }
        }
    }
    mesh_relay_flood(h, payload, plen);
}

uint16_t mesh_get_epoch(void)
{
    return mesh_is_master() ? s_boot_epoch : s_master_epoch;
}

uint8_t mesh_get_parent(void)
{
    return s_route.parent_addr;
}

uint8_t mesh_get_hops(void)
{
    return s_route.self_hops;
}

uint8_t mesh_get_path_cost(void)
{
    return s_route.self_cost;
}

void mesh_handle_rx(const uint8_t *buf, uint16_t len, int16_t rssi, int8_t snr)
{
    mesh_header_t h;
    const uint8_t *payload;
    uint8_t plen;

    if (len < MESH_HEADER_SIZE) {
        return;
    }
    mesh_wire_decode(buf, &h);
    if (h.version != MESH_WIRE_VERSION) {
        return;
    }
    if (h.type >= MESH_TYPE_COUNT) {
        return;
    }
    s_rx_rssi = rssi;
    s_rx_snr  = snr;

    /* Master liveness bookkeeping: refresh the last-heard time for a node as
     * long as it sends anything (beacon, uplink or claim). */
    if (mesh_is_master() && h.src >= MESH_FIRST_SLAVE_ADDR && h.src <= MESH_ADDR_MAX) {
        s_heard_ms[h.src] = millis();
    }

    payload = buf + MESH_HEADER_SIZE;
    plen    = (uint8_t)(len - MESH_HEADER_SIZE);

    /* Link ACK (checked before dedup): clear our in-flight frame when we hear
     * it forwarded (same origin+seq, addressed elsewhere) or explicitly acked
     * by the master, which has no next hop to forward to. */
    if (s_pending.active) {
        uint8_t ack_src = 0xFF, ack_seq = 0xFF;
        if (h.type == MESH_TYPE_DATA_UPLINK) {
            ack_src = h.src;
            ack_seq = h.seq;
        } else if (h.type == MESH_TYPE_LINK_ACK && plen >= 2) {
            ack_src = payload[0];
            ack_seq = payload[1];
        }
        if (ack_src == s_pending.src && ack_seq == s_pending.seq &&
            (h.type == MESH_TYPE_LINK_ACK || h.dst != mesh_get_address())) {
            s_pending.active = false;
        }
    }

    /* Dedup flooded frames on (src,seq).  JOIN_REQ carries src=0 (unassigned)
     * so it is keyed on a hash of its token instead; beacons are link-local
     * and never deduped. */
    if (h.type == MESH_TYPE_JOIN_REQ) {
        if (plen < MESH_TOKEN_LEN || mesh_dedup_check(mesh_token_hash8(payload), h.seq)) {
            return;                     /* too short or duplicate */
        }
    } else if (h.type != MESH_TYPE_BEACON) {
        if (mesh_dedup_check(h.src, h.seq)) {
            return;                     /* duplicate */
        }
    }

    switch (h.type) {
    case MESH_TYPE_BEACON:        mesh_rx_beacon(&h, payload, plen);       break;
    case MESH_TYPE_JOIN_REQ:      mesh_rx_join_req(&h, payload, plen);     break;
    case MESH_TYPE_ADDR_ASSIGN:   mesh_rx_addr_assign(&h, payload, plen);  break;
    case MESH_TYPE_ADDR_TABLE:    mesh_rx_addr_table(&h, payload, plen);   break;
    case MESH_TYPE_DATA_UPLINK:   mesh_rx_data_uplink(&h, payload, plen);  break;
    case MESH_TYPE_DATA_DOWNLINK: mesh_rx_data_downlink(&h, payload, plen);break;
    case MESH_TYPE_LINK_ACK:      /* implicit ACK only (M4) */             break;
    case MESH_TYPE_ADDR_CLAIM:    mesh_rx_addr_claim(&h, payload, plen);   break;
    default:                                                               break;
    }
}
