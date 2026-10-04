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
#include "service_lora_p2p.h"   /* service_lora_p2p_get_sf()/..._get_bandwidth() */
#include "mesh.h"
#include "mesh_alloc.h"
#include "mesh_route.h"
#include "MessageQueue.h"

#define MESH_FLAG_ENABLED       0x01
#define MESH_FLAG_ROOT        0x02
#define MESH_FLAG_STANDBY       0x04    /* passive observer (M7); not the root  */

/* ======================================================================= */
/*  Persistent configuration                                              */
/* ======================================================================= */

/* Persistent layout: magic, version, flags, address, boot counter, device id. */
struct __attribute__((packed)) mesh_flash_config_t {
    uint8_t  magic;
    uint8_t  version;
    uint8_t  flags;      /* MESH_FLAG_* */
    uint8_t  address;    /* own 4-bit address, MESH_ADDR_NONE when unassigned */
    uint16_t boot_count; /* root boot counter: bumped each boot, used as the
                          * beacon epoch so nodes detect a root restart */
    uint8_t  device_id[MESH_NODE_DEVICE_ID_LEN]; /* host-provisioned device id */
};

static mesh_flash_config_t s_cfg;

static void mesh_config_defaults(void)
{
    s_cfg.magic      = MESH_FLASH_MAGIC;
    s_cfg.version    = MESH_FLASH_VERSION;
    s_cfg.flags      = 0;               /* meshing disabled, not root */
    s_cfg.address    = MESH_ADDR_NONE;
    s_cfg.boot_count = 0;
    memset(s_cfg.device_id, 0, MESH_NODE_DEVICE_ID_LEN);
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
static uint8_t  s_bcast_seq;                    /* rotating broadcast seq     */
static uint32_t s_join_next_ms;                 /* next JOIN_REQ tx time      */
static uint32_t s_join_interval_ms;             /* JOIN_REQ backoff interval  */
static uint32_t s_table_next_ms;                /* next ADDR_TABLE flood (mst)*/
static uint16_t s_table_ver_seen;               /* alloc version last flooded   */
static bool     s_table_dirty;                  /* table changed, flood pending*/
static uint32_t s_table_dirty_ms;               /* when the change was noticed */
static uint8_t  s_table_miss;                   /* consecutive table misses   */

/* ======================================================================= */
/*  M3: neighbour table + convergecast routing                            */
/* ======================================================================= */

static mesh_route_neighbor_t s_neigh[MESH_NEIGHBOR_MAX];
static mesh_route_t s_route;                    /* our route to the root    */
static uint32_t s_beacon_next_ms;               /* next beacon transmission   */
static uint32_t s_beacon_interval_ms;           /* adaptive beacon interval   */
static uint32_t s_relay_last_ms;                /* last forward for a child    */
static uint8_t  s_data_seq;                     /* rotating seq for uplink    */
static int16_t  s_rx_rssi;                      /* last received radio metrics*/
static int8_t   s_rx_snr;

/* --- Radio parameters (read from the P2P driver, used for airtime maths) --- */
static uint8_t  s_sf;                           /* active spreading factor    */
static uint8_t  s_bw;                           /* active bandwidth index     */
static uint8_t  s_cr;                           /* coding rate index (1..4)   */
static uint16_t s_ppl;                          /* preamble length            */
static bool     s_lde;                          /* low-datarate optimize (DE) */

/* Control-plane overhead accounting (TX airtime, ms).  s_air_ctrl counts every
 * non-data frame we transmit; s_air_data counts the data frames. */
static uint32_t s_air_ctrl;
static uint32_t s_air_data;

/* --- Channel access (listen before talk) -------------------------------- */
/* After a busy channel the queue drain holds off until s_tx_next_ms, drawing a
 * fresh random window from s_tx_backoff_ms (which doubles per busy attempt). */
static uint32_t s_tx_next_ms;                   /* earliest next drain attempt */
static uint32_t s_tx_backoff_ms;                /* current backoff window      */
static uint32_t s_rand_state;                   /* xorshift PRNG state         */
static bool     s_tx_fast_armed;                /* fast-drain one-shot is armed */

/* Last unicast frame awaiting a hop ACK (M4).  One slot per direction: a relay
 * in the middle of the tree can have an uplink and a downlink in flight at the
 * same time. */
enum { MESH_PEND_UP = 0, MESH_PEND_DOWN, MESH_PEND_N };
typedef struct {
    bool     active;
    uint8_t  frame[MESH_MAX_FRAME];
    uint8_t  len;
    uint8_t  src;                   /* origin address of the awaited frame    */
    uint8_t  seq;                   /* seq of the awaited frame               */
    uint8_t  retries;
    uint32_t next_ms;
} mesh_pending_t;
static mesh_pending_t s_pending[MESH_PEND_N];

/* --- M5: recovery / robustness ----------------------------------------- */
static uint16_t s_boot_epoch;       /* root: this boot's epoch              */
static uint16_t s_root_epoch;     /* node: last epoch heard (0 = unknown)   */
static bool     s_epoch_valid;      /* node: s_root_epoch is meaningful      */
static bool     s_claim_due;        /* node: an ADDR_CLAIM is pending (event)  */
static uint32_t s_alive_next_ms;    /* node: next periodic liveness aggregate  */
static bool     s_had_parent;       /* node: had a parent on the previous tick*/
static uint32_t s_boot_ms;          /* root: uptime reference for RECOVER   */
static uint32_t s_heard_ms[MESH_ADDR_MAX + 1]; /* root: last frame per addr  */
/* Step-2/3 liveness aggregation: per-child subtree bitmap (bit a = "address a
 * is alive in that child's subtree"), when that bitmap was last received, and
 * the last time we heard the child at all.  A child is current while
 * s_child_heard_ms[addr] is within MESH_ALIVE_CHILD_HOLD_MS; its bitmap is
 * trusted only while s_child_bm_ms[addr] is too -- otherwise it is reported as
 * just itself, because a child that stopped sending aggregates may have become
 * a leaf (its subtree gone).  heard_ms == 0 means "not a child".  Indexed by
 * address (2..15). */
static uint16_t s_child_bm[MESH_ADDR_MAX + 1];       /* last subtree bitmap      */
static uint32_t s_child_bm_ms[MESH_ADDR_MAX + 1];    /* when that bitmap arrived */
static uint32_t s_child_heard_ms[MESH_ADDR_MAX + 1]; /* last heard as our child  */
/* Root only (Step 3): the direct child whose liveness aggregate last covered
 * each address (0 = none).  A dead direct child takes its whole covered set with
 * it -- subtree-scoped eviction. */
static uint8_t  s_cover[MESH_ADDR_MAX + 1];

/* M6 topology map (root only): each node reports its parent and the
 * neighbours it hears, rootward along the tree, so the root can stitch the
 * whole graph together.  A node with no report yet (age 0) is absent from the
 * map; a row older than MESH_TOPO_HOLD_MS is dropped.  The root's own row
 * (addr 1) comes from its live neighbour table, not from a report. */
#define MESH_TOPO_NO_LINK   0xFF                    /* s_topo_cost: no link */
static uint8_t  s_topo_parent[MESH_ADDR_MAX + 1];   /* reported parent (0=none) */
static uint32_t s_topo_age_ms[MESH_ADDR_MAX + 1];   /* when the report arrived */
static uint16_t s_topo_link[MESH_ADDR_MAX + 1];     /* bit b = "hears addr b"   */
static uint8_t  s_topo_cost[MESH_ADDR_MAX + 1][MESH_ADDR_MAX + 1]; /* link cost */

/* Node only (M6): a topology report is pending (parent / neighbour change) and
 * the next backstop time. */
static bool     s_topo_due;
static uint32_t s_topo_next_ms;

/* --- M7: standby root (silent shadow root) --------------------------- */
/* A standby transmits nothing (single silence gate in mesh_tx_radio_send).  It
 * otherwise mirrors the root: it shadows the address <-> device-id map and
 * topology from the rootward control frames it overhears, mirrors uplinks to its
 * host, and -- if the active root stops answering -- promotes itself (see the
 * M7 section).  s_root_alive_ms is refreshed by every frame from the root and
 * drives the backstop; the probe tracks one overheard uplink at a time, expecting
 * the application ACK (a downlink back to the origin) within PROBE_MS. */
static bool     s_root_seen;      /* a root frame has been heard (gates probe)*/
static uint32_t s_root_alive_ms;  /* last frame heard from the root         */
static bool     s_probe_active;     /* an uplink probe is in flight             */
static uint8_t  s_probe_src;        /* the probed uplink's origin               */
static uint32_t s_probe_deadline;   /* when an unconfirmed probe becomes a miss  */
static uint8_t  s_probe_miss;       /* consecutive unacked probes               */
static uint32_t s_probe_first_ms;   /* when the current miss run started        */
/* Node only (Step 4) liveness monitor, armed at depth >= 2: we overhear our
 * parent's outgoing ADDR_ALIVE aggregate (a unicast to our grandparent, which
 * LoRa's shared medium lets us see) to confirm it still carries our bit
 * upstream.  s_parent_alive_ms = 0 means "no aggregate overheard yet". */
static uint8_t  s_mon_parent;       /* parent whose aggregate we are watching  */
static uint32_t s_mon_parent_ms;    /* when we adopted that parent             */
static uint32_t s_parent_alive_ms;  /* last parent aggregate overheard (0=none)*/
static uint32_t s_mon_claim_ms;     /* last monitor-triggered re-claim (0=none)*/

/* --- M8: root conflict (lowest device id wins) ----------------------- */
/* A (re)booted root does not serve until it has proven it is alone (or the
 * senior of the two): it starts "contending", broadcasting ROOT_QUERY and
 * listening for a rival before it beacons/answers joins.  s_root_serving is
 * false while contending; s_contend_until_ms == 0 means "not yet armed". */
static bool     s_root_serving;       /* root passed the solo check          */
static uint32_t s_contend_until_ms;     /* probation deadline (0 = not armed)     */
static uint32_t s_contend_next_ms;      /* next ROOT_QUERY retry                */

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
static void mesh_send_alive(void);
static void mesh_send_topology(void);
static void mesh_monitor_tick(uint32_t now);

/* Defined in the M7 section below; used by the timer. */
static void mesh_standby_tick(uint32_t now);
static void mesh_standby_takeover(void);

/* Defined in the M8 section below; used by the timer / RX. */
static void mesh_root_contend_tick(uint32_t now);
static void mesh_rx_root_query(const mesh_header_t *h, const uint8_t *payload, uint8_t plen);
static void mesh_rx_root_announce(const mesh_header_t *h, const uint8_t *payload, uint8_t plen);
static void mesh_root_demote_to_standby(void);
static void mesh_root_announce_send(void);

/* Fast forward path: a point-to-point frame is drained by a one-shot timer
 * rather than waiting for the next housekeeping tick (see mesh_tx_schedule). */
static void mesh_tx_schedule(void);
static void mesh_tx_fast_cb(void *);

/* Radio/beacon helpers defined further down; used by the timer. */
static void mesh_refresh_radio_params(void);
static uint32_t mesh_frame_airtime_ms(uint8_t len);
static uint32_t mesh_link_ack_timeout_for(uint8_t len);
static void mesh_beacon_fast(void);
static bool mesh_is_relay(void);

/* Recompute the join FSM state from the current role / address / device id. */
static void mesh_update_state(void)
{
    if (!mesh_is_enabled() || !mesh_has_node_device_id() ||
        (s_cfg.flags & MESH_FLAG_STANDBY)) {
        /* Without a host-provisioned node device id the mesh does not run: no
         * beacons and no join attempt.  It is how the root identifies us.
         * A standby root (M7) is an observer, not a member of the tree. */
        s_state = MESH_STATE_UNASSIGNED;
    } else if (mesh_is_root() || mesh_get_address() != MESH_ADDR_NONE) {
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
    s_beacon_interval_ms = MESH_BEACON_FAST_MS;  /* start fast, back off */
    s_relay_last_ms  = millis() - MESH_RELAY_HOLD_MS;  /* a fresh node is a leaf */
    s_data_seq       = 0;
    s_rx_rssi        = 0;
    s_rx_snr         = 0;
    s_air_ctrl       = 0;
    s_air_data       = 0;
    s_tx_next_ms     = 0;
    s_tx_backoff_ms  = MESH_TX_BACKOFF_MIN_MS;
    s_tx_fast_armed  = false;
    s_rand_state     = 0x9E3779B9u ^ millis();  /* per-node, non-zero */
    if (s_rand_state == 0) {
        s_rand_state = 0x1234567u;
    }
    memset(s_pending, 0, sizeof(s_pending));

    mesh_refresh_radio_params();

    /* M5 recovery state.  s_boot_epoch is *not* reset here: it is owned by
     * mesh_apply_role() and must survive an enable/disable cycle. */
    s_root_epoch   = 0;
    s_epoch_valid    = false;
    s_claim_due      = false;
    s_alive_next_ms  = 0;
    s_had_parent     = false;
    s_boot_ms        = millis();
    memset(s_heard_ms, 0, sizeof(s_heard_ms));
    memset(s_child_bm, 0, sizeof(s_child_bm));
    memset(s_child_bm_ms, 0, sizeof(s_child_bm_ms));
    memset(s_child_heard_ms, 0, sizeof(s_child_heard_ms));
    memset(s_cover, 0, sizeof(s_cover));
    memset(s_topo_parent, 0, sizeof(s_topo_parent));
    memset(s_topo_age_ms, 0, sizeof(s_topo_age_ms));
    memset(s_topo_link, 0, sizeof(s_topo_link));
    memset(s_topo_cost, MESH_TOPO_NO_LINK, sizeof(s_topo_cost));
    s_topo_due        = false;
    s_topo_next_ms    = 0;

    /* M7 standby state.  s_root_alive_ms starts at the boot reference so the
     * backstop also fires for a standby that boots into a dead network. */
    s_root_seen     = false;
    s_root_alive_ms = millis();
    s_probe_active    = false;
    s_probe_src       = MESH_ADDR_NONE;
    s_probe_deadline  = 0;
    s_probe_miss      = 0;
    s_probe_first_ms  = 0;

    s_mon_parent      = MESH_ADDR_NONE;
    s_mon_parent_ms   = 0;
    s_parent_alive_ms = 0;
    s_mon_claim_ms    = 0;

    /* M8 root conflict.  A root (re)enters its solo check on every fresh
     * start; a non-root never runs it (guarded by mesh_is_root()). */
    s_root_serving  = false;
    s_contend_until_ms = 0;
    s_contend_next_ms  = 0;

    mesh_alloc_reset();
    s_table_ver_seen = mesh_alloc_version();    /* no pending flood after reset */
    s_table_dirty    = false;
    s_table_dirty_ms = 0;
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

/* Small xorshift PRNG: makes the retry backoff random per node so units that
 * collided do not retry in lockstep. */
static uint32_t mesh_rand(void)
{
    uint32_t x = s_rand_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    s_rand_state = x;
    return x;
}

/* Draw a delay in [window/2, window] ms from the current backoff window. */
static uint32_t mesh_tx_backoff_draw(void)
{
    uint32_t half = s_tx_backoff_ms / 2;
    return half + (mesh_rand() % (half + 1));
}

/* Transmit one frame *now* with CAD forced on (listen before talk), whatever
 * the module's persisted CAD setting is.  Returns false when the radio is busy
 * or CAD found the channel busy; the caller decides whether to back off (queued
 * mesh traffic) or report busy to the host (host-originated traffic). */
static bool mesh_tx_radio_send(const uint8_t *frame, uint8_t len)
{
    mesh_header_t h;
    uint32_t air;

    /* THE silence guarantee: a passive standby never puts anything on air.
     * Every transmit path funnels through here (the only api.lora.psend caller),
     * so this single gate is what makes the observer silent -- regardless of
     * which RX handlers or housekeeping it runs. */
    if (mesh_is_standby()) {
        return false;
    }
    if (!api.lora.psend(len, (uint8_t *)frame, true)) {
        return false;
    }
    /* Overhead accounting: bucket the frame's airtime as data or control. */
    mesh_wire_decode(frame, &h);
    air = mesh_frame_airtime_ms(len);
    if (h.type == MESH_TYPE_DATA_UPLINK || h.type == MESH_TYPE_DATA_DOWNLINK) {
        s_air_data += air;
    } else {
        s_air_ctrl += air;
    }
    return true;
}

static void mesh_tx_drain(void)
{
    uint32_t now;

    if (s_txq_count == 0) {
        return;
    }
    now = millis();
    if ((int32_t)(now - s_tx_next_ms) < 0) {
        return;                             /* still in the post-busy backoff */
    }
    if (mesh_tx_radio_send(s_txq[s_txq_head].buf, s_txq[s_txq_head].len)) {
        s_txq_head = (uint8_t)((s_txq_head + 1) % MESH_TX_QUEUE_SIZE);
        s_txq_count--;
        s_tx_backoff_ms = MESH_TX_BACKOFF_MIN_MS;   /* reset after a clean send */
        return;
    }
    /* Channel busy: wait a randomised window (doubling per consecutive busy
     * attempt) before trying the same frame again. */
    s_tx_next_ms = now + mesh_tx_backoff_draw();
    if (s_tx_backoff_ms < MESH_TX_BACKOFF_MAX_MS) {
        s_tx_backoff_ms *= 2;
        if (s_tx_backoff_ms > MESH_TX_BACKOFF_MAX_MS) {
            s_tx_backoff_ms = MESH_TX_BACKOFF_MAX_MS;
        }
    }
}

/* Arm the fast-drain one-shot when there is queued point-to-point work.  Called
 * when such a frame is queued, and again after every fast drain (to send the
 * rest of the queue, or to wait out a busy backoff).  Broadcasts never reach
 * here -- they stay on the periodic tick, whose per-node phase spreads the
 * relays.  Arming is idempotent: a second queued frame does not push the
 * already-scheduled drain further out. */
static void mesh_tx_schedule(void)
{
    uint32_t now, delay;

    if (s_tx_fast_armed || !mesh_is_enabled() || s_txq_count == 0) {
        return;
    }
    now = millis();
    if ((int32_t)(now - s_tx_next_ms) < 0) {
        delay = (uint32_t)(s_tx_next_ms - now);   /* hold off: channel was busy */
    } else {
        delay = MESH_TX_FAST_MS + (mesh_rand() % MESH_TX_FAST_JITTER_MS);
    }
    if (api.system.timer.create(RAK_TIMER_3, mesh_tx_fast_cb, RAK_TIMER_ONESHOT) &&
        api.system.timer.start(RAK_TIMER_3, delay, NULL)) {
        s_tx_fast_armed = true;
    }
}

static void mesh_tx_fast_cb(void *)
{
    s_tx_fast_armed = false;
    mesh_tx_drain();            /* one frame, or a fresh busy backoff          */
    mesh_tx_schedule();         /* more queued, or the backoff to wait out     */
}

static void mesh_timer_cb(void *)
{
    uint32_t now;

    if (!mesh_is_enabled() || !mesh_has_node_device_id()) {
        return;                         /* every mesh device carries a device id */
    }
    mesh_refresh_radio_params();
    mesh_tx_drain();

    now = millis();

    /* Standby root (M7): a silent shadow root.  Run its liveness detector,
     * then fall through the common flow -- a standby is not the root, so it
     * skips the root-only housekeeping below; anything it would transmit is
     * dropped by the silence gate. */
    if (mesh_is_standby()) {
        mesh_standby_tick(now);
    } else if (mesh_is_root() && !s_root_serving) {
        /* A root that has not yet passed its solo check (M8) serves nothing:
         * it only probes for a rival, then asserts or stands down. */
        mesh_root_contend_tick(now);
        return;
    }

    /* Beacons: the root and every routable node advertise periodically so
     * neighbours can pick parents and detect a node going down.  The interval
     * backs off toward MESH_BEACON_MAX_MS while stable, and is reset fast on
     * any topology change (join, re-attach, epoch change). */
    if ((int32_t)(now - s_beacon_next_ms) >= 0) {
        bool recovering = mesh_is_root() &&
                          (uint32_t)(now - s_boot_ms) < MESH_RECOVER_MS;
        uint32_t interval;

        if (recovering) {
            interval = MESH_BEACON_FAST_MS;
        } else {
            interval = s_beacon_interval_ms;
            /* A leaf (no children) is on nobody's path: it only needs to
             * advertise itself as a potential parent occasionally. */
            if (!mesh_is_root() && !mesh_is_relay()) {
                interval *= MESH_BEACON_LEAF_MULT;
            }
        }

        mesh_emit_beacon();
        s_beacon_next_ms = now + interval;

        if (!recovering && s_beacon_interval_ms < MESH_BEACON_MAX_MS) {
            s_beacon_interval_ms *= 2;
            if (s_beacon_interval_ms > MESH_BEACON_MAX_MS) {
                s_beacon_interval_ms = MESH_BEACON_MAX_MS;
            }
        }
    }
    mesh_check_parent_staleness(now);
    mesh_monitor_tick(now);
    mesh_pending_tick(now);

    if (mesh_is_root()) {
        /* Subtree-scoped eviction: a known direct child whose liveness aggregate
         * has been silent for MESH_EVICT_MS takes its whole covered subtree with
         * it, so a dead branch expires as one coherent event rather than as N
         * independent per-address timers.  s_child_heard_ms is refreshed by the
         * child's aggregates and by its beacons (mesh_rx_alive / mesh_rx_beacon),
         * so a child that merely stopped being a relay -- and went quiet on the
         * aggregate timer -- is not mistaken for a dead one. */
        for (uint8_t c = MESH_FIRST_NONROOT_ADDR; c <= MESH_ADDR_MAX; c++) {
            if (s_child_heard_ms[c] == 0 ||
                (uint32_t)(now - s_child_heard_ms[c]) <= MESH_EVICT_MS) {
                continue;
            }
            for (uint8_t a = MESH_FIRST_NONROOT_ADDR; a <= MESH_ADDR_MAX; a++) {
                if (s_cover[a] == c) {
                    mesh_alloc_free(a);
                    s_cover[a]    = MESH_ADDR_NONE;
                    s_heard_ms[a] = now;    /* neutral until re-allocated */
                }
            }
            s_child_heard_ms[c] = 0;
            s_child_bm[c]       = 0;
            s_child_bm_ms[c]    = 0;
        }
        /* Backstop: any still-allocated address that has itself been silent (a
         * leaf directly under the root node has no covering direct child). */
        for (uint8_t a = MESH_FIRST_NONROOT_ADDR; a <= MESH_ADDR_MAX; a++) {
            if (mesh_alloc_occupies(a) &&
                (uint32_t)(now - s_heard_ms[a]) > MESH_EVICT_MS) {
                mesh_alloc_free(a);
                s_cover[a]    = MESH_ADDR_NONE;
                s_heard_ms[a] = now;        /* neutral until re-allocated */
            }
        }
        /* Flood the occupied bitmap when the table actually changed (an address
         * was allocated, claimed or recycled), coalescing a burst of changes
         * into one flood, and otherwise only on the slow backstop timer. */
        {
            uint16_t ver = mesh_alloc_version();
            if (ver != s_table_ver_seen && !s_table_dirty) {
                s_table_dirty    = true;    /* start the coalescing window */
                s_table_dirty_ms = now;
            }
            if ((int32_t)(now - s_table_next_ms) >= 0 ||
                (s_table_dirty &&
                 (uint32_t)(now - s_table_dirty_ms) >= MESH_TABLE_DEBOUNCE_MS)) {
                uint8_t bm[2];
                mesh_alloc_bitmap(bm);
                mesh_send_broadcast(MESH_TYPE_ADDR_TABLE, MESH_DEFAULT_TTL, bm, 2);
                s_table_ver_seen = ver;
                s_table_dirty    = false;
                s_table_next_ms  = now + MESH_TABLE_INTERVAL_MS;
            }
        }
    } else {
        if (s_state == MESH_STATE_JOINING) {
            /* Ask the root for an address, backing off between attempts. */
            if ((int32_t)(now - s_join_next_ms) >= 0) {
                mesh_send_broadcast(MESH_TYPE_JOIN_REQ, MESH_DEFAULT_TTL,
                                    s_cfg.device_id, MESH_NODE_DEVICE_ID_LEN);
                if (s_join_interval_ms < MESH_JOIN_INTERVAL_MAX_MS) {
                    s_join_interval_ms += MESH_JOIN_INTERVAL_MS;
                    if (s_join_interval_ms > MESH_JOIN_INTERVAL_MAX_MS) {
                        s_join_interval_ms = MESH_JOIN_INTERVAL_MAX_MS;
                    }
                }
                s_join_next_ms = now + s_join_interval_ms;
            }
        } else if (s_state == MESH_STATE_JOINED &&
                   mesh_get_address() != MESH_ADDR_NONE) {
            /* Event-driven claim: (re)bind our address to our device id so a
             * restarted root can rebuild its RAM-only table.  Set by an epoch
             * change, a boot, or a re-attach -- never on a timer. */
            if (s_claim_due) {
                mesh_send_claim();
                s_claim_due = false;
            }
            /* Periodic liveness: a subtree aggregate a deep idle node would
             * otherwise never get to the root (its beacon is link-local).
             * Rate is set by MESH_ALIVE_INTERVAL_MS, which the eviction window
             * (MESH_EVICT_MS) must stay comfortably above. */
            if ((int32_t)(now - s_alive_next_ms) >= 0) {
                mesh_send_alive();
            }
            /* Topology report: event-driven (attach / parent change) and a
             * slow backstop, so the root's node/link map stays current. */
            if (s_topo_due || (int32_t)(now - s_topo_next_ms) >= 0) {
                mesh_send_topology();
                s_topo_due = false;
            }
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
    if (s_tx_fast_armed) {
        api.system.timer.stop(RAK_TIMER_3);
        s_tx_fast_armed = false;
    }
}

/* ======================================================================= */
/*  Lifecycle + accessors                                                 */
/* ======================================================================= */

/* --- Radio parameters + LoRa airtime ------------------------------------ */

/* Bandwidth enum index (RAK) to Hz.  7 == 31.25 kHz ("32 kHz" narrowband). */
static uint32_t mesh_bw_hz(uint8_t idx)
{
    switch (idx) {
    case 0: return 125000;  case 1: return 250000;  case 2: return 500000;
    case 3: return 7800;    case 4: return 10400;   case 5: return 15600;
    case 6: return 20800;   case 7: return 31250;   case 8: return 41700;
    case 9: return 62500;   default: return 125000;
    }
}

static void mesh_refresh_radio_params(void)
{
    s_sf  = service_lora_p2p_get_sf();
    s_bw  = (uint8_t)service_lora_p2p_get_bandwidth();
    s_cr  = service_lora_p2p_get_codingrate();
    s_ppl = service_lora_p2p_get_preamlen();
    s_lde = service_lora_p2p_get_low_datarate_optimize();
}

/* LoRa frame airtime in ms (CR 4/(4+cr), CRC on), from the active radio params.
 * Tsym = 2^SF / BW; n = 8 + max(ceil((8L - 4SF + 28 + 16)/(4(SF - 2DE))),0)*(4+CR). */
static uint32_t mesh_frame_airtime_ms(uint8_t len)
{
    uint32_t bw = mesh_bw_hz(s_bw);
    uint32_t sf = (s_sf >= 5) ? s_sf : 7;
    uint32_t cr = (s_cr >= 1 && s_cr <= 4) ? s_cr : 1;
    uint64_t tsym_ns = ((uint64_t)1 << sf) * 1000000000ull / bw;
    uint8_t  de = (s_lde || tsym_ns > 16000000ull) ? 1u : 0u;
    int32_t  num = 8 * (int32_t)len - 4 * (int32_t)sf + 28 + 16; /* CRC on */
    uint32_t den = 4 * (sf - 2 * de);
    int32_t  pl = (num > 0 && den > 0) ? (num + (int32_t)den - 1) / (int32_t)den : 0;
    uint32_t n_payload = 8 + (uint32_t)pl * (cr + 4);
    uint32_t n_total   = (s_ppl >= 5 ? s_ppl : 8) + 4;  /* preamble + ~4.25 hdr */

    return (uint32_t)(((uint64_t)(n_total + n_payload) * tsym_ns) / 1000000ull + 1u);
}

/* Implicit-ACK wait for a frame of `len` bytes: ~2x its airtime plus a tick,
 * floored so very fast datarates still leave room for a reply. */
static uint32_t mesh_link_ack_timeout_for(uint8_t len)
{
    uint32_t t = 2 * mesh_frame_airtime_ms(len) + MESH_TIMER_PERIOD_MS;
    return (t < 500u) ? 500u : t;
}

uint32_t mesh_get_link_ack_timeout_ms(void)
{
    return mesh_link_ack_timeout_for((uint8_t)(MESH_HEADER_SIZE + 32));
}

uint16_t mesh_get_overhead_pct(void)
{
    uint64_t pct;
    if (s_air_data == 0) {
        return 0;
    }
    pct = (uint64_t)s_air_ctrl * 100u / s_air_data;
    return (pct > 65535u) ? 65535u : (uint16_t)pct;
}

/* Reset the adaptive beacon to the fast interval (emit on the next tick). */
static void mesh_beacon_fast(void)
{
    s_beacon_interval_ms = MESH_BEACON_FAST_MS;
    s_beacon_next_ms     = 0;
}

/* A node is a relay while a child has recently declared itself to us: a
 * rootward unicast (uplink or claim) addressed to us, a periodic ADDR_ALIVE
 * aggregate addressed to us, or a beacon from a child we already know.  Only a
 * node that selected us as its parent ever does that.  A childless node is a
 * leaf and beacons more slowly. */
static bool mesh_is_relay(void)
{
    return (uint32_t)(millis() - s_relay_last_ms) < MESH_RELAY_HOLD_MS;
}

/* Record that a child just routed through us: refresh its liveness entry and
 * our relay status.  `child` is the child's address (the header `src` of the
 * rootward unicast or liveness aggregate it sent us).  On the leaf->relay
 * transition, reset the beacon to the normal rate so the new child can track us
 * promptly, and fire a liveness aggregate at once (we now have a subtree to
 * publish). */
static void mesh_note_relay(uint8_t child)
{
    bool was = mesh_is_relay();

    s_relay_last_ms = millis();
    if (child >= MESH_FIRST_NONROOT_ADDR && child <= MESH_ADDR_MAX) {
        if (s_child_heard_ms[child] == 0) {
            s_child_bm[child]    = (uint16_t)(1u << child); /* at least itself */
            s_child_bm_ms[child] = 0;                       /* no subtree yet  */
            if (was) {
                /* A new child under an existing relay: publish the updated
                 * subtree promptly, so the root (liveness) and our parent
                 * (downlink steering) can reach it without waiting a whole
                 * aggregate interval. */
                s_alive_next_ms = 0;
            }
        }
        s_child_heard_ms[child] = millis();
    }
    if (!was) {
        mesh_beacon_fast();
        s_alive_next_ms = 0;            /* publish the new subtree promptly */
    }
}

/* The root always owns MESH_ROOT_ADDR; a demoted root must rejoin. */
static void mesh_apply_role(void)
{
    if (mesh_is_root()) {
        if (mesh_get_address() != MESH_ROOT_ADDR) {
            mesh_set_address(MESH_ROOT_ADDR);
            mesh_config_save();
        }
        /* Give this root run a fresh epoch so nodes detect the restart and
         * re-announce their flash-stored addresses. */
        if (s_boot_epoch == 0) {
            s_cfg.boot_count = (uint16_t)(s_cfg.boot_count + 1);
            if (s_cfg.boot_count == 0) {
                s_cfg.boot_count = 1;   /* wrap past the "unknown" value */
            }
            s_boot_epoch = s_cfg.boot_count;
            mesh_config_save();
        }
    } else if (mesh_get_address() == MESH_ROOT_ADDR) {
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

bool mesh_is_root(void)
{
    return (s_cfg.flags & MESH_FLAG_ROOT) != 0;
}

void mesh_set_root(bool root)
{
    bool was = mesh_is_root();

    if (root) {
        s_cfg.flags |= MESH_FLAG_ROOT;
        s_cfg.flags &= (uint8_t)~MESH_FLAG_STANDBY; /* roles are exclusive */
    } else {
        s_cfg.flags &= (uint8_t)~MESH_FLAG_ROOT;
    }
    if (root != was) {
        /* A role change restarts the solo check: a fresh root must re-prove
         * it is alone before it serves. */
        s_root_serving   = false;
        s_contend_until_ms = 0;
    }
    mesh_apply_role();
    mesh_update_state();
}

bool mesh_is_standby(void)
{
    return (s_cfg.flags & MESH_FLAG_STANDBY) != 0;
}

void mesh_set_standby(bool standby)
{
    if (standby) {
        s_cfg.flags |= MESH_FLAG_STANDBY;
        s_cfg.flags &= (uint8_t)~MESH_FLAG_ROOT;  /* roles are exclusive */
        /* An observer holds no address and is not a tree member. */
        s_cfg.address = MESH_ADDR_NONE;
        s_root_serving = false;       /* no longer a serving root         */
        mesh_state_clear();
    } else {
        s_cfg.flags &= (uint8_t)~MESH_FLAG_STANDBY;
    }
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

/* The active root and a passive standby both maintain the root-owned tables
 * (address <-> device-id map, topology, liveness): a standby is a "shadow root"
 * that mirrors the root's state without transmitting (mesh_tx_radio_send drops
 * every frame it would send). */
static bool mesh_at_root(void)
{
    return mesh_is_root() || mesh_is_standby();
}

/* True when a frame's header `dst` addresses this node.  A standby additionally
 * treats the root's well-known address as its own, so it absorbs the rootward
 * traffic that reaches the root on its final hop (claims, liveness aggregates,
 * topology reports) even though it holds no address of its own. */
static bool mesh_addressed_to_me(uint8_t dst)
{
    return dst != MESH_ADDR_NONE &&
           (dst == mesh_get_address() ||
            (mesh_is_standby() && dst == MESH_ROOT_ADDR));
}

bool mesh_has_node_device_id(void)
{
    for (uint8_t i = 0; i < MESH_NODE_DEVICE_ID_LEN; i++) {
        if (s_cfg.device_id[i] != 0) {
            return true;
        }
    }
    return false;
}

void mesh_set_node_device_id(const uint8_t *device_id)
{
    if (device_id != NULL) {
        memcpy(s_cfg.device_id, device_id, MESH_NODE_DEVICE_ID_LEN);
    } else {
        memset(s_cfg.device_id, 0, MESH_NODE_DEVICE_ID_LEN);
    }
    /* A node device id appearing can start the join; losing it stops the mesh. */
    mesh_update_state();
}

void mesh_get_node_device_id(uint8_t out[MESH_NODE_DEVICE_ID_LEN])
{
    memcpy(out, s_cfg.device_id, MESH_NODE_DEVICE_ID_LEN);
}

bool mesh_lookup_device_id(uint8_t addr, uint8_t out[MESH_NODE_DEVICE_ID_LEN])
{
    if (addr == MESH_ADDR_NONE) {
        return false;
    }
    /* A node always knows its own device id (its own address is its own row). */
    if (addr == mesh_get_address()) {
        if (!mesh_has_node_device_id()) {
            return false;
        }
        mesh_get_node_device_id(out);
        return true;
    }
    /* The active root holds the full map; a standby holds a best-effort shadow
     * of it (learned from overheard ADDR_ASSIGN / ADDR_CLAIM frames). */
    if (!mesh_at_root()) {
        return false;
    }
    return mesh_alloc_device_id(addr, out);
}

/* ======================================================================= */
/*  RX / TX plumbing                                                      */
/* ======================================================================= */

bool mesh_send_frame(const uint8_t *frame, uint8_t len)
{
    mesh_header_t h;

    if (mesh_is_standby()) {
        return false;                   /* an observer enqueues nothing (never transmits) */
    }
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

    /* A point-to-point frame (dst set: data, link-ack, claim, topology) skips
     * the housekeeping tick and is drained by a one-shot timer a few ms from
     * now.  Broadcasts / floods (dst = NONE) stay on the tick -- its per-node
     * phase is what spreads their forwarders, and control latency does not
     * matter. */
    mesh_wire_decode(frame, &h);
    if (h.dst != MESH_ADDR_NONE) {
        mesh_tx_schedule();
    }
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

uint8_t mesh_root_alloc_count(void)
{
    return mesh_alloc_count();
}

/* ======================================================================= */
/*  M2: join / address assignment                                         */
/* ======================================================================= */

/* Build a frame (header + optional control payload) into out; return length. */
static uint8_t mesh_build_frame(uint8_t *out, uint8_t type,
                                uint8_t src, uint8_t dst, uint8_t hops, uint8_t seq,
                                const uint8_t *payload, uint8_t plen)
{
    mesh_header_t h;

    h.version = MESH_WIRE_VERSION;
    h.type    = type;
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
static uint8_t mesh_node_device_id_hash8(const uint8_t device_id[MESH_NODE_DEVICE_ID_LEN]);
static void mesh_mark_seen(uint8_t type, uint8_t src, uint8_t seq, const uint8_t *payload);

/* Flood a control frame with an explicit dst (ADDR_ASSIGN carries the assigned
 * address in the header dst; everything else uses MESH_ADDR_NONE). */
static bool mesh_send_flood(uint8_t type, uint8_t dst, uint8_t hops,
                            const uint8_t *payload, uint8_t plen)
{
    uint8_t frame[MESH_MAX_FRAME];
    uint8_t seq = s_bcast_seq;
    uint8_t len;

    s_bcast_seq = (uint8_t)((s_bcast_seq + 1) & 0x1F);
    len = mesh_build_frame(frame, type, mesh_get_address(), dst, hops, seq,
                           payload, plen);
    mesh_mark_seen(type, mesh_get_address(), seq, payload);
    return mesh_send_frame(frame, len);
}

static bool mesh_send_broadcast(uint8_t type, uint8_t hops,
                                const uint8_t *payload, uint8_t plen)
{
    return mesh_send_flood(type, MESH_ADDR_NONE, hops, payload, plen);
}

/* Rewrite a flooded frame to travel one hop further out, into `frame`; false
 * when the TTL is spent and there is nothing left to relay. */
static bool mesh_relay_frame(const mesh_header_t *h, const uint8_t *payload, uint8_t plen,
                             uint8_t frame[MESH_MAX_FRAME], uint8_t *flen)
{
    if (h->hops <= 1) {
        return false;
    }
    *flen = mesh_build_frame(frame, h->type, h->src, h->dst,
                             (uint8_t)(h->hops - 1), h->seq, payload, plen);
    return true;
}

/* Re-broadcast a flooded control frame once, decrementing the TTL. */
static void mesh_relay_flood(const mesh_header_t *h, const uint8_t *payload, uint8_t plen)
{
    uint8_t frame[MESH_MAX_FRAME];
    uint8_t flen;

    if (!mesh_relay_frame(h, payload, plen, frame, &flen)) {
        return;                         /* TTL exhausted */
    }
    mesh_mark_seen(h->type, h->src, h->seq, payload);
    mesh_send_frame(frame, flen);
}

/* Re-broadcast a flooded frame only if we are not a leaf.  A leaf has no
 * downstream node, so its re-broadcast reaches nobody that has not already seen
 * the frame; the routing tree guarantees every attached node still receives the
 * flood from its own parent (a parent is never a leaf -- it forwarded a rootward
 * unicast from the child that chose it).  Only used for floods whose recipients
 * are all attached (ADDR_TABLE, DATA_DOWNLINK); joins/assigns, which must reach
 * unattached nodes, are relayed by everyone. */
static void mesh_relay_flood_gated(const mesh_header_t *h,
                                   const uint8_t *payload, uint8_t plen)
{
    if (mesh_is_relay()) {
        mesh_relay_flood(h, payload, plen);
    }
}

/* 8-bit hash of a node device id, used to dedup JOIN_REQ (whose src is 0). */
static uint8_t mesh_node_device_id_hash8(const uint8_t device_id[MESH_NODE_DEVICE_ID_LEN])
{
    uint32_t h = 2166136261u;           /* FNV-1a 32-bit offset basis */
    for (uint8_t i = 0; i < MESH_NODE_DEVICE_ID_LEN; i++) {
        h ^= device_id[i];
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
            (void)mesh_dedup_check(mesh_node_device_id_hash8(payload), seq);
        }
    } else if (type != MESH_TYPE_BEACON) {
        (void)mesh_dedup_check(src, seq);
    }
}

/* Flood ADDR_ASSIGN{ devid } so the requesting node learns its address.  The
 * assigned address rides in the header `dst`, so no address byte is needed. */
static void mesh_send_addr_assign(const uint8_t device_id[MESH_NODE_DEVICE_ID_LEN], uint8_t addr)
{
    mesh_send_flood(MESH_TYPE_ADDR_ASSIGN, (uint8_t)(addr & MESH_ADDR_MASK),
                    MESH_DEFAULT_TTL, device_id, MESH_NODE_DEVICE_ID_LEN);
}

/* A joining node asks for an address (payload = devid, flooded).  The root
 * allocates and floods ADDR_ASSIGN; every node relays the request onward. */
static void mesh_rx_join_req(const mesh_header_t *h, const uint8_t *payload, uint8_t plen)
{
    if (plen < MESH_NODE_DEVICE_ID_LEN) {
        return;
    }
    if (mesh_is_root()) {
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

/* Node: adopt an address the root assigned to our node device id (flooded). */
static void mesh_rx_addr_assign(const mesh_header_t *h, const uint8_t *payload, uint8_t plen)
{
    uint8_t addr;

    if (plen < MESH_NODE_DEVICE_ID_LEN) {
        return;
    }
    addr = (uint8_t)(h->dst & MESH_ADDR_MASK);
    if (mesh_is_standby()) {
        /* Shadow root: learn the root's address <-> device-id binding from the
         * assignment it overhears (payload = device id, header dst = address).
         * Record only -- never transmit, so the flood is not relayed. */
        if (addr >= MESH_FIRST_NONROOT_ADDR && addr <= MESH_ADDR_MAX) {
            mesh_alloc_claim(payload, addr);
        }
        return;
    }
    if (!mesh_is_root()) {
        if (memcmp(payload, s_cfg.device_id, MESH_NODE_DEVICE_ID_LEN) == 0) {
            if (addr >= MESH_FIRST_NONROOT_ADDR) {
                if (mesh_get_address() != addr) {
                    mesh_set_address(addr);
                    mesh_config_save();
                }
                mesh_update_state();
                mesh_beacon_fast();     /* announce the new attachment quickly */
            }
        } else if (addr >= MESH_FIRST_NONROOT_ADDR && addr == mesh_get_address()) {
            /* Our address was handed to a different node: relinquish it and
             * re-join so no duplicate address can persist (safety net). */
            mesh_set_address(MESH_ADDR_NONE);
            mesh_config_save();
            mesh_update_state();
            mesh_beacon_fast();
        }
    }
    mesh_relay_flood(h, payload, plen);
}

/* Node: reconcile our address with the root's occupied bitmap (flooded). */
static void mesh_rx_addr_table(const mesh_header_t *h, const uint8_t *payload, uint8_t plen)
{
    uint8_t addr = mesh_get_address();

    if (plen < 2) {
        return;
    }
    if (!mesh_is_root() && addr != MESH_ADDR_NONE) {
        if (payload[addr >> 3] & (uint8_t)(1u << (addr & 7))) {
            s_table_miss = 0;           /* root still knows us */
        } else if (++s_table_miss >= MESH_TABLE_MISS_LIMIT) {
            /* Our address vanished from the root's table.  A single lost
             * flood is normal, so only relinquish after several misses
             * (M5 will refine this with an explicit grace policy). */
            s_table_miss = 0;
            mesh_set_address(MESH_ADDR_NONE);
            mesh_config_save();
            mesh_update_state();
            mesh_beacon_fast();
        }
    }
    mesh_relay_flood_gated(h, payload, plen);
}

/* ======================================================================= */
/*  M3: neighbour table, parent selection and convergecast uplink         */
/* ======================================================================= */

/* Map a smoothed SNR (0.1 dB units) to a small integer link cost.  Prefers
 * an extra reliable hop over a weak direct link (see the design notes). */
static uint8_t mesh_link_cost(int16_t snr_x10)
{
    return mesh_route_link_cost(snr_x10, s_sf);
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

/* --- M6: topology map accessors ----------------------------------------- *
 * Every node keeps a partial map: its own row (fed live from the neighbour
 * table) plus a row for each node whose report has passed through it, i.e. its
 * descendants -- the part of the tree it is on the path for.  The root node
 * is on the path for every report (and also absorbs the flooded fallback and
 * the tree edge carried by claims), so it sees the whole graph. */

/* A row exists for our own address (live) and for any node whose report has
 * not aged out. */
static bool mesh_topo_row_valid(uint8_t addr, uint32_t now)
{
    if (addr == mesh_get_address()) {
        return true;                    /* our own row is always present */
    }
    return s_topo_age_ms[addr] != 0 &&
           (uint32_t)(now - s_topo_age_ms[addr]) <= MESH_TOPO_HOLD_MS;
}

uint8_t mesh_topo_row_count(void)
{
    uint32_t now = millis();
    uint8_t  n = 0;

    for (uint8_t a = MESH_ROOT_ADDR; a <= MESH_ADDR_MAX; a++) {
        if (mesh_topo_row_valid(a, now)) {
            n++;
        }
    }
    return n;
}

bool mesh_topo_row(uint8_t index, mesh_topo_row_t *out)
{
    uint32_t now;
    uint8_t  seen = 0;

    if (out == NULL) {
        return false;
    }
    now = millis();
    for (uint8_t a = MESH_ROOT_ADDR; a <= MESH_ADDR_MAX; a++) {
        if (!mesh_topo_row_valid(a, now)) {
            continue;
        }
        if (seen++ < index) {
            continue;
        }
        memset(out, 0, sizeof(*out));
        out->addr = a;
        if (a == mesh_get_address()) {
            /* Our own row: live parent and links from the neighbour table. */
            out->parent = (a == MESH_ROOT_ADDR) ? MESH_ADDR_NONE
                                                  : mesh_get_parent();
            for (uint8_t i = 0; i < MESH_NEIGHBOR_MAX; i++) {
                if (s_neigh[i].addr == MESH_ADDR_NONE || s_neigh[i].addr == a) {
                    continue;
                }
                out->neighbor[out->count] = s_neigh[i].addr;
                out->cost[out->count]     = s_neigh[i].link_cost;
                out->count++;
            }
        } else {
            out->parent = s_topo_parent[a];
            for (uint8_t b = MESH_FIRST_NONROOT_ADDR;
                 b <= MESH_ADDR_MAX && out->count <= MESH_TOPO_MAX_NEIGH; b++) {
                if (!(s_topo_link[a] & (uint16_t)(1u << b))) {
                    continue;
                }
                out->neighbor[out->count] = b;
                out->cost[out->count]     = s_topo_cost[a][b];
                out->count++;
            }
        }
        return true;
    }
    return false;
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
    uint32_t stale = 3u * s_beacon_interval_ms;     /* 3x the current interval */

    /* Drop neighbours that have gone quiet. */
    for (i = 0; i < MESH_NEIGHBOR_MAX; i++) {
        if (s_neigh[i].addr != MESH_ADDR_NONE &&
            (uint32_t)(now - s_neigh[i].last_ms) > stale) {
            memset(&s_neigh[i], 0, sizeof(s_neigh[i]));
        }
    }
    if (s_route.parent_addr != MESH_ADDR_NONE &&
        mesh_neighbor_find(s_route.parent_addr) == NULL) {
        memset(&s_route, 0, sizeof(s_route));   /* parent lost: re-attach */
        s_route.parent_addr = MESH_ADDR_NONE;
        s_had_parent = false;                   /* claim again when we re-attach */
        mesh_select_parent();
        if (s_route.parent_addr != MESH_ADDR_NONE) {
            mesh_beacon_fast();
        }
    }
}

/* Emit this node's beacon: only routable nodes (root, or a node with a
 * parent) advertise, so advertised costs are always meaningful.  The 1-byte
 * payload packs the root epoch (3 bits) over the path cost (5 bits). */
static void mesh_emit_beacon(void)
{
    uint8_t payload[MESH_BEACON_LEN];

    if (mesh_is_root()) {
        payload[0] = (uint8_t)((s_boot_epoch & 0x07) << 5);     /* cost 0 */
        mesh_send_broadcast(MESH_TYPE_BEACON, 0, payload, MESH_BEACON_LEN);
        return;
    }
    if (s_route.parent_addr != MESH_ADDR_NONE) {
        payload[0] = (uint8_t)(((s_root_epoch & 0x07) << 5) |
                               (s_route.self_cost & 0x1F));
        mesh_send_broadcast(MESH_TYPE_BEACON, s_route.self_hops, payload,
                            MESH_BEACON_LEN);
    }
}

/* Track the root's boot epoch.  A change (or the first one seen while we
 * already hold an address) means the root restarted, so schedule a claim. */
static void mesh_note_epoch(uint16_t epoch)
{
    bool announce = false;

    if (mesh_is_root()) {
        return;
    }
    if (!s_epoch_valid) {
        s_epoch_valid  = true;
        s_root_epoch = epoch;
        announce = (mesh_get_address() != MESH_ADDR_NONE);
    } else if (epoch != s_root_epoch) {
        s_root_epoch = epoch;
        announce = true;
    }
    if (announce && mesh_get_address() != MESH_ADDR_NONE) {
        s_claim_due = true;             /* re-announce on the next timer tick */
        mesh_beacon_fast();
    }
}

static void mesh_rx_beacon(const mesh_header_t *h, const uint8_t *payload, uint8_t plen)
{
    mesh_route_neighbor_t *n;
    int16_t sample;
    uint32_t now = millis();
    uint8_t  old_parent = s_route.parent_addr;
    uint8_t  epoch;

    if (h->src == mesh_get_address() || plen < MESH_BEACON_LEN) {
        return;
    }
    epoch = (uint8_t)((payload[0] >> 5) & 0x07);    /* 3-bit root epoch */
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
    n->cost      = (uint8_t)(payload[0] & 0x1F);
    n->hops      = h->hops;
    n->link_cost = mesh_link_cost(n->snr_x10);
    n->last_ms   = now;

    /* A beacon from a child we already know proves it is still alive and still
     * routing through us, so refresh its liveness/relay timestamps.  Set
     * s_relay_last_ms directly (not via mesh_note_relay) so a periodic child
     * beacon never resets our own beacon back-off. */
    if (h->src >= MESH_FIRST_NONROOT_ADDR && s_child_heard_ms[h->src] != 0) {
        s_child_heard_ms[h->src] = now;
        s_relay_last_ms          = now;
    }

    mesh_check_parent_staleness(now);
    mesh_select_parent();

    /* A parent change (or a fresh attachment) resets the beacon to fast. */
    if (s_route.parent_addr != old_parent) {
        mesh_beacon_fast();
    }

    /* Any (re)attachment -- a fresh one or a switch to a better parent --
     * re-announces our address so the root re-binds us and learns our new
     * parent, and queues a topology report so the map follows the move. */
    if (!mesh_is_root() && s_route.parent_addr != MESH_ADDR_NONE &&
        mesh_get_address() != MESH_ADDR_NONE) {
        if (!s_had_parent || s_route.parent_addr != old_parent) {
            s_claim_due = true;
            s_topo_due  = true;
        }
        s_had_parent = true;
    }
    mesh_note_epoch(epoch);
}

/* --- M4: implicit link ACK + bounded retries ---------------------------- */

/* Remember a unicast frame so the timer can retransmit it if the next hop
 * does not visibly forward it (implicit ACK) within the timeout. */
static void mesh_set_pending(uint8_t slot, const uint8_t *frame, uint8_t len,
                             uint8_t src, uint8_t seq)
{
    mesh_pending_t *p = &s_pending[slot];

    memcpy(p->frame, frame, len);
    p->len     = len;
    p->src     = src;
    p->seq     = seq;
    p->retries = 0;
    p->active  = true;
    p->next_ms = millis() + mesh_link_ack_timeout_for(len);
}

static void mesh_pending_tick(uint32_t now)
{
    for (uint8_t slot = 0; slot < MESH_PEND_N; slot++) {
        mesh_pending_t *p = &s_pending[slot];

        if (!p->active || (int32_t)(now - p->next_ms) < 0) {
            continue;
        }
        if (p->retries >= MESH_LINK_RETRIES) {
            p->active = false;          /* give up; the app-layer ACK covers it */
            continue;
        }
        mesh_send_frame(p->frame, p->len);
        p->retries++;
        p->next_ms = now + mesh_link_ack_timeout_for(p->len);
    }
}

bool mesh_send_uplink(const uint8_t *payload, uint8_t len)
{
    uint8_t frame[MESH_MAX_FRAME];
    uint8_t flen;
    uint8_t seq;

    if (len > MESH_MAX_FRAME - MESH_HEADER_SIZE) {
        return false;
    }
    if (mesh_is_root()) {
        /* The root's own message is delivered straight to its host. */
        mesh_deliver_to_host(payload, len, MESH_ROOT_ADDR);
        return true;
    }
    if (s_state != MESH_STATE_JOINED || s_route.parent_addr == MESH_ADDR_NONE) {
        return false;                   /* no route yet */
    }
    seq = s_data_seq;
    s_data_seq = (uint8_t)((s_data_seq + 1) & 0x1F);
    flen = mesh_build_frame(frame, MESH_TYPE_DATA_UPLINK,
                            mesh_get_address(), s_route.parent_addr,
                            MESH_DEFAULT_TTL, seq, payload, len);

    /* Host message: one immediate attempt.  If the channel is busy we return
     * false so the host backs off and resends -- the module does not queue or
     * retry it. */
    if (!mesh_tx_radio_send(frame, flen)) {
        return false;
    }
    /* Arm the pending entry so a lost hop is still retried by the mesh MAC
     * (implicit link ACK), independent of the host.  A direct child of the
     * root is the exception: its parent is the root, which does not forward,
     * so there is no forward to overhear and the root sends it no LINK_ACK.
     * Its single hop is instead confirmed by the application-layer ACK, so it
     * arms nothing here and the mesh MAC does not retransmit. */
    if (s_route.parent_addr != MESH_ROOT_ADDR) {
        mesh_set_pending(MESH_PEND_UP, frame, flen, mesh_get_address(), seq);
    }
    return true;
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

/* Explicit link ACK: the root has no next hop to forward to, so it acks the
 * uplink it just delivered by (origin,seq); relays clear on the same key.
 * The acked origin rides in the header `dst`, so the payload is just the seq. */
static void mesh_send_link_ack(uint8_t acked_src, uint8_t acked_seq)
{
    uint8_t frame[MESH_MAX_FRAME];
    uint8_t payload[1];
    uint8_t flen;

    payload[0] = acked_seq;
    flen = mesh_build_frame(frame, MESH_TYPE_LINK_ACK, mesh_get_address(),
                            acked_src, 0, 0, payload, 1);
    mesh_send_frame(frame, flen);
}

static void mesh_rx_data_uplink(const mesh_header_t *h, const uint8_t *payload, uint8_t plen)
{
    uint8_t frame[MESH_MAX_FRAME];
    uint8_t flen;

    if (mesh_is_standby()) {
        /* Passive observer (M7): mirror the uplink to our own host exactly as
         * the root delivers the uplinks it receives, then probe for the
         * root's application ACK.  Only one probe is in flight at a time, so
         * a node's two quick uplinks are not counted as two independent trials. */
        mesh_deliver_to_host(payload, plen, h->src);
        if (!s_probe_active && s_root_seen &&
            s_rx_snr >= MESH_STANDBY_MIN_SNR) {
            s_probe_active   = true;
            s_probe_src      = h->src;
            s_probe_deadline = millis() + MESH_STANDBY_PROBE_MS;
        }
        return;                         /* never forward / ack: an observer is silent */
    }
    if (mesh_is_root()) {
        mesh_deliver_to_host(payload, plen, h->src);
        /* A relay node clears its pending on the next hop's forward (implicit
         * ACK); the root has no next hop to overhear, so it answers with an
         * explicit LINK_ACK.  The exception is a direct child of the root: its
         * single hop is confirmed by the application-layer ACK (see
         * mesh_send_uplink), so the root does not spend a LINK_ACK frame on it.
         * A direct child's uplink is the only one that arrives at the full TTL
         * (a relay is only ever in the path of deeper uplinks, which it
         * decrements), so h->hops identifies it exactly. */
        if (h->hops < MESH_DEFAULT_TTL) {
            mesh_send_link_ack(h->src, h->seq);
        }
        return;
    }
    if (h->dst != mesh_get_address()) {
        return;                         /* not addressed to us */
    }
    mesh_note_relay(h->src);            /* a child routed through us */
    if (h->hops <= 1 || s_route.parent_addr == MESH_ADDR_NONE) {
        return;                         /* TTL exhausted / no route */
    }
    flen = mesh_build_frame(frame, MESH_TYPE_DATA_UPLINK, h->src,
                            s_route.parent_addr, (uint8_t)(h->hops - 1), h->seq,
                            payload, plen);
    if (mesh_send_frame(frame, flen)) {
        /* Wait for our parent to forward it (implicit ACK); retry otherwise. */
        mesh_set_pending(MESH_PEND_UP, frame, flen, h->src, h->seq);
    }
}

/* --- Downlink routing (M4) ---------------------------------------------- *
 * The tree the liveness plane already maintains is also a routing table: each
 * node holds, per child, the bitmap of the addresses alive in that child's
 * subtree (s_child_bm, refreshed by ADDR_ALIVE).  A downlink can therefore be
 * steered one hop down the branch that contains its target instead of being
 * flooded: only the node whose child's bitmap covers the target re-broadcasts,
 * so a delivery costs depth(target) transmissions, not one per relay node.
 * The liveness invariants make the lookup exact -- a relay always publishes an
 * aggregate (so its parent's bitmap of it stays fresh), while a leaf's subtree
 * is just itself -- so the bits partition the tree with no ambiguity. */

/* The bitmap of a current child's subtree (0 when `a` is not our child).  The
 * child's own bitmap is trusted only while it is fresh; once stale the child is
 * reported as just itself, because it may have become a leaf (subtree gone). */
static uint16_t mesh_child_bitmap(uint8_t a, uint32_t now)
{
    if (a < MESH_FIRST_NONROOT_ADDR || a > MESH_ADDR_MAX ||
        a == mesh_get_address() || s_child_heard_ms[a] == 0 ||
        (uint32_t)(now - s_child_heard_ms[a]) > MESH_ALIVE_CHILD_HOLD_MS) {
        return 0;
    }
    if (s_child_bm_ms[a] != 0 &&
        (uint32_t)(now - s_child_bm_ms[a]) <= MESH_ALIVE_CHILD_HOLD_MS) {
        return s_child_bm[a];
    }
    return (uint16_t)(1u << a);
}

/* The child whose subtree contains `dst`, or MESH_ADDR_NONE.  A direct child
 * covers itself (its seeded bitmap has its own bit set). */
static uint8_t mesh_child_for(uint8_t dst)
{
    uint32_t now = millis();

    if (dst < MESH_FIRST_NONROOT_ADDR || dst > MESH_ADDR_MAX ||
        dst == mesh_get_address()) {
        return MESH_ADDR_NONE;
    }
    for (uint8_t c = MESH_FIRST_NONROOT_ADDR; c <= MESH_ADDR_MAX; c++) {
        if (mesh_child_bitmap(c, now) & (uint16_t)(1u << dst)) {
            return c;
        }
    }
    return MESH_ADDR_NONE;
}

/* Root: send a payload down to a specific node.  Steered hop by hop via the
 * subtree bitmaps, so the frame is on air once per hop instead of once per
 * relay node. */
bool mesh_send_downlink(uint8_t dst, const uint8_t *payload, uint8_t len)
{
    uint8_t frame[MESH_MAX_FRAME];
    uint8_t flen;
    uint8_t seq;

    if (!mesh_is_root() || dst < MESH_FIRST_NONROOT_ADDR) {
        return false;
    }
    if (len > MESH_MAX_FRAME - MESH_HEADER_SIZE) {
        return false;
    }
    seq = s_data_seq;
    s_data_seq = (uint8_t)((s_data_seq + 1) & 0x1F);
    flen = mesh_build_frame(frame, MESH_TYPE_DATA_DOWNLINK, mesh_get_address(),
                            dst, MESH_DEFAULT_TTL, seq, payload, len);
    mesh_mark_seen(MESH_TYPE_DATA_DOWNLINK, mesh_get_address(), seq, payload);

    /* Host-originated: one immediate attempt; a busy channel goes back to the
     * host as a busy status (it owns the backoff / resend). */
    if (!mesh_tx_radio_send(frame, flen)) {
        return false;
    }
    /* Arm the hop ACK so a lost hop is retried by the mesh MAC (implicit link
     * ACK), the mirror of mesh_send_uplink.  A direct child is the exception:
     * that link is a single hop, so losing it costs the same to retry end to
     * end, and the target sends no LINK_ACK for it (see mesh_rx_data_downlink);
     * arming here would only spend MESH_LINK_RETRIES retransmissions that
     * nothing can confirm. */
    if (mesh_child_for(dst) != dst) {
        mesh_set_pending(MESH_PEND_DOWN, frame, flen, mesh_get_address(), seq);
    }
    return true;
}

/* Node: a downlink is steered down the tree.  The target delivers it to its
 * host; every other node relays it one hop further only when a child's subtree
 * bitmap covers the target, i.e. only the branch that leads to the target ever
 * re-broadcasts.  A node off that branch stays silent, so the frame follows a
 * single path of length depth(target) instead of flooding.  Every hop of that
 * path is then retried on its own: a relay clears its pending by overhearing the
 * next hop's forward, and the target -- which has no next hop -- clears its
 * parent's with an explicit LINK_ACK.  That recovers a lost hop with one
 * retransmission instead of the origin's whole uplink-and-downlink round trip,
 * which is what the application-layer ACK would otherwise take. */
static void mesh_rx_data_downlink(const mesh_header_t *h, const uint8_t *payload, uint8_t plen)
{
    uint8_t frame[MESH_MAX_FRAME];
    uint8_t flen;

    if (mesh_is_standby()) {
        /* The application ACK we are waiting for is a downlink from the root
         * back to the origin of the probed uplink: a live, responsive root.
         * Confirm the probe and reset the consecutive-miss run. */
        if (s_probe_active && h->src == MESH_ROOT_ADDR && h->dst == s_probe_src) {
            s_probe_active = false;
            s_probe_miss   = 0;
        }
        return;                         /* never relay: an observer is not in the tree */
    }
    if (h->dst == mesh_get_address()) {
        mesh_deliver_to_host(payload, plen, h->src);
        /* The path ends here, so there is no next hop whose forward our parent
         * could overhear; we ack the final hop ourselves, the mirror of the
         * root acking an uplink it terminated.  A direct child of the root
         * sees the full TTL (no relay touched the frame) and is skipped: that
         * hop is the root's own and it did not arm it (mesh_send_downlink). */
        if (h->hops < MESH_DEFAULT_TTL) {
            mesh_send_link_ack(h->src, h->seq);
        }
        return;
    }
    if (mesh_is_root()) {
        return;                         /* the root originates downlinks */
    }
    if (mesh_child_for(h->dst) != MESH_ADDR_NONE &&
        mesh_relay_frame(h, payload, plen, frame, &flen)) {
        mesh_mark_seen(h->type, h->src, h->seq, payload);
        if (mesh_send_frame(frame, flen)) {
            /* Wait for the next hop to carry it on (implicit ACK) -- or, when
             * the next hop *is* the target, for the target's LINK_ACK. */
            mesh_set_pending(MESH_PEND_DOWN, frame, flen, h->src, h->seq);
        }
    }
}

/* ======================================================================= */
/*  M5: recovery / robustness                                             */
/* ======================================================================= */

/* Record one node's reported parent (the tree edge).  Shared by the root (a
 * claim it absorbs) and by any relay on a claim's rootward path, so every node
 * learns its descendants' tree edges, not just the root. */
static void mesh_topo_note_parent(uint8_t addr, uint8_t parent)
{
    if (addr < MESH_FIRST_NONROOT_ADDR || addr > MESH_ADDR_MAX) {
        return;
    }
    parent = (uint8_t)(parent & MESH_ADDR_MASK);
    if (parent == addr) {
        parent = MESH_ADDR_NONE;        /* a node is never its own parent */
    }
    s_topo_parent[addr] = parent;
    s_topo_age_ms[addr] = millis();
}

/* Root: apply a node's claim to the RAM-only table.  Adopt the claimed
 * address when it is free, re-assert our existing assignment when the node device
 * id is already known, and fall back to a fresh allocation when the claimed address
 * is already taken.  Shared by the unicast and flooded claim paths. */
static void mesh_root_apply_claim(const mesh_header_t *h, const uint8_t *payload, uint8_t plen)
{
    uint8_t want = (uint8_t)(h->src & MESH_ADDR_MASK);  /* node's own address */
    uint8_t have;

    if (want < MESH_FIRST_NONROOT_ADDR || want > MESH_ADDR_MAX) {
        return;
    }
    /* A claim also reports the node's parent (the topology map's tree edge). */
    if (plen >= MESH_NODE_DEVICE_ID_LEN + 1) {
        mesh_topo_note_parent(want, payload[MESH_NODE_DEVICE_ID_LEN]);
    }
    have = mesh_alloc_lookup(payload);
    if (have == want) {
        /* already known -- nothing to do */
    } else if (have != MESH_ADDR_NONE) {
        /* node device id known under another address: re-assert ours */
        mesh_send_addr_assign(payload, have);
    } else if (!mesh_alloc_claim(payload, want)) {
        /* claimed address taken by another node: assign a fresh one */
        uint8_t na = mesh_alloc_assign(payload);
        if (na != MESH_ADDR_NONE) {
            mesh_send_addr_assign(payload, na);
        }
    }
}

/* Node: re-announce our flash-stored address so a restarted root can rebuild
 * its RAM-only table.  Our address is already in the header `src`; the payload
 * is just our node device id.
 *
 * This is a rootward message: it only has to reach the root, so we send it
 * hop-by-hop toward our parent (same discipline as DATA_UPLINK) rather than
 * flooding -- the network-wide cost is O(hops) per claim instead of O(N).  When
 * we have no route yet (just booted, or just lost our parent) we fall back to a
 * flood so a route-less node is still discoverable.  Best-effort either way.
 *
 * Claims are event-driven (boot, epoch change, re-attach); the recurring
 * rootward traffic is the far cheaper single-hop ADDR_ALIVE aggregate
 * (mesh_send_alive). */
static void mesh_send_claim(void)
{
    uint8_t frame[MESH_MAX_FRAME];
    uint8_t claim[MESH_NODE_DEVICE_ID_LEN + 1];
    uint8_t flen;
    uint8_t seq;

    if (mesh_is_root() || mesh_get_address() == MESH_ADDR_NONE) {
        return;
    }
    /* Payload is our device id plus our current parent, so the root learns
     * both the address binding and the tree edge (its topology map) in one
     * message.  Parent 0 when we have no route yet. */
    memcpy(claim, s_cfg.device_id, MESH_NODE_DEVICE_ID_LEN);
    claim[MESH_NODE_DEVICE_ID_LEN] = s_route.parent_addr;
    if (s_state != MESH_STATE_JOINED || s_route.parent_addr == MESH_ADDR_NONE) {
        /* No route: flood so the root can still find us. */
        mesh_send_broadcast(MESH_TYPE_ADDR_CLAIM, MESH_DEFAULT_TTL,
                            claim, sizeof(claim));
    } else {
        /* Rootward unicast to the parent. */
        seq = s_bcast_seq;
        s_bcast_seq = (uint8_t)((s_bcast_seq + 1) & 0x1F);
        flen = mesh_build_frame(frame, MESH_TYPE_ADDR_CLAIM,
                                mesh_get_address(), s_route.parent_addr,
                                MESH_DEFAULT_TTL, seq, claim, sizeof(claim));
        mesh_send_frame(frame, flen);
    }
    /* The claim is itself a liveness signal; don't follow it immediately with
     * a redundant aggregate. */
    s_alive_next_ms = millis() + MESH_ALIVE_INTERVAL_MS;
}

/* Node: topology report.  Rootward (to the root) it carries our current
 * parent and every neighbour we hear, with its link cost, so the root can
 * assemble the whole node/link graph.  The header `src` is our address, so the
 * payload is `parent(1) | {addr(1), cost(1)}*` (no count: the length implies
 * it).  Event-driven (attach, parent change) plus a slow backstop; sent only
 * once we hold a parent, so it is an O(hops) rootward unicast, never a flood.
 * Best-effort -- the backstop recovers a lost report. */
static void mesh_send_topology(void)
{
    uint8_t frame[MESH_MAX_FRAME];
    uint8_t payload[1 + 2 * MESH_NEIGHBOR_MAX];
    uint8_t flen;
    uint8_t plen = 1;
    uint8_t seq;

    s_topo_next_ms = millis() + MESH_TOPO_INTERVAL_MS;

    if (mesh_is_root() || mesh_get_address() == MESH_ADDR_NONE) {
        return;
    }
    if (s_state != MESH_STATE_JOINED || s_route.parent_addr == MESH_ADDR_NONE) {
        return;                         /* no route yet: the claim flood covers us */
    }
    payload[0] = s_route.parent_addr;
    for (uint8_t i = 0; i < MESH_NEIGHBOR_MAX; i++) {
        if (s_neigh[i].addr == MESH_ADDR_NONE || s_neigh[i].addr == mesh_get_address()) {
            continue;
        }
        payload[plen++] = s_neigh[i].addr;
        payload[plen++] = s_neigh[i].link_cost;
    }
    seq = s_bcast_seq;
    s_bcast_seq = (uint8_t)((s_bcast_seq + 1) & 0x1F);
    flen = mesh_build_frame(frame, MESH_TYPE_TOPOLOGY, mesh_get_address(),
                            s_route.parent_addr, MESH_DEFAULT_TTL, seq,
                            payload, plen);
    mesh_send_frame(frame, flen);
}

/* Node: periodic liveness **aggregate**.  A single-hop unicast to our parent
 * carrying a 2-byte subtree bitmap (bit a = "address a is alive in our
 * subtree").  We set our own bit and OR in the bitmap reported by every current
 * child, so one frame covers the whole subtree -- the parent absorbs it (never
 * forwards it), and the union of the root's direct children therefore covers
 * every non-root node each round.
 *
 * Only a **relay** (a node with children) sends one: a leaf sends nothing, so
 * the steady-state cost is (relays) single-hop frames per round, not N.  A leaf
 * is covered by its parent, which learns it when it joins / re-attaches (the
 * child's event-driven ADDR_CLAIM is addressed to and forwarded by that parent)
 * and keeps it fresh from its link-local beacon (mesh_rx_beacon).  If that early
 * claim is lost, the leaf's bit simply stays clear and the ADDR_TABLE miss path
 * makes it re-join and re-claim -- the outer backstop.
 *
 * A child is "current" while we heard it within MESH_ALIVE_CHILD_HOLD_MS.  A
 * child's *bitmap* is trusted only while s_child_bm_ms is fresh too; when it
 * goes stale (the child stopped sending aggregates -- it may have become a leaf
 * whose subtree is gone) we report the child as just itself, so a vanished
 * subtree clears instead of lingering.  Only sent once we hold a parent -- a
 * route-less node re-attaches (and re-claims) instead. */
static void mesh_send_alive(void)
{
    uint8_t  frame[MESH_MAX_FRAME];
    uint8_t  flen;
    uint8_t  seq;
    uint8_t  payload[2];
    uint16_t bm;
    uint32_t now = millis();

    s_alive_next_ms = now + MESH_ALIVE_INTERVAL_MS;

    if (mesh_is_root() || mesh_get_address() == MESH_ADDR_NONE) {
        return;
    }
    if (s_state != MESH_STATE_JOINED || s_route.parent_addr == MESH_ADDR_NONE) {
        return;                         /* re-attaching: the claim path covers us */
    }
    if (!mesh_is_relay()) {
        return;                         /* leaf: our parent covers us */
    }
    bm = (uint16_t)(1u << mesh_get_address());
    for (uint8_t a = MESH_FIRST_NONROOT_ADDR; a <= MESH_ADDR_MAX; a++) {
        bm |= mesh_child_bitmap(a, now); /* self, plus each current child's subtree */
    }
    payload[0] = (uint8_t)(bm & 0xFF);
    payload[1] = (uint8_t)(bm >> 8);
    seq = s_bcast_seq;
    s_bcast_seq = (uint8_t)((s_bcast_seq + 1) & 0x1F);
    flen = mesh_build_frame(frame, MESH_TYPE_ADDR_ALIVE,
                            mesh_get_address(), s_route.parent_addr,
                            MESH_DEFAULT_TTL, seq, payload, 2);
    mesh_send_frame(frame, flen);
}

/* Forward a rootward message one hop toward our parent (or convert it to a
 * flood if we have lost our parent).  Used by ADDR_CLAIM only: an ADDR_ALIVE is
 * a single-hop subtree aggregate consumed by the parent, never forwarded. */
static void mesh_forward_rootward(const mesh_header_t *h,
                                  const uint8_t *payload, uint8_t plen)
{
    uint8_t frame[MESH_MAX_FRAME];
    uint8_t flen;

    mesh_note_relay(h->src);            /* a child routed through us */
    if (h->hops <= 1) {
        return;                         /* TTL exhausted */
    }
    if (s_route.parent_addr == MESH_ADDR_NONE) {
        /* We cannot forward rootward right now: fall back to a flood (origin's
         * src preserved, dst cleared) so the message can still reach the root
         * via another path. */
        flen = mesh_build_frame(frame, h->type, h->src,
                                MESH_ADDR_NONE, (uint8_t)(h->hops - 1), h->seq,
                                payload, plen);
        mesh_send_frame(frame, flen);
        return;
    }
    flen = mesh_build_frame(frame, h->type, h->src,
                            s_route.parent_addr, (uint8_t)(h->hops - 1), h->seq,
                            payload, plen);
    mesh_send_frame(frame, flen);       /* best effort; no pending retry */
}

/* Handle an incoming ADDR_CLAIM.  The root applies it to its table; a standby
 * records the binding (shadow root); an intermediate node forwards a rootward
 * unicast one hop toward its parent (or converts it to a flood if it has lost its
 * own parent); the flooded fallback is relayed by everyone. */
static void mesh_rx_addr_claim(const mesh_header_t *h, const uint8_t *payload, uint8_t plen)
{
    if (plen < MESH_NODE_DEVICE_ID_LEN) {
        return;
    }

    if (mesh_is_standby()) {
        /* Shadow root: learn the address <-> device-id binding (device id in the
         * payload, the node's own address in header `src`) and the tree edge,
         * without allocating or responding -- that is the active root's job. */
        uint8_t addr = (uint8_t)(h->src & MESH_ADDR_MASK);
        if (addr >= MESH_FIRST_NONROOT_ADDR && addr <= MESH_ADDR_MAX) {
            mesh_alloc_claim(payload, addr);
            if (plen >= MESH_NODE_DEVICE_ID_LEN + 1) {
                mesh_topo_note_parent(addr, payload[MESH_NODE_DEVICE_ID_LEN]);
            }
        }
        return;
    }

    /* Flooded fallback (sent by a node that has no route yet): everyone relays
     * and the root rebuilds. */
    if (h->dst == MESH_ADDR_NONE) {
        if (mesh_is_root()) {
            mesh_root_apply_claim(h, payload, plen);
        }
        mesh_relay_flood(h, payload, plen);
        return;
    }

    /* Rootward unicast: only the addressed next hop acts. */
    if (!mesh_addressed_to_me(h->dst)) {
        return;                         /* not addressed to us */
    }
    if (mesh_is_root()) {
        mesh_root_apply_claim(h, payload, plen);
        return;
    }
    /* We are on the claim's rootward path: record its tree edge, then forward. */
    if (plen >= MESH_NODE_DEVICE_ID_LEN + 1) {
        mesh_topo_note_parent(h->src, payload[MESH_NODE_DEVICE_ID_LEN]);
    }
    mesh_forward_rootward(h, payload, plen);
}

/* Absorb one node's topology report -- its parent plus the neighbours it hears
 * and their link costs.  A report replaces that node's whole row.  Called by
 * any node on the reporter's rootward path: the root node (which sees every
 * report) and each relay ancestor (which sees its own descendants). */
static void mesh_apply_topology(uint8_t src, const uint8_t *payload, uint8_t plen)
{
    uint8_t n;

    if (src < MESH_FIRST_NONROOT_ADDR || src > MESH_ADDR_MAX || plen < 1) {
        return;
    }
    mesh_topo_note_parent(src, payload[0]);
    s_topo_link[src] = 0;
    memset(s_topo_cost[src], MESH_TOPO_NO_LINK, sizeof(s_topo_cost[src]));

    n = (uint8_t)((plen - 1) / 2);              /* entries implied by the length */
    if (n > MESH_TOPO_MAX_NEIGH) {
        n = MESH_TOPO_MAX_NEIGH;
    }
    for (uint8_t i = 0; i < n; i++) {
        uint8_t a = (uint8_t)(payload[1 + i * 2] & MESH_ADDR_MASK);
        uint8_t c = payload[2 + i * 2];
        if (a < MESH_FIRST_NONROOT_ADDR || a > MESH_ADDR_MAX || a == src) {
            continue;
        }
        s_topo_link[src]   |= (uint16_t)(1u << a);
        s_topo_cost[src][a] = c;
    }
    /* A parent is by definition a neighbour: make sure the tree edge shows up
     * even if the neighbour list was truncated. */
    if (s_topo_parent[src] >= MESH_FIRST_NONROOT_ADDR &&
        s_topo_parent[src] <= MESH_ADDR_MAX) {
        s_topo_link[src] |= (uint16_t)(1u << s_topo_parent[src]);
    }
}

/* Handle an incoming topology report.  Every node the report passes through on
 * its rootward path absorbs it -- so the root node learns the whole graph and
 * a relay learns its own subtree -- and a non-root forwards it one hop toward
 * its parent.  The flooded fallback (a relay that lost its own parent) is
 * absorbed only by a root (everyone else would pollute its map with nodes
 * that are not its descendants) and relayed by everyone. */
static void mesh_rx_topology(const mesh_header_t *h, const uint8_t *payload, uint8_t plen)
{
    if (plen < 1) {
        return;
    }
    if (h->dst == MESH_ADDR_NONE) {
        if (mesh_at_root()) {
            mesh_apply_topology(h->src, payload, plen);
        }
        mesh_relay_flood(h, payload, plen);
        return;
    }
    if (!mesh_addressed_to_me(h->dst)) {
        return;                         /* not addressed to us (or our shadow root) */
    }
    /* We are a hop on the reporter's rootward path: absorb its row. */
    mesh_apply_topology(h->src, payload, plen);
    if (mesh_at_root()) {
        return;                         /* the root does not forward; a standby is silent */
    }
    mesh_forward_rootward(h, payload, plen);
}

/* --- Node liveness monitor (Step 4) -------------------------------------- *
 * A node at depth >= 2 cannot observe the root directly, and under
 * relay-only liveness a *leaf* sends no aggregate of its own, so its only
 * upstream proof of life is its parent's aggregate carrying its bit.  The
 * monitor watches that aggregate -- a single-hop unicast to the grandparent,
 * which LoRa's shared medium lets us overhear -- and re-claims when the proof
 * disappears: either the parent stops aggregating altogether (it rebooted and
 * lost its RAM-only child table, so it is no longer a relay), or it keeps
 * aggregating but has dropped our bit.  A claim re-registers us at every hop
 * (mesh_forward_rootward -> mesh_note_relay) and re-adopts our address at the
 * root, so it is safe even if the eviction timer has already fired.  Steady
 * state costs nothing; only a node that has actually lost coverage emits. */

/* Fire a re-claim, rate-limited to one per liveness interval so a broken or
 * marginal parent link cannot turn into a claim storm. */
static void mesh_monitor_trigger(uint32_t now)
{
    if (s_mon_claim_ms != 0 &&
        (uint32_t)(now - s_mon_claim_ms) < MESH_ALIVE_INTERVAL_MS) {
        return;
    }
    s_mon_claim_ms = now;
    s_claim_due    = true;              /* re-claim on the next timer tick */
}

/* Per-tick: (re)arm for the current parent, and fire if its aggregate has gone
 * missing for too long.  A parent that rebooted and forgot its children stops
 * aggregating entirely, so we never overhear one and the watchdog catches it. */
static void mesh_monitor_tick(uint32_t now)
{
    uint8_t  parent = s_route.parent_addr;
    uint32_t last;

    if (mesh_is_root() || s_state != MESH_STATE_JOINED ||
        parent < MESH_FIRST_NONROOT_ADDR || parent > MESH_ADDR_MAX) {
        s_mon_parent = MESH_ADDR_NONE;  /* parent is the root / none: exempt */
        return;
    }
    if (parent != s_mon_parent) {
        s_mon_parent      = parent;     /* adopted a parent: restart the watch */
        s_mon_parent_ms   = now;
        s_parent_alive_ms = 0;
        return;
    }
    last = (s_parent_alive_ms != 0) ? s_parent_alive_ms : s_mon_parent_ms;
    if ((uint32_t)(now - last) > MESH_MONITOR_TIMEOUT_MS) {
        mesh_monitor_trigger(now);      /* parent went quiet */
    }
}

/* We overheard our parent's aggregate (addressed to our grandparent): refresh
 * the watch, and react if our own bit is missing from it. */
static void mesh_monitor_alive(const mesh_header_t *h,
                               const uint8_t *payload, uint8_t plen)
{
    uint16_t bm;
    uint8_t  me  = mesh_get_address();
    uint32_t now = millis();

    if (plen < 2 || me == MESH_ADDR_NONE ||
        s_route.parent_addr < MESH_FIRST_NONROOT_ADDR ||
        s_route.parent_addr > MESH_ADDR_MAX ||
        h->src != s_route.parent_addr) {
        return;
    }
    if (s_mon_parent != h->src) {       /* first aggregate from a new parent */
        s_mon_parent    = h->src;
        s_mon_parent_ms = now;
    }
    s_parent_alive_ms = now;            /* parent is still aggregating */
    bm = (uint16_t)(payload[0] | ((uint16_t)payload[1] << 8));
    if (bm & (uint16_t)(1u << me)) {
        return;                         /* still covered */
    }
    /* Our bit is missing.  Allow one full interval after (re)attaching so our
     * own claim can land before we react. */
    if ((uint32_t)(now - s_mon_parent_ms) > MESH_ALIVE_INTERVAL_MS) {
        mesh_monitor_trigger(now);
    }
}

/* Handle an incoming ADDR_ALIVE: a single-hop subtree-liveness aggregate
 * addressed to us (its parent).  We never forward it.
 *   - root (the root, or a standby shadowing it): book liveness for every
 *     address the child reported alive (so the eviction loop sees the whole
 *     subtree refreshed, not just the header src), record the sender as a known
 *     direct child, and remember which direct child covers each address (s_cover)
 *     for subtree-scoped eviction.
 *   - relay: remember this child's subtree bitmap for our own next aggregate. */
static void mesh_rx_alive(const mesh_header_t *h, const uint8_t *payload, uint8_t plen)
{
    uint16_t bm;
    uint8_t  c;

    if (!mesh_addressed_to_me(h->dst)) {
        /* Not addressed to us.  If it is our parent's own aggregate (a unicast
         * to our grandparent) we overhear it to monitor whether we are still
         * covered upstream (see mesh_monitor_alive). */
        if (!mesh_at_root() && h->src == s_route.parent_addr) {
            mesh_monitor_alive(h, payload, plen);
        }
        return;                         /* single-hop aggregate: only the parent acts */
    }
    if (plen < 2) {
        return;
    }
    bm = (uint16_t)(payload[0] | ((uint16_t)payload[1] << 8));
    c  = h->src;

    if (mesh_at_root()) {
        uint32_t now = millis();
        if (c < MESH_FIRST_NONROOT_ADDR || c > MESH_ADDR_MAX) {
            return;
        }
        s_child_heard_ms[c] = now;      /* c is a known direct child */
        for (uint8_t a = MESH_FIRST_NONROOT_ADDR; a <= MESH_ADDR_MAX; a++) {
            if (bm & (uint16_t)(1u << a)) {
                s_heard_ms[a] = now;
                s_cover[a]    = c;      /* a is covered by branch c */
            }
        }
        return;
    }
    if (c >= MESH_FIRST_NONROOT_ADDR && c <= MESH_ADDR_MAX) {
        mesh_note_relay(c);             /* refresh child + our relay status */
        s_child_bm[c]    = bm;          /* adopt the richer subtree bitmap  */
        s_child_bm_ms[c] = millis();
    }
}

/* ======================================================================= */
/*  M7: standby root (passive observer)                                 */
/* ======================================================================= */

/* Per-tick liveness check for the standby.  Two independent ways to declare the
 * active root dead:
 *   - the application-ACK probe: an uplink we probed got no downlink back to its
 *     origin within MESH_STANDBY_PROBE_MS -> a miss.  MESH_STANDBY_MISS_LIMIT
 *     misses in a row, spread over at least MESH_STANDBY_MIN_WINDOW_MS (so a
 *     reboot shorter than that window does not promote), is "dead".
 *   - the backstop: no frame from the root for MESH_STANDBY_BACKSTOP_MS (covers
 *     an idle network and a standby that boots into a dead one -- the clock runs
 *     from boot until the first root frame, because s_root_alive_ms starts
 *     there). */
static void mesh_standby_tick(uint32_t now)
{
    if (s_probe_active && (int32_t)(now - s_probe_deadline) >= 0) {
        s_probe_active = false;         /* expired unconfirmed: a miss */
        s_probe_miss++;
        if (s_probe_miss == 1) {
            s_probe_first_ms = now;
        }
        if (s_probe_miss >= MESH_STANDBY_MISS_LIMIT &&
            (uint32_t)(now - s_probe_first_ms) >= MESH_STANDBY_MIN_WINDOW_MS) {
            mesh_standby_takeover();
            return;
        }
    }
    if ((uint32_t)(now - s_root_alive_ms) >= MESH_STANDBY_BACKSTOP_MS) {
        mesh_standby_takeover();
    }
}

/* Promote the standby to root.  Clearing the standby role stops the observer
 * paths; mesh_set_root(true) gives us address 1 and -- via mesh_apply_role --
 * a fresh boot epoch, so every node sees the restart, re-adopts its address and
 * re-claims (the root's table is RAM-only).  mesh_state_clear() drops the
 * observer state and schedules the first beacon / ADDR_TABLE immediately, and
 * the save persists the promotion across a power-cycle. */
static void mesh_standby_takeover(void)
{
    s_cfg.flags &= (uint8_t)~MESH_FLAG_STANDBY;
    /* Force a fresh epoch even if this device was an active root earlier in the
     * power cycle (mesh_apply_role only bumps while s_boot_epoch is still 0), so
     * the nodes always see a change and re-claim. */
    s_boot_epoch = 0;
    mesh_set_root(true);
    mesh_state_clear();
    mesh_update_state();
    mesh_config_save();
}

/* ======================================================================= */
/*  M8: root conflict (lowest device id wins)                            */
/* ======================================================================= */

/* Total order on two device ids: -1 if a < b, +1 if a > b, 0 if equal.  Both
 * roots compute it identically, so exactly one steps down and the result is
 * stable -- no tie-break, no oscillation. */
static int mesh_devid_cmp(const uint8_t *a, const uint8_t *b)
{
    int c = memcmp(a, b, MESH_NODE_DEVICE_ID_LEN);
    return (c < 0) ? -1 : (c > 0) ? 1 : 0;
}

/* Advertise our identity so a rival root can compare device ids. */
static void mesh_root_announce_send(void)
{
    uint8_t own[MESH_NODE_DEVICE_ID_LEN];

    mesh_get_node_device_id(own);
    mesh_send_broadcast(MESH_TYPE_ROOT_ANNOUNCE, 1, own, sizeof(own));
}

/* A root with a lower device id exists: stand down and observe it.  We become
 * a standby (M7) -- silent, shadowing, and able to take over again if that root
 * later dies -- and arm its heartbeat (we have just heard a root). */
static void mesh_root_demote_to_standby(void)
{
    mesh_set_standby(true);             /* clears root + address, resets state */
    s_root_seen     = true;
    s_root_alive_ms = millis();
    mesh_config_save();                 /* persist the role across a reboot      */
}

/* A rival asked "who is root?".  Lowest device id wins: if we are junior we
 * stand down; if we are senior we stop contending and answer. */
static void mesh_rx_root_query(const mesh_header_t *h, const uint8_t *payload, uint8_t plen)
{
    uint8_t own[MESH_NODE_DEVICE_ID_LEN];

    if (!mesh_is_root() || h->src != MESH_ROOT_ADDR ||
        plen < MESH_NODE_DEVICE_ID_LEN) {
        return;                         /* a query is for roots only */
    }
    mesh_get_node_device_id(own);
    if (mesh_devid_cmp(payload, own) < 0) {
        mesh_root_demote_to_standby();
        return;
    }
    s_root_serving   = true;          /* we are senior: assert and reply */
    s_contend_until_ms = 0;
    mesh_root_announce_send();
}

/* A rival stated its identity.  If it is senior, stand down; else it is the one
 * that yields (it will hear our announce / query in turn). */
static void mesh_rx_root_announce(const mesh_header_t *h, const uint8_t *payload, uint8_t plen)
{
    uint8_t own[MESH_NODE_DEVICE_ID_LEN];

    if (!mesh_is_root() || h->src != MESH_ROOT_ADDR ||
        plen < MESH_NODE_DEVICE_ID_LEN) {
        return;
    }
    mesh_get_node_device_id(own);
    if (mesh_devid_cmp(payload, own) < 0) {
        mesh_root_demote_to_standby();
    }
}

/* Tick for a root that has not yet proven it is alone: broadcast the query so
 * a rival answers, then assert if nothing senior turns up within the window. */
static void mesh_root_contend_tick(uint32_t now)
{
    uint8_t own[MESH_NODE_DEVICE_ID_LEN];

    if (s_contend_until_ms == 0) {      /* arm on the first tick */
        s_contend_until_ms = now + MESH_ROOT_QUERY_WINDOW_MS;
        /* Jitter the first query so two roots booting together do not answer
         * each other in lockstep. */
        s_contend_next_ms  = now + (mesh_rand() % MESH_ROOT_QUERY_RETRY_MS);
        return;
    }
    if ((int32_t)(now - s_contend_next_ms) >= 0) {
        mesh_get_node_device_id(own);
        mesh_send_broadcast(MESH_TYPE_ROOT_QUERY, 1, own, sizeof(own));
        s_contend_next_ms = now + MESH_ROOT_QUERY_RETRY_MS;
    }
    if ((int32_t)(now - s_contend_until_ms) >= 0) {
        s_root_serving   = true;      /* nobody senior answered: we are root */
        s_contend_until_ms = 0;
    }
}

uint16_t mesh_get_epoch(void)
{
    return mesh_is_root() ? s_boot_epoch : s_root_epoch;
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

    /* Root liveness bookkeeping: refresh the last-heard time for a node as
     * long as it sends anything (beacon, uplink, claim or liveness aggregate);
     * mesh_rx_alive additionally refreshes every bit of a child's aggregate. */
    if (mesh_is_root() && h.src >= MESH_FIRST_NONROOT_ADDR && h.src <= MESH_ADDR_MAX) {
        s_heard_ms[h.src] = millis();
    }
    /* Standby root (M7): any frame from the root (beacon, downlink, ACK,
     * table) refreshes the backstop heartbeat. */
    if (mesh_is_standby() && h.src == MESH_ROOT_ADDR) {
        s_root_alive_ms = millis();
        s_root_seen     = true;
    }

    payload = buf + MESH_HEADER_SIZE;
    plen    = (uint8_t)(len - MESH_HEADER_SIZE);

    /* Link ACK (checked before dedup): clear an in-flight frame when we hear it
     * carried on, or -- when it has no next hop left to be carried to --
     * explicitly acked by whoever terminates it.  Both directions ack alike (see
     * mesh_wire_ack_key); a pending is matched on the origin it is keyed to, so
     * an uplink's ACK can never clear a downlink's pending or vice versa. */
    for (uint8_t slot = 0; slot < MESH_PEND_N; slot++) {
        mesh_pending_t *pend = &s_pending[slot];
        uint8_t ack_src, ack_seq;

        if (pend->active &&
            mesh_wire_ack_key(&h, payload, plen, mesh_get_address(), &ack_src, &ack_seq) &&
            ack_src == pend->src && ack_seq == pend->seq) {
            pend->active = false;
        }
    }

    /* Dedup flooded frames on (src,seq).  JOIN_REQ carries src=0 (unassigned)
     * so it is keyed on a hash of its node device id instead; beacons are link-local
     * and never deduped. */
    if (h.type == MESH_TYPE_JOIN_REQ) {
        if (plen < MESH_NODE_DEVICE_ID_LEN || mesh_dedup_check(mesh_node_device_id_hash8(payload), h.seq)) {
            return;                     /* too short or duplicate */
        }
    } else if (h.type != MESH_TYPE_BEACON) {
        if (mesh_dedup_check(h.src, h.seq)) {
            return;                     /* duplicate */
        }
    }

    /* A root that has not yet passed its solo check (M8) serves nothing: it
     * handles only the conflict handshake, so a booting root cannot hand out
     * addresses or relay while it is still undecided. */
    if (mesh_is_root() && !s_root_serving &&
        h.type != MESH_TYPE_ROOT_QUERY &&
        h.type != MESH_TYPE_ROOT_ANNOUNCE) {
        return;
    }

    /* A frame sourced from address 1 that we did not send (a node never hears
     * its own frames) proves a rival root exists.  A serving root answers a
     * rival's beacon with its identity (M8) so the two can compare and the
     * junior one stand down; a contending root waits for the handshake above. */
    if (mesh_is_root() && s_root_serving && h.src == MESH_ROOT_ADDR &&
        h.type == MESH_TYPE_BEACON) {
        mesh_root_announce_send();
    }

    /* A standby root (M7) is a silent shadow root, not a routing node: it
     * skips the join / neighbour / routing control types entirely and only
     * observes -- it delivers data and absorbs the rootward control frames that
     * reach the root (ADDR_ASSIGN / ADDR_CLAIM / ALIVE / TOPOLOGY, handled via
     * mesh_addressed_to_me / mesh_at_root inside the handlers). */
    if (mesh_is_standby() &&
        (h.type == MESH_TYPE_BEACON ||
         h.type == MESH_TYPE_JOIN_REQ ||
         h.type == MESH_TYPE_ADDR_TABLE)) {
        return;
    }

    switch (h.type) {
    case MESH_TYPE_BEACON:        mesh_rx_beacon(&h, payload, plen);       break;
    case MESH_TYPE_JOIN_REQ:      mesh_rx_join_req(&h, payload, plen);     break;
    case MESH_TYPE_ADDR_ASSIGN:   mesh_rx_addr_assign(&h, payload, plen);  break;
    case MESH_TYPE_ADDR_TABLE:    mesh_rx_addr_table(&h, payload, plen);   break;
    case MESH_TYPE_DATA_UPLINK:   mesh_rx_data_uplink(&h, payload, plen);  break;
    case MESH_TYPE_DATA_DOWNLINK: mesh_rx_data_downlink(&h, payload, plen);break;
    case MESH_TYPE_LINK_ACK:      /* cleared the pending above; nothing else */ break;
    case MESH_TYPE_ADDR_CLAIM:    mesh_rx_addr_claim(&h, payload, plen);   break;
    case MESH_TYPE_ADDR_ALIVE:    mesh_rx_alive(&h, payload, plen);        break;
    case MESH_TYPE_TOPOLOGY:      mesh_rx_topology(&h, payload, plen);     break;
    case MESH_TYPE_ROOT_QUERY:  mesh_rx_root_query(&h, payload, plen);    break;
    case MESH_TYPE_ROOT_ANNOUNCE: mesh_rx_root_announce(&h, payload, plen);break;
    default:                                                               break;
    }
}
