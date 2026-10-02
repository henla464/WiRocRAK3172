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
#define MESH_FLAG_MASTER        0x02

/* ======================================================================= */
/*  Persistent configuration                                              */
/* ======================================================================= */

/* Persistent layout: magic, version, flags, address, boot counter, device id. */
struct __attribute__((packed)) mesh_flash_config_t {
    uint8_t  magic;
    uint8_t  version;
    uint8_t  flags;      /* MESH_FLAG_* */
    uint8_t  address;    /* own 4-bit address, MESH_ADDR_NONE when unassigned */
    uint16_t boot_count; /* master boot counter: bumped each boot, used as the
                          * beacon epoch so nodes detect a master restart */
    uint8_t  device_id[MESH_NODE_DEVICE_ID_LEN]; /* host-provisioned device id */
};

static mesh_flash_config_t s_cfg;

static void mesh_config_defaults(void)
{
    s_cfg.magic      = MESH_FLASH_MAGIC;
    s_cfg.version    = MESH_FLASH_VERSION;
    s_cfg.flags      = 0;               /* meshing disabled, not master */
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
static mesh_route_t s_route;                    /* our route to the master    */
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

/* Radio/beacon helpers defined further down; used by the timer. */
static void mesh_refresh_radio_params(void);
static uint32_t mesh_frame_airtime_ms(uint8_t len);
static uint32_t mesh_link_ack_timeout_for(uint8_t len);
static void mesh_beacon_fast(void);
static bool mesh_is_relay(void);

/* Recompute the join FSM state from the current role / address / device id. */
static void mesh_update_state(void)
{
    if (!mesh_is_enabled() || !mesh_has_node_device_id()) {
        /* Without a host-provisioned node device id the mesh does not run: no
         * beacons and no join attempt.  It is how the master identifies us. */
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
    s_beacon_interval_ms = MESH_BEACON_FAST_MS;  /* start fast, back off */
    s_relay_last_ms  = millis() - MESH_RELAY_HOLD_MS;  /* a fresh node is a leaf */
    s_data_seq       = 0;
    s_rx_rssi        = 0;
    s_rx_snr         = 0;
    s_air_ctrl       = 0;
    s_air_data       = 0;
    s_tx_next_ms     = 0;
    s_tx_backoff_ms  = MESH_TX_BACKOFF_MIN_MS;
    s_rand_state     = 0x9E3779B9u ^ millis();  /* per-node, non-zero */
    if (s_rand_state == 0) {
        s_rand_state = 0x1234567u;
    }
    memset(&s_pending, 0, sizeof(s_pending));

    mesh_refresh_radio_params();

    /* M5 recovery state.  s_boot_epoch is *not* reset here: it is owned by
     * mesh_apply_role() and must survive an enable/disable cycle. */
    s_master_epoch   = 0;
    s_epoch_valid    = false;
    s_claim_next_ms  = 0;
    s_had_parent     = false;
    s_boot_ms        = millis();
    memset(s_heard_ms, 0, sizeof(s_heard_ms));

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

static void mesh_timer_cb(void *)
{
    uint32_t now;

    if (!mesh_is_enabled() || !mesh_has_node_device_id()) {
        return;
    }
    mesh_refresh_radio_params();
    mesh_tx_drain();

    now = millis();

    /* Beacons: the master and every routable node advertise periodically so
     * neighbours can pick parents and detect a node going down.  The interval
     * backs off toward MESH_BEACON_MAX_MS while stable, and is reset fast on
     * any topology change (join, re-attach, epoch change). */
    if ((int32_t)(now - s_beacon_next_ms) >= 0) {
        bool recovering = mesh_is_master() &&
                          (uint32_t)(now - s_boot_ms) < MESH_RECOVER_MS;
        uint32_t interval;

        if (recovering) {
            interval = MESH_BEACON_FAST_MS;
        } else {
            interval = s_beacon_interval_ms;
            /* A leaf (no children) is on nobody's path: it only needs to
             * advertise itself as a potential parent occasionally. */
            if (!mesh_is_master() && !mesh_is_relay()) {
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
            /* Ask the master for an address, backing off between attempts. */
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

/* A node is a relay while it has recently forwarded a rootward unicast (an
 * uplink or a claim) addressed to it: only a node that selected us as its parent
 * ever does that.  A childless node is a leaf and beacons more slowly. */
static bool mesh_is_relay(void)
{
    return (uint32_t)(millis() - s_relay_last_ms) < MESH_RELAY_HOLD_MS;
}

/* Record that we just forwarded for a child.  On the leaf->relay transition,
 * reset the beacon to the normal rate so the new child can track us promptly. */
static void mesh_note_relay(void)
{
    bool was = mesh_is_relay();

    s_relay_last_ms = millis();
    if (!was) {
        mesh_beacon_fast();
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
    len = mesh_build_frame(frame, type, 0, mesh_get_address(), dst, hops, seq,
                           payload, plen);
    mesh_mark_seen(type, mesh_get_address(), seq, payload);
    return mesh_send_frame(frame, len);
}

static bool mesh_send_broadcast(uint8_t type, uint8_t hops,
                                const uint8_t *payload, uint8_t plen)
{
    return mesh_send_flood(type, MESH_ADDR_NONE, hops, payload, plen);
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

/* A joining node asks for an address (payload = devid, flooded).  The master
 * allocates and floods ADDR_ASSIGN; every node relays the request onward. */
static void mesh_rx_join_req(const mesh_header_t *h, const uint8_t *payload, uint8_t plen)
{
    if (plen < MESH_NODE_DEVICE_ID_LEN) {
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

/* Node: adopt an address the master assigned to our node device id (flooded). */
static void mesh_rx_addr_assign(const mesh_header_t *h, const uint8_t *payload, uint8_t plen)
{
    uint8_t addr;

    if (plen < MESH_NODE_DEVICE_ID_LEN) {
        return;
    }
    addr = (uint8_t)(h->dst & MESH_ADDR_MASK);
    if (!mesh_is_master()) {
        if (memcmp(payload, s_cfg.device_id, MESH_NODE_DEVICE_ID_LEN) == 0) {
            if (addr >= MESH_FIRST_SLAVE_ADDR) {
                if (mesh_get_address() != addr) {
                    mesh_set_address(addr);
                    mesh_config_save();
                }
                mesh_update_state();
                mesh_beacon_fast();     /* announce the new attachment quickly */
            }
        } else if (addr >= MESH_FIRST_SLAVE_ADDR && addr == mesh_get_address()) {
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

/* Node: reconcile our address with the master's occupied bitmap (flooded). */
static void mesh_rx_addr_table(const mesh_header_t *h, const uint8_t *payload, uint8_t plen)
{
    uint8_t addr = mesh_get_address();

    if (plen < 2) {
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

/* Emit this node's beacon: only routable nodes (master, or a node with a
 * parent) advertise, so advertised costs are always meaningful.  The 1-byte
 * payload packs the master epoch (3 bits) over the path cost (5 bits). */
static void mesh_emit_beacon(void)
{
    uint8_t payload[MESH_BEACON_LEN];

    if (mesh_is_master()) {
        payload[0] = (uint8_t)((s_boot_epoch & 0x07) << 5);     /* cost 0 */
        mesh_send_broadcast(MESH_TYPE_BEACON, 0, payload, MESH_BEACON_LEN);
        return;
    }
    if (s_route.parent_addr != MESH_ADDR_NONE) {
        payload[0] = (uint8_t)(((s_master_epoch & 0x07) << 5) |
                               (s_route.self_cost & 0x1F));
        mesh_send_broadcast(MESH_TYPE_BEACON, s_route.self_hops, payload,
                            MESH_BEACON_LEN);
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
    epoch = (uint8_t)((payload[0] >> 5) & 0x07);    /* 3-bit master epoch */
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

    mesh_check_parent_staleness(now);
    mesh_select_parent();

    /* A parent change (or a fresh attachment) resets the beacon to fast. */
    if (s_route.parent_addr != old_parent) {
        mesh_beacon_fast();
    }

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
    s_pending.next_ms = millis() + mesh_link_ack_timeout_for(len);
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
    s_pending.next_ms = now + mesh_link_ack_timeout_for(s_pending.len);
}

bool mesh_send_uplink(const uint8_t *payload, uint8_t len)
{
    uint8_t frame[MESH_MAX_FRAME];
    uint8_t flen;
    uint8_t seq;

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
    s_data_seq = (uint8_t)((s_data_seq + 1) & 0x1F);
    flen = mesh_build_frame(frame, MESH_TYPE_DATA_UPLINK, MESH_FLAG_ACK_REQ,
                            mesh_get_address(), s_route.parent_addr,
                            MESH_DEFAULT_TTL, seq, payload, len);

    /* Host message: one immediate attempt.  If the channel is busy we return
     * false so the host backs off and resends -- the module does not queue or
     * retry it. */
    if (!mesh_tx_radio_send(frame, flen)) {
        return false;
    }
    /* It is on the air: arm the pending entry so a lost hop is still retried
     * by the mesh MAC (implicit link ACK), independent of the host. */
    mesh_set_pending(frame, flen, mesh_get_address(), seq);
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

/* Explicit link ACK: the master has no next hop to forward to, so it acks the
 * uplink it just delivered by (origin,seq); relays clear on the same key.
 * The acked origin rides in the header `dst`, so the payload is just the seq. */
static void mesh_send_link_ack(uint8_t acked_src, uint8_t acked_seq)
{
    uint8_t frame[MESH_MAX_FRAME];
    uint8_t payload[1];
    uint8_t flen;

    payload[0] = acked_seq;
    flen = mesh_build_frame(frame, MESH_TYPE_LINK_ACK, 0, mesh_get_address(),
                            acked_src, 0, 0, payload, 1);
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
    mesh_note_relay();                  /* a child routed through us */
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
    s_data_seq = (uint8_t)((s_data_seq + 1) & 0x1F);
    flen = mesh_build_frame(frame, MESH_TYPE_DATA_DOWNLINK, 0, mesh_get_address(),
                            dst, MESH_DEFAULT_TTL, seq, payload, len);
    mesh_mark_seen(MESH_TYPE_DATA_DOWNLINK, mesh_get_address(), seq, payload);

    /* Host-originated: one immediate attempt; a busy channel goes back to the
     * host as a busy status (it owns the backoff / resend). */
    return mesh_tx_radio_send(frame, flen);
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
    mesh_relay_flood_gated(h, payload, plen);
}

/* ======================================================================= */
/*  M5: recovery / robustness                                             */
/* ======================================================================= */

/* Master: apply a node's claim to the RAM-only table.  Adopt the claimed
 * address when it is free, re-assert our existing assignment when the node device
 * id is already known, and fall back to a fresh allocation when the claimed address
 * is already taken.  Shared by the unicast and flooded claim paths. */
static void mesh_master_apply_claim(const mesh_header_t *h, const uint8_t *payload)
{
    uint8_t want = (uint8_t)(h->src & MESH_ADDR_MASK);  /* node's own address */
    uint8_t have;

    if (want < MESH_FIRST_SLAVE_ADDR || want > MESH_ADDR_MAX) {
        return;
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

/* Node: re-announce our flash-stored address so a restarted master can rebuild
 * its RAM-only table.  Our address is already in the header `src`; the payload
 * is just our node device id.
 *
 * This is a rootward message: it only has to reach the master, so we send it
 * hop-by-hop toward our parent (same discipline as DATA_UPLINK) rather than
 * flooding -- the network-wide cost is O(hops) per claim instead of O(N).  When
 * we have no route yet (just booted, or just lost our parent) we fall back to a
 * flood so a route-less node is still discoverable.  Best-effort either way: the
 * claim repeats on the periodic safety net. */
static void mesh_send_claim(void)
{
    uint8_t frame[MESH_MAX_FRAME];
    uint8_t flen;
    uint8_t seq;

    if (mesh_is_master() || mesh_get_address() == MESH_ADDR_NONE) {
        return;
    }
    if (s_state != MESH_STATE_JOINED || s_route.parent_addr == MESH_ADDR_NONE) {
        /* No route: flood so the master can still find us. */
        mesh_send_broadcast(MESH_TYPE_ADDR_CLAIM, MESH_DEFAULT_TTL,
                            s_cfg.device_id, MESH_NODE_DEVICE_ID_LEN);
    } else {
        /* Rootward unicast to the parent. */
        seq = s_bcast_seq;
        s_bcast_seq = (uint8_t)((s_bcast_seq + 1) & 0x1F);
        flen = mesh_build_frame(frame, MESH_TYPE_ADDR_CLAIM, 0,
                                mesh_get_address(), s_route.parent_addr,
                                MESH_DEFAULT_TTL, seq, s_cfg.device_id, MESH_NODE_DEVICE_ID_LEN);
        mesh_send_frame(frame, flen);
    }
    s_claim_next_ms = millis() + MESH_CLAIM_INTERVAL_MS;
}

/* Handle an incoming ADDR_CLAIM.  The master applies it to its table; an
 * intermediate node forwards a rootward unicast one hop toward its parent (or
 * converts it to a flood if it has lost its own parent); the flooded fallback
 * is relayed by everyone. */
static void mesh_rx_addr_claim(const mesh_header_t *h, const uint8_t *payload, uint8_t plen)
{
    uint8_t frame[MESH_MAX_FRAME];
    uint8_t flen;

    if (plen < MESH_NODE_DEVICE_ID_LEN) {
        return;
    }

    /* Flooded fallback (sent by a node that has no route yet): everyone relays
     * and the master rebuilds. */
    if (h->dst == MESH_ADDR_NONE) {
        if (mesh_is_master()) {
            mesh_master_apply_claim(h, payload);
        }
        mesh_relay_flood(h, payload, plen);
        return;
    }

    /* Rootward unicast: only the addressed next hop acts. */
    if (h->dst != mesh_get_address()) {
        return;                         /* not addressed to us */
    }
    if (mesh_is_master()) {
        mesh_master_apply_claim(h, payload);
        return;
    }
    mesh_note_relay();                  /* a child routed through us */
    if (h->hops <= 1) {
        return;                         /* TTL exhausted */
    }
    if (s_route.parent_addr == MESH_ADDR_NONE) {
        /* We cannot forward rootward right now: fall back to a flood (origin's
         * src preserved, dst cleared) so the claim can still reach the master
         * via another path. */
        flen = mesh_build_frame(frame, MESH_TYPE_ADDR_CLAIM, 0, h->src,
                                MESH_ADDR_NONE, (uint8_t)(h->hops - 1), h->seq,
                                payload, plen);
        mesh_send_frame(frame, flen);
        return;
    }
    flen = mesh_build_frame(frame, MESH_TYPE_ADDR_CLAIM, 0, h->src,
                            s_route.parent_addr, (uint8_t)(h->hops - 1), h->seq,
                            payload, plen);
    mesh_send_frame(frame, flen);       /* best effort; no pending retry */
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
        } else if (h.type == MESH_TYPE_LINK_ACK && plen >= 1) {
            ack_src = h.dst;            /* acked origin is in the header dst */
            ack_seq = payload[0];
        }
        if (ack_src == s_pending.src && ack_seq == s_pending.seq &&
            (h.type == MESH_TYPE_LINK_ACK || h.dst != mesh_get_address())) {
            s_pending.active = false;
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
