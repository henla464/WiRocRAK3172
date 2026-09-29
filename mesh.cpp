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
#include "mesh.h"
#include "mesh_alloc.h"
#include "mesh_route.h"
#include "MessageQueue.h"

#define MESH_FLAG_ENABLED       0x01
#define MESH_FLAG_MASTER        0x02

/* ======================================================================= */
/*  Persistent configuration                                              */
/* ======================================================================= */

/* Persistent layout (4 bytes). */
struct __attribute__((packed)) mesh_flash_config_t {
    uint8_t magic;
    uint8_t version;
    uint8_t flags;      /* MESH_FLAG_* */
    uint8_t address;    /* own 5-bit address, MESH_ADDR_NONE when unassigned */
};

static mesh_flash_config_t s_cfg;

static void mesh_config_defaults(void)
{
    s_cfg.magic   = MESH_FLASH_MAGIC;
    s_cfg.version = MESH_FLASH_VERSION;
    s_cfg.flags   = 0;                  /* meshing disabled, not master */
    s_cfg.address = MESH_ADDR_NONE;
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

/* Defined in the M3 section below; used by the timer. */
static void mesh_emit_beacon(void);
static void mesh_check_parent_staleness(uint32_t now);
static void mesh_deliver_to_host(const uint8_t *payload, uint8_t len, uint8_t src);

/* Defined in the M2 section below; used by the timer. */
static bool mesh_send_broadcast(uint8_t type, uint8_t hops,
                                const uint8_t *payload, uint8_t plen);

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
        mesh_emit_beacon();
        s_beacon_next_ms = now + MESH_BEACON_INTERVAL_MS;
    }
    mesh_check_parent_staleness(now);

    if (mesh_is_master()) {
        /* Periodically flood the occupied bitmap so nodes can reconcile. */
        if ((int32_t)(now - s_table_next_ms) >= 0) {
            uint8_t bm[4];
            mesh_alloc_bitmap(bm);
            mesh_send_broadcast(MESH_TYPE_ADDR_TABLE, MESH_DEFAULT_TTL, bm, 4);
            s_table_next_ms = now + MESH_TABLE_INTERVAL_MS;
        }
    } else if (s_state == MESH_STATE_JOINING) {
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
    }
    /* M4: link-ACK retry timers and implicit-ACK handling go here. */
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
 * Derived from the STM32 hardware id via FNV-1a so it is unique per board and
 * needs no LoRaWAN/EUI support (this build is P2P-only). */
static void mesh_token_load(void)
{
    String id = api.system.chipId.get();
    uint64_t h = 1469598103934665603ULL;    /* FNV-1a 64-bit offset basis */
    uint8_t i;

    for (unsigned k = 0; k < id.length(); k++) {
        h ^= (uint8_t)id[k];
        h *= 1099511628211ULL;
    }
    for (i = 0; i < MESH_TOKEN_LEN; i++) {
        s_token[i] = (uint8_t)(h >> (i * 8));
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
static bool mesh_send_broadcast(uint8_t type, uint8_t hops,
                                const uint8_t *payload, uint8_t plen)
{
    uint8_t frame[MESH_MAX_FRAME];
    uint8_t seq = s_bcast_seq;
    uint8_t len;

    s_bcast_seq = (uint8_t)((s_bcast_seq + 1) & 0x0F);
    len = mesh_build_frame(frame, type, 0, mesh_get_address(),
                           MESH_ADDR_NONE, hops, seq, payload, plen);
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
    flen = mesh_build_frame(frame, h->type, h->flags, h->src, MESH_ADDR_NONE,
                            (uint8_t)(h->hops - 1), h->seq, payload, plen);
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

/* A joining node asks for an address (payload = token, flooded).  The master
 * allocates and floods ADDR_ASSIGN; every node relays the request onward. */
static void mesh_rx_join_req(const mesh_header_t *h, const uint8_t *payload, uint8_t plen)
{
    if (plen < MESH_TOKEN_LEN) {
        return;
    }
    if (mesh_is_master()) {
        uint8_t addr = mesh_alloc_assign(payload);
        if (addr != MESH_ADDR_NONE) {
            uint8_t out[MESH_TOKEN_LEN + 1];
            memcpy(out, payload, MESH_TOKEN_LEN);
            out[MESH_TOKEN_LEN] = addr;
            mesh_send_broadcast(MESH_TYPE_ADDR_ASSIGN, MESH_DEFAULT_TTL,
                                out, (uint8_t)(MESH_TOKEN_LEN + 1));
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
    if (!mesh_is_master() && memcmp(payload, s_token, MESH_TOKEN_LEN) == 0) {
        addr = (uint8_t)(payload[MESH_TOKEN_LEN] & MESH_ADDR_MASK);
        if (addr >= MESH_FIRST_SLAVE_ADDR) {
            if (mesh_get_address() != addr) {
                mesh_set_address(addr);
                mesh_config_save();
            }
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
        mesh_select_parent();
    }
}

/* Emit this node's beacon: only routable nodes (master, or a node with a
 * parent) advertise, so advertised costs are always meaningful. */
static void mesh_emit_beacon(void)
{
    uint8_t payload[1];

    if (mesh_is_master()) {
        payload[0] = 0;                 /* master: cost 0, hops 0 */
        mesh_send_broadcast(MESH_TYPE_BEACON, 0, payload, 1);
        return;
    }
    if (s_route.parent_addr != MESH_ADDR_NONE) {
        payload[0] = s_route.self_cost;
        mesh_send_broadcast(MESH_TYPE_BEACON, s_route.self_hops, payload, 1);
    }
}

static void mesh_rx_beacon(const mesh_header_t *h, const uint8_t *payload, uint8_t plen)
{
    mesh_route_neighbor_t *n;
    int16_t sample;
    uint32_t now = millis();

    if (h->src == mesh_get_address() || plen < 1) {
        return;
    }
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
    n->cost      = payload[0];
    n->hops      = h->hops;
    n->link_cost = mesh_link_cost(n->snr_x10);
    n->last_ms   = now;

    mesh_check_parent_staleness(now);
    mesh_select_parent();
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
    s_data_seq = (uint8_t)((s_data_seq + 1) & 0x0F);
    flen = mesh_build_frame(frame, MESH_TYPE_DATA_UPLINK, 0, mesh_get_address(),
                            s_route.parent_addr, MESH_DEFAULT_TTL, seq, payload, len);
    return mesh_send_frame(frame, flen);
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

static void mesh_rx_data_uplink(const mesh_header_t *h, const uint8_t *payload, uint8_t plen)
{
    uint8_t frame[MESH_MAX_FRAME];
    uint8_t flen;

    if (mesh_is_master()) {
        mesh_deliver_to_host(payload, plen, h->src);
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
    mesh_send_frame(frame, flen);
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

    payload = buf + MESH_HEADER_SIZE;
    plen    = (uint8_t)(len - MESH_HEADER_SIZE);

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
    case MESH_TYPE_BEACON:       mesh_rx_beacon(&h, payload, plen);       break;
    case MESH_TYPE_JOIN_REQ:     mesh_rx_join_req(&h, payload, plen);     break;
    case MESH_TYPE_ADDR_ASSIGN:  mesh_rx_addr_assign(&h, payload, plen);  break;
    case MESH_TYPE_ADDR_TABLE:   mesh_rx_addr_table(&h, payload, plen);   break;
    case MESH_TYPE_DATA_UPLINK:  mesh_rx_data_uplink(&h, payload, plen);  break;
    case MESH_TYPE_DATA_DOWNLINK:/* M4 */                                 break;
    case MESH_TYPE_LINK_ACK:     /* M4 */                                 break;
    case MESH_TYPE_MAP_REQ:      /* later */                              break;
    default:                                                              break;
    }
}
