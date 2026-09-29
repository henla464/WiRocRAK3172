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

/* Defined in the M2 section below; used by the timer. */
static bool mesh_send_broadcast(uint8_t type, const uint8_t *payload, uint8_t plen);

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
    if (mesh_is_master()) {
        /* Periodically flood the occupied bitmap so nodes can reconcile. */
        if ((int32_t)(now - s_table_next_ms) >= 0) {
            uint8_t bm[4];
            mesh_alloc_bitmap(bm);
            mesh_send_broadcast(MESH_TYPE_ADDR_TABLE, bm, 4);
            s_table_next_ms = now + MESH_TABLE_INTERVAL_MS;
        }
    } else if (s_state == MESH_STATE_JOINING) {
        /* Ask the master for an address, backing off between attempts. */
        if ((int32_t)(now - s_join_next_ms) >= 0) {
            mesh_send_broadcast(MESH_TYPE_JOIN_REQ, s_token, MESH_TOKEN_LEN);
            if (s_join_interval_ms < MESH_JOIN_INTERVAL_MAX_MS) {
                s_join_interval_ms += MESH_JOIN_INTERVAL_MS;
                if (s_join_interval_ms > MESH_JOIN_INTERVAL_MAX_MS) {
                    s_join_interval_ms = MESH_JOIN_INTERVAL_MAX_MS;
                }
            }
            s_join_next_ms = now + s_join_interval_ms;
        }
    }
    /* Later milestones: beacon emission, retries, parent staleness here. */
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

/* Broadcast a control frame (dst = NONE) with an auto-incremented seq. */
static bool mesh_send_broadcast(uint8_t type, const uint8_t *payload, uint8_t plen)
{
    uint8_t frame[MESH_MAX_FRAME];
    uint8_t seq = s_bcast_seq;
    uint8_t len;

    s_bcast_seq = (uint8_t)((s_bcast_seq + 1) & 0x0F);
    len = mesh_build_frame(frame, type, 0, mesh_get_address(),
                           MESH_ADDR_NONE, 0, seq, payload, plen);
    return mesh_send_frame(frame, len);
}

/* Master-only: a joining node asks for an address (payload = token). */
static void mesh_rx_join_req(const uint8_t *payload, uint8_t plen)
{
    uint8_t addr;
    uint8_t out[MESH_TOKEN_LEN + 1];

    if (!mesh_is_master() || plen < MESH_TOKEN_LEN) {
        return;
    }
    addr = mesh_alloc_assign(payload);
    if (addr == MESH_ADDR_NONE) {
        return;                         /* pool exhausted */
    }
    memcpy(out, payload, MESH_TOKEN_LEN);
    out[MESH_TOKEN_LEN] = addr;
    mesh_send_broadcast(MESH_TYPE_ADDR_ASSIGN, out, (uint8_t)(MESH_TOKEN_LEN + 1));
}

/* Node: adopt an address the master assigned to our token. */
static void mesh_rx_addr_assign(const uint8_t *payload, uint8_t plen)
{
    uint8_t addr;

    if (mesh_is_master() || plen < MESH_TOKEN_LEN + 1) {
        return;
    }
    if (memcmp(payload, s_token, MESH_TOKEN_LEN) != 0) {
        return;                         /* addressed to another node */
    }
    addr = (uint8_t)(payload[MESH_TOKEN_LEN] & MESH_ADDR_MASK);
    if (addr < MESH_FIRST_SLAVE_ADDR) {
        return;
    }
    if (mesh_get_address() != addr) {
        mesh_set_address(addr);
        mesh_config_save();
    }
    mesh_update_state();
}

/* Node: reconcile our address with the master's occupied bitmap. */
static void mesh_rx_addr_table(const uint8_t *payload, uint8_t plen)
{
    uint8_t addr = mesh_get_address();

    if (mesh_is_master() || plen < 4) {
        return;
    }
    if (addr == MESH_ADDR_NONE) {
        mesh_update_state();
        return;
    }
    if (payload[addr >> 3] & (uint8_t)(1u << (addr & 7))) {
        s_table_miss = 0;               /* master still knows us */
        return;
    }
    /* Our address vanished from the master's table.  A single lost flood is
     * normal, so only relinquish after MESH_TABLE_MISS_LIMIT consecutive
     * misses (M5 will refine this with an explicit grace policy). */
    if (++s_table_miss >= MESH_TABLE_MISS_LIMIT) {
        s_table_miss = 0;
        mesh_set_address(MESH_ADDR_NONE);
        mesh_config_save();
        mesh_update_state();
    }
}

void mesh_handle_rx(const uint8_t *buf, uint16_t len)
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
    if (mesh_dedup_check(h.src, h.seq)) {
        return;                         /* duplicate */
    }

    payload = buf + MESH_HEADER_SIZE;
    plen    = (uint8_t)(len - MESH_HEADER_SIZE);

    switch (h.type) {
    case MESH_TYPE_JOIN_REQ:    mesh_rx_join_req(payload, plen);    break;
    case MESH_TYPE_ADDR_ASSIGN: mesh_rx_addr_assign(payload, plen); break;
    case MESH_TYPE_ADDR_TABLE:  mesh_rx_addr_table(payload, plen);  break;
    case MESH_TYPE_BEACON:      /* M3 */                            break;
    case MESH_TYPE_DATA_UPLINK: /* M3 */                            break;
    case MESH_TYPE_DATA_DOWNLINK:/* M4 */                           break;
    case MESH_TYPE_LINK_ACK:    /* M4 */                            break;
    case MESH_TYPE_MAP_REQ:     /* later */                         break;
    default:                                                        break;
    }
}
