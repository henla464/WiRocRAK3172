/*
 * mesh.h
 *
 *  LoRa mesh mode for WiRocRAK3172.
 *
 *  Convergecast tree rooted at a single master (gateway). 4-bit addresses are
 *  assigned by the master; all nodes route their messages toward the master.
 *
 *  The flash-persisted configuration holds mesh enabled / role / own address
 *  and the host-provisioned 6-byte identity token; the accessors here are used
 *  by the custom AT commands.
 */

#ifndef INC_MESH_H_
#define INC_MESH_H_

#include <stdint.h>
#include <stdbool.h>
#include "mesh_wire.h"

/* --- Address space (4 bits, values 0..15) ------------------------------- */
#define MESH_ADDR_BITS          4
#define MESH_ADDR_MASK          0x0F
#define MESH_ADDR_MAX           0x0F

#define MESH_ADDR_NONE          0x00    /* unassigned / "default: send to master" */
#define MESH_MASTER_ADDR        0x01    /* well-known root (gateway) address       */
#define MESH_FIRST_SLAVE_ADDR   0x02    /* first auto-assigned slave address       */
#define MESH_MAX_SLAVES         (MESH_ADDR_MAX - MESH_FIRST_SLAVE_ADDR + 1) /* 14 */

/* Broadcast is expressed as a message type, not as an address. */

/* Node identity token length (bytes). Provisioned by the host (e.g. from the
 * host's own 6-byte device id) and persisted in flash; it rides only the
 * JOIN/ASSIGN/CLAIM control frames, never the data frames. 48 bits over 14
 * nodes gives a collision probability of ~3e-13, so no hashing is needed. */
#define MESH_TOKEN_LEN          6

/* --- Join / allocator state --------------------------------------------- */
/* Join FSM (see "Join / address assignment" in the design). */
typedef enum {
    MESH_STATE_UNASSIGNED = 0,  /* mesh disabled                                   */
    MESH_STATE_JOINING,         /* enabled, no address yet: broadcasting JOIN_REQ  */
    MESH_STATE_JOINED           /* has an address (or is the master)               */
} mesh_state_t;

/* --- Persistent (flash) configuration ----------------------------------- */
/* User flash partition: offset + length must stay below 0x7800 on STM32WLE. */
#define MESH_FLASH_OFFSET       0x0100
#define MESH_FLASH_MAGIC        0xA5
#define MESH_FLASH_VERSION      0x03

/* --- Lifecycle ---------------------------------------------------------- */
/* Load persisted config (or defaults: disabled, not master, unassigned). */
bool mesh_init(void);

/* --- Configuration accessors -------------------------------------------- */
bool    mesh_is_enabled(void);
void    mesh_set_enabled(bool enabled);

bool    mesh_is_master(void);
void    mesh_set_master(bool master);

uint8_t mesh_get_address(void);
void    mesh_set_address(uint8_t address);

/* Identity token (MESH_TOKEN_LEN bytes), provisioned by the host.  A device
 * without a token never starts the mesh / never joins. */
bool    mesh_has_token(void);
void    mesh_set_token(const uint8_t *token);   /* NULL clears it */
void    mesh_get_token(uint8_t out[MESH_TOKEN_LEN]);

/* Persist the current configuration to flash. */
bool    mesh_config_save(void);

/* --- M1 plumbing -------------------------------------------------------- */
#define MESH_TX_QUEUE_SIZE      8       /* queued outgoing mesh frames        */
#define MESH_MAX_FRAME          64      /* header + path + payload, in bytes  */
#define MESH_DEDUP_SIZE         32      /* (src,seq) duplicate cache depth    */
#define MESH_TIMER_PERIOD_MS    200     /* mesh housekeeping period           */

/* --- M2 join / address assignment --------------------------------------- */
#define MESH_JOIN_INTERVAL_MS   3000    /* initial JOIN_REQ period            */
#define MESH_JOIN_INTERVAL_MAX_MS 15000 /* JOIN_REQ backoff ceiling           */
#define MESH_TABLE_INTERVAL_MS  300000  /* master ADDR_TABLE backstop period   */
#define MESH_TABLE_DEBOUNCE_MS  500     /* coalesce table changes into a flood */
#define MESH_TABLE_MISS_LIMIT   3       /* misses before a node re-joins      */

/* --- M3 tree / uplink --------------------------------------------------- */
#define MESH_BEACON_MAX_MS      90000   /* adaptive beacon back-off ceiling    */
/* A *leaf* -- a joined node that no other node routes through -- is on nobody's
 * path, so it only needs to advertise itself as a *potential* parent: it beacons
 * at LEAF_MULT x the normal back-off.  A node counts as a *relay* while it has
 * recently forwarded a rootward unicast (uplink / claim) addressed to it, which
 * is the only signal that another node has selected it as its parent. */
#define MESH_BEACON_LEAF_MULT   3       /* leaf beacon interval multiplier     */
#define MESH_RELAY_HOLD_MS      200000  /* "recently forwarded for a child"    */
#define MESH_NEIGHBOR_MAX       8       /* tracked neighbours per node        */
#define MESH_DEFAULT_TTL        4       /* uplink hop limit (max 4)           */
#define MESH_PARENT_HYSTERESIS  1       /* cost margin required to switch     */

/* Join state of this node (UNASSIGNED / JOINING / JOINED). */
mesh_state_t mesh_get_state(void);

/* Number of addresses currently allocated by the master (diagnostics). */
uint8_t mesh_master_alloc_count(void);

/* Routing diagnostics. */
uint8_t mesh_get_parent(void);          /* current parent address (0 = none)  */
uint8_t mesh_get_hops(void);            /* hop-count to the master            */
uint8_t mesh_get_path_cost(void);       /* cost to the master                 */
uint8_t mesh_get_neighbor_count(void);  /* live neighbours                    */

/* Running control-plane overhead since enable, in percent of the data-frame
 * airtime transmitted (100 == as much control airtime as data airtime).
 * Diagnostic for the narrowband (31.25 kHz) 10% budget. */
uint16_t mesh_get_overhead_pct(void);

/* Enqueue a WiRoc payload toward the master (non-master) or to the local host
 * (master). Returns false when there is no route / the frame is too large. */
bool    mesh_send_uplink(const uint8_t *payload, uint8_t len);

/* --- M4 MAC / downlink -------------------------------------------------- */
#define MESH_LINK_RETRIES        3      /* retransmits before giving up       */

/* Channel access: every transmission is preceded by a CAD (listen before
 * talk), forced on regardless of the module's persisted CAD setting.  When the
 * channel is busy a *queued* (module-generated) frame is retried after a
 * randomised backoff window that doubles per consecutive busy attempt.  Host
 * messages are not queued: they are tried once and a busy channel is reported
 * back to the host, which owns the backoff/resend. */
#define MESH_TX_BACKOFF_MIN_MS   40     /* smallest randomised window (ms)    */
#define MESH_TX_BACKOFF_MAX_MS   1280   /* ceiling of the doubling backoff    */

/* Master only: flood a payload down to a specific node (0 on failure). */
bool    mesh_send_downlink(uint8_t dst, const uint8_t *payload, uint8_t len);

/* Derived per-hop ACK timeout (ms) for the current datarate: ~2x the airtime
 * of a maximum-length data frame. Reported by ATC+MESHMAP. */
uint32_t mesh_get_link_ack_timeout_ms(void);

/* --- M5 recovery / robustness ------------------------------------------- */
#define MESH_BEACON_LEN          1      /* epoch[3] | cost[5]                 */
#define MESH_BEACON_FAST_MS      1000   /* beacon period while (re)attaching  */
#define MESH_CLAIM_INTERVAL_MS   90000  /* node re-announces its address      */
#define MESH_EVICT_MS            180000 /* master frees a silent node's addr  */
#define MESH_RECOVER_MS          10000  /* master defers new allocs after boot*/

/* Master boot epoch last heard (0 while unknown). */
uint16_t mesh_get_epoch(void);

/* Parse an incoming mesh frame (already gated on mesh_is_enabled() by caller).
 * M1: validates + dedups.  M2: join/assign/table.  M3: beacon + uplink.
 * M4: downlink + implicit link-ACK retries.  M5: epoch/claim recovery. */
void    mesh_handle_rx(const uint8_t *buf, uint16_t len, int16_t rssi, int8_t snr);

/* Enqueue a fully-built mesh frame for transmission (drained by the timer). */
bool    mesh_send_frame(const uint8_t *frame, uint8_t len);

/* Number of frames currently queued for transmission (diagnostics). */
uint8_t mesh_tx_queue_count(void);

#endif /* INC_MESH_H_ */
