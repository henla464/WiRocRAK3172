/*
 * mesh.h
 *
 *  LoRa mesh mode for WiRocRAK3172.
 *
 *  Convergecast tree rooted at a single master (gateway). 5-bit addresses are
 *  assigned by the master; all nodes route their messages toward the master.
 *
 *  This header currently covers the M0 scaffolding: address constants, the
 *  flash-persisted configuration (mesh enabled / role / own address) and the
 *  accessors used by the custom AT commands.
 */

#ifndef INC_MESH_H_
#define INC_MESH_H_

#include <stdint.h>
#include <stdbool.h>
#include "mesh_wire.h"

/* --- Address space (5 bits, values 0..31) ------------------------------- */
#define MESH_ADDR_BITS          5
#define MESH_ADDR_MASK          0x1F
#define MESH_ADDR_MAX           0x1F

#define MESH_ADDR_NONE          0x00    /* unassigned / "default: send to master" */
#define MESH_MASTER_ADDR        0x01    /* well-known root (gateway) address       */
#define MESH_FIRST_SLAVE_ADDR   0x02    /* first auto-assigned slave address       */
#define MESH_MAX_SLAVES         (MESH_ADDR_MAX - MESH_FIRST_SLAVE_ADDR + 1) /* 30 */

/* Broadcast is expressed as a message type, not as an address. */

/* --- Persistent (flash) configuration ----------------------------------- */
/* User flash partition: offset + length must stay below 0x7800 on STM32WLE. */
#define MESH_FLASH_OFFSET       0x0100
#define MESH_FLASH_MAGIC        0xA5
#define MESH_FLASH_VERSION      0x01

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

/* Persist the current configuration to flash. */
bool    mesh_config_save(void);

/* --- M1 plumbing -------------------------------------------------------- */
#define MESH_TX_QUEUE_SIZE      4       /* queued outgoing mesh frames        */
#define MESH_MAX_FRAME          64      /* header + path + payload, in bytes  */
#define MESH_DEDUP_SIZE         16      /* (src,seq) duplicate cache depth    */
#define MESH_TIMER_PERIOD_MS    200     /* mesh housekeeping period           */

/* Parse an incoming mesh frame (already gated on mesh_is_enabled() by caller).
 * M1: validates + dedups only; routing is added in later milestones. */
void    mesh_handle_rx(const uint8_t *buf, uint16_t len);

/* Enqueue a fully-built mesh frame for transmission (drained by the timer). */
bool    mesh_send_frame(const uint8_t *frame, uint8_t len);

/* Number of frames currently queued for transmission (diagnostics). */
uint8_t mesh_tx_queue_count(void);

#endif /* INC_MESH_H_ */
