# LoRa mesh mode

WiRocRAK3172 normally runs as a **P2P** LoRa modem driven by the WiRoc host over
AT commands: every node broadcasts, and a node only reaches the host if it is in
direct radio range. This document describes the optional **mesh mode**, which adds
automatic multi-hop routing so that distant nodes can still reach the gateway.

Mesh mode is **opt-in** and off by default: with it disabled the firmware behaves
exactly as before (legacy P2P), so existing deployments are unaffected.

## Model

* **Exactly one master** (gateway), designated by configuration -- there is no
  election. The master is the only node that feeds its host.
* **5-bit addresses (0-31)** assigned automatically by the master; the master
  itself always owns address `1`.
* **Convergecast** routing: every node forwards toward the master, multi-hop.
* The master can send a payload **down** to a specific node (needed for
  application-layer ACKs).

```
            (master, addr 1)
              /        \
        (addr 4)      (addr 5)
             |
        (addr 7)   <-- out of the master's range: reaches it via addr 4
```

## Enabling

| Command | Meaning |
|---------|---------|
| `ATC+MESH=<0\|1>` | Enable / disable mesh mode (persisted). `ATC+MESH?` reads it. |
| `ATC+MASTER=<0\|1>` | Designate this node as the master (persisted). `ATC+MASTER?` reads it. |
| `ATC+MESHMAP=?` | Diagnostics (see below). |
| `ATC+P2P=...,<mesh>:<master>` | The two mesh flags are also the trailing params of the P2P config command. |

A device only joins the network once mesh mode is enabled and it is not the
master. Enabling/disabling takes effect immediately (no reboot required).

`ATC+MESHMAP=?` returns
`MESHMAP=<enabled>:<master>:<addr>:<txq>:<state>:<alloc>:<parent>:<hops>:<cost>:<neigh>:<epoch>`
where `state` is `0`=unassigned, `1`=joining, `2`=joined; `alloc` is the number of
addresses the master has handed out; `parent` is the current next hop (`0`=none);
`hops`/`cost` are the route to the master; `neigh` is the live neighbour count and
`epoch` is the master boot epoch last seen.

## Host interface changes

Addresses travel in the AT interface, so the host is updated in lock-step:

* **`ATC+SEND=<dest>:<hexpayload>`** -- optional leading destination address.
  `dest=0` means "the master / default" and is what legacy P2P and every
  non-master mesh node use. A non-zero `dest` is honoured **only by the master**
  as a downlink to that node; on a non-master it is ignored and the payload goes
  to the master anyway. The legacy single-parameter form `ATC+SEND=<hexpayload>`
  stays valid (`dest=0`).
* **`ATC+REC`** appends one **source-address** byte to both response forms:

  ```
  hex:    OK:<hex>:<rssi>:<snr>:<status>:<srcaddr>
  binary: OK + data + rssiH + rssiL + snr + status + srcaddr
  ```

  `srcaddr` is the mesh source of the delivered frame (the node that injected it).
  In P2P mode it is `0`. The master uses it to learn which node to ACK.

## Wire format

Mesh mode is exclusive: with it enabled **every** on-air frame is a mesh frame, so
no magic bytes are needed (the LoRa PHY CRC plus the packed version/type fields act
as the sanity gate). The header is **3 bytes**, MSB-first:

| bits | field | notes |
|------|-------|-------|
| 3 | type | `BEACON`, `JOIN_REQ`, `ADDR_ASSIGN`, `ADDR_TABLE`, `DATA_UPLINK`, `DATA_DOWNLINK`, `LINK_ACK`, `ADDR_CLAIM` |
| 2 | flags | `ACK_REQ` (0x01), `HAS_PATH` (0x02, reserved) |
| 5 | src | source address |
| 5 | dst | destination (ignored by broadcast types) |
| 3 | hops | beacon: hop-count to master; data: TTL (max 7) |
| 4 | seq | dedup key `(src,seq)`, 16-value window |
| 2 | version | protocol version |

The WiRoc payload follows verbatim and is stripped of the header before delivery to
the host. Control beacons additionally carry a 3-byte control payload after the
header: `epoch[2]` (little-endian) + `path cost[1]`.

## Join / address assignment

1. A node with no stored address enters **JOINING** and floods
   `JOIN_REQ{ token[12] }` from the mesh timer (start ~3 s, backing off to 15 s).
   The `token` is the STM32 **96-bit unique device id** (UID) read straight from the
   device registers, so it is guaranteed unique per die and needs no hashing. It is
   *not* a secret, only used to correlate an assignment back to the requester.
   > Note: `api.system.chipId.get()` must **not** be used for this. In this RUI3
   > build it returns the compile-time constant `"stm32wle5xx"`, identical on every
   > board, which would make every node collide on one identity.
2. The master keeps a RAM-only token->address table, allocates the lowest free
   address in 2-31 and floods `ADDR_ASSIGN{ token, addr }`.
3. The matching node adopts and **persists** the address (flash), emits
   `ADDR_CLAIM`, and becomes **JOINED**.
4. The master periodically floods `ADDR_TABLE{ occupied_bitmap[4] }`; a node that
   sees its own bit cleared for several intervals relinquishes its address and
   re-joins.

The master owns address `1`. Its table is **RAM-only**; nodes are the source of
truth for their own address via flash (see *Recovery*).

## Routing and the link-quality metric

All routable nodes emit a **beacon** every `MESH_BEACON_INTERVAL_MS` advertising
`path cost = link_cost(self,parent) + parent.path_cost` and their hop-count. A node
picks as parent the neighbour minimising `link_cost + n.path_cost`.

**SNR is preferred to RSSI** (RSSI saturates and is dominated by the noise floor;
SNR reflects the demodulation margin). Per-neighbour SNR is smoothed with an EWMA
and mapped to a small link cost:

| EWMA SNR | link cost |
|----------|-----------|
| >= +5 dB | 1 (good) |
| 0 .. +5 dB | 2 (fair) |
| -5 .. 0 dB | 3 (poor) |
| < -5 dB | 6 (avoid) |

Because the metric is a summed cost, a **2-hop all-good path (cost 2) beats a weak
direct link (cost 6)**: the node deliberately chooses the extra reliable hop.
Parent switching requires a hysteresis margin to avoid flapping, and a stale parent
(no beacon for `MESH_BEACON_STALE_MS`) is dropped immediately.

### Uplink

The origin builds `DATA_UPLINK{ src, dst=parent, ttl, ACK_REQ, payload }` and
unicasts it to its parent; each relay dedups `(src,seq)` and forwards to its own
parent (TTL - 1). The master queues the payload to its host with `srcaddr` = the
origin.

### Downlink

The master floods `DATA_DOWNLINK{ src=master, dst=target, ttl }`; the node whose
address matches delivers it to its host and everyone else relays it one hop
further (bounded by TTL and dedup).

## MAC / link reliability

Uplink unicasts are remembered in a single pending slot. A relay clears its pending
frame when it **overhears the next hop forward the same `(origin,seq)`** -- an
implicit ACK -- and otherwise retransmits up to `MESH_LINK_RETRIES` times at
`MESH_LINK_ACK_TIMEOUT_MS`, then gives up. The **master has no next hop to
overhear**, so it emits an explicit `LINK_ACK{ origin, seq }` after delivering.

Per-hop ACKs are *not* a liveness mechanism (a quiet node is indistinguishable from
a dead one); liveness is beacon-driven, and the end-to-end WiRoc ACK is
application-layer (the RAK does not fabricate it in mesh mode).

## Recovery

**Node / link down.** A node that stops hearing its parent invalidates it and
re-attaches to the best remaining neighbour, then re-announces its address. If it
has no neighbour at all it keeps listening until beacons return.

**Master restart.** The master's table is RAM-only. To recover it:

* The master persists a **boot counter** and advertises it as the beacon **epoch**,
  beacons fast for `MESH_RECOVER_MS` after boot, and defers new allocations during
  that window so returning nodes win back their own addresses first.
* A node re-announces its flash-stored address with a flooded
  `ADDR_CLAIM{ token, addr }` when it sees a new epoch (fast path), on its first
  beacon after boot, after re-attaching a lost parent, and periodically as a
  safety net (`MESH_CLAIM_INTERVAL_MS`).
* The master rebuilds its table from the claims: it adopts the claimed address when
  free, re-asserts its own assignment when the token is already known, and hands
  out a fresh address when the claimed one is already taken.
* Routing rebuilds on its own: the master is beaconing again within one interval and
  routing state is node-side.

**Eviction / recycling.** The master frees the address of a node it has not heard
from for `MESH_EVICT_MS`. A live node re-claims well inside that window, so an
idle-but-alive node is never dropped.

**Duplicate addresses.** If a node hears its own address handed to a different token
it relinquishes it and re-joins, so no duplicate address can persist.

## Tunables

All intervals live in `mesh.h` and can be adjusted without touching logic:

| Constant | Default | Purpose |
|----------|---------|---------|
| `MESH_BEACON_INTERVAL_MS` | 5000 | steady beacon period |
| `MESH_BEACON_FAST_MS` | 1000 | master beacon period while recovering |
| `MESH_BEACON_STALE_MS` | 15000 | parent timeout (3x beacon) |
| `MESH_JOIN_INTERVAL_MS` / `MESH_JOIN_INTERVAL_MAX_MS` | 3000 / 15000 | join backoff |
| `MESH_TABLE_INTERVAL_MS` | 60000 | ADDR_TABLE flood period |
| `MESH_CLAIM_INTERVAL_MS` | 60000 | node safety-net re-announce |
| `MESH_EVICT_MS` | 180000 | master eviction grace |
| `MESH_RECOVER_MS` | 10000 | master post-boot recovery window |
| `MESH_LINK_RETRIES` / `MESH_LINK_ACK_TIMEOUT_MS` | 3 / 1500 | link retransmits |
| `MESH_DEFAULT_TTL` | 7 | max hops |
| `MESH_NEIGHBOR_MAX` | 8 | tracked neighbours per node |
| `MESH_PARENT_HYSTERESIS` | 1 | cost margin required to switch parent |

## Files

| File | Role |
|------|------|
| `mesh.h` / `mesh.cpp` | mesh engine: config, join, routing, MAC, recovery |
| `mesh_wire.h` / `mesh_wire.cpp` | pure 3-byte header codec (host-tested) |
| `mesh_alloc.h` / `mesh_alloc.cpp` | pure master address allocator (host-tested) |
| `mesh_route.h` / `mesh_route.cpp` | pure link metric + parent selection (host-tested) |
| `custom_at.cpp` | AT integration (`MESH`/`MASTER`/`MESHMAP`, `SEND`/`REC`) |
| `MessageQueue.h` | `SourceAddr` carried from RX to `ATC+REC` |
| `test/` | host `g++` unit tests for the pure modules |

## Testing

* **Host unit tests** (no target needed):
  `g++ -std=c++11 -Wall -Wextra -I. test/mesh_wire_test.cpp mesh_wire.cpp -o /tmp/t && /tmp/t`
  (same for `mesh_alloc_test`, `mesh_route_test`).
* **Build check** against the real target flags: compile the sketch sources with
  `-fsyntax-only` using the generated `compile_commands.json`.
* **Lab, 2 devices**: master + slave -> join assigns an address; slave `ATC+SEND`
  arrives at the master `ATC+REC` with `srcaddr` = the slave; an app-level ACK sent
  back with `ATC+SEND=<slave>:...` reaches the slave.
* **Lab, multi-hop**: three nodes in a line with the far one out of the master's
  range -> it reaches the master through the middle node; perturb SNR and confirm
  the cost-based parent choice and hysteresis.
* **Failure / recovery**: power off a relay -> children re-attach; reboot the
  master -> nodes re-claim on the new epoch and are back within one or two beacon
  intervals. Reboot a slave -> it keeps / re-joins its address.
* **Regression**: with mesh disabled the P2P punch / double-punch ACK+hash path is
  unchanged (and no `srcaddr` byte is added).

## Limitations

* The master is a single point of failure by design.
* 5-bit space caps the network at 30 slaves; addresses are recycled on eviction.
* `hops` is 3 bits (max 7 hops) and `seq` is 4 bits (16-value dedup window).
* Downlink uses a controlled flood rather than a recorded reverse source-route; the
  `HAS_PATH` flag is reserved for a future source-route optimisation.
* Mesh and legacy P2P must not share a channel at the same time (mode is exclusive,
  and no magic byte distinguishes the two framings).
