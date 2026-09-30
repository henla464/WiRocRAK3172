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
* **4-bit addresses (0-15)** assigned automatically by the master; the master
  itself always owns address `1`, slaves use `2`-`15` (14 nodes max).
* **Convergecast** routing: every node forwards toward the master, multi-hop
  (up to `4` hops).
* Each node has a **host-provisioned 6-byte identity token** used to claim an
  address; a node with no token never starts the mesh.
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
| `ATC+MESHTOKEN=<12 hex>` | Set the 6-byte identity token (persisted). `ATC+MESHTOKEN?` reads it back. |
| `ATC+MESHMAP=?` | Diagnostics (see below). |
| `ATC+P2P=...,<mesh>:<master>[:<token>]` | Mesh flags and token as trailing params of the P2P config command. |

A device only joins the network once mesh mode is enabled **and** it has a token
and it is not the master. A node with **no token** neither beacons nor joins
(the mesh does not start). Enabling/disabling takes effect immediately (no reboot
required).

**Token.** The token replaces the old device-derived identity. It is 6 bytes
(12 hex chars) provisioned by the host -- e.g. the host's own device id -- so
identity is deterministic and needs no hashing. 48 bits over <=14 nodes gives a
collision probability of ~3e-13. It rides only the `JOIN_REQ` / `ADDR_ASSIGN` /
`ADDR_CLAIM` control frames, never the data frames.

**`ATC+P2P` tail.** The legacy 12/13-argument forms are unchanged; the mesh tail
is:

| argc | params |
|------|--------|
| 12 | base only |
| 13 (`argv[12]=="0"`) | base + runtime-config selector `0` |
| 14 | base + `<mesh>:<master>` (token unchanged) |
| 15 | base + `<mesh>:<master>:<token>` |
| 16 (`argv[15]=="0"`) | base + `<mesh>:<master>:<token>` + runtime-config selector `0` |
| 13 (`argv[12]=="1"`) / 16 (`argv[15]=="1"`) | as above, with the runtime-config selector `1` |

The token is 12 hex chars, so it is trivially distinguishable from the 1-char
selector.

`ATC+MESHMAP=?` returns
`MESHMAP=<enabled>:<master>:<addr>:<txq>:<state>:<alloc>:<parent>:<hops>:<cost>:<neigh>:<epoch>:<overhead%>:<ackms>`
where `state` is `0`=unassigned, `1`=joining, `2`=joined; `alloc` is the number of
addresses the master has handed out; `parent` is the current next hop (`0`=none);
`hops`/`cost` are the route to the master; `neigh` is the live neighbour count;
`epoch` is the master boot epoch last seen; `overhead%` is the measured
control-plane airtime as a percentage of the data-frame airtime transmitted; and
`ackms` is the derived per-hop ACK timeout for the current datarate.

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
as the sanity gate). The header is **3 bytes** (24 bits, no waste), MSB-first:

| bits | field | notes |
|------|-------|-------|
| 3 | type | message type, `0`-`7` (see **Message types** below) |
| 2 | flags | `ACK_REQ` (0x01), `HAS_PATH` (0x02, reserved) |
| 4 | src | source address (0-15) |
| 4 | dst | destination (0-15; also carries the assigned address / acked origin) |
| 3 | hops | beacon: hop-count to master; data: TTL (max 4) |
| 5 | seq | dedup key `(src,seq)`, 32-value window |
| 3 | version | protocol version (currently `1`) |

```
byte0 = type<<5 | flags<<3 | src>>1
byte1 = (src&1)<<7 | dst<<3 | hops
byte2 = seq<<3 | version
```

The WiRoc payload follows verbatim and is stripped of the header before delivery to
the host. Control beacons carry a **1-byte** control payload after the header:
`epoch[3]` (bits 7-5) packed over `path cost[5]` (bits 4-0).

### Message types

The 3-bit `type` field carries one of the following values. `MESH_TOKEN_LEN` = 6
(the host-provisioned identity token); the value itself is part of the wire format
and must not change without a version bump. An address that is already in the
header is **not** repeated in the payload (header-field reuse).

| id | type | on-air size (B) | delivery | payload | purpose |
|----|------|-----------------|----------|---------|---------|
| 0 | `BEACON` | 4 | link-local, not relayed, not deduped | `epoch[3]\|cost[5]` (1 B); header `hops` = hop-count to master | Liveness and routing advertisement. Emitted on an adaptive interval by the master and by any node that has a parent. Neighbours use it to select a parent and to detect a node going quiet. |
| 1 | `JOIN_REQ` | 9 | flooded | `token[6]`; `src` = 0 | An unassigned node asks the master for an address, retrying with backoff until assigned. Because `src` is 0, it is deduped on a hash of the token instead of `(src,seq)`. |
| 2 | `ADDR_ASSIGN` | 9 | flooded | `token[6]`; **assigned address in header `dst`** | The master's answer to a `JOIN_REQ` (and its re-assertion after an `ADDR_CLAIM`). The node whose token matches adopts and persists the address. A node that sees its own address given to a *different* token relinquishes it. |
| 3 | `ADDR_TABLE` | 5 | flooded | `occupied_bitmap[2]` (16-bit; the master bit is always set) | The master periodically floods its occupied-address bitmap so nodes can reconcile. A node that sees its own bit clear for `MESH_TABLE_MISS_LIMIT` intervals relinquishes its address and re-joins. |
| 4 | `DATA_UPLINK` | 3 + N | unicast hop-by-hop (`src` = origin, `dst` = parent) | WiRoc payload (N bytes) | A WiRoc payload travelling toward the master, carrying `ACK_REQ`. Each relay dedups `(src,seq)`, decrements the TTL and forwards to its parent. |
| 5 | `DATA_DOWNLINK` | 3 + N | flooded with `dst` = target | WiRoc payload (N bytes) | A WiRoc payload from the master to one specific node. The target delivers it to its host; everyone else relays it one hop further. |
| 6 | `LINK_ACK` | 4 | broadcast (single hop) | `acked_seq[1]`; **acked origin in header `dst`** | Explicit per-hop ACK, used only by the master (it has no next hop whose forward it could overhear, so relays rely on the implicit ACK instead). |
| 7 | `ADDR_CLAIM` | 9 | flooded | `token[6]`; **own address already in header `src`** | A node re-announces its flash-stored address so a restarted master can rebuild its RAM-only table. Sent on a new boot epoch, on re-attach, and periodically as a safety net. |

All sizes include the fixed 3-byte header. The control types (0, 1, 2, 3, 6, 7) have
a fixed length; the two data types (4, 5) are `3 + N`, where `N` is the verbatim
WiRoc payload (WiRoc punch payloads are typically 15 or 27 bytes, giving 18 or 30
bytes on air). A frame cannot exceed `MESH_MAX_FRAME` = 64 bytes, so `N` <= 61 for
data frames (and the control payloads above are well within that).

`MESH_TYPE_COUNT` (= 8) is not a wire value: the 3-bit field encodes only `0`-`7`,
so it is used as the "invalid / out of range" bound when validating a frame.

## Join / address assignment

1. A node with no stored address enters **JOINING** and floods
   `JOIN_REQ{ token[6] }` from the mesh timer (start ~3 s, backing off to 15 s).
   The `token` is the **6-byte host-provisioned identity token** (see
   `ATC+MESHTOKEN`), so identity is deterministic and needs no hashing. It is
   *not* a secret, only used to correlate an assignment back to the requester.
   A node with no token set never enters JOINING.
2. The master keeps a RAM-only token->address table, allocates the lowest free
   address in 2-15 and floods `ADDR_ASSIGN{ token }` (the address is the header `dst`).
3. The matching node adopts and **persists** the address (flash), emits
   `ADDR_CLAIM`, resets its beacon to fast, and becomes **JOINED**.
4. The master periodically floods `ADDR_TABLE{ occupied_bitmap[2] }`; a node that
   sees its own bit cleared for several intervals relinquishes its address and
   re-joins.

The master owns address `1`. Its table is **RAM-only**; nodes are the source of
truth for their own address via flash (see *Recovery*).

## Routing and the link-quality metric

All routable nodes emit a **beacon** advertising
`path cost = link_cost(self,parent) + parent.path_cost` and their hop-count. A node
picks as parent the neighbour minimising `link_cost + n.path_cost`.

**SNR is preferred to RSSI** (RSSI saturates and is dominated by the noise floor;
SNR reflects the demodulation margin). Per-neighbour SNR is smoothed with an EWMA.
The link cost is a **margin over the current spreading factor's LoRa demodulation
floor** (which depends on SF, not bandwidth), so the metric stays meaningful when
the datarate is retuned:

| SF | demod floor | cost 1 (good) | cost 2 (fair) | cost 3 (poor) | cost 6 (avoid) |
|----|-------------|---------------|---------------|---------------|----------------|
| 5  | -2.5 dB | margin >= 12.5 dB | >= 7.5 dB | >= 2.5 dB | below |
| 6  | -5.0 dB | margin >= 12.5 dB | >= 7.5 dB | >= 2.5 dB | below |
| 7  | -7.5 dB | margin >= 12.5 dB | >= 7.5 dB | >= 2.5 dB | below |
| 8  | -10.0 dB | margin >= 12.5 dB | >= 7.5 dB | >= 2.5 dB | below |

At SF7 this reproduces the classic fixed thresholds (`>= +5`, `0..+5`, `-5..0`,
`< -5` dB); at other SFs the whole scale shifts by the difference in demod floor.

Because the metric is a summed cost, a **2-hop all-good path (cost 2) beats a weak
direct link (cost 6)**: the node deliberately chooses the extra reliable hop.
Parent switching requires a hysteresis margin to avoid flapping, and a stale parent
(no beacon for `3x` the current interval) is dropped immediately.

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
implicit ACK -- and otherwise retransmits up to `MESH_LINK_RETRIES` times at the
per-hop ACK timeout, then gives up. The **master has no next hop to overhear**, so
it emits an explicit `LINK_ACK` (acked origin in the header `dst`, acked seq in the
payload) after delivering. The timeout is **derived from the datarate**: it is
`~2x` the airtime of the frame being sent plus one timer tick, floored at 500 ms
(reported as `ackms` by `ATC+MESHMAP?`).

Per-hop ACKs are *not* a liveness mechanism (a quiet node is indistinguishable from
a dead one); liveness is beacon-driven, and the end-to-end WiRoc ACK is
application-layer (the RAK does not fabricate it in mesh mode).

## Recovery

**Node / link down.** A node that stops hearing its parent invalidates it and
re-attaches to the best remaining neighbour, then re-announces its address. If it
has no neighbour at all it keeps listening until beacons return.

**Master restart.** The master's table is RAM-only. To recover it:

* The master persists a **boot counter** and advertises its low **3 bits** as the
  beacon **epoch**, beacons fast for `MESH_RECOVER_MS` after boot, and defers new
  allocations during that window so returning nodes win back their own addresses
  first. The 3-bit epoch is enough because its only job is *fast* reboot detection;
  a rare miss (the master rebooting a multiple of 8 times while a node was deaf) is
  caught by the periodic `ADDR_CLAIM` safety net.
* A node re-announces its flash-stored address with a flooded
  `ADDR_CLAIM{ token }` (its own address is already in the header `src`) when it
  sees a new epoch (fast path), on its first
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
| `MESH_BEACON_FAST_MS` | 1000 | fast beacon interval (start / after any topology change / master recovering) |
| `MESH_BEACON_MAX_MS` | 60000 | adaptive beacon back-off ceiling (interval doubles while stable) |
| `MESH_JOIN_INTERVAL_MS` / `MESH_JOIN_INTERVAL_MAX_MS` | 3000 / 15000 | join backoff |
| `MESH_TABLE_INTERVAL_MS` | 60000 | ADDR_TABLE flood period |
| `MESH_CLAIM_INTERVAL_MS` | 60000 | node safety-net re-announce |
| `MESH_EVICT_MS` | 180000 | master eviction grace |
| `MESH_RECOVER_MS` | 10000 | master post-boot recovery window |
| `MESH_LINK_RETRIES` | 3 | link retransmits (timeout is derived, see MAC) |
| `MESH_DEFAULT_TTL` | 4 | max hops |
| `MESH_NEIGHBOR_MAX` | 8 | tracked neighbours per node |
| `MESH_PARENT_HYSTERESIS` | 1 | cost margin required to switch parent |

**Adaptive beacon.** Each node's beacon interval starts at `MESH_BEACON_FAST_MS`
and **doubles per stable interval** up to `MESH_BEACON_MAX_MS`, and is **reset to
fast** on join, re-attach, parent change, epoch change or when a better candidate
appears. Parent staleness is `3x` the *current* interval. The master keeps its
`MESH_RECOVER_MS` fast window after boot, then backs off too. Trade-off (accepted):
failure detection slows down once backed off.

## Narrowband (31.25 kHz) operation

The mesh is designed to run at **31.25 kHz bandwidth** (the RAK bandwidth *enum*
index `7`; "32 kHz" is really 31.25 kHz) with SF5-SF8. Set it with
`ATC+P2P=<freq>:<sf>:7:<cr>:...` -- `service_lora_p2p_set_bandwidth()` takes the
raw index, so `7` passes straight through.

At 31.25 kHz every frame's airtime is `4x` the 125 kHz value, so the control plane
dominates. The firmware computes airtimes from the live radio parameters
(`service_lora_p2p_get_sf()/_get_bandwidth()`) using
`Tsym = 2^SF / BW` and
`n = 8 + max(ceil((8L - 4SF + 28 + 16)/(4(SF - 2DE))), 0) * (CR + 4)`.
For the v2 frame sizes (CR 4/5, preamble 8, CRC on), approximate airtimes in ms are:

| on-air frame | B | SF5 | SF6 | SF7 | SF8 |
|---|---|---|---|---|---|
| BEACON | 4 | 36 | 72 | 123 | 246 |
| LINK_ACK | 4 | 36 | 72 | 123 | 246 |
| ADDR_TABLE | 5 | 41 | 72 | 123 | 246 |
| JOIN_REQ / ADDR_ASSIGN / ADDR_CLAIM | 9 | 46 | 82 | 164 | 287 |
| DATA (15 B payload) | 18 | 67 | 113 | 205 | 369 |
| DATA (27 B payload) | 30 | 92 | 154 | 287 | 492 |

Two structural facts matter: the **fixed per-frame cost** (preamble + header symbols
= 12.25 symbols) is large -- at SF7 a 4-byte ACK spends ~40% of its airtime on the
preamble, so *fewer, larger frames beat many small frames* -- and the **proportional**
terms (the 3-byte header and the per-uplink ACK) never amortise, while only the
**fixed-rate** control terms do.

The overhead budget is `[header + per-uplink ACK + beacons + claims + table] / data
airtime`. At the old defaults (5 s beacons, standalone 5-byte ACKs, 30 slaves) a
15-byte-payload traffic pattern reached ~235% overhead at SF7 -- far above 10%. The
M7 design changes cut this several ways: **4-bit addresses (14 slaves max)** roughly
halve the beacon term, the **adaptive beacon** back-off cuts the steady-state beacon
term by the interval ratio, **frame slimming** (1-byte beacons, 2-byte ADDR_TABLE,
header-field reuse) shrinks every control frame, and the host can **measure** the
resulting ratio live via `ATC+MESHMAP?` (`overhead%`). The 10% target is only
plausible for a data-dominant, few-node, fat-payload regime -- the header and the
per-uplink ACK are proportional and set the floor.

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
* **Lab, 2 devices**: set a token on each (`ATC+MESHTOKEN=<12 hex>`), enable mesh,
  designate one master -> the slave joins and is assigned an address; slave
  `ATC+SEND` arrives at the master `ATC+REC` with `srcaddr` = the slave; an
  app-level ACK sent back with `ATC+SEND=<slave>:...` reaches the slave.
* **Narrowband**: run two devices at `ATC+P2P=...,7,<SF>,...` (31.25 kHz) for SF5
  and SF8; verify join, uplink, downlink, and that `ATC+MESHMAP?` reports the derived
  `ackms` and the measured `overhead%`.
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
* The 4-bit space caps the network at 14 slaves; addresses are recycled on eviction.
* `hops` is 3 bits (max 7 hops, we cap at 4) and `seq` is 5 bits (32-value dedup
  window, matching `MESH_DEDUP_SIZE`).
* The beacon `cost` is 5 bits, so the path cost must stay `<= 31`: with `hops <= 4`
  and a max link cost of 6 this holds with headroom (keep the link-cost map <= 7).
* The beacon epoch is 3 bits, so a master reboot that is a multiple of 8 while a
  node was deaf is caught only by the periodic `ADDR_CLAIM` safety net.
* Downlink uses a controlled flood rather than a recorded reverse source-route; the
  `HAS_PATH` flag is reserved for a future source-route optimisation.
* Mesh and legacy P2P must not share a channel at the same time (mode is exclusive,
  and no magic byte distinguishes the two framings).
