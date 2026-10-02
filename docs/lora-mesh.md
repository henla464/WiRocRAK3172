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
* Each node has a **host-provisioned 6-byte node device id** used to claim an
  address; a node with no node device id never starts the mesh.
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
| `ATC+MESHNODEDEVICEID=<12 hex>` | Set the 6-byte node device id (persisted). `ATC+MESHNODEDEVICEID?` reads it back. |
| `ATC+MESHMAP=?` | Diagnostics (see below). |
| `ATC+P2P=<runcfg>:...,[:<mesh>:<master>:<deviceid>]` | Mesh flags and node device id as trailing params of the P2P config command. |

A device only joins the network once mesh mode is enabled **and** it has a node
device id and it is not the master. A node with **no node device id** neither
beacons nor joins (the mesh does not start). Enabling/disabling takes effect
immediately (no reboot required).

**Node device id.** The node device id is a 6-byte (12 hex chars) identifier
provisioned by the host from the host's own Bluetooth address, so it is unique
per node. It rides only the `JOIN_REQ` / `ADDR_ASSIGN` / `ADDR_CLAIM` control
frames, never the data frames.

**`ATC+P2P` layout.** The runtime-config selector is the **first parameter and
is mandatory** (`0` = use the flash-stored config, `1` = use the runtime config);
the 12 radio parameters follow, then the optional mesh tail. `ATC+P2P=?` returns
the same layout -- its first field is the active selector -- so the query output
can be fed straight back as a set command.

| argc | params |
|------|--------|
| 13 | `<runcfg>` + 12 radio params |
| 16 | `<runcfg>` + 12 radio params + `<mesh>:<master>:<deviceid>` |

`<runcfg>` is a single digit at `argv[0]`; the 12 radio parameters and the mesh
tail are unchanged from the legacy `ATC+P2P`. Any other argument count is
rejected. (Mesh can also be enabled and given a device id with the standalone
`ATC+MESH` / `ATC+MASTER` / `ATC+MESHNODEDEVICEID` commands, which is how a node
changes one field without resending the whole P2P config.)

`ATC+MESHMAP=?` returns
`MESHMAP=<enabled>:<master>:<addr>:<txq>:<state>:<alloc>:<parent>:<hops>:<cost>:<neighbour-count>:<epoch>:<overhead%>:<ackms>`
where `state` is `0`=unassigned, `1`=joining, `2`=joined; `alloc` is the number of
addresses the master has handed out; `parent` is the current next hop (`0`=none);
`hops`/`cost` are the route to the master; `neighbour-count` is the live neighbour
count; `epoch` is the master's **boot number** -- a counter the master increments
on every boot -- as last seen by this node (only its low 3 bits are advertised;
see *Recovery*); `overhead%` is the measured control-plane airtime as a
percentage of the data-frame airtime transmitted; and `ackms` is the derived
per-hop ACK timeout for the current datarate.

## Host interface changes

Addresses travel in the AT interface, so the host is updated in lock-step:

* **`ATC+SEND=<dest>:<hexpayload>`** -- optional leading destination address.
  `dest=0` means "the master / default" and is what legacy P2P and every
  non-master mesh node use. A non-zero `dest` is honoured **only by the master**
  as a downlink to that node; on a non-master it is ignored and the payload goes
  to the master anyway. The legacy single-parameter form `ATC+SEND=<hexpayload>`
  stays valid (`dest=0`).
  In **both** modes a busy channel (CAD) is reported back as `AT_BUSY_ERROR`; the
  host backs off and resends (see "Channel access").
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
| 2 | flags | reserved; always `0` today (see **Header flags**) |
| 4 | src | source address (0-15) |
| 4 | dst | destination (0-15; also carries the assigned address / acked origin) |
| 3 | hops | beacon: hop-count to master; data: TTL (max 4) |
| 5 | seq | dedup key `(src,seq)`, 32-value window |
| 3 | version | protocol version (currently `2`) |

```
byte0 = type<<5 | flags<<3 | src>>1
byte1 = (src&1)<<7 | dst<<3 | hops
byte2 = seq<<3 | version
```

**Header flags.** The 2-bit `flags` field is **reserved and always `0`** today.
There is no "ack requested" flag: the master answers **every** delivered uplink
with an explicit `LINK_ACK`, since it has no next hop whose forward it could
overhear the way a relay does. The only defined bit, `HAS_PATH` (0x02), is a
reserved hook for carrying an explicit source path in the payload instead of
relying on per-hop routing state; encoders send `0` and receivers ignore it.

The WiRoc payload follows verbatim and is stripped of the header before delivery to
the host. Control beacons carry a **1-byte** control payload after the header:
`epoch[3]` (bits 7-5) packed over `path cost[5]` (bits 4-0). The 3-bit `epoch` is
the low bits of the master's **boot number** -- incremented by the master on each
boot -- so nodes spot a restart when it changes.

### Duplicate suppression (dedup)

A flooded or multi-hop frame can reach the same node more than once -- a relay
loop, two paths through the mesh, or a retransmit -- so every receiver drops a
frame it has already seen. The key is the `(src, seq)` pair: the origin's address
plus a 5-bit sequence number the origin allocates per frame. Relays copy both
fields unchanged, so every copy of a frame carries the same key.

The cache is a fixed **32-entry ring** (`MESH_DEDUP_SIZE`). On receive the node
scans it; if the pair is present the frame is dropped, otherwise the pair is
recorded (evicting the oldest entry) and the frame is accepted. A key is
therefore only remembered for the next 32 *received* frames, and `seq` itself
wraps at 32 (`2^5`); the TTL / hop cap is what keeps a stale duplicate from
surviving long enough to be re-accepted.

Two frames key differently:

* `JOIN_REQ` is flooded by nodes that have **no address yet**, so its header
  `src` is `0` for all of them. It is deduped on an 8-bit hash of the node device
  id instead of `(src, seq)`, so two distinct joiners are not mistaken for each
  other.
* `BEACON` is link-local and never relayed, so it is **not** deduped at all.

A node also records its own outbound floods as "seen" the moment it transmits, so
the copies echoed back to it are ignored and cannot start a loop.

### Message types

The 3-bit `type` field carries one of the following values. `MESH_NODE_DEVICE_ID_LEN`
= 6 (the host-provisioned node device id); the value itself is part of the wire
format and must not change without a version bump. An address that is already in the
header is **not** repeated in the payload (header-field reuse).

| id | type | on-air size (B) | delivery | payload | purpose |
|----|------|-----------------|----------|---------|---------|
| 0 | `BEACON` | 4 | link-local, not relayed, not deduped | `epoch[3]\|cost[5]` (1 B); header `hops` = hop-count to master | Liveness and routing advertisement. Emitted on an adaptive interval by the master and by any node that has a parent. Neighbours use it to select a parent and to detect a node going quiet. |
| 1 | `JOIN_REQ` | 9 | flooded | `devid[6]`; `src` = 0 | An unassigned node asks the master for an address, retrying with backoff until assigned. Because `src` is 0, it is deduped on its node device id instead of `(src,seq)`. |
| 2 | `ADDR_ASSIGN` | 9 | flooded | `devid[6]`; **assigned address in header `dst`** | The master's answer to a `JOIN_REQ` (and its re-assertion after an `ADDR_CLAIM`). The node whose node device id matches adopts and persists the address. A node that sees its own address given to a *different* node device id relinquishes it. |
| 3 | `ADDR_TABLE` | 5 | flooded (relayed by relays only) | `occupied_bitmap[2]` (16-bit; the master bit is always set) | The master floods its occupied-address bitmap whenever the table **changes** (debounced) and as a slow periodic backstop, so nodes can reconcile. A node that sees its own bit clear for `MESH_TABLE_MISS_LIMIT` intervals relinquishes its address and re-joins. A *leaf* does not relay it (see "Flood relay"). |
| 4 | `DATA_UPLINK` | 3 + N | unicast hop-by-hop (`src` = origin, `dst` = parent) | WiRoc payload (N bytes) | A WiRoc payload travelling toward the master. Each relay dedups `(src,seq)`, decrements the TTL and forwards to its parent. |
| 5 | `DATA_DOWNLINK` | 3 + N | flooded with `dst` = target (relayed by relays only) | WiRoc payload (N bytes) | A WiRoc payload from the master to one specific node. The target delivers it to its host; any other node that has children relays it one hop further (a *leaf* does not relay). |
| 6 | `LINK_ACK` | 4 | broadcast (single hop) | `acked_seq[1]`; **acked origin in header `dst`** | Explicit per-hop ACK, emitted only by the master, which has no next hop whose forward it could overhear; it sends one for every `DATA_UPLINK` it delivers. Relays use the implicit ACK instead. |
| 7 | `ADDR_CLAIM` | 9 | rootward unicast (flood fallback) | `devid[6]`; **own address already in header `src`** | A node re-announces its flash-stored address so a restarted master can rebuild its RAM-only table. Sent hop-by-hop toward the parent (on a new boot epoch, on re-attach, and periodically as a safety net); a node with no route yet -- or a relay that has lost its own parent -- floods it instead. |

All sizes include the fixed 3-byte header. The control types (0, 1, 2, 3, 6, 7) have
a fixed length; the two data types (4, 5) are `3 + N`, where `N` is the verbatim
WiRoc payload (WiRoc punch payloads are typically 15 or 27 bytes, giving 18 or 30
bytes on air). A frame cannot exceed `MESH_MAX_FRAME` = 64 bytes, so `N` <= 61 for
data frames (and the control payloads above are well within that).

`MESH_TYPE_COUNT` (= 8) is not a wire value: the 3-bit field encodes only `0`-`7`,
so it is used as the "invalid / out of range" bound when validating a frame.

## Join / address assignment

1. A node with no stored address enters **JOINING** and floods
   `JOIN_REQ{ devid[6] }` from the mesh timer (start ~3 s, backing off to 15 s).
   The `devid` is the **6-byte host-provisioned node device id** (see
   `ATC+MESHNODEDEVICEID`); it is *not* a secret, only used to correlate an
   assignment back to the requester. A node with no node device id set never
   enters JOINING.
2. The master keeps a RAM-only node-device-id->address table, allocates the
   lowest free address in 2-15 and floods `ADDR_ASSIGN{ devid }` (the address is
   the header `dst`).
3. The matching node adopts and **persists** the address (flash), emits
   `ADDR_CLAIM`, resets its beacon to fast, and becomes **JOINED**.
4. The master floods `ADDR_TABLE{ occupied_bitmap[2] }` **whenever the table
   changes** (debounced), with a slow periodic backstop; a node whose own bit is
   cleared for several intervals relinquishes its address and re-joins.

The master owns address `1`. Its table is **RAM-only**; nodes are the source of
truth for their own address via flash (see *Recovery*).

## Routing and the link-quality metric

All routable nodes emit a **beacon** advertising
`path cost = link_cost(self,parent) + parent.path_cost` and their hop-count (a
childless *leaf* advertises at a reduced rate -- see "Adaptive beacon" below). A
node picks as parent the neighbour minimising `link_cost + n.path_cost`.

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

The origin builds `DATA_UPLINK{ src, dst=parent, ttl, payload }` and unicasts it
to its parent; each relay dedups `(src,seq)` and forwards to its own parent
(TTL - 1). The master queues the payload to its host with `srcaddr` = the origin.

### Downlink

The master floods `DATA_DOWNLINK{ src=master, dst=target, ttl }`; the node whose
address matches delivers it to its host and any node with children relays it one
hop further (bounded by TTL and dedup; a *leaf* does not relay -- see "Flood
relay").

## Channel access (listen before talk)

Every transmission is preceded by a **CAD** (channel activity detection) that is
**forced on in firmware** -- `api.lora.psend(len, frame, true)` -- so it does not
depend on the module's persisted `AT+CAD` setting. When CAD detects activity the
frame is **not** put on the air (`psend` returns false).

How a busy channel is handled depends on who wants the transmission:

* **Module-generated traffic** (beacons, joins, claims, table floods, relays,
  link ACKs and link retransmits) is queued and re-tried by the mesh timer. After
  a busy verdict the drain holds off for a **randomised backoff** that starts at
  `MESH_TX_BACKOFF_MIN_MS` and **doubles per consecutive busy attempt** up to
  `MESH_TX_BACKOFF_MAX_MS`, resetting after a clean send. The random draw (from a
  per-node xorshift PRNG) stops nodes that just collided from retrying in lockstep
  on the 200 ms tick.
* **Host-originated traffic** (`ATC+SEND`, i.e. an uplink or a master downlink) is
  **not queued**. The frame is tried **once, immediately**: if the channel is busy
  the AT handler returns `AT_BUSY_ERROR` and the **host owns the backoff and
  resend**, exactly like legacy P2P mode. If the frame does go out, it is still
  armed in the pending slot, so a lost *hop* is retried by the mesh MAC (below)
  independently of the host.

The legacy P2P `ATC+SEND` and the built-in punch auto-ACK also pass `true`, so CAD
is checked first there too.

## MAC / link reliability

Uplink unicasts are remembered in a single pending slot. A relay clears its pending
frame when it **overhears the next hop forward the same `(origin,seq)`** -- an
implicit ACK -- and otherwise retransmits up to `MESH_LINK_RETRIES` times at the
per-hop ACK timeout, then gives up. The **master has no next hop to overhear**, so
it emits an explicit `LINK_ACK` (acked origin in the header `dst`, acked seq in the
payload) after delivering every uplink. The timeout is **derived from the datarate**: it is
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
* A node re-announces its flash-stored address with an `ADDR_CLAIM{ devid }` (its
  own address is already in the header `src`) when it sees a new epoch (fast
  path), on its first beacon after boot, after re-attaching a lost parent, and
  periodically as a safety net (`MESH_CLAIM_INTERVAL_MS`). The claim is a
  **rootward unicast**: it travels hop-by-hop toward the parent (like an uplink),
  so the network-wide cost is `O(hops)` per claim rather than the `O(N)` of a
  flood. Only a node with no route yet (just booted / parent lost), or a relay
  that has lost its own parent, falls back to flooding it.
* The master rebuilds its table from the claims: it adopts the claimed address when
  free, re-asserts its own assignment when the node device id is already known, and
  hands out a fresh address when the claimed one is already taken.
* Routing rebuilds on its own: the master is beaconing again within one interval and
  routing state is node-side.

**Eviction / recycling.** The master frees the address of a node it has not heard
from for `MESH_EVICT_MS`. A live node re-claims well inside that window, so an
idle-but-alive node is never dropped.

**Duplicate addresses.** If a node hears its own address handed to a different node
device id it relinquishes it and re-joins, so no duplicate address can persist.

## Tunables

All intervals live in `mesh.h` and can be adjusted without touching logic:

| Constant | Default | Purpose |
|----------|---------|---------|
| `MESH_BEACON_FAST_MS` | 1000 | fast beacon interval (start / after any topology change / master recovering) |
| `MESH_BEACON_MAX_MS` | 90000 | adaptive beacon back-off ceiling (interval doubles while stable) |
| `MESH_BEACON_LEAF_MULT` | 3 | a childless *leaf* beacons at this multiple of its back-off interval |
| `MESH_RELAY_HOLD_MS` | 200000 | a node counts as a *relay* while it forwarded for a child within this window |
| `MESH_JOIN_INTERVAL_MS` / `MESH_JOIN_INTERVAL_MAX_MS` | 3000 / 15000 | join backoff |
| `MESH_TABLE_INTERVAL_MS` | 300000 | ADDR_TABLE flood backstop period (a table change also triggers an immediate flood) |
| `MESH_TABLE_DEBOUNCE_MS` | 500 | window that coalesces table changes into one ADDR_TABLE flood |
| `MESH_CLAIM_INTERVAL_MS` | 90000 | node safety-net re-announce |
| `MESH_EVICT_MS` | 180000 | master eviction grace |
| `MESH_RECOVER_MS` | 10000 | master post-boot recovery window |
| `MESH_LINK_RETRIES` | 3 | link retransmits (timeout is derived, see MAC) |
| `MESH_TX_BACKOFF_MIN_MS` / `MESH_TX_BACKOFF_MAX_MS` | 40 / 1280 | queued-frame backoff after a busy CAD (randomised window, doubles per attempt) |
| `MESH_DEFAULT_TTL` | 4 | max hops |
| `MESH_NEIGHBOR_MAX` | 8 | tracked neighbours per node |
| `MESH_PARENT_HYSTERESIS` | 1 | cost margin required to switch parent |

**Adaptive beacon.** Each node's beacon interval starts at `MESH_BEACON_FAST_MS`
and **doubles per stable interval** up to `MESH_BEACON_MAX_MS`, and is **reset to
fast** on join, re-attach, parent change, epoch change or when a better candidate
appears. Parent staleness is `3x` the *current* interval. The master keeps its
`MESH_RECOVER_MS` fast window after boot, then backs off too. Trade-off (accepted):
failure detection slows down once backed off.

**Leaf suppression.** A **leaf** -- a joined node that no other node routes
through -- is on nobody's path, so it only needs to advertise itself as a
*potential* parent rather than maintain a live route. It therefore beacons at
`MESH_BEACON_LEAF_MULT` x its back-off interval (3x, so ~270 s in steady state),
cutting the dominant control-plane term (most nodes in a convergecast tree are
leaves). A node is a **relay** while it has recently forwarded a rootward unicast
(an uplink or a `ADDR_CLAIM`) **addressed to it** -- the only signal that another
node has selected it as its parent -- for `MESH_RELAY_HOLD_MS`; relays and the
master beacon at the full `MESH_BEACON_MAX_MS` rate. On the leaf -> relay
transition the beacon resets to fast so the new child can track us promptly.
Trade-off (accepted): discovering a *leaf* as a parent takes up to one leaf
interval, so re-parenting onto a former leaf is slower (its parent-side liveness
is unaffected -- claims make the leaf a relay's child within one interval, and
the leaf's own 90 s claims keep it alive at the master regardless).

**Flood relay.** A flood is re-broadcast once per receiving node (deduped on
`(src,seq)`), so one flood costs one transmission per relay. Floods whose
recipients are all **attached** nodes -- `ADDR_TABLE` and `DATA_DOWNLINK` -- are
relayed only by **relays** (non-leaves, the same signal as leaf suppression): a
node with no children has no downstream node, so its re-broadcast reaches nobody
that has not already seen the frame, while the routing tree guarantees every
attached node still receives the flood from its own parent. Floods that must
reach an **unattached** node -- `JOIN_REQ`, `ADDR_ASSIGN` and the flooded
`ADDR_CLAIM` fallback -- are still relayed by **everyone**, because the target may
not yet have a parent whose relay it could rely on.

## Narrowband (31.25 kHz) operation

The mesh is designed to run at **31.25 kHz bandwidth** (the RAK bandwidth *enum*
index `7`; "32 kHz" is really 31.25 kHz) with SF5-SF8. Set it with
`ATC+P2P=0:<freq>:<sf>:7:<cr>:...` (the leading `0` is the mandatory
runtime-config selector) -- `service_lora_p2p_set_bandwidth()` takes the raw
index, so `7` passes straight through.

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
| DATA (7 B payload) | 10 | 52 | 93 | 165 | 289 |
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
halve the beacon term, the **adaptive beacon** back-off plus **leaf suppression**
(only relays and the master beacon at the full rate) cut the steady-state beacon
term by the interval ratio, **frame slimming** (1-byte beacons, 2-byte ADDR_TABLE,
header-field reuse) shrinks every control frame, the **rootward `ADDR_CLAIM`**
keeps the claim plane `O(N)` rather than the `O(N^2)` of a flood (it floods only
when a node is route-less), and the host can **measure** the resulting ratio live
via `ATC+MESHMAP?` (`overhead%`). The 10% target is only
plausible for a data-dominant, few-node, fat-payload regime -- the header and the
per-uplink ACK are proportional and set the floor.

### Control-plane occupancy

The tables above are per-frame costs; these measure how much of the channel the
**module-generated** control plane occupies by itself (no data traffic), as a
percentage of wall-clock time -- i.e. the fraction of the time the single channel is
busy with mesh housekeeping. They count **beacons, `ADDR_CLAIM`s and the `ADDR_TABLE`
flood**, and exclude `LINK_ACK` (which only exists alongside an uplink) and
application data. Everything is derived from the airtimes above and the live tunables:
relays and the master beacon every `MESH_BEACON_MAX_MS` (90 s), a leaf every `3x` that
(270 s); every joined node claims once per `MESH_CLAIM_INTERVAL_MS` (90 s) over `d`
hops; the master floods `ADDR_TABLE` on the `MESH_TABLE_INTERVAL_MS` backstop (300 s),
relayed by non-leaves only. `<slaves>` is `N` (non-master nodes); the tree is rooted at
the master with depth `<= 4`. In the topology table below, `SD` is also the total
number of `ADDR_CLAIM` hops per claim round.

| topology | F = non-leaves (incl. master) | L = leaves | SD = sum of node depths |
|---|---|---|---|
| deep tree (max uplink hops -- fewest relays) | 4 | N-3 | 4N-6 |
| average 1.5 hops (2-level: floor(N/2) at depth 1, the rest at depth 2) | 1+floor(N/2) | ceil(N/2) | ~1.5N |
| average 2 hops (balanced 3-level) | ~1+2N/3 | ~N/3 | ~2N |

**Control-plane occupancy, % of wall-clock time (LINK_ACK and data excluded)**

| topology | slaves | SF5 | SF6 | SF7 | SF8 |
|---|---|---|---|---|---|
| **deep tree**              | 4  | 0.7% | 1.4% | 2.6% | 4.7% |
|                           | 9  | 1.8% | 3.3% | 6.5% | 11.5% |
|                           | 14 | 2.9% | 5.3% | 10.3% | 18.4% |
| **average 1.5 hops**       | 4  | 0.5% | 0.9% | 1.7% | 3.2% |
|                           | 9  | 1.1% | 1.9% | 3.7% | 6.7% |
|                           | 14 | 1.6% | 2.9% | 5.6% | 10.2% |
| **average 2 hops**         | 4  | 0.6% | 1.2% | 2.3% | 4.1% |
|                           | 9  | 1.3% | 2.4% | 4.7% | 8.5% |
|                           | 14 | 2.1% | 3.8% | 7.3% | 13.3% |

At SF5-SF6 the idle control plane stays within a few percent even at 14 nodes. At SF8
the deep 14-node tree reaches 18.4%, but that is dominated by the **claim plane**:
50 claim-hops per 90 s is ~16% on its own (beacons 2.1%, table 0.3%), so a deep
14-node SF8 network should be avoided or the claim interval lengthened. Churn adds
change-triggered `ADDR_TABLE` floods on top of the backstop counted here.

### Punch throughput

A WiRoc *punch* round trip costs: one **uplink** `DATA` (15 B payload -> 18 B frame)
over `d_avg` hops; one **`LINK_ACK`** (4 B) from the master (relays use the implicit
ACK, which costs no frame); and one **downlink** ACK `DATA` (7 B payload -> 10 B
frame) that is *flooded* back, so it is relayed by every non-leaf -- `F`
transmissions. Fitting exchanges into the time left after the control plane (table
above), assuming an error-free channel (no retransmits) and one exchange at a time,
gives the network-wide rate below.

**Punches per minute, network-wide** (15 B up / 7 B down, `LINK_ACK` included).

| topology | slaves | SF5 | SF6 | SF7 | SF8 |
|---|---|---|---|---|---|
| **deep tree**              | 4  | 145 | 81 | 45 | 25 |
|                           | 9  | 126 | 71 | 38 | 20 |
|                           | 14 | 121 | 67 | 36 | 18 |
| **average 1.5 hops**       | 4  | 204 | 114 | 64 | 35 |
|                           | 9  | 148 | 83 | 46 | 25 |
|                           | 14 | 107 | 59 | 32 | 17 |
| **average 2 hops**         | 4  | 174 | 98 | 54 | 30 |
|                           | 9  | 111 | 62 | 34 | 18 |
|                           | 14 | 85 | 47 | 25 | 13 |

The **downlink ACK flood dominates** once the tree has many relays (see below).
Retries, busy backoff and per-hop implicit-ACK waits make these figures upper bounds.

### Why the deep tree beats the balanced ones

The two costs move in opposite directions, because the uplink is a **unicast** (one
frame per hop) while the downlink ACK is a **flood** (one frame per *relay*). A
balanced tree is shallower per node but has many more relays, so its ACK flood is
bigger even though its uplinks are shorter. The three N=14 shapes (`r` = relay /
non-leaf, `l` = leaf):

deep tree (max uplink hops):

```
       M r
       |
       a r
       |
       b r
       |
       c r
  +-+-+-+-+-+-+-+-+-+-+
  l l l l l l l l l l l   (11 leaves)
relays = 4 (M,a,b,c)   d_avg = 3.6
```

average 1.5 hops:

```
          M r
  +-+-+-+-+-+-+-+
  a b c d e f g         (7 relays at depth 1)
  | | | | | | |
  l l l l l l l         (7 leaves at depth 2)
relays = 8 (M + 7)   d_avg = 1.5
```

average 2 hops:

```
            M r
  +-+-+-+-+
  a b c d               (4 relays at depth 1)
  | | | | \
  e f g h  i            (5 relays at depth 2)
  | | | |  |
  l l l l  l            (5 leaves at depth 3)
relays = 10 (M + 4 + 5)   d_avg = 2.1
```

| tree | uplink unicast `d_avg x 369 ms` | `LINK_ACK` (master) | downlink ACK flood `relays x 289 ms` | exchange | punches/min |
|---|---|---|---|---|---|
| deep tree | 1317 ms | 246 ms | **4 x 289 = 1156 ms** | 2719 ms | **18** |
| average 1.5 hops | 554 ms | 246 ms | **8 x 289 = 2312 ms** | 3112 ms | 17 |
| average 2 hops | 712 ms | 246 ms | **10 x 289 = 2890 ms** | 3848 ms | **13** |

(`LINK_ACK` is a single 4 B frame from the master per uplink -- 246 ms at SF8; the older
`exchange` figures folded it in silently, which is why they exceed uplink + downlink.)

So the **deep tree** is the *best* case for the downlink flood (fewest relays) even
though it is the worst case for hops -- and for the idle control plane, where the
claim term dominates. It is the flood, not the hop count, that caps throughput. A
reverse source-route ACK (the reserved `HAS_PATH`) would cost `d_avg` hops instead of
`relays` and flip the ordering back to the intuitive one (shorter tree -> more
throughput).

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
* **Lab, 2 devices**: set a node device id on each (`ATC+MESHNODEDEVICEID=<12 hex>`), enable mesh,
  designate one master -> the slave joins and is assigned an address; slave
  `ATC+SEND` arrives at the master `ATC+REC` with `srcaddr` = the slave; an
  app-level ACK sent back with `ATC+SEND=<slave>:...` reaches the slave.
* **Narrowband**: run two devices at `ATC+P2P=0:<freq>:<sf>:7:<cr>:...` (31.25 kHz) for SF5
  and SF8; verify join, uplink, downlink, and that `ATC+MESHMAP?` reports the derived
  `ackms` and the measured `overhead%`.
* **Lab, multi-hop**: three nodes in a line with the far one out of the master's
  range -> it reaches the master through the middle node; perturb SNR and confirm
  the cost-based parent choice and hysteresis.
* **Failure / recovery**: power off a relay -> children re-attach; reboot the
  master -> nodes re-claim on the new epoch and are back within one or two beacon
  intervals. Reboot a slave -> it keeps / re-joins its address.
* **Flood pruning / change-triggered table**: add a node and confirm the master
  emits an `ADDR_TABLE` within ~`MESH_TABLE_DEBOUNCE_MS` (sniff, or watch the new
  node's fast adoption) rather than waiting for the backstop; confirm a fresh
  multi-hop node keeps reconciling (no rising table-miss / no spurious re-join),
  i.e. its parent still relays the table. Confirm a *leaf* does not re-broadcast
  `ADDR_TABLE` / `DATA_DOWNLINK` while a node with children does.
* **Channel access (CAD / backoff)**: hold the channel busy with a second
  transmitter and confirm a host `ATC+SEND` returns `AT_BUSY_ERROR` with no
  module-side resend, while module-generated traffic (beacons, joins) keeps
  retrying after the randomised backoff once the channel clears. Confirm CAD is
  applied even with `AT+CAD=0` persisted (i.e. listen before talk is always on).
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
