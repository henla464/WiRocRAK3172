# LoRa mesh mode

WiRocRAK3172 normally runs as a **P2P** LoRa modem driven by the WiRoc host over
AT commands: every node broadcasts, and a node only reaches the host if it is in
direct radio range. This document describes the optional **mesh mode**, which adds
automatic multi-hop routing so that distant nodes can still reach the gateway.

Mesh mode is **opt-in** and off by default: with it disabled the firmware behaves
exactly as before (legacy P2P), so existing deployments are unaffected.

## Model

* **Exactly one master node** (gateway), designated by configuration -- there is no
  election. The master node is the only node that feeds its host.
* **4-bit addresses (0-15)** assigned automatically by the master node; the master node
  itself always owns address `1`, all other nodes use `2`-`15` (14 nodes max).
* **Convergecast** routing: every node forwards toward the master node, multi-hop
  (up to `4` hops).
* Each node has a **host-provisioned 6-byte node device id** used to claim an
  address; a node with no node device id never starts the mesh.
* The master node can send a payload **down** to a specific node (needed for
  application-layer ACKs).

```
            (master node, addr 1)
                 /        \
        (addr 4)      (addr 5)
             |
        (addr 7)   <-- out of the master node's range: reaches it via addr 4
```

## Enabling

| Command | Meaning |
|---------|---------|
| `ATC+MESH=<0\|1>` | Enable / disable mesh mode (persisted). `ATC+MESH?` reads it. |
| `ATC+MASTER=<0\|1>` | Designate this node as the master node (persisted). `ATC+MASTER?` reads it. |
| `ATC+MESHNODEDEVICEID=<12 hex>` | Set the 6-byte node device id (persisted). `ATC+MESHNODEDEVICEID?` reads it back. |
| `ATC+MESHMAP=?` | Diagnostics (see below). |
| `ATC+MESHTOPO=?` | Dump the topology map this node knows, one line per node (see below). |
| `ATC+P2P=<runcfg>:...,[:<mesh>:<master>:<deviceid>]` | Mesh flags and node device id as trailing params of the P2P config command. |

A device only joins the network once mesh mode is enabled **and** it has a node
device id and it is not the master node. A node with **no node device id** neither
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
addresses the master node has handed out; `parent` is the current next hop (`0`=none);
`hops`/`cost` are the route to the master node; `neighbour-count` is the live neighbour
count; `epoch` is the master node's **boot number** -- a counter the master node increments
on every boot -- as last seen by this node (only its low 3 bits are advertised;
see *Recovery*); `overhead%` is the measured control-plane airtime as a
percentage of the data-frame airtime transmitted; and `ackms` is the derived
per-hop ACK timeout for the current datarate.

`ATC+MESHTOPO=?` returns the topology map **this node** knows, one line per known
node:

```
ATC+MESHTOPO=<addr>:<parent>:<nbr>=<cost>,<nbr>=<cost>,...
```

`addr` is the node, `parent` is its parent address (`0` = none), and the trailing
list is the neighbours it hears with their link costs (omitted when none). Every
node serves its **own** row (live from its neighbour table) plus a row for each
**descendant** whose report it has passed rootward, so the master node -- being the
root -- covers the whole network while any other node sees only the part of the
tree it is on the path for. See *Topology map* below.

## Host interface changes

Addresses travel in the AT interface, so the host is updated in lock-step:

* **`ATC+SEND=<dest>:<hexpayload>`** -- optional leading destination address.
  `dest=0` means "the master node / default" and is what legacy P2P and every
  non-master mesh node use. A non-zero `dest` is honoured **only by the master node**
  as a downlink to that node; on a non-master it is ignored and the payload goes
  to the master node anyway. The legacy single-parameter form `ATC+SEND=<hexpayload>`
  stays valid (`dest=0`).
  In **both** modes a busy channel (CAD) is reported back as `AT_BUSY_ERROR`; the
  host backs off and resends (see "Channel access").
* **`ATC+REC`** appends one **source-address** byte to both response forms:

  ```
  hex:    OK:<hex>:<rssi>:<snr>:<status>:<srcaddr>
  binary: OK + data + rssiH + rssiL + snr + status + srcaddr
  ```

  `srcaddr` is the mesh source of the delivered frame (the node that injected it).
  In P2P mode it is `0`. The master node uses it to learn which node to ACK.

## Wire format

Mesh mode is exclusive: with it enabled **every** on-air frame is a mesh frame, so
no magic bytes are needed (the LoRa PHY CRC plus the packed version/type fields act
as the sanity gate). The header is **3 bytes** (24 bits, no waste), MSB-first:

| bits | field | notes |
|------|-------|-------|
| 4 | type | message type, `0`-`9` (see **Message types** below) |
| 1 | flags | reserved; always `0` today (see **Header flags**) |
| 4 | src | source address (0-15) |
| 4 | dst | destination (0-15; also carries the assigned address / acked origin) |
| 3 | hops | beacon: hop-count to the master node; data: TTL (max 4) |
| 5 | seq | dedup key `(src,seq)`, 32-value window |
| 3 | version | protocol version (currently `3`) |

```
byte0 = type<<4 | flags<<3 | src>>1
byte1 = (src&1)<<7 | dst<<3 | hops
byte2 = seq<<3 | version
```

**Header flags.** The 1-bit `flags` field is **reserved and always `0`** today.
There is no "ack requested" flag: the master node answers **every** delivered uplink
with an explicit `LINK_ACK`, since it has no next hop whose forward it could
overhear the way a relay node does. The only defined bit, `HAS_PATH` (0x01), is a
reserved hook for carrying an explicit source path in the payload instead of
relying on per-hop routing state; encoders send `0` and receivers ignore it.

The WiRoc payload follows verbatim and is stripped of the header before delivery to
the host. Control beacons carry a **1-byte** control payload after the header:
`epoch[3]` (bits 7-5) packed over `path cost[5]` (bits 4-0). The 3-bit `epoch` is
the low bits of the master node's **boot number** -- incremented by the master node on each
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

The 4-bit `type` field carries one of the following values. `MESH_NODE_DEVICE_ID_LEN`
= 6 (the host-provisioned node device id); the value itself is part of the wire
format and must not change without a version bump. An address that is already in the
header is **not** repeated in the payload (header-field reuse).

| id | type | on-air size (B) | delivery | payload | purpose |
|----|------|-----------------|----------|---------|---------|
| 0 | `BEACON` | 4 | link-local, not relayed, not deduped | `epoch[3]\|cost[5]` (1 B); header `hops` = hop-count to the master node | Liveness and routing advertisement. Emitted on an adaptive interval by the master node and by any node that has a parent. Neighbours use it to select a parent and to detect a node going quiet. |
| 1 | `JOIN_REQ` | 9 | flooded | `devid[6]`; `src` = 0 | An unassigned node asks the master node for an address, retrying with backoff until assigned. Because `src` is 0, it is deduped on its node device id instead of `(src,seq)`. |
| 2 | `ADDR_ASSIGN` | 9 | flooded | `devid[6]`; **assigned address in header `dst`** | The master node's answer to a `JOIN_REQ` (and its re-assertion after an `ADDR_CLAIM`). The node whose node device id matches adopts and persists the address. A node that sees its own address given to a *different* node device id relinquishes it. |
| 3 | `ADDR_TABLE` | 5 | flooded (relayed by relay nodes only) | `occupied_bitmap[2]` (16-bit; the master-node bit is always set) | The master node floods its occupied-address bitmap whenever the table **changes** (debounced) and as a slow periodic backstop, so nodes can reconcile. A node that sees its own bit clear for `MESH_TABLE_MISS_LIMIT` intervals relinquishes its address and re-joins. A leaf node does not relay it (see "Flood relay"). |
| 4 | `DATA_UPLINK` | 3 + N | unicast hop-by-hop (`src` = origin, `dst` = parent) | WiRoc payload (N bytes) | A WiRoc payload travelling toward the master node. Each relay node dedups `(src,seq)`, decrements the TTL and forwards to its parent. |
| 5 | `DATA_DOWNLINK` | 3 + N | steered hop-by-hop down the tree (`dst` = target) | WiRoc payload (N bytes) | A WiRoc payload from the master node to one specific node. The target delivers it to its host; any other node re-broadcasts it one hop further **only when one of its children's subtree bitmaps covers the target**, so the frame follows the single branch that leads to the target (`depth(target)` transmissions) instead of flooding the tree. Best-effort (no per-hop retry); the host resends if the application-layer ACK does not return. |
| 6 | `LINK_ACK` | 4 | broadcast (single hop) | `acked_seq[1]`; **acked origin in header `dst`** | Explicit per-hop ACK, emitted only by the master node, which has no next hop whose forward it could overhear; it sends one for every `DATA_UPLINK` it delivers. Relay nodes use the implicit ACK instead. |
| 7 | `ADDR_CLAIM` | 10 | rootward unicast (flood fallback) | `devid[6]\|parent[1]`; **own address already in header `src`** | A node (re)announces its flash-stored address and binds it to its node device id, so a restarted master node can rebuild its RAM-only table; the trailing byte reports its **current parent** (`0`=none), so a claim doubles as a parent report for the topology map. **Event-driven** -- sent on a new boot epoch, on boot, on re-attach, on a parent change, and when the depth >= 2 liveness watchdog sees our parent stop covering us; never on a plain timer. Rootward hop-by-hop toward the parent; a node with no route yet -- or a relay node that has lost its own parent -- floods it instead. |
| 8 | `ADDR_ALIVE` | 5 | single hop to the parent (absorbed, never forwarded) | `subtree_bitmap[2]` (16-bit; bit `a` = "address `a` is alive in my subtree") | Periodic liveness **aggregate**, so the master node does not evict a deep idle node (whose beacon is link-local and never reaches it). Each node sets its own bit and ORs in the bitmap reported by each of its children, so **one frame covers a whole subtree**; the parent absorbs it and does not forward it, so the union of the master node's direct children's bitmaps covers every non-master node each round. **Only a relay node sends one** (a leaf is covered by its parent from its beacon); the master node refreshes last-heard for **every** set bit. Sent while the node holds a parent. |
| 9 | `TOPOLOGY` | 4 + 2K | rootward unicast (`dst` = parent) | `parent[1]\|{addr[1]\|cost[1]}*` (K neighbours) | The reporting node's view of the graph: its current parent and the neighbours it hears, each with its link cost. **Event-driven** (attach / re-parent) plus a slow backstop; **best-effort** (the backstop recovers a loss). Every node on its rootward path absorbs it, so the master node sees the whole network and a relay node sees its own subtree; a relay node also forwards it one hop rootward. Read the local view with `ATC+MESHTOPO?`. |

All sizes include the fixed 3-byte header. The control types (0, 1, 2, 3, 6, 7, 8) have
a fixed length; the two data types (4, 5) are `3 + N`, where `N` is the verbatim
WiRoc payload (WiRoc punch payloads are typically 15 or 27 bytes, giving 18 or 30
bytes on air), and `TOPOLOGY` (9) is `4 + 2K` for `K` neighbours. A frame cannot
exceed `MESH_MAX_FRAME` = 64 bytes, so `N` <= 61 for
data frames (and the control payloads above are well within that).

`MESH_TYPE_COUNT` (= 10) is not a wire value: the 4-bit field encodes only `0`-`9`,
so it is used as the "invalid / out of range" bound when validating a frame.

## Join / address assignment

1. A node with no stored address enters **JOINING** and floods
   `JOIN_REQ{ devid[6] }` from the mesh timer (start ~3 s, backing off to 15 s).
   The `devid` is the **6-byte host-provisioned node device id** (see
   `ATC+MESHNODEDEVICEID`); it is *not* a secret, only used to correlate an
   assignment back to the requester. A node with no node device id set never
   enters JOINING.
2. The master node keeps a RAM-only node-device-id->address table, allocates the
   lowest free address in 2-15 and floods `ADDR_ASSIGN{ devid }` (the address is
   the header `dst`).
3. The matching node adopts and **persists** the address (flash), emits
   `ADDR_CLAIM`, resets its beacon to fast, and becomes **JOINED**.
4. The master node floods `ADDR_TABLE{ occupied_bitmap[2] }` **whenever the table
   changes** (debounced), with a slow periodic backstop; a node whose own bit is
   cleared for several intervals relinquishes its address and re-joins.

The master node owns address `1`. Its table is **RAM-only**; nodes are the source of
truth for their own address via flash (see *Recovery*).

## Routing and the link-quality metric

All routable nodes emit a **beacon** advertising
`path cost = link_cost(self,parent) + parent.path_cost` and their hop-count (a
leaf node advertises at a reduced rate -- see "Adaptive beacon" below). A
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
to its parent; each relay node dedups `(src,seq)` and forwards to its own parent
(TTL - 1). The master node queues the payload to its host with `srcaddr` = the origin.

### Downlink

The master node sends `DATA_DOWNLINK{ src=master, dst=target, ttl }`; the node whose
address matches delivers it to its host, and any other node relays it one hop further
**only when one of its children's subtree bitmaps contains the target** (bounded by TTL
and dedup). The subtree bitmaps carried by the `ADDR_ALIVE` aggregates therefore double
as a routing table: at each hop exactly one node -- the one on the branch that leads to
the target -- re-broadcasts, so the frame travels a single path of `depth(target)` hops
instead of flooding. A relay node re-publishes its aggregate promptly when a **new**
child appears (not just on the periodic interval), so a freshly attached node becomes
routable without waiting a whole interval. The steer is best-effort (there is no forward
to overhear at the last hop, so no per-hop retry); the host resends if no
application-layer ACK comes back.

## Channel access (listen before talk)

Every transmission is preceded by a **CAD** (channel activity detection) that is
**forced on in firmware** -- `api.lora.psend(len, frame, true)` -- so it does not
depend on the module's persisted `AT+CAD` setting. When CAD detects activity the
frame is **not** put on the air (`psend` returns false).

How a busy channel is handled depends on who wants the transmission:

* **Module-generated traffic** (beacons, joins, claims, table floods, relay forwards,
  link ACKs and link retransmits) is queued and re-tried by the mesh timer. After
  a busy verdict the drain holds off for a **randomised backoff** that starts at
  `MESH_TX_BACKOFF_MIN_MS` and **doubles per consecutive busy attempt** up to
  `MESH_TX_BACKOFF_MAX_MS`, resetting after a clean send. The random draw (from a
  per-node xorshift PRNG) stops nodes that just collided from retrying in lockstep
  on the 200 ms tick.
* **Host-originated traffic** (`ATC+SEND`, i.e. an uplink or a master-node downlink) is
  **not queued**. The frame is tried **once, immediately**: if the channel is busy
  the AT handler returns `AT_BUSY_ERROR` and the **host owns the backoff and
  resend**, exactly like legacy P2P mode. If the frame does go out, it is still
  armed in the pending slot, so a lost *hop* is retried by the mesh MAC (below)
  independently of the host.

The legacy P2P `ATC+SEND` and the built-in punch auto-ACK also pass `true`, so CAD
is checked first there too.

## MAC / link reliability

Uplink unicasts are remembered in a single pending slot. A relay node clears its pending
frame when it **overhears the next hop forward the same `(origin,seq)`** -- an
implicit ACK -- and otherwise retransmits up to `MESH_LINK_RETRIES` times at the
per-hop ACK timeout, then gives up. The **master node has no next hop to overhear**, so
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

**Master node restart.** The master node's table is RAM-only. To recover it:

* The master node persists a **boot counter** and advertises its low **3 bits** as the
  beacon **epoch**, beacons fast for `MESH_RECOVER_MS` after boot, and defers new
  allocations during that window so returning nodes win back their own addresses
  first. The 3-bit epoch is enough because its only job is *fast* reboot detection;
  a rare miss (the master node rebooting a multiple of 8 times while a node was deaf)
  is caught instead by the `ADDR_TABLE` bitmap -- the rebooted master forgets the
  node, its bit stays clear for `MESH_TABLE_MISS_LIMIT` floods, and the node
  relinquishes its address and re-joins.
* A node (re)binds its flash-stored address with an `ADDR_CLAIM{ devid, parent }`
  (its own address is already in the header `src`) when it sees a new epoch (fast
  path), on its first beacon after boot, and after re-attaching a lost parent (and
  on switching to a better parent).
  Claims are **event-driven, not periodic**. The claim is a **rootward unicast**:
  it travels hop-by-hop toward the parent (like an uplink), so the network-wide
  cost is `O(hops)` per claim rather than the `O(N)` of a flood. Only a node with
  no route yet (just booted / parent lost), or a relay node that has lost its own
  parent, falls back to flooding it.
* **Liveness is a separate, cheaper message, and it is aggregated.** A deep idle
  node's beacon is link-local, so it would otherwise go unheard at the master node;
  the node instead sends an `ADDR_ALIVE` **subtree aggregate** to its parent every
  `MESH_ALIVE_INTERVAL_MS` (300 s). The 2-byte payload is a bitmap of the
  addresses alive in that node's subtree: the node sets its own bit and ORs in the
  bitmap each child reported. The parent **absorbs** the frame (it is a single
  hop, never forwarded) and folds it into its own next aggregate; the master node
  refreshes last-heard for **every** set bit. So one frame covers a whole subtree
  and the liveness plane costs `O(relays)` single-hop frames per round --
  independent of tree depth and node count, unlike the old per-hop keepalive.
  **Only relay nodes send one:** a leaf sends nothing and is covered by its parent,
  which learns it when it joins / re-attaches (the child's event-driven
  `ADDR_CLAIM` is addressed to and forwarded by that parent) and keeps it fresh
  from its link-local beacon. If that early claim is lost the leaf's bit stays
  clear and the `ADDR_TABLE` miss path makes it re-join and re-claim. (Splitting
  liveness from the claim is what lets the claim be event-driven and the liveness
  be aggregated.)
* **A node at depth >= 2 watches its own coverage, so relay-only liveness cannot
  strand it.** A deep node (one whose parent is not the master node) **overhears its
  parent's `ADDR_ALIVE` aggregate** -- a single-hop unicast to the grandparent, which
  the shared medium lets it see -- and checks whether its own bit is still set. If the
  parent stops aggregating altogether (it rebooted and lost its RAM-only relay role) or
  keeps aggregating but drops the bit, the node re-claims. A claim re-registers it at
  every hop (`mesh_note_relay`) *and* re-adopts its address at the master node, so the
  watchdog is safe even if eviction has already fired. Steady state costs nothing; a
  parent aggregate has to be missing for `MESH_MONITOR_TIMEOUT_MS` (1.5x the aggregate
  interval), and each re-claim is rate-limited to one per interval so a marginal link
  cannot storm. A depth-1 node is exempt: the master node hears its beacon directly.
* The master node rebuilds its table from the claims: it adopts the claimed address when
  free, re-asserts its own assignment when the node device id is already known, and
  hands out a fresh address when the claimed one is already taken.
* Routing rebuilds on its own: the master node is beaconing again within one interval and
  routing state is node-side.

**Eviction / recycling.** A node is refreshed well inside the `MESH_EVICT_MS` (610 s)
window either by a frame carrying its own `src`, or by any ancestor's `ADDR_ALIVE`
aggregate (every `MESH_ALIVE_INTERVAL_MS`, 300 s) that carries its bit -- so an
idle-but-alive node is never dropped and even a single missed aggregate is tolerated.
A relay node drops a child that has been silent for `MESH_ALIVE_CHILD_HOLD_MS` (610 s)
from its own aggregate, so a dead node's bit stops being reported and its ancestors'
coverage shrinks toward the master node. Eviction is then **subtree-scoped**: the
master node records which **direct child** covered each address (`s_cover`), so when a
direct child's aggregates have been silent for `MESH_EVICT_MS` it frees that child **and
its whole covered subtree at once** -- one coherent event per branch -- with a
per-address timer as the backstop for a leaf sitting directly under the master node.
A relay's bitmap is trusted only while it keeps being refreshed (`s_child_bm_ms`), so a
child that has stopped aggregating -- because it *became* a leaf -- is reported as just
itself instead of a stale subtree.

**Relay-only liveness, and its watchdog.** Because a leaf sends no liveness of its own,
its place in the tree rests on its event-driven `ADDR_CLAIM` plus the parent's
beacon-driven refresh of it. A relay that reboots while its children are all *quiet
leaves* would otherwise make its whole branch invisible to the master node until the
eviction timer (610 s) plus three `ADDR_TABLE` misses pushed the children to re-join.
The depth >= 2 watchdog (above) closes that window: a node notices within
`MESH_MONITOR_TIMEOUT_MS` (450 s) -- before eviction -- and re-claims, and the claim also
re-registers it with its parent, so the parent becomes a relay again and re-covers the
branch. The `ADDR_TABLE` miss path stays as the outer backstop (a lost initial claim, or
a node too far from its parent to overhear the aggregate).

**Duplicate addresses.** If a node hears its own address handed to a different node
device id it relinquishes it and re-joins, so no duplicate address can persist.

## Topology map

The node/link map is assembled from two reports, both travelling **rootward**
toward the master node:

* every `ADDR_CLAIM` (sent when a node (re)attaches or re-parents) carries the
  sender's **parent**, so a tree edge is learned the moment a node attaches; and
* a dedicated rootward `TOPOLOGY` frame carries the sender's **parent and the
  neighbours it hears** (with link costs), so the off-tree links are learned too.

The map has one row per node: its reported parent (`0`=none) and the set of
neighbours it hears, each with the link cost that node measured. A report
**replaces** that node's whole row; a row is dropped after `MESH_TOPO_HOLD_MS`
without a report (`MESH_TOPO_INTERVAL_MS` is the backstop, and a re-parent reports
immediately).

Both report types are hop-by-hop **unicasts along the tree**, so each one is
absorbed by every node on the reporter's rootward path -- the reporter's parent,
grandparent, and so on up to the master node -- and by no one else. It follows
that **each node sees the part of the tree it is on the path for**: its own row
(live) plus a row for every **descendant** (whose reports pass through it). The
**master node**, being the root, is on every report's path and so covers the whole
network. Read the local view with `ATC+MESHTOPO?` on any node.

The map is a **diagnostic**: links are measured one-way and are noisy/asymmetric,
and the reports are best-effort (a missed report is recovered by the backstop).
Nothing in the routing path depends on it.

## Tunables

All intervals live in `mesh.h` and can be adjusted without touching logic:

| Constant | Default | Purpose |
|----------|---------|---------|
| `MESH_BEACON_FAST_MS` | 1000 | fast beacon interval (start / after any topology change / master node recovering) |
| `MESH_BEACON_MAX_MS` | 90000 | adaptive beacon back-off ceiling (interval doubles while stable) |
| `MESH_BEACON_LEAF_MULT` | 3 | a leaf node beacons at this multiple of its back-off interval |
| `MESH_RELAY_HOLD_MS` | 400000 | a node counts as a relay node while a child declared itself to it within this window (>= `MESH_ALIVE_INTERVAL_MS` so liveness aggregates keep it refreshed) |
| `MESH_JOIN_INTERVAL_MS` / `MESH_JOIN_INTERVAL_MAX_MS` | 3000 / 15000 | join backoff |
| `MESH_TABLE_INTERVAL_MS` | 300000 | ADDR_TABLE flood backstop period (a table change also triggers an immediate flood) |
| `MESH_TABLE_DEBOUNCE_MS` | 500 | window that coalesces table changes into one ADDR_TABLE flood |
| `MESH_ALIVE_INTERVAL_MS` | 300000 | **relay** liveness **aggregate** period (must stay well under `MESH_EVICT_MS`) |
| `MESH_ALIVE_CHILD_HOLD_MS` | 610000 | a relay node drops a child from its aggregate after this long without hearing it |
| `MESH_EVICT_MS` | 610000 | master-node eviction grace |
| `MESH_MONITOR_TIMEOUT_MS` | 450000 | depth >= 2 node: no parent aggregate for this long -> re-claim (1.5x `MESH_ALIVE_INTERVAL_MS`; must stay under `MESH_EVICT_MS`) |
| `MESH_RECOVER_MS` | 10000 | master-node post-boot recovery window |
| `MESH_LINK_RETRIES` | 3 | link retransmits (timeout is derived, see MAC) |
| `MESH_TX_BACKOFF_MIN_MS` / `MESH_TX_BACKOFF_MAX_MS` | 40 / 1280 | queued-frame backoff after a busy CAD (randomised window, doubles per attempt) |
| `MESH_DEFAULT_TTL` | 4 | max hops |
| `MESH_NEIGHBOR_MAX` | 8 | tracked neighbours per node |
| `MESH_PARENT_HYSTERESIS` | 1 | cost margin required to switch parent |
| `MESH_TOPO_INTERVAL_MS` | 300000 | topology-report backstop period (a re-parent also triggers an immediate report) |
| `MESH_TOPO_HOLD_MS` | 900000 | the master node drops a node's map row after this long without a report |
| `MESH_TOPO_MAX_NEIGH` | 8 | max neighbours carried in one `TOPOLOGY` report (= `MESH_NEIGHBOR_MAX`) |

**Adaptive beacon.** Each node's beacon interval starts at `MESH_BEACON_FAST_MS`
and **doubles per stable interval** up to `MESH_BEACON_MAX_MS`, and is **reset to
fast** on join, re-attach, parent change, epoch change or when a better candidate
appears. Parent staleness is `3x` the *current* interval. The master node keeps its
`MESH_RECOVER_MS` fast window after boot, then backs off too. Trade-off (accepted):
failure detection slows down once backed off.

**Leaf suppression.** A **leaf node** -- a joined node that no other node routes
through -- is on nobody's path, so it only needs to advertise itself as a
*potential* parent rather than maintain a live route. It therefore beacons at
`MESH_BEACON_LEAF_MULT` x its back-off interval (3x, so ~270 s in steady state),
cutting the dominant control-plane term (most nodes in a convergecast tree are
leaf nodes). A node is a **relay node** while a child declared itself to it **addressed
to it** within `MESH_RELAY_HOLD_MS` -- via a rootward unicast (uplink / claim), a
liveness aggregate, or that child's own beacon (the only signal that another node has
selected it as its parent); relay nodes and the
master node beacon at the full `MESH_BEACON_MAX_MS` rate. On the leaf-node -> relay-node
transition the beacon resets to fast so the new child can track us promptly.
Trade-off (accepted): discovering a leaf node as a parent takes up to one leaf-node
interval, so re-parenting onto a former leaf node is slower (its parent-side liveness
is unaffected -- the leaf node's own `ADDR_ALIVE` aggregate keeps it alive at the relay
node within one interval, and at the master node -- via that relay node -- regardless).

**Flood relay.** A flood is re-broadcast once per receiving node (deduped on
`(src,seq)`), so one flood costs one transmission per relay node. Floods whose
recipients are all **attached** nodes -- `ADDR_TABLE` -- are
relayed only by **relay nodes** (not leaf nodes, the same signal as leaf suppression): a
node with no children has no downstream node, so its re-broadcast reaches nobody
that has not already seen the frame, while the routing tree guarantees every
attached node still receives the flood from its own parent. `DATA_DOWNLINK` is not a
flood at all: it is **steered** down the tree, re-broadcast only by the node whose
child's subtree bitmap contains the target (see the message table and "Downlink").
Floods that must
reach an **unattached** node -- `JOIN_REQ`, `ADDR_ASSIGN` and the flooded
`ADDR_CLAIM` fallback -- are still relayed by
**every node**, because the target may
not yet have a parent whose forward it could rely on.

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
For the v3 frame sizes (CR 4/5, preamble 8, CRC on), approximate airtimes in ms are:

| on-air frame | B | SF5 | SF6 | SF7 | SF8 |
|---|---|---|---|---|---|
| BEACON | 4 | 36 | 72 | 123 | 246 |
| LINK_ACK | 4 | 36 | 72 | 123 | 246 |
| ADDR_TABLE | 5 | 41 | 72 | 123 | 246 |
| ADDR_ALIVE | 5 | 41 | 72 | 123 | 246 |
| JOIN_REQ / ADDR_ASSIGN | 9 | 46 | 82 | 164 | 287 |
| ADDR_CLAIM | 10 | 52 | 93 | 165 | 289 |
| TOPOLOGY (8 neighbours, worst case) | 20 | 71 | 120 | 219 | 390 |
| DATA (15 B payload) | 18 | 67 | 113 | 205 | 369 |
| DATA (7 B payload) | 10 | 52 | 93 | 165 | 289 |
| DATA (27 B payload) | 30 | 92 | 154 | 287 | 492 |

Two structural facts matter: the **fixed per-frame cost** (preamble + header symbols
= 12.25 symbols) is large -- at SF7 a 4-byte ACK spends ~40% of its airtime on the
preamble, so *fewer, larger frames beat many small frames* -- and the **proportional**
terms (the 3-byte header and the per-uplink ACK) never amortise, while only the
**fixed-rate** control terms do.

The overhead budget is `[header + per-uplink ACK + beacons + liveness + table] / data
airtime`. The design cuts it several ways: **4-bit addresses (14 non-master nodes max)** roughly
halve the beacon term, the **adaptive beacon** back-off plus **leaf suppression**
(only relay nodes and the master node beacon at the full rate) cut the steady-state beacon
term by the interval ratio, **frame slimming** (1-byte beacons, 2-byte ADDR_TABLE,
header-field reuse) shrinks every control frame, the **rootward `ADDR_CLAIM`** reaches
the master node hop-by-hop (flooding only when a node is route-less) instead of the
`O(N)` of a flood, and the **aggregated `ADDR_ALIVE`** collapses per-node liveness into
one single-hop subtree bitmap per **relay** node (a leaf sends nothing), so the liveness
plane is `O(F)` transmissions per round independent of tree depth. The host can
**measure** the resulting ratio live via `ATC+MESHMAP?` (`overhead%`). The header and
the per-uplink ACK are proportional terms (one per data frame), so they set a floor on
this ratio.

### Control-plane occupancy

The tables above are per-frame costs; these measure how much of the channel the
**module-generated** control plane occupies by itself (no data traffic), as a
percentage of wall-clock time -- i.e. the fraction of the time the single channel is
busy with mesh housekeeping. They count **beacons, `ADDR_ALIVE` aggregates and the
`ADDR_TABLE` flood**, and exclude `LINK_ACK` (which only exists alongside an uplink) and
application data. `ADDR_CLAIM`s and `TOPOLOGY` reports are event-driven (plus a slow
backstop) and contribute negligibly in steady state, so they are not counted.
Everything is derived from the airtimes above and the
live tunables: relay nodes and the master node beacon every `MESH_BEACON_MAX_MS` (90 s),
a leaf node every `3x` that (270 s); every **relay** node sends a single-hop `ADDR_ALIVE`
aggregate (5 B) once per `MESH_ALIVE_INTERVAL_MS` (300 s) -- **one frame per relay node,
independent of depth** -- while a leaf node is covered by its parent; the master node
floods `ADDR_TABLE` on the
`MESH_TABLE_INTERVAL_MS` backstop (300 s), relayed by non-leaf nodes only. `<nodes>` is
`N` (non-master nodes); the tree is rooted at the master node with depth `<= 4`.

With `B4` = airtime of a 4 B beacon, `A5` = airtime of a 5 B `ADDR_ALIVE`, `T5` = airtime
of a 5 B `ADDR_TABLE`, and `F` / `L` the non-leaf / leaf counts:

```
beacon%   = F*B4/900  + L*B4/2700
liveness% = (F-1)*A5/3000        (one single-hop aggregate per non-master relay;
                                  the master node does not send one)
table%    = F*T5/3000            (master flood + one relay per non-leaf node)
```

`SD` below is the sum of node depths (the uplink hop-cost); unlike the old per-hop
keepalive, liveness no longer scales with it.

| topology | F = non-leaf nodes (incl. master node) | L = leaf nodes | SD = sum of node depths |
|---|---|---|---|
| deep tree (max uplink hops -- fewest relay nodes) | 4 | N-3 | 4N-6 |
| average 1.5 hops (2-level: floor(N/2) at depth 1, the rest at depth 2) | 1+floor(N/2) | ceil(N/2) | ~1.5N |
| average 2 hops (balanced 3-level) | ~1+2N/3 | ~N/3 | ~2N |

**Control-plane occupancy, % of wall-clock time (LINK_ACK and data excluded)**

| topology | nodes | SF5 | SF6 | SF7 | SF8 |
|---|---|---|---|---|---|
| **deep tree**              | 4  | 0.3% | 0.5% | 0.9% | 1.8% |
|                           | 9  | 0.3% | 0.6% | 1.1% | 2.2% |
|                           | 14 | 0.4% | 0.8% | 1.3% | 2.7% |
| **average 1.5 hops**       | 4  | 0.2% | 0.4% | 0.7% | 1.4% |
|                           | 9  | 0.4% | 0.7% | 1.3% | 2.6% |
|                           | 14 | 0.6% | 1.2% | 2.0% | 4.1% |
| **average 2 hops**         | 4  | 0.3% | 0.5% | 0.8% | 1.6% |
|                           | 9  | 0.5% | 1.0% | 1.6% | 3.3% |
|                           | 14 | 0.7% | 1.4% | 2.4% | 4.9% |

The idle control plane stays within a few percent even at SF8. The worst cell -- the
average-2-hop 14-node tree -- reaches 4.9%, now **beacon-dominated** (beacons 3.2%,
table 0.8%, liveness 0.8%); aggregation made the liveness term `O(F)` (relay count) and
depth-free, and restricting it to relay nodes dropped the leaf frames entirely, so
the adaptive/leaf beacon back-off is the remaining lever on the worst case. Churn adds
change-triggered `ADDR_TABLE` floods on top of the backstop counted here.

**Control-plane target.** The design goal is that this module-generated traffic
(beacons, liveness aggregates and the table flood -- everything counted here) stays
**under 10%
of total wall-clock time**. `LINK_ACK` is deliberately excluded from both the
target and the table: it is a per-uplink cost that scales with data traffic, not a
fixed control-plane load. At the 300 s liveness-aggregate interval the target holds for
**every
topology up to 14 nodes across the whole SF5-SF8 range** (worst cell: the
average-2-hop 14-node tree at SF8, 4.9%).

### Punch throughput

A WiRoc *punch* round trip costs: one **uplink** `DATA` (15 B payload -> 18 B frame)
over `d_avg` hops; one **`LINK_ACK`** (4 B) from the master node (relay nodes use the implicit
ACK, which costs no frame); and one **downlink** ACK `DATA` (7 B payload -> 10 B
frame) **steered back down the tree**, so it also costs `d_avg` hops rather than one
transmission per relay node. One exchange is therefore `d_avg*(air18 + air10) + air4`
of airtime, and the network-wide rate (one exchange at a time, error-free channel, no
retransmits) is `60 / exchange` punches per minute. Retries, busy backoff and per-hop
implicit-ACK waits make these figures upper bounds, and the steer is best-effort (a
lost hop costs the ACK, which the host then resends).

**Punches per minute, network-wide** (15 B up / 7 B down, `LINK_ACK` included).

| topology | nodes | `d_avg` | SF5 | SF6 | SF7 | SF8 |
|---|---|---|---|---|---|---|
| **deep tree**              | 4  | 2.5  | 180 | 102 | 57 | 32 |
|                           | 9  | 3.3  | 139 | 79 | 44 | 25 |
|                           | 14 | 3.6  | 130 | 74 | 42 | 23 |
| **average 1.5 hops**       | 4  | 1.5  | 280 | 157 | 88 | 49 |
|                           | 9  | 1.6  | 271 | 153 | 86 | 47 |
|                           | 14 | 1.5  | 280 | 157 | 88 | 49 |
| **average 2 hops**         | 4  | 2.0  | 219 | 124 | 70 | 38 |
|                           | 9  | 2.0  | 219 | 124 | 70 | 38 |
|                           | 14 | 2.1  | 210 | 119 | 67 | 37 |

Because the ACK is steered, the **number of relay nodes no longer enters the exchange
cost** -- only the hop count `d_avg` does (see below).

### Why the balanced trees now win

Both legs are **unicasts of `d_avg` hops**, so the exchange cost tracks the hop count
and nothing else. The flood -- which used to scale with the number of relay nodes and
so rewarded the deep tree -- is gone. A balanced tree puts most nodes one or two hops
from the master node, so both the uplink and the steered ACK are short. The three N=14
shapes (`r` = relay node, `l` = leaf node):

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
  l l l l l l l l l l l   (11 leaf nodes)
relay nodes = 4 (M,a,b,c)   d_avg = 3.6
```

average 1.5 hops:

```
          M r
  +-+-+-+-+-+-+-+
  a b c d e f g         (7 relay nodes at depth 1)
  | | | | | | |
  l l l l l l l         (7 leaf nodes at depth 2)
relay nodes = 8 (M + 7)   d_avg = 1.5
```

average 2 hops:

```
            M r
  +-+-+-+-+
  a b c d               (4 relay nodes at depth 1)
  | | | | \
  e f g h  i            (5 relay nodes at depth 2)
  | | | |  |
  l l l l  l            (5 leaf nodes at depth 3)
relay nodes = 10 (M + 4 + 5)   d_avg = 2.1
```

| tree | uplink unicast `d_avg x 369 ms` | `LINK_ACK` (master node) | steered ACK unicast `d_avg x 289 ms` | exchange | punches/min |
|---|---|---|---|---|---|
| deep tree | 1328 ms | 246 ms | **3.6 x 289 = 1040 ms** | 2614 ms | 23 |
| average 1.5 hops | 554 ms | 246 ms | **1.5 x 289 = 434 ms** | 1234 ms | **49** |
| average 2 hops | 775 ms | 246 ms | **2.1 x 289 = 607 ms** | 1628 ms | 37 |

(`LINK_ACK` is a single 4 B frame from the master node per uplink -- 246 ms at SF8.)

So the **average 1.5-hop tree** is the best case: it has the shortest uplinks *and* the
shortest steered ACK. The deep tree is now the worst case for throughput (longest paths
on both legs), even though it stays the best for the idle control plane, where the
relay-count terms (beacons, aggregates, table flood) dominate -- the two optima differ.
A flood reversed this relation, because the ACK then cost one frame per relay node.
Steering the ACK along the subtree bitmaps removes that term with no wire change: the
bitmaps are already on air in the `ADDR_ALIVE` aggregates, so no reverse source-route
(`HAS_PATH`) is needed.

## Files

| File | Role |
|------|------|
| `mesh.h` / `mesh.cpp` | mesh engine: config, join, routing, MAC, recovery, topology map |
| `mesh_wire.h` / `mesh_wire.cpp` | pure 3-byte header codec (host-tested) |
| `mesh_alloc.h` / `mesh_alloc.cpp` | pure master-node address allocator (host-tested) |
| `mesh_route.h` / `mesh_route.cpp` | pure link metric + parent selection (host-tested) |
| `custom_at.cpp` | AT integration (`MESH`/`MASTER`/`MESHMAP`/`MESHTOPO`, `SEND`/`REC`) |
| `MessageQueue.h` | `SourceAddr` carried from RX to `ATC+REC` |
| `test/` | host `g++` unit tests for the pure modules |

## Testing

* **Host unit tests** (no target needed):
  `g++ -std=c++11 -Wall -Wextra -I. test/mesh_wire_test.cpp mesh_wire.cpp -o /tmp/t && /tmp/t`
  (same for `mesh_alloc_test`, `mesh_route_test`).
* **Build check** against the real target flags: compile the sketch sources with
  `-fsyntax-only` using the generated `compile_commands.json`.
* **Lab, 2 devices**: set a node device id on each (`ATC+MESHNODEDEVICEID=<12 hex>`), enable mesh,
  designate one as the master node -> the other node joins and is assigned an address; its
  `ATC+SEND` arrives at the master node's `ATC+REC` with `srcaddr` = its own address; an
  app-level ACK sent back with `ATC+SEND=<addr>:...` reaches it.
* **Narrowband**: run two devices at `ATC+P2P=0:<freq>:<sf>:7:<cr>:...` (31.25 kHz) for SF5
  and SF8; verify join, uplink, downlink, and that `ATC+MESHMAP?` reports the derived
  `ackms` and the measured `overhead%`.
* **Lab, multi-hop**: three nodes in a line with the far one out of the master node's
  range -> it reaches the master node through the middle relay node; perturb SNR and confirm
  the cost-based parent choice and hysteresis.
* **Topology map**: confirm `ATC+MESHTOPO?` on the master node lists every attached
  node with its parent and heard neighbours; force a node to re-parent (move it or
  perturb SNR) and confirm its row's parent updates within one report (immediately,
  via the re-claim), and that a powered-off node's row disappears after
  `MESH_TOPO_HOLD_MS`. On a non-master node `ATC+MESHTOPO?` shows its own row plus
  its descendants' rows and nothing outside its subtree; on the master node it shows
  every node.
* **Failure / recovery**: power off a relay node -> its children re-attach; reboot the
  master node -> nodes re-claim on the new epoch and are back within one or two beacon
  intervals. Reboot a non-master node -> it keeps / re-joins its address.
* **Flood pruning / change-triggered table**: add a node and confirm the master node
  emits an `ADDR_TABLE` within ~`MESH_TABLE_DEBOUNCE_MS` (sniff, or watch the new
  node's fast adoption) rather than waiting for the backstop; confirm a fresh
  multi-hop node keeps reconciling (no rising table-miss / no spurious re-join),
  i.e. its parent still relays the table. Confirm a leaf node does not re-broadcast
  `ADDR_TABLE` while a relay node does, and that a `DATA_DOWNLINK` is re-broadcast only
  by the branch whose subtree bitmap contains the target (a relay node off that branch
  stays silent).
* **Channel access (CAD / backoff)**: hold the channel busy with a second
  transmitter and confirm a host `ATC+SEND` returns `AT_BUSY_ERROR` with no
  module-side resend, while module-generated traffic (beacons, joins) keeps
  retrying after the randomised backoff once the channel clears. Confirm CAD is
  applied even with `AT+CAD=0` persisted (i.e. listen before talk is always on).
* **Regression**: with mesh disabled the P2P punch / double-punch ACK+hash path is
  unchanged (and no `srcaddr` byte is added).

## Limitations

* The master node is a single point of failure by design.
* The 4-bit space caps the network at 14 non-master nodes; addresses are recycled on eviction.
* `hops` is 3 bits (max 7 hops, we cap at 4) and `seq` is 5 bits (32-value dedup
  window, matching `MESH_DEDUP_SIZE`).
* The beacon `cost` is 5 bits, so the path cost must stay `<= 31`: with `hops <= 4`
  and a max link cost of 6 this holds with headroom (keep the link-cost map <= 7).
* The beacon epoch is 3 bits, so a master-node reboot that is a multiple of 8 while a
  node was deaf is caught by the `ADDR_TABLE` bitmap miss (the node sees its bit clear
  for `MESH_TABLE_MISS_LIMIT` floods and re-joins), not by a periodic claim.
* Downlink is steered along the tree by the `ADDR_ALIVE` subtree bitmaps rather than a
  recorded reverse source-route (the `HAS_PATH` flag stays reserved). The steer is
  best-effort: it needs the bitmaps to be current, so a node whose parent has not yet
  reported it -- e.g. a relay node that just rebooted and has not heard a silent leaf
  child re-claim -- is unreachable by downlink until the next aggregate or re-join; the
  application-layer ACK / retry is the backstop.
* The topology map is **diagnostic only** and, on a non-master node, **partial**: a
  node learns its own row plus its descendants' rows (the reports it forwards
  rootward), while the master node -- being on every report's path -- sees the whole
  graph. It is built from best-effort reports, its links are measured one-way (each
  node reports what *it* hears, so the graph is directed and may be asymmetric), and
  only a node's own row is live -- every other row is as fresh as that node's last
  report (until `MESH_TOPO_HOLD_MS`). Nothing in the routing path depends on it.
* Mesh and legacy P2P must not share a channel at the same time (mode is exclusive,
  and no magic byte distinguishes the two framings).
