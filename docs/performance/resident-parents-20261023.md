# Resident original-color parents — 2026-10-23

Exact original-parent preparation passed the frozen 17-case corpus on both
recorded AMD and Intel runtimes: **34 processes, 30 normal results and four
required unusable results**. Every independent CPU comparison passed. The
[machine-readable archive](resident-parents-20261023.json) retains all comparison
payloads, sanitized normal/fault metadata, output digests and runtime/source seals.
This closes the recorded parent-map/count/record boundary only. It establishes
no GPU timing, FPS, throughput, speedup, physical source or full-station acceptance.

## Exact boundary

The [module](../../ros2/src/so101_station_vulkan/include/so101_station_vulkan/parents.hpp)
borrows the completed producer's original one-hot color mask, preserving colored
pixels with invalid depth. Exact eight-connected parents use true minimum 2×2
raster-block keys; node zero is encoded as one. All parents receive per-color
GRANA IDs before area filtering. The full label map retains small parents, and
ordered eight-U32 records preserve original IDs, area, bounds and block keys.
Color ordinals are red, green, blue, yellow. Positive U32 thresholds above image
area, including UINT_MAX, leave full IDs/counts valid and emit no kept records.

The frozen corpus includes all eight committed parent-order witnesses, odd
shapes, false occupied-block and true diagonal edges, four colors in one block,
a 640×480 all-red parent, 4096 isolated records, above-area thresholds and both
negative cases. All N label words, every ordered record field and all four
parent counts compare exactly with explicit OpenCV `CCL_GRANA` expectations.
The CPU factory uses the committed fixture and retained CPU trace; no GPU/native
output can generate or replace expected values. Input regeneration is not a
replacement for the archived GPU output digests.

On each device, unsupported mask value 3 and two retained parents with capacity
one produce unusable output. `finish`, `resident_buffer`,
`resident_records_buffer`, `original_mask_buffer` and `records` all reject; no
map/count/record arrays are published. Valid cases also exercise wrong-pool
identity at the same Context/index, Core faults, both calibration generations,
pending source fences, copied leases, all-three-slot retention and dropped-source
collection. A weak producer token reserves no frame slot; actual source leases
remain retained through the parent and diagnostic consumer fences.

All-root storage is image-bounded independently of the compact cap. The
4096-record cap and 32 hook/flatten rounds are explicit budgets, not universal
scene admission or convergence proofs. Every actual required edge/root is
validated and incomplete convergence makes computed output unusable. Injected
host tests cover nonconvergence; the two GPU negative classes are mask and
compact overflow. Ordinary host readback remains compact only. The cold arena's
transfer-source capability serves the separate N×4-byte diagnostic copy.

## Recorded runtimes and ownership

| Profile | Actual hardware | Actual Vulkan loader | Float64 supported/enabled | Int64 supported/enabled |
|---|---|---|---|---|
| AMD | Radeon 8060S, 1002:1586, RADV | 1.4.341 | true/true | true/true |
| Intel | Graphics ADL-N, 8086:46d1 | 1.3.275 | false/false | true/true |

All nine parent SPVs validate with Shader capability only; the parent stage
requires neither Float64 nor Int64. The JSON preserves each device/driver/API,
compiler/build/ELF/library/ICD/DRM identity and all nine shader hashes. Per-process
loader and hardware selection were frozen; no global environment override was
part of this diagnostic.

Every child exited zero without watchdog signals or descendants. The real
external process-group owner provided a 90-second deadline, TERM/KILL/drain,
pre/post source/runtime seals and no-children checks. Own fences/polls had
10-second bounds; these do not bound cold driver calls or teardown. The command's
acknowledgment alone is not a watchdog.

Raw maps and post identities were archived before map-policy validation. Seven
last raw snapshots were empty because the process exited during observation;
first snapshots and observed map unions remain archived. Required executable,
loader/driver and critical libraries passed union/post-hash validation for all
34 processes. No complete final snapshot is claimed for those seven cases, and
no missing snapshot was fabricated or rerun.

The historical qualification context was HEAD
`a20f5ac2b7f6ed4ebdf6b6992911554485229201` plus uncommitted source additions.
All 21 relevant source files matched both frozen staged copies, and all 340
strict staged artifacts remained sealed. The later published source checkpoint
`f275c9afaa32dcf9e29e9ddcd45df3a520836a4c` matches all 21 source pins; it does not
change the historical measured context or establish a main/station release.

## CPU and build evidence

Canonical CPU tests passed 3979 with 136 documented skips and 48 passing subtests.
The native lane passed 347 with zero skips. All 93,528 upstream TestFloat vectors
executed; they were not silently skipped. Forty relevant before/after file hashes
were unchanged. A fresh full native package build/install passed all 15 CPU
CTests. Build/install contain 31/30 SPVs: all prior 17/16 legacy plus five
software-binary64 SPVs remain byte-identical, with nine new Shader-only parents.

Independent correctness and resource-flow reviews passed. The actual-module
synthetic host harness ran 10,000 operations each for empty, one-record and
4096-record results: 30,000 total, zero hot heap/Buffer allocations or waits and
73 dispatches per operation. It uses a fake Vulkan ABI, injected completion and
CPU compact copies; it excludes the real driver/GPU. Modeled root reads and
these host results do not establish GPU latency, speedup or a station budget.
Compiler-content/version/mode counter tests and source/SPV cache mutations
passed; provenance publishes only after matching outputs complete.

The archive keeps original report/output digest values unchanged. Public path
aliases and the renamed `procedural_source_tag` remove private locations and
identity fields; diagnostic stamps/generations/tags are explicitly synthetic.
Those digests identify original unredacted artifacts, not a rehash of the
sanitized text. Large dense arrays are represented by exact comparisons and
lossless output digests, rather than duplicated JSON dumps.

## Remaining work

Depth children still need exact tight crop/parity/default-CCL ordering and local
support/halo evidence. Parent global GRANA IDs cannot substitute for crop-local
children. Ordinary closed memberships, support-plane fitting, contours, PCA,
complete object/report parity and source/order/epoch/freshness/TF/world admission
remain incomplete. Consumers must retain their own lease through their fence.
No default-camera wiring, full objects, main release, physical cycles or station
qualification follows. Actual transfer-inclusive useful performance remains a
separate gate.

Reproduction and manual native build instructions are in the
[diagnostic README](../../tools/performance/README.md#resident-original-color-parent-diagnostic).
The maintained [integration plan](../superpowers/plans/2026-10-23-native-geometry-integration.md#4-specify-and-implement-resident-evidence-not-diagnostic-downloads)
records the completed boundary and remaining work.
