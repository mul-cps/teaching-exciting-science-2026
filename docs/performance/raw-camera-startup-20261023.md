# Raw-camera startup and local live smoke — 2026-10-23

CPU, strict Vulkan and auto each passed one 15-second steady smoke on the actual
local D435i / AMD Radeon 8060S. This is provisional local integration evidence,
not fwmini4 camera qualification, a matched speedup study, soak, full geometry or actuation.
[Machine archive](raw-camera-startup-20261023.json) retains source/runtime hashes,
proof aggregates and original report SHA256 values. Earlier
[raw-resident failures and offline correctness](raw-resident-20261023.md) remain separate.

| Sequential live mode | CPU | Strict Vulkan | Auto |
|---|---:|---:|---:|
| Backend / alignment | cpu / vendor_cpu | vulkan / raw_vulkan | vulkan / raw_vulkan |
| Steady seconds | 15.003 | 15.030 | 15.003 |
| Unique original cohorts | 449 | 451 | 451 |
| Container process CPU ms / output | 14.120 | 3.792 | 3.304 |
| Collector host-receipt age p95, ms | 15.261 | 4.665 | 4.649 |

Both GPU fallback reasons were empty. Each passed its three-live-original-cohort
cold gate: exact SDK-aligned Z16, finite XYZ error ≤1e-7 m, at least 24 native and
SDK-aligned nonzero samples per cohort, exact independent CPU compact output and
coherent metadata/zero faults. Constructor completion and raw-ready status prove
completion; dense maximum XYZ error was not separately logged.

Both source IDs strictly increased; each run retained one generation, distinct across
fresh starts. Geometry stayed unqualified and execution ineligible. No unknown/fatal,
fallback, replacement, slot drop or duplicate/regression was observed; cleanup and all
source/runtime postchecks passed. All owned processes and camera handles closed.

Each ready status starts at received=1/completed=0: the received event is the startup
seed. Ready-to-last periodic-status deltas were 450/450 in each mode, whereas collector
windows saw 449/451/451 outputs. These are distinct asynchronous observation windows,
not a claim of lossless DDS reconciliation or lifetime counters as steady offered frames.

## Preserved failure chronology and correction

The earlier V2 CPU run rejected a present depth ID 0. Capture-only SDK traces then
reproduced zero/regressed IDs before alignment/Core: two completed capture-only trials
and one CPU-load trial, out of four starts. Trial 2 failed diagnostic generation
bookkeeping; its failure is retained. Late 2–3 s windows had 30 clean cohorts each.
No timestamp-domain transition was observed; firmware-versus-SDK mechanism is unresolved.
All 75 CPU workloads preserved original metadata through alignment and Core.

The reviewed startup gate quarantines output as starting/ineligible. It requires at
least 2 s elapsed, 0.5 s stable advancing metadata and five observations, within a
10 s deadline. Zero/reset/repeat events reset cold admission; steady guards remain strict.
V3 stopped because the old collector rejected legitimate starting status, before any
ready window. The corrected collector accepts explicit safe starting only before ready,
starts measurement only on the required backend/alignment, and still rejects unknown,
fatal, unsafe, wrong-backend and post-ready starting states. V4 then passed all modes.

CPU verification recorded 14 native CTests, latest 3,838 canonical tests with 136 skips
and 48 subtests, plus the prior 347 native tests. Startup helper medians were
5.45448/3.99440 ns per observation with zero own heap allocation; collector classifier
cost was approximately 0.740–0.753 µs starting and 2.177–2.199 µs ready.
These isolated helper costs exclude SDK/ROS/driver/GPU work and are not GPU performance.

Live CPU accounting uses actual container process ticks (10 ms granularity), excluding
startup, driver IRQ/kworkers and other processes. Age is host receipt to collector receipt,
not exposure age. Scenes/exposure were uncontrolled and sequential; no causal speedup
percentage or general latency guarantee follows. No services, actuators or deployment changed.
Private serials/calibration, raw frames, event paths and per-frame samples are not archived.

The focused source review found no new blockers in the startup/collector delta.
Its remaining limits include observed ordering rather than continuous capture cadence,
no minimum-FPS assertion in collector PASS alone, and an external watchdog for hung GPU teardown.
The [archived offline driver](../../tools/performance/raw_resident_reference.cpp) is byte-identical
to the eight-case GPU qualification source; a fresh compile passed all eight CPU SDK cases.
[Reproduction instructions](../../tools/performance/README.md) distinguish offline correctness
from live-camera timing. The archive retains the review digest and driver identity.
