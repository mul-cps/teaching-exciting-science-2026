# Vulkan offload strategy for resource-constrained platforms

As of **2026-10-23**. This playbook separates measured results from proposed next steps.
The workshop experiments support target-specific decisions; they do not qualify a complete station.
Use [resident processing](resident-20261023.md), [camera profiles](d435i-profiles-20261023.md),
[exact alignment](alignment-20261023.md), and the companion
[raw resident bridge report](raw-resident-20261023.md) for evidence and boundaries.

## Choose the budget before choosing the backend

Treat latency, unique-output throughput, process CPU demand, and control QoS as separate contracts.
GPU offload can release CPU capacity while increasing frame latency or reducing delivered rate.
Specify the required unique rate, maximum age, CPU allowance, and failure behavior first.
Measure the actual target GPU, driver, SDK, and mixed workload; vendor identity is not a prediction.
For this station, attempt Vulkan first and expose the reason for any CPU fallback.
A slower GPU can satisfy the CPU budget, provided it also meets the required rate and age limits.
Lower process CPU time alone does not establish lower power use or improved control timing.

| Intel prealigned workload after exact-division correction | CPU → GPU process ms/completed | Delivery | Interpretation |
|---|---:|---|---|
| Recorded, offered at 30 Hz | 6.112 → 1.620 | Both 1800/1800, 30 Hz | 73.50% less process CPU/frame |
| Dense, offered at 30 Hz | 24.725 → 2.221 | CPU 30.00, GPU 23.07 fps | 91.02% less CPU/frame, unequal service |

Source: [corrected Intel Core paced trials](intel-core-shared-divide-20261023.md).
These figures cover a prealigned subset, excluding capture, alignment, ROS and complete geometry.
Preserve the [earlier paced trials](resident-20261023.md#paced-cpu-headroom-and-delivery)
as historical evidence rather than mixing them into the corrected baseline.
A matched whole-camera CPU/GPU comparison remains unmeasured. A later
[local AMD camera smoke](raw-camera-startup-20261023.md) observed about 30 unique outputs/s
in CPU, strict Vulkan and auto, one 15-second run each. Those sequential observations
remain provisional and do not establish a matched-input speedup or station qualification.

| Historical native workload | Intel CPU / GPU p95 ms | AMD CPU / GPU p95 ms |
|---|---:|---:|
| Recorded | 5.118 / 13.812 | 1.943 / 0.482 |
| Dense | 25.287 / 41.978 | 11.211 / 1.060 |

Source: [native paired latency](resident-20261023.md#native-paired-latency).
These are medians of trial p95s, exclude upload memcpy, and are not complete frame latency.

The [current Intel Core remeasurement](intel-core-shared-divide-20261023.md) after shared exact division
records 13.926/42.148 ms GPU p95 for recorded/dense input. Process CPU cost per completed output
is 73.50%/91.02% lower than native CPU, but dense GPU delivery is 23.07 fps (1,384/1,800).
This remains a prealigned Core subset; the historical comparison does not isolate helper cost
and supplies no live-camera performance result.

## Optimize the measured hot stage

Earlier attribution put connected components near 85% of the GPU interval despite only 444 active
pixels out of 307,200; fixed full-frame passes still dominated. Transfers were about 1.75%, so
zero-copy was not the first target. See the historical attribution in the
[raw resident companion](raw-resident-20261023.md), separate from current performance claims.
Atomic-min flatten fusion removed 32 parent-plane copies and 64 barriers: 39.32 MB copy payload,
or 78.64 MB combined logical read/write traffic per frame. Intel dense delivery improved
20.4 → 23.4 fps, still below 30 Hz ([resident evidence](resident-20261023.md)).
These byte counts describe logical operations, not measured DRAM traffic.
Avoid fixed full-frame work on sparse inputs when the algorithm permits a bounded alternative.
Fuse only after proving invariants and equivalent interleavings: associative, commutative,
idempotent unsigned min justified this fusion. Concurrent writes alone do not establish safety.

## Keep ownership bounded and residency purposeful

The raw bridge uploads RGB 921,600 B and Z16 614,400 B once per plane at 640×480;
it copies 614,400 B Z16 on device, then reads back a 4 B producer result and 163,856 B compact output.
Use three bounded slots and a typed source lease retained through the **consumer fence**.
Keep one active submission and one replaceable, unsubmitted latest pending input; replaced pending
inputs must never incur GPU submission. Publish each completed source cohort once.
Source: [raw resident companion](raw-resident-20261023.md); queue semantics also appear in
[resident ownership evidence](resident-20261023.md#latest-frame-software-boundary).
Under sustained overload, keep capture rate and unique processed-output rate separate.
Replace pending work before submission, process the newest available cohort, and report skipped
inputs and frame age. Publishing an old result again does not count as another processed frame.
Accept this policy only when its measured unique rate and age still meet the application budget.
CPU capture, SDK RGB conversion, ROS, and control still consume CPU; new GPU compute uses
Vulkan/Slang/SPIR-V, and complete geometry remains unfinished.
Zero own allocations/waits in a fake-driver hot path excludes SDK, RMW, real-driver behavior,
and memcpy costs. Avoid whole-device waits in steady operation; qualify the real configuration.

## Preserve arithmetic and failure contracts

`NoContraction` does not require correctly rounded division. Reciprocal multiplication and double
rounding caused exact Z16 rectangle and Core XYZ discrepancies; a correctly rounded reciprocal
alone did not repair the demonstrated Core error. Preserve thresholds and far-depth output semantics.
The canonical float32 helper verifies an integer quotient from a native seed, corrects at most
four times, then falls back to 23 restoring steps; it needs neither FP64 nor Int64 on Intel.
The Core convert SPIR-V grew about 3.05×; its subsequent GPU remeasurement is linked above.
CPU helper nanoseconds and correctness-loop duration cannot predict frame latency or FPS.
Sources: [exact alignment](alignment-20261023.md) and [raw resident companion](raw-resident-20261023.md).
Both GPUs passed eight SDK full raw-bridge fixtures with exact arrays and 65,536 division bit cases.
The first live CPU prototype rejected depth ID 4 → 0 before a valid steady window.
Native SDK traces reproduced startup resets before alignment/processing. A bounded observed
stability phase then enabled the three-mode [live smoke](raw-camera-startup-20261023.md).
Keep that failure and its resolution distinct from the later timing observations.
Do not weaken steady provenance, regression, or unknown/restart-required failure gates.

Distinguish physical support from features enabled on the logical device. Optional video
negotiation must not erase independent compute capabilities: the
[compute-feature correction](compute-feature-baseline-20261023.md) preserves supported FP64
without granting unsupported FP64 to Intel. Probe driver-specific test controls before relying
on them; the generic Mesa version override was not implemented by this RADV version.
Inventory individual physical features before designing an emulation path: both tested GPUs
support Int64 even though Intel lacks FP64. The historical committed compute-only baseline
at `c53d8d5` left Int64 disabled. The later [explicit software binary64 prototype](software-binary64-20261023.md#bounded-gpu-conformance-and-preparation) enables supported Int64 and passes bounded
primitive/preparation accuracy on both tested GPUs, including Intel without Float64.
Initial [isolated preparation timing](geometry-preparation-timing-20261023.md) now records host latency and process CPU demand. Transfer-inclusive fresh-input/full-pipeline cost, unique delivery and age remain unmeasured, so it is not the default path.
Capability, enablement, accuracy and useful performance are separate evidence.

## Interpret isolated preparation budgets

The [two-case preparation experiment](geometry-preparation-timing-20261023.md#timing-and-process-cpu-demand)
used one already-resident synthetic 640×480 source, serial reuse, eight warmups and three 32-sample trials.
For full XYZ rotation, median trial p95 was 0.318740 ms software AMD, 0.218732 ms native AMD
and 13.101365 ms software Intel. Median whole-trial process CPU divided by 32 was 0.011474,
0.010819 and 0.478461 ms respectively. These separate latency/CPU budgets do not measure
capture, new input transfers, unique delivery, complete geometry or control QoS.
Do not add Core/alignment/preparation p95s from separate experiments to derive FPS.

Calibration structure belongs in the test matrix: axis-aligned sign/unit coefficients exercise
different arithmetic paths from a proper XYZ rotation with all nine coefficients nonzero/nonunit.
Retain both cases and their source identities; one calibration is not a general workload model.
Requested 50 µs polling produced about 99–104 µs observed sleep intervals, so sub-millisecond
host-observed tails include polling/scheduling granularity. The experiment did not isolate
DVFS causality or supply a CPU baseline; native/software AMD comparisons remain descriptive.

## Account for camera and system work

AMD camera testing used 18 actual trials, three repeats per condition. At roughly 29.98 unique fps,
SDK capture plus alignment cost 0.36 logical CPU / 11.3 ms alignment p95 at 640×480,
0.80 / 26.4 ms at 1280×720, and 0.96 / 27.4 ms for RGB1080p with depth720p.
Observed counter gaps were zero. These are AMD CPU measurements, not fwmini4 results;
the native GPU path supports only 640×480 ([profile evidence](d435i-profiles-20261023.md)).
More pixels can exhaust CPU headroom before downstream processing starts.
Application queue capacity 1 and SDK sensor queue capacity 16 are different layers.
Do not infer capture age by subtracting clocks from different domains.

## Reproduce decisions, then qualify the station

Freeze inputs, compiler, SPIR-V, transitive helper hashes, loaded libraries/driver, frame identities,
and protocol before each comparison; helper-only source changes must invalidate cached evidence.
Exclude startup, warmup, and drain from steady timing; retain offered/completed/replaced counts.
Measure CLOCK_PROCESS CPU and control jitter separately; report median trial p95s, not pooled tails.
Intel paced samples observed 600 versus 750 MHz, without isolating DVFS causality; an unpaced
33.9% gain did not translate to paced tail improvement ([resident evidence](resident-20261023.md)).
Whole-host versus process CPU accounting remains unresolved: claim process budget only.
Zero misses in a synthetic 125 Hz control loop does not prove physical control safety.
Capability extensions do not qualify codecs: Intel H.264 corruption retains CPU fallback;
VAAPI interoperability remains unqualified ([raw resident companion](raw-resident-20261023.md)).

For future projects, execute this proposed sequence:

1. Define CPU/control budgets and unique-rate/age contracts; freeze an equivalent CPU baseline.
2. Optimize the measured hot stage; batch or fuse proven work and minimize required readback.
3. Bound ownership and latest-pending behavior; enforce correctness, precision, and failure gates.
4. Measure paced mixed workloads, CPU demand, and safety/control QoS on each target configuration.
5. Keep explicit target-specific fallbacks; finish full-station soak, rehearsal, and physical cycles.

## Reusable experiment record

These dated platform results are evidence to guide future experiments, not portable speedup promises.
Process CPU reduction and the unique-rate/frame-age contract jointly determine useful offload.
A slower GPU path may be acceptable when it frees required CPU headroom and still satisfies the
application's rate, age, latency and correctness constraints. A control-loop benefit requires measured
control evidence. In the [dense comparison](intel-core-shared-divide-20261023.md), 91.02% CPU savings per completed
frame came with unequal service: 23.07 versus 30 fps. Preserve both facts when assessing that saving.
Use the [measurement boundaries](geometry-preparation-timing-20261023.md) and the
[ownership/rate contracts](resident-20261023.md#latest-frame-software-boundary) to fill this record:

```text
Project / date / acceptance owner:
Exact source + transitive helper hashes / compiler identity + flags:
SPIR-V hashes / loaded driver + library hashes / device identity + enabled features:
Input resolution / density / calibration identity + structure:
Matched CPU/GPU operation scope / precision / unchanged correctness threshold + result:
Measurement window / warmup / trials / samples / clocks / polling policy:
Offered / completed / replaced counts and rates / unique-output rate:
Frame-age p95 / clock domain / application rate + age limits:
Process CPU seconds per completed output / seconds per wall-second (CPU and GPU paths):
Latency trial p95s + aggregation rule (separate from process CPU):
Transfers (bytes + direction) / allocations / synchronization + readback per output:
Concurrent control-loop p95 / p99 / deadline misses / required limits:
Fallback reason / acceptance decision / unmet constraints / evidence links:
```
