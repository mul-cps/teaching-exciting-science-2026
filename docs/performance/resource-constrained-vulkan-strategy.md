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

| Historical Intel prealigned workload | CPU → GPU process ms/completed | Delivery | Interpretation |
|---|---:|---|---|
| Recorded, offered at 30 Hz | 6.141550 → 1.664821 | Both 1800/1800 | 72.89% less process CPU/frame |
| Dense, offered at 30 Hz | 24.448020 → 2.156685 | CPU 30.0000, GPU 23.3667 fps | 91.18% less CPU/frame, unequal service |
| Dense process demand | 0.733440 → 0.050395 logical CPUs | Same paced experiment | CPU budget saved, rate contract failed |

Source: [resident paced trials](resident-20261023.md#paced-cpu-headroom-and-delivery).
These figures precede the latest shared exact-division Core correction.
They cover a prealigned subset, excluding alignment and complete geometry; they are not current Core timings.
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
support Int64 even though Intel lacks FP64. Its compute-only logical device currently leaves
Int64 disabled. Capability, enablement and qualified arithmetic are three separate facts.

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
