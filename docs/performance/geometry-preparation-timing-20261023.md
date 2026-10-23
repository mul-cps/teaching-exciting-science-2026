# Isolated geometry preparation timing — 2026-10-23

**Three independent correctness gates and six timing processes passed.** This is an initial budget for repeated, already-resident synthetic input. It does not establish unique-frame delivery, capture/frame age, transfer-inclusive performance, full geometry or station acceptance. The [sanitized archive](geometry-preparation-timing-20261023.json) retains all 576 timing samples, trial aggregates, actual device/compiler/driver/SPV identities and digests of the unchanged original private reports.

## Workload and correctness

Each case is 640×480 with 307,200 independently expected valid pixels and 4,800 support points. `all_valid` is byte-identical to the earlier qualified fixture. `full_xyz_rotation` uses a proper XYZ rotation with all nine rotation coefficients nonzero and nonunit. The [factory](../../tools/performance/prepare_geometry_benchmark.py) derives expected arrays and counts from the retained CPU oracle; no GPU output supplies a golden.

Before timing, the existing qualifier ran the new full-XYZ case in software AMD, native Float64 AMD and software Intel modes. All three passed the unchanged absolute **1e-12 m**, rtol zero coordinate comparison, with maximum **2.220446049250313e-16 m**. Valid/vivid/ROI flags, selected-pixel membership/order, counts and arena size matched exactly; generation, metadata, source-fault, pending-source fence, copied-lease saturation and dropped-source contracts also passed. These three additional cases are separate from the [earlier sixteen-case accuracy record](software-binary64-20261023.md#bounded-gpu-conformance-and-preparation); its frozen maximum remains 2.7755575615628914e-17 m.

## Timing and process CPU demand

Every timing process uploaded and completed one source once, retained that source, performed eight untimed warmups, then three trials of 32 samples. It serially reused the source-index arena within fixed three-arena storage, with at most one operation in flight. Initialization, upload, warmup and drain were excluded. Only the ordinary 16-byte completion counts were read; every measured iteration returned the independent expected counts.

Latency is host-observed `CLOCK_MONOTONIC` before submit through after finish. The reported tail is nearest-rank p95 within each 32-sample trial, rank 31, followed by the median of those three p95s; tails are never pooled. CPU demand uses `CLOCK_PROCESS_CPUTIME_ID`: the CPU column is median whole-trial process CPU divided by 32, including loop/checksum/budget/instrumentation and all process threads.

| Device / precision | Synthetic case | Median trial p95 (ms) | Median trial CPU / iteration (ms) | Mean observed sleep (µs) |
|---|---|---:|---:|---:|
| AMD / software binary64 | `all_valid` | 0.288794 | 0.011118 | 99.822 |
| AMD / software binary64 | `full_xyz_rotation` | 0.318740 | 0.011474 | 99.478 |
| AMD / native Float64 | `all_valid` | 0.214785 | 0.012160 | 99.936 |
| AMD / native Float64 | `full_xyz_rotation` | 0.218732 | 0.010819 | 103.588 |
| Intel / software binary64 | `all_valid` | 10.268876 | 0.378912 | 99.470 |
| Intel / software binary64 | `full_xyz_rotation` | 13.101365 | 0.478461 | 99.306 |

All 18 trial records reported zero ordinary-C++ operator-new calls and zero station Buffer allocation growth, mean 0 per iteration. This counter does not cover driver malloc. All six processes stayed below their 2,000 ms measured process-CPU guard. Per-trial CPU totals, raw samples and allocation/sleep records remain in the archive.

Requested polling sleep was 50 µs; observed syscall wall intervals averaged about 99–104 µs. These host-observed latencies include polling and scheduling granularity. Frequency endpoints were read only as diagnostics; DVFS causality was not isolated. Native/software AMD rows are descriptive, with no CPU baseline or speedup claim. CPU demand and elapsed latency are separate budgets. Do not add these percentiles to Core/alignment percentiles from other experiments to infer end-to-end FPS.

## Actual hardware and resource boundary

| Device scope | Vendor/device | Physical / enabled Int64 | Physical / enabled Float64 | Driver integer | API integer |
|---|---|---|---|---:|---:|
| Radeon 8060S / RADV STRIX_HALO |1002:1586|true / true|true / true|109060099|4211042|
| Intel Graphics ADL-N |8086:46d1|true / true|false / false|104865800|4211006|

Native Intel was not run. Actual per-executable loader and critical driver/DRM/core-library mappings were checked against registered hashes; AMD used loader 1.4.357 and Intel 1.3.275. Both sides prehashed their available canonical DRM libraries. Raw maps and post-critical identities were archived before mapping validation. Ordinary nonexecuting, nonlibrary, nonpinned mapped data was recorded separately from source identity.

All nine children exited 0 under independently owned 90-second process-group deadlines with TERM/KILL/drain protection. No timeout or kill was needed; groups were empty and subsequent checks found zero staged GPU children. No camera, ROS, actuator, service, deployment or default-backend selection was involved.

## Measured source and current tree

The measured snapshot was HEAD `1908b3423fa1e57a5344d0adb4434fd1c511fc16` plus benchmark source SHA256 `ce12e7c4eb25c8cc93f0c912539222ff6649b20a09dc143810808e3a6bad8dfb`. All 21 relevant source pins and critical artifact identities matched before and after the successful protocol. Immutable source/header copies and the original post-run seal were preserved.

The measured `pipeline.hpp` was identical in committed 99acf7c and 1908b34: git blob `95a33ed31a0eb8c79ddc447d4c8f8c5af58f379f`, SHA256 `48cc86a081499b61a9b7a816c4c737034abf07908a15fe3d2e996b681b2dcb30`. `FramePoolToken` was intentionally added after timing and its completed source seal. That later additive API is outside the measured version, not historical runtime identity drift.

The [current benchmark CPP](../../ros2/src/so101_station_vulkan/test/benchmark_geometry.cpp) still has the recorded CPP hash; its current transitive header/runtime is a separate source version. The public JSON distinguishes historical measured pins from current source observed at archival. It does not assert that the old header hash matches the current tree or that the new API was timed.

## Reproduction and remaining gates

See [benchmark usage](../../tools/performance/README.md#geometry-preparation-timing) for independent fixture generation, the native CMake target, offline validation and the external-watchdog/explicit-precision contract. CMake stamps meaningful source/compiler/flags provenance; runtime output records the actual fixture/SPV hashes. A new source/library/driver version needs its own frozen identities and review.

Transfer-inclusive fresh-input performance, unique-output rate and age, mixed-workload control QoS, camera integration, parent/plane/contour/PCA/object processing and physical-station acceptance remain open. This initial preparation budget does not select the default camera/backend or qualify a full pipeline. Original private archival bytes remain unchanged; only their digests and sanitized numerical/identity evidence are published.
