# fwmini4 higher-resolution checkpoint — 27 October 2026

**720p and 1080p capability is implemented; 30 Hz processing is not achieved.**
The default is now 720p RGB/depth. The 1080p profile uses native 1920×1080 RGB
and 1280×720 depth, aligned onto the RGB grid without resizing RGB. Native
profile/stride checks, bounded storage, three-slot ownership and the original
exact-Z16, XYZ and compact-result cold gates remain intact.

All eight short live collectors passed profile/output/allocation-safety checks.
Those passes do **not** qualify station performance. The optimized system SDK CPU
baseline produced 29.89 outputs/s at 720p and 17.49 at 1080p, versus strict
Vulkan's 19.09 and 8.71. **GPU output was slower at both profiles.** At 1080p,
GPU also used more CPU time per output (80.61 ms versus 62.17 ms); its lower CPU
occupancy per second partly reflects fewer completed outputs. This does not
establish uniformly efficient offload.

| Profile / mode / SDK | Unique outputs/s | Process CPU, % of total four-core capacity | CPU ms/output | Host-receipt age p95 ms | RSS / HWM MiB |
|---|---:|---:|---:|---:|---:|
| 720p CPU / scalar 2.58.3 | 15.790 | 31.713 | 80.338 | 126.275 | 281.2 / 281.2 |
| 720p strict Vulkan / scalar 2.58.3 | 19.091 | 9.745 | 20.418 | 82.543 | 194.9 / 387.2 |
| 720p auto → Vulkan / scalar 2.58.3 | 19.086 | 9.609 | 20.139 | 82.695 | 194.8 / 387.0 |
| 720p CPU / optimized system 2.58.4 | 29.893 | 24.916 | 33.341 | 66.015 | 282.3 / 282.3 |
| 1080p CPU / scalar 2.58.3 | 11.849 | 38.543 | 130.112 | 168.121 | 549.5 / 549.5 |
| 1080p strict Vulkan / scalar 2.58.3 | 8.714 | 17.561 | 80.611 | 146.460 | 324.5 / 772.3 |
| 1080p auto → Vulkan / scalar 2.58.3 | 8.794 | 17.639 | 80.227 | 142.894 | 322.7 / 770.4 |
| 1080p CPU / optimized system 2.58.4 | 17.488 | 27.180 | 62.167 | 117.703 | 536.0 / 536.0 |

All measurements were **without RViz**. The requested actual GPU RViz 30 fps
at VGA, 720p and 1080p remains unvalidated. Higher-resolution capability meets
the requested image extents; processing throughput still needs optimization.
720p Vulkan also remains below the earlier 20 Hz target.

CPU is steady container-PID `utime+stime`, divided by elapsed time and four
logical cores; it excludes startup, driver IRQ/kworker and other processes.
RSS is an after-steady snapshot; HWM includes startup and cold diagnostics.
The separate host-busy snapshot includes startup/other processes over a
different interval. Neither is interchangeable with the steady process metric.
These were sequential uncontrolled live inputs without matched-pixel replay or
CPU reservation. The six scalar runs used isolated SDK 2.58.3; the two optimized
CPU baselines used unmodified system SDK 2.58.4. SDK and input differences limit
causal comparisons; the scalar CPU baseline is not the fastest legacy CPU path.

Age is **host receipt ROS time to collector receipt**, not processing latency or
exposure age. The 1080p strict run's 146.46 ms exceeds 100 ms on that receipt-age
measure; this cannot establish physical latency. Throughput comes from unique
outputs divided by the measured interval, independently of age percentiles.

The latest-frame queue skips older inputs under load. Between first and last
observed source IDs, strict Vulkan had 161 missing IDs at 720p and 316 at 1080p
for both color and depth. These are bounded ID-window gaps, not whole-window
producer skip totals. Periodically sampled received/completed/replaced counters
have different timing and must not be combined to infer exact steady throughput.
Vulkan allocation count stayed at **35** throughout all four GPU windows.

A separate recorded Core probe reused one frozen, full-resolution **prealigned**
input per profile. CPU and GPU each measured 20 frames after five warmups:

| Profile | CPU Core p95 ms | GPU compact Core p95 ms | GPU process CPU p95 ms | GPU CC mean ms |
|---|---:|---:|---:|---:|
| 720p | 14.437 | 41.361 | 1.498 | 31.722 |
| 1080p | 32.303 | 96.270 | 3.357 | 74.922 |

GPU connected components consumed about 77–78% of summed native stage means;
upload was 0.85/1.79 ms and compact readback about 0.036 ms. GPU Core was slower
than CPU Core on these frozen inputs while using less process CPU. Two-frame
dense comparisons (one warmup) passed all arrays/compact morphology at both
profiles; their percentile estimates are not robust. This probe excludes capture,
alignment and full geometry. It does not derive FPS from timing percentiles.
Images and calibration stay private on fwmini4; only summary/script are retained.

Synthetic native alignment passed eight frozen cases per profile (VGA, 720p,
1080p) on both Intel and AMD: **48 exact Z16 comparisons, maximum XYZ error 0**.
These are software-device fixtures, not physical-camera equivalence tests.
Independent Core, Alignment and camera resource/performance reviews are retained.

The isolated native package built in 39.3 s from a content bundle before commit;
all 103 bundled source files were subsequently verified byte-identical to pushed
checkpoint `5344d665…`. Final canonical CPU verification passed **4333 tests,
133 skipped and 48 subtests in 206.10 s**, with all 464 before/after source pins
unchanged. Skipped native ROS/physical checks remain unqualified. Earlier direct
pytest collection failure (19 environment errors) and the canonical run's two
stale identity-guard failures are preserved; the latter were corrected in tests
and the final canonical run passed without numerical/product changes.

At 13:32:28 UTC the old owner unit remained active with PID 181501 and its
22 October start time; no process held the camera video nodes. No production
service or canonical deployment was promoted. Markers still use CPU OpenCV;
full geometry, control, soak and physical station acceptance remain separate.
`execution_eligible=false` and whole-station qualification remains false.

[Retained raw evidence and selfcheck](evidence/fwmini4-high-resolution-20261027/README.md)
and the [JSON summary](fwmini4-high-resolution-20261027.json) bind the measurements,
source identities and failed-attempt history. This is a capability/numerical/live
observation checkpoint, not an adoption or performance qualification.
