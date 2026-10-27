# fwmini4 live camera observation — 27 October 2026

The required quality target is **720p/1080p**. The 640×480 runs below are a
baseline only; higher-resolution GPU processing and qualification remain pending.

The native 640×480 camera/resident-core path sustained about **30 unique outputs/s**
in three sequential 15-second steady windows on `cps-wkstn-fwmini4.local`.
Strict Vulkan used **18.654% of one CPU core**, versus **70.425%** for CPU mode
with the same scalar SDK. That is **73.51% less steady container CPU time per
wall second**; CPU time per output fell **73.57%**, from 23.489 to 6.208 ms.
These describe the component-container process, not whole-machine CPU use.

The scalar-SDK CPU run is **not the fastest observed CPU baseline**. Unmodified,
optimized system SDK 2.58.4 CPU mode sustained 30.016 outputs/s at 36.872% of one
core and 12.284 ms CPU/output. Strict Vulkan with scalar SDK 2.58.3 used 49.41%
less CPU per wall second than that run, but the SDKs and sequential live inputs
differ. This is an observed combination comparison, not a controlled speedup.

| SDK / requested mode | Observed path | Unique outputs/s | One-core CPU % | CPU ms/output | Receipt age p95 ms |
|---|---|---:|---:|---:|---:|
| Scalar 2.58.3 / CPU | vendor CPU alignment | 29.982 | 70.425 | 23.489 | 20.248 |
| Scalar 2.58.3 / Vulkan | raw Vulkan | 30.046 | 18.654 | 6.208 | 22.599 |
| Scalar 2.58.3 / auto | raw Vulkan | 30.050 | 18.123 | 6.031 | 22.262 |
| System 2.58.4 / CPU | vendor CPU alignment | 30.016 | 36.872 | 12.284 | 11.335 |

All three scalar-SDK collectors passed. Color and depth source IDs are contiguous
through each measured window, with no missing IDs or regressions. Final status
reported no drops, replacements or duplicates. Received/completed status counts
can differ by one normal in-flight frame; collector windows count unique outputs.

CPU figures use actual container PID `utime+stime`, excluding startup, driver
IRQ/kworker and other processes, with process-tick quantization. On the four-core
host, scalar CPU/strict Vulkan/auto correspond to 17.606% / 4.663% / 4.531% of
total CPU capacity. The retained host-busy snapshot includes startup and other
processes over a different interval; it must not be compared directly with these
steady process figures. Receipt age means **host receipt ROS time to collector
receipt**, not exposure age or processing latency. There was no CPU reservation
or frozen equal-input replay.

System SDK 2.58.4 strict Vulkan refused the alignment convention. Its auto run
explicitly fell back to CPU with that reason; the collector correctly failed its
required-Vulkan condition. Both attempts are retained, alongside the earlier
numeric-serial launch failure. Commit `2e578b8…` fixes serial typing; compiled C++
identity remains `0542aba…`. Only the installed launch file was patched.

The passing profile privately copied one SDK 2.58.3 library, SHA-256
`067df4f44d66500dc30186dd2e1f6f3e7b8654fd758c1044714157ffcc3d252d`, while using
native ROS Jazzy/FastDDS, system libstdc++, and domain 179. System SDK files were
unchanged. Admitted `raw_vulkan` status requires the original three cold cohorts:
exact loaded-SDK aligned Z16, XYZ absolute error ≤1e-7, and independent core
compact comparison. This evidence is the unchanged code invariant plus admitted
status; no separate cold pixel dump was collected and no gate was weakened.

Separate SDK acquisition-only checks obtained 300 unique pairs at about 29.98 Hz:
640×480 used 9.20% of one core; RGB 1920×1080 plus depth 1280×720 used 51.73%.
Those checks exclude alignment, perception and ROS. **High-resolution GPU
processing was not qualified**; the live GPU observation is 640×480 only.

The retained final-state check at 12:51:17 UTC shows the original owner unit
active with PID 181501 and its 22 October start time, no test containers, USB
5000 Mb/s, and no holders on all six camera video nodes. The camera was released.

[Raw successes, failures, scripts, SDK results and hashes](evidence/fwmini4-camera-live-20261027/README.md)
and the [machine-readable summary](fwmini4-camera-live-20261027.json) preserve the
bounded result. Markers still use CPU OpenCV. Full geometry, control, soak and
physical station acceptance are outside this observation;
`execution_eligible=false`, geometry admission remains unqualified, and full
station qualification remains false.
