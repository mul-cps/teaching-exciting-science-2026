# fwmini4 RViz coexistence checkpoint — 27 October 2026

**Native Intel OpenGL RViz maintained about 31.25 swaps/s at all three camera
profiles; higher-resolution perception did not sustain 30 unique outputs/s.**
These are six short sequential windows, not station or physical qualification.
RViz used direct `Mesa Intel(R) Graphics (ADL-N)` on the same Intel GPU as Vulkan
compute. The main window was 1600×900 (confirmed in the separate UI check). GLX
drawables were 574×810 (main viewport), 640×314 (RGB dock), and 1×1 (dummy). Rates are measured separately;
the dummy is excluded from the primary viewport, and rates are **never summed**.

| Profile/backend | Unique perception outputs/s | Primary RViz swaps/s | Perception CPU, % of total four-core capacity | CPU ms/output | Receipt-age p95 ms |
|---|---:|---:|---:|---:|---:|
| vga strict GPU | 30.029 | 31.252 | 4.844 | 6.452 | 27.218 |
| vga optimized CPU | 29.947 | 31.254 | 10.282 | 13.733 | 13.215 |
| 720p strict GPU | 13.649 | 31.254 | 10.553 | 30.927 | 103.436 |
| 720p optimized CPU | 25.399 | 31.247 | 25.199 | 39.685 | 75.118 |
| 1080p strict GPU | 6.199 | 31.248 | 18.713 | 120.753 | 193.055 |
| 1080p optimized CPU | 13.892 | 31.277 | 27.119 | 78.086 | 141.208 |

The CPU baseline uses optimized system SDK 2.58.4; strict GPU uses isolated scalar
SDK 2.58.3. Inputs were sequential physical-camera captures, without equal-pixel
replay, CPU reservation or controlled power. GPU uses less process CPU per second,
but at 1080p also completes fewer outputs and uses **120.75 CPUms/output versus
CPU's 78.09**. Lower CPU occupancy does not establish uniformly efficient offload.
Receipt age is host receipt ROS time→collector receipt, not exposure age or
processing latency. RViz swap return is not physical scanout, nor evidence of a
fresh camera frame on every redraw. Full-resolution inputs are preserved without
RGB resizing; the displayed RGB dock itself is 640×314.

The Image display subscribes to **actual resident camera RGB** through the custom
FrameLease→sensor_msgs/Image DDS adapter. Robot poses, 20k-point world cloud and
four brick markers are synthetic visual load. They cannot qualify measured geometry.
A separate **VGA-only post-matrix UI check** used the private Weston compositor
screenshooter. Root inspected the 1920×1080 capture: 1600×900 RViz, actual RGB,
robot, colored world surface and brick markers visible, Global Status OK, 31fps
status bar. Its private screenshot hash and reported inspection are retained,
without publishing the image. Earlier Qt/rootless-X11 capture was black/invalid
and remains a failed method. The UI readback/debug compositor run was outside
all timed windows; it neither proves all three UI profiles independently nor
qualifies geometry or control.

| Profile/backend | External target CPU % | Approx. perception+external CPU % | Host busy % | RViz RSS MiB* | Observer begin/end offset ms |
|---|---:|---:|---:|---:|---:|
| vga strict GPU | 7.576 | 12.420 | 17.666 | 208.5 | -24.0 / -28.6 |
| vga optimized CPU | 7.693 | 17.975 | 20.704 | 208.5 | -21.8 / -34.6 |
| 720p strict GPU | 8.574 | 19.127 | 23.905 | 210.0 | -22.5 / -26.3 |
| 720p optimized CPU | 11.041 | 36.240 | 41.237 | 209.9 | +14.1 / +26.2 |
| 1080p strict GPU | 8.825 | 27.539 | 29.436 | 223.0 | -44.4 / -32.9 |
| 1080p optimized CPU | 12.039 | 39.158 | 44.583 | 222.7 | -5.6 / -36.6 |

Percentages use all four logical cores. External targets are RViz, scene publisher,
robot_state_publisher, Weston, Xwayland, desktop and keyboard; PID aliases are
not double-counted. Their nearest-sample windows differ from the collector windows:
all six explicit bounds/offsets, CPUms, starttime identity, per-process RSS and
DRM engine deltas are retained in JSON. **Combined CPU is an approximate addition
across different windows**, not exact simultaneous accounting. Host busy includes
all tasks, idle+iowait excluded; sampler CPU is absent from target sums but included
in host busy. Sampling read-cost means were 4.93–5.97ms, maxima 6.28–11.62ms, excluding
JSON writes; these are wall costs, not measured sampler CPU time. *RSS pages are
authoritative; derived MiB assumes 4096-byte pages. Camera RSS/HWM snapshots are in
JSON; HWM includes startup. RViz RSS was 208–223MiB across the six windows.

DRM counters cover per-client render engines for RViz, Weston and Xwayland;
other sampled engines were zero. Camera/Vulkan compute engine accounting was not
sampled. These counters are not complete GPU occupancy, wall-time, power or control
headroom. No such qualification is inferred.

Earlier observations without RViz, shown only as context:

| Profile | GPU perception Hz: prior → with RViz | Optimized CPU Hz: prior → with RViz |
|---|---:|---:|
| vga | 30.046 → 30.029 | 30.016 → 29.947 |
| 720p | 19.091 → 13.649 | 29.893 → 25.399 |
| 1080p | 8.714 → 6.199 | 17.488 → 13.892 |

This is not a controlled before/after causal measurement: live inputs and timing
differ, VGA also used an earlier source checkpoint. The added workload combines
RViz graphics, ROS Image compatibility serialization/upload, synthetic scene, RSP
and compositing; its effect cannot be attributed solely to OpenGL rendering.

Every measured source-ID window, gap/duplicate/regression count and allocation
sample is retained. GPU allocation count stayed 35, CPU 0. VGA had no source gaps;
GPU 720p/1080p had 242/352 missing IDs between first/last observed outputs. These
are bounded source-ID gaps, not whole producer-window skip totals. Periodic producer
received/completed/replaced counters are not used to derive unique throughput.

Three compositor setup failures preceded camera measurement and remain in the raw
archive: kiosk crash, missing theme, wrong theme parent path. The successful setup
uses full `WESTON_DATA_DIR=.../runtime/usr/share/weston`. A post-run native-library
maps attempt found no surviving processes; its empty result is **not SDK identity
proof**. Runtime launch paths and prior native-source/SDK provenance are retained.

Native source checkpoint is `5344d665…`, bound by the prior 103-file native manifest.
The model cache has upstream revision `58318c9…` plus two local xacro edits; source
hashes and patch are retained, meshes omitted. The exact generated bundle hashes
are the authority. The GLX hook passed independent CPU resource review before runtime: fixed
16 drawable slots, no hot heap/readback/wait calls, 500k-call host benchmarks,
UBSan and five rejected mutants. Independent review also measured sampler and
scene costs; ROS serialization allocates by design. The offline analyzer passed
17 host tests; its reviewed metadata/input guards leave all six run subtrees
unchanged. Artifact checks verify the six windows; this data-only publication
does not rerun the full CPU suite. It preserves the source 5344 baseline before future
profile/frame-skip changes. Markers remain CPU OpenCV; full geometry, control, soak,
physical acceptance and production deployment remain unqualified.
`qualified=false`, `execution_eligible=false`.

[JSON and per-run accounting](fwmini4-rviz-coexistence-20261027.json) ·
[raw evidence, scripts, reconstruction and selfcheck](evidence/fwmini4-rviz-coexistence-20261027/README.md)
