# Resident RGB-D compute boundary

`so101_station_vulkan` is an experimental native C++ ROS package. It is **not a qualified replacement for the station perception pipeline** and is not wired into the production launch. Its ordinary output topics live under `/station/resident`; measurements never admit robot execution.

The Core GPU API accepts **already aligned RGB8 and Z16**, at most 1920×1080 pixels, subject to device storage and dispatch limits checked before pool allocation. The camera adapter connects shared-device raw Alignment to Core through a typed device-depth lease, without a CPU aligned-depth roundtrip. Native depth and RGB may have different extents; alignment uses their original intrinsics and the selected RGB output grid. One shared Vulkan context owns the instance/device/compute queue, with hardware-only device selection. Three slots allocate upload, device input, resident scratch, compact reduction, descriptors, commands and readback at construction. A copied `FrameLease` retains its slot through late consumers. Recycling additionally requires successful per-frame fence completion. Latest scheduling keeps one active submission and one replaceable unsubmitted frame. The core performs no frame-time Vulkan allocation or device/queue-wide waits. Shutdown drains per-slot fences explicitly. The optional diagnostic readback is a separate qualification mode, not the compact production path.

Offline Slang compilation emits SPIR-V for eight entry points. The source/compiler/flags provenance file is installed alongside the binaries. Deployment does not need SlangPy or a Slang compiler. Connectivity uses tiled 16×16 shared-memory ping-pong with a local 256-step bound, then 32 phases of immutable A→B compression; connect atomically flattens A from frozen B before boundary hooks, eliminating canonical B→A copyback. A final edge check reports a convergence fault; parent traversal does not read atomically modified parents.

An optional `RESIDENT_SLANG_CPU=ON` build adds explicit `cpu-slang` execution to
the prealigned Core API and benchmark. It uses native Slang LLVM conversion and
candidate stages with existing C++ completion, without creating a Vulkan device.
Automatic selection retains hardware Vulkan first and the existing C++ CPU
fallback. Camera admission does not yet accept `cpu-slang`. See the
[build instructions and measured comparison](performance/native-slang-cpu-20261027.md).

Separate child preparation now keeps one-based parent membership immutable in
existing workspace through gather and preserves uniform reducer barrier flow.
The [Intel parent-map experiment](performance/intel-child-parent-cache-20261027.md)
records the original failure and the retained 58-case child-preparation matrix passing
on both Intel and AMD. Full geometry and performance qualification remain open.

| Stage | Implemented boundary | Qualification limitation |
|---|---|---|
| Native RGB/Z16 upload | Padded native rows copied once; GPU byte extraction/conversion | USB import/DMA-BUF unavailable; upload cost must be measured |
| Conversion | Native Z16 scale to Float32; invalid zero preserved | Already aligned input only |
| Deprojection | Pinhole original integer pixel centers | Nonzero distortion unsupported; complete retained geometry unqualified |
| Color | Retained OpenCV integer HSV thresholds/table rounding | Compare every pixel against independent OpenCV |
| Depth splitting | Valid .10–1.2 m, four-neighbor 12 mm cuts on both sides before connectivity | Parent/support/rejection geometry still belongs to retained path |
| Connectivity | Eight-connected per-color minimum-index roots; bounded GPU union/compress; explicit convergence fault | Fixed rounds, fault instead of silent incomplete output |
| Reduction | Compact >=24-pixel component count, root, color, pixel bounds, min/max depth | Capacity 4096; overflow fails, never truncates silently |
| Per-piece morphology | Exact tight-bbox 3×3 closing implemented with separate overlapping root memberships, original OpenCV borders, max nine source roots per pixel | Exact per-pixel memberships/counts match native CPU and retained OpenCV on both AMD/Intel bounded offline matrix; full geometry remains open |
| Support/contours/geometry | Preparation only: FP64 cached coordinates, known-hue vivid 7×7 preparation and row-major stride 8 | AMD eight-fixture preparation passes at 1e-12; Intel rejects unsupported FP64. RANSAC, support contours, PCA and complete object reports are not implemented |
| Raw alignment | Separate Alignment API implemented for zero distortion, checked against scalar SDK oracle | [Exact alignment qualification](performance/alignment-20261023.md) now passes all eight CPU/SDK/GPU cases on AMD and Intel: exact Z16, maximum XYZ error 0 m, unchanged outer 1e-7 m gate; current 640×480 zero-coefficient profile/loaded SDK only. [Machine evidence](performance/alignment-20261023.json); shared-device camera adapter and local AMD strict/auto cold gates now pass the [bounded startup/live smoke](performance/raw-camera-startup-20261023.md). This does not qualify another camera/host or full geometry |
| Encoding/recording | Shared-device GPU conversion and H.264 implemented; see [resident video](resident-video.md) | AMD 60 encoded / 60 CPU-decoded / 60 hardware-decoded frames pass hashes and luma; Intel first packet is corrupt and uses CPU fallback. No caller recording-service integration |
| Occupancy mapping | Not implemented | No free-space evidence or OctoMap qualification |

Modes are `cpu`, `auto`, and strict `vulkan`. Native CPU mode executes equivalent conversion/masks/depth cuts/connectivity/compact reduction. `auto` tries a hardware Vulkan device first and reports the initialization failure reason when CPU is selected. Runtime dispatch/fence/source errors invalidate the frame, publish unknown/restart-required and stop owned capture; **runtime CPU recovery has not been qualified**. Strict `vulkan` never silently runs CPU compute.

The camera component enumerates the actual sensor profiles before starting a single RealSense owner. Its `profile` argument selects `vga` (640×480 RGB/depth), `720p` (1280×720 RGB/depth, the default), or `1080p` (1920×1080 RGB and 1280×720 depth), all at 30 Hz with RGB8/Z16. Missing camera or profile causes a clear initialization error; no image is resized to satisfy a request. Auto tries Vulkan first: supported source/SDK/profile admission, bounded startup stability and three original live-cohort SDK/depth/XYZ/compact cold checks precede raw Vulkan readiness. Strict Vulkan refuses failed admission and never silently falls back. Explicit CPU uses vendor CPU alignment and native Core; auto initialization fallback reports a concrete reason. Shared Alignment→Core device-depth ownership lasts through the consuming Core fence. It publishes adapted RGB compatibility images only when ordinary ROS subscribers request serialization. Internal composed consumers receive the lease through intra-process ROS type adaptation. Calibration generation, hardware time/domain, source frame number, device serial, original acquisition dimensions and host receipt time remain attached to the lease. Raw hardware timestamps are not relabeled as ROS-clock timestamps.

The [higher-resolution fwmini4 observation](performance/fwmini4-high-resolution-20261027.md) records exact alignment on Intel/AMD and live full-resolution processing. Intel Vulkan delivered about 19.1 unique outputs/s at 720p and 8.7 at 1080p, with older pending frames replaced. This preserves image resolution and CPU headroom but does not meet 30 Hz processing or qualify the full station. The [640×480 observation](performance/fwmini4-camera-live-20261027.md) remains a separate baseline.

The [RViz coexistence observation](performance/fwmini4-rviz-coexistence-20261027.md) measured the same Intel GPU rendering the robot, a synthetic world surface and actual camera RGB at about 31.25 redraws/s. Vulkan perception produced 30.03, 13.65 and 6.20 unique outputs/s at VGA, 720p and 1080p. These short headless graphics measurements distinguish rendering, acquisition and fresh perception results; they do not qualify control or physical display presentation.

The [bounded fwmini4 camera smoke](performance/fwmini4-camera-live-20261027.md)
ran the actual D435i at 640×480×30 with scalar SDK 2.58.3 and native ROS.
All three 15-second CPU, strict Vulkan and auto runs delivered about 30 unique
outputs/s. Vulkan used 18.7% of one core (4.7% of four-core capacity), versus
70.4% (17.6%) for CPU with the same SDK. Optimized system SDK 2.58.4 CPU also
sustained 30 Hz at 36.9% of one core; scalar CPU is not the fastest baseline.
System-SDK strict mode refused its alignment convention and auto correctly fell
back. This qualifies process-local SDK selection, not deployment; full geometry,
marker CPU costs, control and soak remain unmeasured.

Build with the existing native ROS wrapper and isolated prefixes:

```bash
ros2/conda/so101-conda.sh run colcon --log-base /tmp/station-vulkan-log build \
  --base-paths ros2/src --packages-select so101_station_vulkan \
  --build-base /tmp/station-vulkan-build --install-base /tmp/station-vulkan-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  -DRMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -DSLANGC_EXECUTABLE=$HOME/.local/opt/slang-2026.19/bin/slangc
```

On this host, Conda's loader and system RADV need process-local system libdrm libraries. Restrict the driver and preload for the tested command only:

```bash
ros2/conda/so101-conda.sh run env \
  VK_DRIVER_FILES=/usr/share/vulkan/icd.d/radeon_icd.x86_64.json \
  LD_PRELOAD=/usr/lib64/libdrm.so.2:/usr/lib64/libdrm_amdgpu.so.1 \
  /tmp/station-vulkan-build/so101_station_vulkan/benchmark_rgbd \
  --backend compare --frames 60 --warmup 10 --output /tmp/resident-synthetic.json
```

For an unchanged recorded 640×480 fixture with `color.png`, `aligned_depth-raw.npy` and explicitly calibrated `capture.json`:

```bash
ros2/conda/so101-conda.sh run python tools/benchmark_resident_rgbd.py \
  --binary /tmp/station-vulkan-build/so101_station_vulkan/benchmark_rgbd \
  --fixture /absolute/recorded/fixture --amd-system-driver \
  --frames 60 --warmup 10 --output /tmp/resident-recorded.json
```

The wrapper rejects resize, inferred depth units, unmatched images and nonzero distortion. It hashes original inputs, carries source stamps/calibration into its report, compares native CPU/GPU full arrays and independently checks masks, depth bits, cuts and canonical all-pixel component membership against OpenCV. It additionally invokes the retained `split_depth_coherent_components` implementation and checks exact retained piece membership/areas. The historical seven-stage morphology comparison passed on AMD and Intel for every independently closed piece membership against retained per-piece OpenCV closing, including overlaps. Full geometry parity remains unqualified. Diagnostic comparisons and compact-only timings must be reported separately. The local AMD three-mode camera smoke below establishes only its bounded integration boundary; complete station latency/CPU reduction, long soak, recording on/off and physical sorting gates remain open.


## Historical eight-stage performance checkpoint, 2026-10-23 (pre-shared-divide)

These timings precede the shared exact-division Core correction. See the [current Intel prealigned Core remeasurement](performance/intel-core-shared-divide-20261023.md); neither dataset is whole-camera timing.

The [durable Wave6 report](performance/resident-20261023.md) and [machine evidence](performance/resident-20261023.json) record the atomic-fusion candidate. Immutable A→B compression is retained; connect atomically flattens A from frozen B before boundary hooks, removing 32 full-plane parent copies and 64 transfer barriers. The [Wave5 software checkpoint](software-checkpoint-20261023.md) is historical copyback evidence. Both GPUs pass seven exact compute cases, actual capacity fault 2, and the production-reduction nonconvergence fault 1/canonical fault 0 probe. All rounds, thresholds and tolerances remain unchanged; universal 32-round convergence is not claimed.

Median-of-three native trial p95 is AMD dense/recorded 1.059647/0.482329 ms and Intel 41.977849/13.812418 ms. At offered 30 Hz, Intel dense completes 1402/1800 frames (23.3667 fps), replacing 393 pending frames, with age p95 76.233 ms: this still fails 30 Hz processing. Intel recorded completes 1800/1800 with no replacement and age p95 24.868 ms; paced submit/finish p95 is 23.752 ms versus old 23.868 ms. The large unpaced gain does not imply a comparable paced gain. Observed 600 versus 750 MHz nonzero GT medians make DVFS plausible, without complete causal proof.

Intel recorded uses 1.664821 ms process CPU/completed frame versus CPU mode 6.141550 ms (72.89% less); dense uses 2.156685 versus 24.448020 ms, with different delivered throughput. These are perception-process metrics, not whole-host capacity. Separate 125 Hz control trials are synthetic, not physical control qualification. Normal upload/readback remain 1,536,000/163,856 B per frame; diagnostics read back 34,570,256 B.

The latest-frame software policy retains one active and one replaceable pending frame within three leases, publishes unique completed cohorts, and preserves original source hardware metadata and pre-alignment receipt time. Its CPU review does not establish live camera throughput. That historical dataset does not qualify raw camera integration. The later local AMD smoke below adds bounded startup/unique-cohort evidence; matched whole-camera performance, video integration, full geometry, complete station CPU reduction, soak and physical acceptance remain unqualified; there is no canonical deployment.

## Current local live integration boundary, 2026-10-23

The [startup and V4 live archive](performance/raw-camera-startup-20261023.md) records one sequential 15-second steady smoke each for CPU, strict Vulkan and auto on the actual local D435i/Radeon 8060S. All passed; strict/auto selected raw Vulkan without fallback after three live-cohort exact SDK-depth, XYZ ≤1e-7 m and independent CPU compact checks. CPU/strict/auto observed 449/451/451 unique original cohorts, process CPU 14.120/3.792/3.304 ms per output and host-receipt age p95 15.261/4.665/4.649 ms. These uncontrolled sequential scenes do not establish a matched speedup, exposure age, soak or general performance guarantee; they do not qualify the fwmini4 camera.

Startup quarantine handles observed native SDK zero/regressed source IDs before admission without weakening steady source guards. The earlier V2 source failure and V3 collector-starting-status failure remain in the archive. Initial received=1 is the startup seed; periodic counter deltas and collector-window cohorts are distinct observations. All identity/generation/safety/cleanup checks passed. Geometry remains unqualified and execution false; Intel FP64 preparation limitations and unimplemented full geometry remain unchanged. No robot actuation or full-station p95/CPU claim follows.

## Historical seven-stage qualification baseline, 2026-10-22

This entire section describes only historical shader source `04c5b21ba975057197ed539e5b8c4febfccea6aa4691b9c4c61475c656694ccd`. Its seven-shader layout, atomic path splitting and unexecuted GPU nonconvergence-fault probe claims do not describe current code. The timings remain valid evidence of that failed baseline.

The [machine-readable evidence summary](resident-vulkan-qualification-20261022.json) pins source/SPIR-V/input hashes, each matrix result and timestamp stage means. Shader source SHA256 is `04c5b21ba975057197ed539e5b8c4febfccea6aa4691b9c4c61475c656694ccd`, Slang2026.19, precise floating-point/O2. Evidence uses the compute-only Context before optional video integration changes. Earlier six-stage results are superseded for this computation boundary.

Each successful case ran60 measured frames after10 warmup. Both Radeon8060S(RADV) and Intel Graphics(ADL-N) passed native CPU/GPU full-array and compact comparisons, unchanged XYZ1e-7 tolerance, and independent retained/OpenCV closing for holes, rings, diagonals, depth splits, long snakes, overlapping red/green checkerboard closings and the unchanged640x480 recorded fixture. Masks, depth bits, cuts and every canonical and closed membership had zero independent mismatches.5120separate24pixel components correctly failed capacity4096 with fault2 on both GPUs.

| Historical seven-stage case | AMD p50/p95ms | Intel p50/p95ms |
|---|---:|---:|
|Dense640x480 synthetic, diagnostics|5.551 /6.390|560.365 /561.272|
|Unchanged640x480 recorded, diagnostics|3.411 /4.332|15.236 /15.710|
|Same recorded, compact-only GPU|0.528 /0.629|13.049 /13.152|
|Same recorded, native CPU|1.856 /1.917|5.086 /5.125|

**Intel fails the station performance target on the dense synthetic scene.** Its compact GPU recorded processing is also2.56x slower than native CPU. AMD results do not qualify Intel or the full station. Normal640x480 transfers are1,536,000B upload and163,856B compact download/frame; diagnostics download33,341,456B/frame. Neither benchmark includes capture/raw alignment, complete geometry, publication age, video or station concurrency. Serial frozen-input submit/finish timing excludes upload memcpy and CPU oracle comparisons.

Optional `--timestamps 1` preallocates queries, collects GPU timestamps only after the existing frame fence, and never adds query waits or frame-time allocation. Intel synthetic compact GPU means: connect/compress542.464ms, reduction7.283ms, morphology4.537ms, convert1.044ms, cuts0.603ms, upload0.320ms, gather0.554ms and readback0.037ms. Diagnostic readback rises to2.250ms while CC remains543.626ms. Actual i915 render-engine busy was99.3–100%, native CPU~3%, RSS~67MiB. The measured bottleneck is global CC, not morphology or dense download. Tiled CC is only a potential next experiment; no such optimization is qualified.

Native `/usr/bin/time` recorded compact runs (70total frames including warmup) used user+system CPU0.04s/RSS90936KiB on AMD and0.11s/RSS66132KiB on Intel, versus native CPU0.15s/87540KiB and0.41s/86880KiB. These aggregate short-process costs include initialization/reference work and do not establish full-station CPU reduction. AMD diagnostic runs additionally used26–30CPU seconds outside the small per-frame timing interval; that aggregate cost is not attributed to a steady station tick.

Root traversal has a monotone pixel-count bound with atomic path splitting;32union/compression rounds and all admission thresholds remain unchanged. Long-chain/snake cases converged. An actual GPU nonconvergence fault was not forced; production decoder fault1/2/4/8 refusal was exercised separately. A CPU overflow safety regression (4096valid pieces followed by one5120pixel piece) originally segfaulted; clearing the rejected root-map lane fixes it without admitting extra components. Native regression now checks explicit fault2 and all retained counts24. CPU morphology also takes66ms on120nested640x480 rings, so the fallback has its own scene-dependent20Hz failure.

All raw alignment/distortion, full geometry, live30Hz/unique frame/publish-age, >=20Hz/p95<100ms complete station, >=30%stationCPU reduction, video/recording,10min soak and physical sorting gates remain open. No camera or robot was started for these offline tests.

## Compute-feature prerequisite and CPU geometry contract, 2026-10-23

The [baseline checkpoint](performance/compute-feature-baseline-20261023.md) records supported FP64 preserved independently of optional video, the final 3,868-test CPU pass and separate offline GPU/provenance outcomes. The [12-case retained CPU matrix](../tools/native_geometry_matrix.py), [procedural golden](../tests/fixtures/native_geometry/matrix-v1.json) and [native geometry integration plan](superpowers/plans/2026-10-23-native-geometry-integration.md) define the next contract. Strict provenance identity and cross-runtime payload compatibility remain separate, with unchanged numerical/array tolerances. Native plane fitting, contours, PCA and complete reports are not implemented; Intel hardware FP64 remains unsupported and its verified refusal is not geometry success.

## Current offline build boundary

A deployment compiler is optional: `-DPRECOMPILED_SHADER_DIR=/absolute/offline/shaders` requires the current parent/helper composite SHA256 (`resident.slang` plus `exact_divide.slang`) in `provenance.txt`, exactly eight expected compute `.spv` files and matching per-binary `.spv.sha256` sidecars. Wrong source or output digest fails configuration. `CMAKE_CONFIGURE_DEPENDS` refreshes provenance when parent or included helper source changes. The historical isolated Intel seven-stage build and offline comparisons passed; its wrapper uses absolute isolated package paths and a process-only `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` while the existing deployment's custom GID middleware has no development CMake config. No deployment or service changes are authorized by this package.
