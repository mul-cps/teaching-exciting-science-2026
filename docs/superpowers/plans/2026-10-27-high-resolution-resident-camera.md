# Higher-resolution resident camera implementation plan

> For agentic workers: use subagent-driven development with independent resource
> review before native GPU validation. The supervisor owns integration and commits.

**Goal:** Preserve full 720p/1080p RGB quality in the resident Vulkan camera path.

**Architecture:** Keep one camera owner and a shared Vulkan context. Separate
native-depth and aligned-color extents, retain the three-slot pools and bounded
latest-frame scheduling, and reject unsupported device resources before allocation.

**Tech stack:** C++17, ROS 2 Jazzy, librealsense, Vulkan, offline Slang/SPIR-V.

**Spec:** The accepted workshop plan plus the user's 27 October requirement for
720p or 1080p image quality; the native profile enumeration and live acquisition
evidence in `docs/performance/fwmini4-camera-live-20261027.md`.

User requirement: operate at 720p or 1080p for better image quality. The passing
640×480 observations are a baseline, not the desired workshop resolution.

Use explicit 30 Hz profiles: `vga` (640×480 RGB/depth), `720p` (1280×720 RGB/depth),
and `1080p` (1920×1080 RGB, 1280×720 depth). The D435i advertises all of these with
zero distortion coefficients. Its highest pair already passed SDK acquisition;
GPU processing at either higher resolution remains unqualified until measured.
Align native depth to the selected RGB grid without resizing the RGB image.

1. Expand Core's bounded admission to at most 1920×1080 pixels. Check storage,
   dispatch and push-constant limits before allocating the three-slot pool.
   Preserve all component capacities, arithmetic, thresholds and fault behavior.
2. Make Alignment's input/output extents independent. Allocate rays/input by
   depth extent and aligned depth/XYZ/scratch by color extent. Pass extents to
   both kernels, preserve scalar arithmetic, the 4096-pixel projection bound,
   exact Z16 comparison and the existing XYZ limits. Extend frozen SDK/reference
   tests to the two higher-resolution pairs, retaining the original cases.
3. Expose a named profile in launch and camera configuration; verify exact
   resolved/active profiles and row strides, copy the correct native plane sizes,
   and report actual extents and transfer bytes. Cold checks compare the whole
   selected grids and count native-depth coverage independently. Preserve source
   locking, calibrated intrinsics, latest scheduling and visible CPU fallback.
4. Before GPU testing, independently review new paths for allocation, waits,
   synchronization and readback; add size/cost guards. Rebuild pinned shaders,
   run CPU contracts, then bounded Intel/AMD numerical checks. On fwmini4 run
   CPU/strict Vulkan/auto at both profiles. Record unique throughput, skipped
   source frames, CPU time, frame age, RSS and device allocations separately.
   If processing cannot sustain acquisition, retain the bounded latest-frame
   policy and report skips; never count repeated publications as new frames.

The Intel N100 reports a 4,294,967,295-byte storage-buffer range and 256-byte
push-constant limit; 1080p Core's 149,299,200-byte normal scratch fits this host.
Devices with smaller limits must refuse or visibly fall back before allocation.
Full geometry, CPU marker registration, control qualification and physical
station acceptance retain their existing separate boundaries.

## File and interface ownership

- Core: `pipeline.hpp`, `frame_pool.cpp`, `gpu_pipeline.cpp`; introduce
  `kMaxFramePixels`, preserve `Pipeline`/`FramePool` public signatures, and extend
  the existing synthetic device-depth/admission contracts.
- Alignment: `alignment.cpp`, `alignment_reference.cpp`, `alignment.slang`,
  `alignment_oracle.cpp`; append the same four U32 fields to both push-constant
  layouts: `depth_width`, `depth_height`, `color_width`, `color_height`. The
  existing `AlignmentCalibration` supplies both grids. Add oracle `--profile`
  while retaining the default VGA cases and `--vulkan` interface.
- Camera: `camera_component.cpp`, `raw_camera_admission.hpp/.cpp`, and
  `resident_rgbd.launch.py`; add `profile=vga|720p|1080p` (default `720p`), extend
  cold-check dimensions compatibly, and retain all original source safety gates.
- Verification: focused CPU host contracts and all original frozen numerical
  cases precede native GPU tests. The supervisor collects live high-resolution
  metrics only after independent resource review accepts the complete data path.

## Review focus

- Unequal native-depth and RGB grids must not overrun input or diagnostic copies.
- Invalid or silently substituted profile pairs must fail before publication.
- Storage/dispatch limits must fail before pool allocation and expose fallback.
- Profile changes must bind the correct intrinsics, source generation and strides.
- Partial tile edges, invalid depth and frame skipping must preserve fault and
  timestamp semantics; reported throughput counts distinct original source IDs.
