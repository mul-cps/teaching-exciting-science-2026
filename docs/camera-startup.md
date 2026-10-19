# Persistent camera startup

The station and standalone perception launch defaults now request RGB
**1920×1080 RGB8 at 30 Hz**, depth **1280×720 Z16 at 30 Hz**, and `enable_sync=false`.
These match the user-requested profiles already exercised manually on D435i
serial 243322071783. Actual frame delivery still requires live qualification;
requesting 30 Hz does not establish sustained 30 Hz observations or confirmed tracks.
The independent `color_profile`, `depth_profile`, `enable_sync` and
`far_depth_m` launch arguments remain explicit. The default far clip is 1.2 m and
is also forwarded to object observation admission. `config_file` is empty, since
vendor launch YAML overrides explicit profile arguments. The conservative
`ros2/config/realsense.yaml` remains available for deliberate manual use.

## One explicit build, then startup validation

From the repository checkout:

```bash
ros2/conda/so101-conda.sh realsense-cpu-build
ros2/conda/so101-conda.sh camera-preflight
ros2/conda/so101-conda.sh station-live
```

The build action invokes the existing pinned CPU-only builder into this
checkout's private `ros2/.cache/realsense-cpu-texture-bounds` cache, then validates
its result. It does not activate a package, open a device, change global loader
settings or install into a shared environment. It may download the pinned source
archive during that explicitly requested build. Launch and preflight never build
or download anything. The standard station wrapper supplies absolute repair/SDK
prefixes, independent of the caller's working directory.

For a separately built private repair, pass
`camera_repair_prefix:=/absolute/cache/install` and
`camera_sdk_prefix:=/absolute/so101-jazzy`. A direct `ros2 launch` defaults the
repair prefix to the current checkout directory's cache; use an explicit prefix
when launching elsewhere. To validate an alternate prefix without starting it:

```bash
ros2/conda/so101-conda.sh camera-preflight --prefix /absolute/cache/install
```

Every camera launch checks the 201-file exact source inventory/digest, pinned
wrapper revision/builder/patch, actual source bytes, CMake CPU-only flag/hash,
wrapper SO and CPU SDK 2.58.3 hashes. Source files are bounded at 64 MiB each and
128 MiB total; manifest size is bounded at 256 KiB. Checks run once during launch
preparation, with no source hashing/builds or loader probes in ROS/image callbacks.
The installed camera package must resolve to the same SDK prefix/executable that
is inspected. Missing or mismatched artifacts raise a clear error before any
camera action is returned; no silent unpatched fallback is allowed.

An ELF-loader `--list` subprocess (5 s timeout) verifies the **actual dependency
resolution** under the camera's exact `LD_PRELOAD`. It does not execute the camera
entrypoint or SDK initializers. The verified repair SO preload is applied only
inside the scoped vendor-camera launch group. It replaces inherited preloads for
that child and is restored before marker, arm or RViz processes are launched.
No repair prefix is globally activated and no `LD_LIBRARY_PATH` override is used
to select this SO: existing executable RPATH can defeat that approach. Both the
selected wrapper and selected CPU SDK must match their validated paths/hashes.

## Startup wait and reconnect semantics

`camera_wait_timeout_s` defaults to 10 s (allowed 1..120) and maps to the vendor's
`wait_for_device_timeout`. `camera_reconnect_timeout_s` defaults to 2 s and accepts
integer 1..10, mapping to `reconnect_timeout`. The pinned wrapper source
`realsense_node_factory.cpp:390–430` treats reconnect as a sleep duration;
zero does **not** disable retries, and fractional values below 1 s can truncate to
zero in its timeout branch. No respawn is requested.

The vendor exits with code 1 when its ordinary no-device wait expires. This is not
a hard deadline for a wedged SDK/native call or its exception loop: that upstream
exception branch bypasses the timeout check. Existing downstream source freshness
and readiness gates remain responsible for rejecting missing/stale observations.
No new process supervisor or automatic camera reset is introduced.

## Verification scope

Focused tests inspect launch actions without executing them, verify scoped loader
environment restoration, profile forwarding and missing/mismatched provenance,
source/SO/SDK/CPU flags and loader identity. A static guard confines hashing to
startup. `tools/benchmark_camera_startup.py` measures the full 201-source+SO/SDK
validation and ELF trace; it never opens a camera. This work changes no C++ texture
predicate or PointCloud2 row layout. The known CPU `row_step` metadata discrepancy
is separate and remains unresolved here. No physical stream restart, fresh
calibration, GPU validation or pick/place qualification is claimed.

Camera/perception launch preparation now precedes owner/leader launch actions.
This makes missing repair fail early; it does not guarantee camera readiness
before the owner starts. `use_camera=false` retains the camera-free bypass.
