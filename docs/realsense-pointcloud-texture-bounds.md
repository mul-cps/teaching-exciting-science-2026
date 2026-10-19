# RealSense CPU pointcloud texture bounds repair

This optional wrapper overlay repairs unchecked texture indexing in ROS wrapper
4.58.4 (`9a11121700cb4780e273e34141f6402fe184321d`). It reuses the installed
CPU librealsense 2.58.3 and message interfaces 4.58.4. It does not enable GPU
processing, activate an overlay, access a camera, or change calibration.

## Repair

`PointcloudFilter::Publish` previously accepted normalized coordinates equal to
one, then multiplied by texture dimensions and copied bytes without validating
the resulting pixel or storage offset. The patch requires finite UV coordinates
in `[0,1)`, retains the original float multiplication, and rejects products that
round to the width/height before integer conversion. Integer pixel bounds,
actual row stride and available storage are checked before copying. Arithmetic
is bounded before offset multiplication/addition. No UV clamping or fabricated
edge colors are introduced. Invalid texture samples follow the existing
organized-cloud and allow-no-texture behavior.

Frame properties are fetched once outside the point loop. The inline predicate
adds scalar checks without allocations or per-point SDK calls. Existing texture
format handling otherwise remains unchanged.

## Reproduce the candidate

From a clean checkout containing these four files:

- `ros2/conda/build-realsense-cpu-texture-bounds.sh`
- `ros2/patches/realsense-ros-4.58.4-pointcloud-texture-bounds.patch`
- `tests/test_realsense_pointcloud_texture_bounds.py`
- this document

Run:

```bash
python -m pytest -q tests/test_realsense_pointcloud_texture_bounds.py
ros2/conda/build-realsense-cpu-texture-bounds.sh
```

The builder requires the existing dedicated `so101-jazzy` Conda environment
(or `SO101_CONDA_PREFIX`), CMake/Ninja/compiler and installed dependencies. It
checks the wrapper archive digest, exact source inventory and source bytes,
then applies only the pinned texture patch. GLSL and build tests remain off;
compilation uses nice 10 and at most two jobs. It installs only into
`ros2/.cache/realsense-cpu-texture-bounds/install`, recording source inventory,
builder, ELF and dependency hashes in `build-provenance.json`. It performs no
automatic activation or live validation. The shared experimental GLSL builder's
default `--cpu-wrapper` mode delegates to this standalone builder.

## Operator activation contract

Activation is an explicit operator step after review. Putting the candidate
prefix first in `AMENT_PREFIX_PATH` and its library directory first in
`LD_LIBRARY_PATH` expresses loader intent; neither setting proves selection.
Both the initial library-path-only probe and the subsequent two-prefix probe
still loaded the old Conda camera library. Preserve both as failed activation.

The generated standalone node passes a library basename to its class loader.
Inherited ELF loader search paths can therefore select the Conda plugin despite
the requested environment. Ament resource selection is relevant for component
loading paths but must not be assumed to determine this standalone executable's
loaded library. Explicit preload or an absolute plugin path requires a separate
operator decision and runtime proof; this builder performs neither.

The third controlled probe selected the reviewed CPU candidate only after the
operator supplied an absolute, process-scoped preload in addition to the two
prefix settings. For an already reviewed camera executable and arguments, the
manual invocation has this form (paths and executable are operator supplied):

```bash
env LD_PRELOAD="$CPU_PREFIX/lib/librealsense2_camera.so" \
    LD_LIBRARY_PATH="$CPU_PREFIX/lib:$ENV_PREFIX/lib" \
    AMENT_PREFIX_PATH="$CPU_PREFIX:$ENV_PREFIX" \
    "$REVIEWED_CAMERA_EXECUTABLE" "${REVIEWED_CAMERA_ARGUMENTS[@]}"
```

`CPU_PREFIX` must be absolute. Supply these variables only to that reviewed
process invocation; do not export a preload globally or add it to a system
loader configuration. This instruction does not restart a process. The third
probe's `runtime-libraries.json` and `camera-launched.json` under
`overnight/camera-cpu-bounds/` record mapped camera SHA256 `dbe15530…` and base
CPU SDK SHA256 `067df4f4…`, with no old Conda camera library mapped. That proves
activation for that process; the sustained soak remains a separate result.

Before accepting a runtime result, retain the actual process environment, ament
component resource selected from the candidate prefix, and `/proc/<pid>/maps`.
Verify that the mapped `librealsense2_camera.so` path, device/inode and SHA256
identify the reviewed candidate, and that mapped librealsense identifies the
reviewed base CPU SDK. Reject deleted/replaced or unexpected mapped files.
Record hashes through the process's filesystem namespace when necessary. A
version string, successful build, candidate file existence, or a supplied launch
environment alone does not prove which plugin was loaded.

The same loader-selection risk and mapped-library proof applies to a future GLSL overlay,
including its SDK and GL extension dependencies. That is an additional deployment
gate; it does not authorize activating the experimental GPU wrapper.

## Evidence and limits

The compiled predicate regression covers all corners, equality to one,
negative/NaN/Inf UVs, `nextafter(1,0)`, invalid dimensions/stride/storage, padded
rows, and 100,000 deterministic samples compared with the original float-product
pixel index. The source checker is exercised with an injected extra source file.

The independent 921,600-point valid-UV copy benchmark produced equal checksums:
old/new mean 3.017/4.117 ms. Its p95 10.952/11.635 ms was affected by a concurrent
build; these numbers do not establish live camera latency. Evidence is under
`gpu-perception/texture-bounds-independent/` in the 2026-10-19 station artifacts.

The reviewed CPU candidate ELF SHA256 is
`dbe15530ee5aa2855b6b05ab43d7c1f7c6404234b04bfc60dd04c79758699261`;
patch SHA256 is
`008b46ff14fcb1c999a28935101608a18b8d41fafbd1bd4a866716bf262e3476`.
That candidate was built with the prior shared builder
`90f7f213d8cba7e12bbccf0f25381a73d1db6cf6c0bb4c11a80b72a44ebd3879`.
The standalone extraction has syntax/static/compiled-predicate verification;
it has not rebuilt the candidate during the live camera probe.

The captured core places the fault in texture source copying. The precise
faulting UV was unavailable, so this repair is not asserted to explain the
exclusive crash cause. Full SDK processing, missing/mismatched frames, ROS
serialization and a sustained camera soak remain separate runtime gates.
GPU wrapper deployment additionally requires applying this texture repair to
that separate wrapper source and independent validation; the experimental GLSL
build alone does not provide that repair.
