# Actual upstream GPD backend: optional offline CPU reference

The active LEGO station still uses its existing geometric planner. This optional
reference runs **actual upstream GPG/GPD candidate generation and the original
trained classifier** on recorded point clouds. It also exposes a separate ROS
`GraspPlanning` service. It does not open a camera/serial device, fit calibration,
subscribe to live images, replace station candidates, or execute robot motion.

## What was proved

A clean private-cache build and actual recorded H00 RGBD cloud run succeeded:

| Run | Actual classifier images | Scores ≥ original threshold 500 | Returned grasps | Geometric fallback | C++ total |
|---|---:|---:|---:|---|---:|
| First station H00 | 91 | 6 | 5 | No | 699.8 ms |
| Clean-cache H00 replay | 80 | 9 | 5 | No | 629.9 ms |
| Upstream tabletop fixture | 32 | 1 | 5 | Yes | 399.3 ms |

These C++ totals exclude PCD read/process startup and external Caffe parity
validation. Audited runs also include bounded audit writing. Upstream random
sampling can produce different candidates on another run; each saved audit binds
the exact actual images used in that run. These are feasibility results, not
physical qualification or a sorting success rate.

The actual ROS service run returned four neural-classified grasps from 92 images
(10 above threshold), preserved the original H00 acquisition timestamp, and
published `recorded_source=true`, `qualified=false`, `target_association_verified=false`,
`tool_mapping_qualified=false`. The existing live adapter **rejected** those old
observations. Its 0.5 s freshness bound was not changed. The current whole-scene
reference is too slow for that live admission budget.

Evidence: station evidence directory `2026-10-19-sorting/intel-grasp-backend/`,
including `native-ROS-proof-001/ROS-PROOF.json`, first actual image audits,
`repro-H00-001/recorded-result.json`, and the original failed batch-mode backtrace.
A machine-readable summary is in [intel-grasp-backend-verification.json](intel-grasp-backend-verification.json).

## Pinned sources, licenses and dependencies

| Component | Source revision |
|---|---|
| Intel ROS library | `980b7ddd4348d9aa78c6e84c42da1984746867db` |
| sharronliu GPD engine | `fdd699308eee846bffd93b8efd4029469e6aef0b` |
| atenpas GPG, also available at pull/7 head | `3dcd656d70f095ad1bda3d2fb597a994198466ab` |

Source URLs and every model file hash are embedded in the builder/converter and
recorded in `build-manifest.json`. Intel's wrapper is Apache-2.0. GPG/GPD root
licenses are BSD-2-Clause; original individual source notices are preserved in
the fetched repositories. Models are bundled in that BSD repository; no separate
model-only license file was found. The builder retains and hashes upstream license
files alongside the private artifacts. No upstream code or weights are vendored
into this repository and no dependency is installed globally.

The archived ROS node itself was **not built**. Its original instructions require
Ubuntu 18.04, ROS 2 Dashing, PCL 1.8.1 EXACT and OpenVINO 2019 R3.1. We compile the
standalone pinned GPG/GPD engine with a small independent build wrapper, then
provide our own offline ROS boundary. Tested native dependencies already present
in the isolated Jazzy Conda environment were PCL 1.15.1, Eigen 5.0.1, Boost 1.90,
OpenCV 5.0.0, GCC 15.3 and CMake 4.4.4. Exact package builds/hashes are in the
build manifest. This proves that configuration, not general legacy ROS compatibility.

Compatibility changes are narrow and recorded:

- Two GPG `std::set` comparators gain `const` call operators for C++17.
- Explicit standard/Boost headers replace old transitive-header assumptions.
- The original GPD classifier interface gets a CPU OpenCV DNN implementation.
  OpenCL is explicitly disabled; no ROCm/AMD GPU acceleration is claimed.
- OpenCV 5 removed its Caffe reader. The pinned IR-v2 graph is represented as an
  ONNX graph with **bit-identical** original weights, without training or changing
  mathematical layers. Convolution/pooling/FC/ReLU geometry is fixed to this model.
- The original optional `create_image_batches` branch incorrectly indexes all
  candidates against one sub-batch, causing a reproducible segmentation fault.
  The reference keeps the upstream **default false**. Inference itself uses a
  fixed batch of eight with zero padding only for unused inference slots.

## Exact classifier conventions

Input is the upstream **15-channel grasp feature image**, not RGB. Original
channel order is preserved; HWC uint8 becomes NCHW float32. There is no BGR swap,
RGB mean/std normalization, or additional scale. The pinned inference prototxt's
`MemoryData` layer has no `transform_param` (default scale 1, mean 0). A separate
training prototxt contains a scale and is not substituted for the deployment
contract. This integration does not qualify the original model's physical accuracy.

The score is **positive raw logit[1] minus negative raw logit[0]**, not probability.
The original 500 threshold is unchanged. Upstream falls back to geometric
hypotheses when two or fewer images pass it; every exported grasp records
`origin` and `neural_accepted`, and the classification report records
`used_geometric_fallback` and actual threshold counts.

All 91 actual H00 grasp images were compared with original OpenCV 4 Caffe CPU
inference: maximum absolute logit error 0.000702 and score error 0.000977, with
**identical threshold decisions**. All 32 actual tabletop images also matched
threshold decisions. These checks use actual generated feature images, not only
random inputs. Source weights/IR and all Caffe weight arrays were also checked
bit-for-bit. Audits retain raw images, original logits/scores and their hashes.

## Build in a private cache

The converter needs an independent Python with NumPy, OpenCV 4's Caffe reader,
and ONNX 1.19.1. On this host NumPy/OpenCV 4 already exist in base Python; a private
venv with `--system-site-packages` was used. Do not downgrade the live Conda
OpenCV installation. The builder removes foreign ROS/Conda Python ABI paths when
calling the converter, and retains the Conda native build environment for C++.

```bash
GPD_CONVERTER="$HOME/.cache/exciting-science/gpd-converter"
GPD_CACHE="$HOME/.cache/exciting-science/gpd-reference"
python -m venv --system-site-packages "$GPD_CONVERTER"
"$GPD_CONVERTER/bin/python" -m pip install onnx==1.19.1
ROS_DOMAIN_ID=177 ROS_LOCALHOST_ONLY=1 ros2/conda/so101-conda.sh run \
  python tools/intel_grasp_backend/build_reference.py --cache "$GPD_CACHE" \
    --converter-python "$GPD_CONVERTER/bin/python" --jobs 2
```

The builder refuses an unrelated nonempty cache or unexpected source edits. It
fetches exact revisions, verifies all model hashes, applies only the checked
comparator patch, builds at low priority with at most two jobs, and writes
`build-manifest.json`. Missing native dependencies are explicit failures in
`cache/logs`; it never invokes apt or changes the active Conda environment.

## Run recorded RGBD and the ROS API

Export an existing native capture (original camera frame/time and RGB preserved):

```bash
python tools/intel_grasp_backend/capture_to_pcd.py /path/to/H00 /path/to/H00.pcd
ROS_DOMAIN_ID=177 ROS_LOCALHOST_ONLY=1 ros2/conda/so101-conda.sh run \
  python tools/intel_grasp_backend/run_reference.py \
    --build-manifest "$GPD_CACHE/build-manifest.json" --pcd /path/to/H00.pcd \
    --provenance /path/to/H00.provenance.json --output-directory /path/to/new-attempt \
    --audit --converter-python "$GPD_CONVERTER/bin/python"
```

Only finite optical points with 0 < depth ≤ 1.2 m are exported; source and output
hashes, source model hash, native point-cloud provenance and original acquisition
nanoseconds accompany the PCD. Existing output files are never overwritten.
Each exported grasp carries its source PCD hash/frame/time and unqualified flags.

To expose the actual engine as a diagnostic ROS service:

```bash
ROS_DOMAIN_ID=177 ROS_LOCALHOST_ONLY=1 ros2/conda/so101-conda.sh run \
  python tools/intel_grasp_backend/ros_offline_backend.py \
    --build-manifest "$GPD_CACHE/build-manifest.json" --pcd /path/to/H00.pcd \
    --provenance /path/to/H00.provenance.json --output-root /path/to/ros-attempts
```

Call `/offline/intel_grasp/plan` with `moveit_msgs/srv/GraspPlanning`, group
`offline_reference` and target ID `offline-pcd:<pcd_sha256>` from provenance.
The target is an exact recorded dataset, never a station track. Optional geometry
is rejected. A caller cannot replace the source timestamp with now. Each request
runs the actual bounded CPU subprocess (one outstanding request, ten-second
subprocess deadline by default), not a fake detector or a prerecorded grasp list.

Topics: `/offline/intel_grasp/poses` (PoseArray) and `/offline/intel_grasp/status`
(JSON String). Poses are GPD hand-base poses in the original optical frame:
X=binormal/closing, Y=hand axis, Z=approach, with zero EEF offset. They are not
qualified robot TCP or contact targets. No jaw postures, robot EEF parent, target
association or physical gap mapping are invented. Approach/retreat distances are
upstream reference defaults and remain unqualified. Default station launch does
not start this service and existing station topics are unchanged.

A self-contained actual service check uses an isolated nonzero domain (it refuses
the live domain 76), records the RPC/subscriber proof and verifies rejection by
the existing live freshness gate:

```bash
ROS_DOMAIN_ID=177 ROS_LOCALHOST_ONLY=1 ros2/conda/so101-conda.sh run \
  python tools/intel_grasp_backend/check_offline_ros.py \
    --build-manifest "$GPD_CACHE/build-manifest.json" --pcd /path/to/H00.pcd \
    --provenance /path/to/H00.provenance.json --output /path/to/ros-proof
```

## Compute evidence and remaining gates

A fixed 91-image actual H00 audit, identical batch-eight policy and five measured
samples after warmup gave C++ OpenCV 5 ONNX mean/p95 **112.0/113.2 ms**, versus
original OpenCV 4 Caffe CPU **151.6/155.4 ms**. All threshold decisions matched.
This measures classifier inference only under shared host load, excluding point
cloud processing, model/input I/O, audit I/O and DDS. It is not a GPU result or a
qualified live frame rate. Static guards check CPU selection, bounded audit/batch
sizes, no live sensor/action imports and no shell execution hooks.

Whole-scene GPD is optional and currently exceeds live freshness. A future live
backend needs a bounded target region from the **same acquisition** with proven
station track association, plus bounded end-to-end processing time. Neither an
ID echo nor the generic CNN establishes that association. Camera/arm registration,
actual gripper geometry/TCP/jaw calibration, scene clearance and existing motion
qualification gates still apply. No physical pick/place was attempted by this work.

Focused automated checks (no hardware):

```bash
python -m pytest -q tests/test_intel_grasp_backend.py
ROS_DOMAIN_ID=177 ROS_LOCALHOST_ONLY=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  ros2/conda/so101-conda.sh run python -m pytest -q \
    tests/test_intel_grasp_backend.py tests/test_intel_grasp_offline_native.py \
    tests/test_intel_grasp_adapter.py tests/test_station_action_shell.py
```

The native suite includes synthetic service contract tests; the separate
`check_offline_ros.py` evidence above executes the real compiled engine. Neither
software result establishes physical-station acceptance.

## Bounded PCD input contract

The hardened reference entrypoints accept only the station exporter's little-endian
binary `x y z rgba` schema: four 4-byte scalar fields (`F F F U`), identity
VIEWPOINT, header at most 4096 bytes, 1–1,000,000 points, exact WIDTH×HEIGHT=POINTS,
exact 16-byte point storage, and finite XYZ. Python validates this before any
engine subprocess. The C++ executable independently decodes this schema before
constructing a PCL cloud; it never invokes PCL's generic file parser. File reads
are capped at 16,004,096 bytes. Unknown, ASCII and compressed PCD formats are
explicitly unsupported. Earlier upstream tabletop ASCII runs remain historical
compatibility evidence; current supported actual-engine replay uses the finite
H00 binary export. No input format conversion silently removes nonfinite data.

The Python finite check uses a zero-copy strided NumPy XYZ view with at most a
3 MB boolean temporary. Maximum supported one-million-point admission (file
read/header/finite scan only) measured 9.67 ms mean, 13.10 ms maximum across ten
shared-host samples; SHA and engine/DDS are excluded. A static test forbids a
per-point Python decoding loop. Run `benchmark_pcd_guard.py --output PATH` to
repeat this bounded synthetic benchmark.
