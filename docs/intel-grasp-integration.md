# Optional Intel grasp proposals

## Scope and compatibility

This station integrates the standard `moveit_msgs/srv/GraspPlanning` boundary of
[Intel ros2_grasp_library](https://github.com/intel/ros2_grasp_library), inspected at
`980b7ddd4348d9aa78c6e84c42da1984746867db`. The upstream repository is Apache-2.0,
archived April 8, 2024, and discontinued. No upstream source is vendored here.

The integration is an optional client bridge to an **independently installed**
backend, not a bundled detector. Upstream `grasp_ros2/src/grasp_planner.cpp` exposes
`plan_grasps` as `moveit_msgs/srv/GraspPlanning`; `moveit_msgs_light` is a separate
legacy compatibility package and must not shadow Jazzy's real `moveit_msgs`.
The adapter uses installed Jazzy messages. Native synthetic services test this
boundary, not the upstream C++ binary.

Upstream build concerns observed directly:

- `grasp_ros2/CMakeLists.txt` requires **PCL 1.8.1 EXACT**, GPG and GPD libraries.
- Its legacy OpenVINO/GPD detection stack has not been built or qualified on this
  station's ROS 2 Jazzy / AMD gfx1151. It is not a ROCm backend. No acceleration is
  claimed by this adapter.
- Its service callback blocks until a grasp appears and has no empty-result
  deadline. Our request timeout does **not** cancel that backend computation.
  The bridge drops late responses, clears local pending-request bookkeeping, and
  never retries automatically. A timeout latches the bridge against further calls,
  so repeated requests cannot accumulate work in a hung backend. Inspect/restart
  both the external backend and adapter before attempting another request.
- Its planner applies configurable EEF translation and yaw, finger joint names and
  postures. These defaults are not SO-ARM101 calibration. Reported poses remain
  **upstream EEF parent poses**, not station pinch/contact targets.

## Start and call (ROS only)

Default station launch is unchanged and incurs no adapter overhead. The optional
launch defaults to `enabled:=false`, which starts no node. Use a nonzero ROS domain
(the station uses 76) and local discovery settings consistent with the station.

Build `so101_station_scene` in your normal overlay, then:

```bash
ros2 launch so101_station_scene intel_grasp.launch.py enabled:=true \
  backend_service:=/intel_grasp/plan_grasps frame_id:=base_link
```

This launches only the bridge. Start the external detector/planner separately in
its own namespace. It does not start a camera, robot driver or execution client.

Service: `/station/intel_grasp/plan`, `moveit_msgs/srv/GraspPlanning`.
Quick action:

```bash
ros2/scripts/station-action.sh intel-grasp /absolute/path/source-request.json
```

The JSON is the standard ROS request representation, for example:

```json
{
  "group_name": "manipulator",
  "target": {
    "header": {"frame_id": "base_link", "stamp": {"sec": 123, "nanosec": 456}},
    "id": "red-1"
  }
}
```

The example stamp is deliberately stale. Generate the request from a **current
tracked object observation**, copying its original acquisition stamp and frame.
The CLI never overwrites that stamp with now. Only target ID/header and group are accepted;
request age is at most 0.5 s. Shape/subframe geometry, nondefault target pose/type,
support surfaces, `candidate_grasps` and `movable_obstacles` are rejected because
the upstream planner does not use them. `grasp_service` passes only `target.id`
to detector `start()`. In the default random-pick build, that ID does **not**
restrict point-cloud grasp detection to the station track. Only the optional
`BUILD_RECOGNIZE_PICK`/`object_detect` path compares it to object classifier
`object_name` strings in `people_msgs/ObjectsInMasks`; these are not station track
UUIDs. Thus echoing `request_target_id` is request provenance, not object association.
An independent association check remains required.

Default backend deadline: 0.4 s, one outstanding request, maximum eight results.
Forwarded messages are bounded too: at most 8 joint names and 4 points per posture,
8 touch-object names, 128 characters per string, finite numerical fields and
valid approach/retreat directions. All supplied posture arrays must match their
joint-name count, and durations must be nonnegative, ordered and at most 10 s.
These checks bound proposal payload cost; they do not qualify those postures
for physical execution. Deadline and acquisition freshness are checked again
after result materialization immediately before publishing.

The adapter returns standard failure/error codes when disabled, absent, busy,
stale, malformed or timed out. Monitor `/station/intel_grasp/status` (`std_msgs/String`)
for the reason. No latest-TF substitution is performed: request and result frames
must match the configured frame exactly. Backend acquisition stamps must be fresh,
not older than the request's source stamp, and equal across a returned batch.
This intentionally rejects backend output lacking reliable source provenance.

## Validation handoff

Accepted service results retain complete upstream `Grasp` messages. Their poses
also appear on **`/station/intel_grasp/poses`** (`geometry_msgs/PoseArray`) with the
original acquisition header. The companion status explicitly reports
`qualified=false`, `tool_mapping_qualified=false`, `ik_validated=false`,
`collision_checked=false`, `execution_enabled=false` and
`backend_runtime_verified=false`, `eef_parent_link_verified=false`, and
`target_association_verified=false`. Every status includes the machine-readable
`target_id_semantics` explaining that the backend does not establish track
association; a proposal echoes only `request_target_id`. The inspected revision is documentation
provenance, not a claim about the connected backend's binary.

These topics are isolated from `/so101/grasp_candidates`, the active geometric
planner, workflow state machine and arm actions. There is no automatic backend
fallback, candidate replacement, calibration-file write or motion.

Before using proposals for even IK comparison, verify the backend's configured
EEF parent transform against `gripper_frame_link`, gripper closing/approach axes,
actual jaw gap/TCP calibration and camera-to-arm registration. A MoveIt grasp
pose is the grasp endpoint, **not** the pregrasp pose. Derive the pregrasp by
subtracting `pre_grasp_approach.desired_distance * direction` in its stated frame;
then pass that explicit target to the existing station IK/state-validity/planning
path. Do not directly remap the raw PoseArray to the station pregrasp validator.
Physical execution must still pass existing qualification and scene/trajectory
checks. No physical pick/place or upstream detector qualification has occurred.

## Hand-eye calibration

The upstream `grasp_utils/handeye_dashboard` and `handeye_tf_service` are useful
references, but are not launched or installed. This integration does not fit an
unknown tag mount or use repeated unassociated AprilTag ID 0 instances as link
observations. A later hand-eye integration needs unique physical tag association,
known target geometry/mount, synchronized robot/image acquisition, enough excited
poses, a frozen independent held-out pose and a qualified transform direction.

## Focused verification

Run the synthetic ROS tests in an isolated nonzero domain, not the live station
network. They open no camera or serial device:

```bash
ROS_DOMAIN_ID=177 ROS_LOCALHOST_ONLY=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  ros2/conda/so101-conda.sh run python -m pytest tests/test_intel_grasp_adapter.py -q
ROS_DOMAIN_ID=177 ROS_LOCALHOST_ONLY=1 ros2/conda/so101-conda.sh run \
  python tools/benchmark_intel_grasp_adapter.py
```

### Recorded verification (2026-10-19)

- Synthetic native ROS suite: **37 passed, 0 skipped** (0.99 s), with plugin
  autoload disabled. Includes CLI subprocess → adapter → synthetic upstream →
  proposal subscriber, source-stamp preservation, default-off/absent backend,
  full forwarded-message bounds, invalid request/result rejection, busy/timeout
  latch, late-response cleanup, slow admission/result/copy deadlines, nonzero
  domain defaults/rejection and constructor failure cleanup.
- Whole-payload CPU benchmark (900 measured samples after warmup): default batch
  of 8, mean **366.86 µs**, p95 **422.69 µs**; configured maximum batch of 32 with
  maximum posture arrays and 4-byte Unicode strings, mean **2421.97 µs**, p95
  **2770.63 µs**. Serialized response/poses/status: **2461/484/773 bytes** typical,
  **534037/1828/8326 bytes** maximum. These measurements exclude DDS transport and
  backend inference; host load varies. Earlier 13.8 µs timing measured only the
  original narrow validation subset and is superseded by this complete path.
- Static guards reject action clients, camera imports, blocking waits, unbounded
  `while` loops and device synchronization/readback operations in the adapter.
- Default optional launch starts zero nodes. Direct disabled node has no timer.

Configure the external backend to publish in the adapter's exact `frame_id` and
set GPD `num_selected` no greater than `max_candidates` (upstream default is 5).
Oversized batches are rejected rather than silently truncated. Upstream EEF
mapping, target association, detector inference, hand-eye calibration and real
pick/place remain unqualified.

- Independent review replay: 35 new adapter cases plus 2 existing shell tests,
  no blocking findings; typical/max whole-path p95 **441.6/3208.7 µs** with matching
  serialized byte counts. These are CPU wrapper measurements under shared host load.
- Fresh isolated scene build: 1 package in **1.29 s**; installed executable listed
  by `ros2 pkg executables`, actual installed-prefix module import and disabled
  node construction verified (`timer=None`). Existing CMake minimum-version and
  overlay-prefix warnings were emitted. No live process/configuration was changed.
