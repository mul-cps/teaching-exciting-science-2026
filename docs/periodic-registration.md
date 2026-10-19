# Periodic stationary registration diagnostic

This optional ROS-only diagnostic compares the **current expanded URDF visual
meshes** with observed camera depth points and the depth-backed ChArUco world
anchor. It publishes an unqualified bounded SE(3) proposal. It never publishes
TF, changes calibration/model/joint zeros, or calls motion services.

The node is default off. When enabled it attempts one job every30seconds.
`/so101/registration_diagnostic/trigger` (`std_srvs/Trigger`) requests the same
operation, with the same30second cooldown. One job may be collecting/fitting;
requests during a job/cooldown are rejected without a queue. `enable` (`SetBool`)
controls periodic jobs; disabling cancels collection. `reset` (`Trigger`) reloads
the explicit source pins, invalidates the model cache and stationary history.
Results and unavailable reasons are JSON on `status` and `result` in that namespace.
The trigger accepts work asynchronously; its response is not a successful fit.

## Inputs and bounds

- PointCloud2 `/camera/depth/color/points`, native depth `CameraInfo` on
  `/camera/depth/camera_info`, `/follower/joint_states`, transient-local actual
  `/follower/robot_description`, and `/so101_perception/charuco_world/status`.
- Six fresh finite joints: a fully covered0.5second preceding window, no gap over
 100ms and no joint span over0.004rad; nearest cloud/joint/CameraInfo/board stamps
 within50ms. Future inputs are rejected. Board must be `tracking`,
 `observation_valid=true`, `plane_source=aligned_depth`.
- Every TF is looked up at the **original cloud acquisition stamp**. Per-link
 transforms are used directly, with all mesh URI/origin/scale terms preserved.
- Up to12 bounded owner-state file reads20Hz during a single approximately0.6s
 collection. No continuous owner-file polling. Real UTC/monotonic read brackets
 must agree with the measured owner's clocks; measured owner samples must bracket
 exposure within100ms each, reject active trajectory/component/recovery phases,
 match all six raw counts and pinned identity/source
 hashes and the same explicitly pinned nonempty owner_session_id. Body angles
 must agree with ROS. An observed owner restart invalidates the job/cache/history
 until explicit repin/reset, even when raw counts and code hashes are unchanged. Final owner counts and source files are
 rechecked after fitting. ROS and owner UTC clocks must agree. Complete normal
 admitted bounds, raw admission, empty violations and measured torque are required.
 Failed endpoint holds are blocked. Retained components with known `complete`
 status and an actual past completion timestamp are idle history; active, failed,
 unknown or pending-restoration components are rejected.
- Input source identity/model/publisher changes produce `unavailable`; explicit
 repin/reset is required. Raw owner/config/calibration files are read only.
- Up to32visuals/600000binarySTLtriangles are parsed/hash-pinned once outside an
 active job;4000area-distributed link-local samples are cached. Per job, bounded
 per-link transforms and vectorized array work reuse that cache. No vertex or
 observed-point Python loops, STL loads, GPU reads or thread pool in the fitter.
- Front-facing sampled-surface z-buffer provides approximate self-occlusion
 culling. Observed cloud is sampled once to≤4000points, model-near population is
 fixed at≤2000points, and at least40correspondences are needed within20mm.
- One robust fixed-correspondence point-to-plane optimization, at most20function
 evaluations, candidate translation norm≤10mm and rotation norm≤2°. An internal
 wall deadline is checked before stages and **inside every optimizer residual**;
 default/max fit budget200ms. A missed deadline discards the proposal. These are
 cooperative native-call checks, not an OS hard real-time guarantee. Collection
 has a separate700ms wall limit, and final source age must remain≤500ms.

Sample-nearest median/p95 distance, point-to-plane p95, overlap, per-link support,
scaled Jacobian singular values/rank, optimizer/bound state, elapsed time and
source provenance are reported. These are quality metrics, not a generic
confidence percentage or exact-triangle error. Rank-deficient geometry produces
`insufficient_observability` without a candidate. A low residual alone does not
establish correct correspondence or physical calibration. External objects near
the model, partial views, sparse sampling and surface symmetry remain limitations.
No hidden surfaces or missing observations are synthesized.

The board contribution reports the current world/camera transform and drift
relative to the first admitted cohort after reset. The candidate world/base
transform is reported separately. Neither is applied.

## Explicit source cohort

Prepare pins with the installed read-only ROS quick action; hashes and identities
are derived from actual current runtime inputs, never copied from an old model:

```bash
ROS_DOMAIN_ID=76 ROS_LOCALHOST_ONLY=1 ros2 run so101_station_perception registration_capture_sources --output /absolute/current-source.json
```

The one-shot helper listens to the current expanded description, original cloud,
native CameraInfo, joint history and board status. It performs one `DeviceInfo`
request on the unique cloud publisher node and binds its serial to the actual
MessageInfo GID. Exact cloud-stamp base/camera TF is required. Default paths are
the station config, its referenced calibration, the live owner file and
`~/.local/share/exciting-science/station/calibration/robot_base_registration.json`.
Flags can override paths. Collection takes about1.2seconds; DeviceInfo/TF waiting
is bounded by4seconds. Output and sibling `.evidence.json` refuse overwrite.
No actuator services, direct SDK access, source mutation or automatic monitor start
is involved. Owner/config/calibration and canonical base hashes must stay stable.
The actual base TF must match the canonical nonidentity matrix within1e−8.
Model hash and serial are discovered dynamically. An older model label in the
base artifact remains provenance, never rewritten or promoted. A failed hold or
missing paired board can be pinned truthfully, but `fit_eligibility_at_capture`
reports unavailable; the monitor independently rechecks all gates before fitting.

The generated core schema is shown below with illustrative, nonoperational values:

```json
{
  "schema": "so101-registration-source-v1",
  "model_sha256": "actual-expanded-URDF-SHA256",
  "arm_id": "actual-owner-arm-id",
  "owner_session_id": "actual-current-owner-generation",
  "calibration_sha256": "actual-calibration-SHA256",
  "owner_source_sha256": {"arm": "actual-source-SHA256", "arm_session": "actual-source-SHA256"},
  "camera_serial": "actual-DeviceInfo-serial",
  "cloud_publisher_gid": "actual-live-MessageInfo-publisher-GID-hex",
  "station_config": "/absolute/current/station.json",
  "station_config_sha256": "actual-current-station-config-SHA256",
  "base_registration_path": "/absolute/current/base-registration.json",
  "base_registration_sha256": "actual-base-seed-SHA256",
  "world_from_base": [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],
  "calibration_path": "/absolute/current/calibration.json"
}
```

`owner_source_sha256` must include the entire actual dictionary, not just the two
illustrative keys. The serial is a provenance assertion bound by the prepared
artifact to the actual cloud GID; the monitor does not query camera SDK/services
or infer a serial from topic names. It refuses a changed/missing publisher GID.
The artifact SHA, actual owner identity, current model+mesh hashes, station config,
camera GID/serial, original timestamp and monitor source hashes accompany results.
The world/base seed file is hash-checked each job, and actual world/base TF must
match its explicitly pinned matrix within absolute1e−8 (zero relative tolerance).
A mismatch invalidates the cache and stationary history until repin/reset. The
example identity matrix is illustrative and must be replaced by the actual seed.

A changed model, camera publisher or owner generation requires fresh actual pins
and explicit reset. There is no hardcoded model hash or installed default artifact.

```bash
ROS_DOMAIN_ID=76 ROS_LOCALHOST_ONLY=1 ros2 launch so101_station_perception periodic_registration.launch.py enabled:=true source_cohort:=/absolute/source.json owner_state_file:=/dev/shm/actual-owner.json
ros2 service call /so101/registration_diagnostic/trigger std_srvs/srv/Trigger '{}'
ros2 service call /so101/registration_diagnostic/enable std_srvs/srv/SetBool '{data: false}'
```

Domain0 is rejected by the launch file. No browser/AI agent is needed.

## Attached-marker alignment

Existing tag observations are **unassociated ID0 depth centers**, not6DoF
link-mounted observations. They cannot establish a marker-to-model mount. The
normal result is `unavailable_missing_mount_or_observation`.

An optional `marker_mount` file may explicitly supply `schema=so101-link-marker-v1`,
`model_sha256`, `link`, `dictionary`, `marker_id`, `link_from_marker` (proper4×4SE3),
and `qualified` with actual calibration provenance. The file SHA is reported.
The optional `/so101_perception/associated_marker_pose` JSON observation must give
`frame_id` matching the actual cloud optical frame, original finite `stamp`,
`association=unique_id`, `detection_count=1`, matching dictionary/ID and a proper
`camera_from_marker`6DoF pose. It must be at most50ms before cloud acquisition.
The node reports predicted-vs-observed translation/rotation residual and mount
qualification separately. It does not create this association, invent a mount,
fit it from a center point, or promote an unqualified mount. No existing producer
currently supplies that stronger observation contract.

## Flat-ground constraint diagnostics

The user declared the physical base flat on its support plane. The existing
optimizer still produces a free6DoF diagnostic proposal, never an applicable
physical correction. `ground_constraint` reports current/candidate base-up tilt
and, when base_link visuals are available, the transformed origin of the CAD
minimum-z plane. That floor origin is a CAD feature, not an observed floor
measurement. Candidate tilt or floor height above1e−5 diagnostic tolerance is
flagged `candidate_ground_violation`; the candidate remains unqualified/appliedfalse.
This does not constrain the optimizer or silently level/translate its result.
A future XY/yaw-only fitting mode would require a separately approved change.
Explicit marker mounts may refer to any cached actual URDF link, including the
nonvisual gripper_frame_link; at most one extra exact-stamp TF is needed.
