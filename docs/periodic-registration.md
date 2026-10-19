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

## Explicit derived ROCm diagnostics

The direct camera path is unchanged. A private derived cloud is accepted only
when the source-capture CLI **and** the monitor explicitly opt in. There is no
DeviceInfo impersonation or automatic topic fallback. This does not enable
recovery/motion consumers or qualify camera geometry, the TCP, model offsets or
registration. Results remain `qualified=false`, `applied=false`.

After separately starting the reviewed private producer, prepare pins with:

```bash
ROS_DOMAIN_ID=76 ROS_LOCALHOST_ONLY=1 ros2 run so101_station_perception \
  registration_capture_sources --allow-derived-source \
  --cloud-topic /so101_perception/rocm/points \
  --output /absolute/new-derived-source.json
```

The helper keeps native `/camera/depth/camera_info`, adds only native color
CameraInfo and `/so101_perception/rocm/lineage` subscriptions, and queries actual
ROS graph endpoints. It subscribes to no extra raw images. Its own DeviceInfo
request goes to the **actual native camera**, with all four native image/info
publisher GIDs bound before and after the asynchronous request. The derived cloud
publisher never acts as the camera identity service.

The source artifact retains the common `so101-registration-source-v1` fields and
adds an explicit `derived_source` pin. Loading that artifact without
`allow_derived_source=true` fails. To run the optional monitor:

```bash
ROS_DOMAIN_ID=76 ROS_LOCALHOST_ONLY=1 ros2 launch so101_station_perception \
  periodic_registration.launch.py enabled:=true allow_derived_source:=true \
  cloud_topic:=/so101_perception/rocm/points \
  source_cohort:=/absolute/new-derived-source.json \
  owner_state_file:=/dev/shm/so101-sorting-20261016/rviz-arm-state.json
```

Both switches default false; disabled launch creates no monitor. Existing owner,
stationary, model, base/board, ground and marker-mount gates remain required.
Source pin preparation can still record a failed endpoint hold truthfully; it
never admits that state to a fit.

### Source chain checks

Only per-cohort `so101-derived-rocm-rgbd-cohort-v1` lineage is proof input. The
producer's separate heartbeat/status topic cannot substitute for it. Eight
lineage records and bounded native CameraInfo buffers are retained. The newest
cloud must have exact integer-nanosecond original depth-stamp lineage; missing
lineage cannot fall back to an older cloud/status. Sequence and **both** depth/RGB
exposures must advance. A replay does not renew its initial receipt time.

Pins cover producer/source generations, all four native image/info GIDs, both
derived cloud/lineage GIDs, serial/node binding, the fixed six core/worker source
hashes, worker executable/runtime and compatibility hash, and the complete
projection calibration. Native generation is SHA256 of Python's default
`json.dumps(snapshot, sort_keys=True).encode()` for the exact
`{node,endpoints:{depth,color,depth_info,color_info:{node,gid}}}` snapshot. Only
native zero padding of a 16-byte GID into 24 bytes is normalized.

Every selected job independently checks the six actual unique graph publishers,
actual native CameraInfo original stamps/frames/dimensions/K/zero distortion and
exact cloud-stamp depth-to-color TF. Intrinsics compare at float32 precision;
TF tolerances are 2e-6 for rotation entries and 2e-8 m for translation, accounting
for the producer's original float32 calibration rather than fitting anything.
The configuration-file SHA is a **reported immutable pin**, not reconstructed from
a partial JSON calibration subset. No paths reported by lineage are opened.

Original depth, RGB and their native CameraInfo stamps must be positive integers,
not future, at most 500 ms old and paired within 40 ms. Producer identity is
bound to its original request plus 1 s; each reported native stream receipt has
a 250 ms lease. The consumer additionally retains its own actual lineage receipt
clock and rechecks expiry after graph/TF access and before publishing a result.
Publisher/source/core/calibration changes latch unavailable until explicit
repin/reset; reset with obsolete pins does not admit a replacement publisher.
Stale/malformed messages clear usable lineage rather than refreshing old proof.

These are **trusted same-host local ROS provenance assertions**, not cryptographic
attestation or independent proof of image pixels. Original image receipts and
startup source/runtime hashes are asserted by the reviewed producer; graph,
CameraInfo and TF metadata are checked independently. Host monotonic clocks must
be shared. No raw-image subscriptions are added just to duplicate lineage.

Every derived diagnostic result retains the exact selected per-cohort lineage
statement, its initially observed receipt monotonic clock and actual status GID
under `derived_source.lineage`. This includes both original image and native
CameraInfo stamps, sequence, source/identity leases and backend metrics; result
publication never refreshes a lineage receipt.

The launch and both diagnostic CLI mains normalize integer ROS domains, default
to 76/localhost-only, and reject all zero spellings (`0`, `00`, `+0`) or values
outside 1..232 before ROS initialization. Disabled launch still creates no node.
