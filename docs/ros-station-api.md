# Public station APIs

The native ROS interfaces below serve RViz, Blockly and custom clients. Start the deployment with [the station CLI](station-startup.md). One resident follower owns serial access, one camera process owns capture, and the workflow API owns one EV3 connection. Clients never acquire these devices independently. All physical clients share the manifest's domain (live **76**); use **177** only for explicitly isolated fixtures.

Names below are defaults; launch parameters can remap configured topics/actions. `Trigger` means `std_srvs/srv/Trigger`, `SetBool` means `std_srvs/srv/SetBool`. Service `success` reports the documented operation or admission result. For asynchronous jobs, acceptance is not completion: track the exact UUID to terminal state and inspect the result/fault. A timeout after possible delivery is unknown, not permission to retry.

## Arm and continuous gripper

| Action | Type | Contract |
|---|---|---|
| `/follower/arm_trajectory_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | Five arm joints, radians and timed points; validates fresh state, bounds, motion profile and collision path before execution. |
| `/follower/gripper_controller/gripper_cmd` | `control_msgs/action/GripperCommand` | Continuous physical jaw gap in **metres**, with qualified SI gap calibration. |
| `/follower/moveit_gripper_controller/gripper_cmd` | `control_msgs/action/GripperCommand` | MoveIt gripper joint in **radians**, with separately qualified joint mapping. |

The five arm joints are `shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_roll`. `gripper` is a separate joint. Gripper actions share the resident owner reservation with arm motion and teleoperation. Missing calibration rejects the corresponding action; raw encoder counts are not SI gap values. Inspect the action's terminal status and `reached_goal`/`stalled` or trajectory error fields. Cancellation retains the executor's existing measured hold semantics; explicit `/follower/stop` disarms.

| Follower service | Type | Purpose |
|---|---|---|
| `/follower/status` | Trigger | Cached owner identity, generation, acquisition clocks, measured bounds and faults; observation, not motion permission. |
| `/follower/gripper_calibration_status` | Trigger | JSON `so101-gripper-calibration-status-v1`: independent SI gap and MoveIt joint calibration flags and units; combined success requires both. |
| `/follower/stop` | Trigger | Priority cancellation/disarm with measured acknowledgement. |
| `/follower/home`, `/follower/prepare` | Trigger | Identity-bound measured HOME/recovery; prepare serializes recovery, home and readiness. |
| `/follower/recover_camera`, `/follower/sorting_ready` | Trigger | Guarded recovery or read-only sorting readiness. |
| `/follower/reconnect`, `/follower/reset_owner` | Trigger | Explicit owner recovery, retaining pending-stop and fault barriers. |
| `/follower/damping_enable`, `/follower/damping_inventory` | Trigger | Native damping and read-only RAM inventory. |
| `/follower/rearm_motion_profile` | Trigger | Explicit profile rearm after required checks. |
| `/follower/operator_maintenance_enable` | `so101_station_operator/srv/OperatorMaintenance` | Token-scoped maintenance reservation: `enable`, `token`, `operation`, exact `unit`; not arbitrary process control. |
| `/follower/gripper_diagnostics`, `/follower/joint_diagnostics` | Trigger | Passive bounded register observations; no calibration or motion authority. |
| `/follower/commission_gripper_step`, `/follower/commission_gripper_held_step` | `so101_station_workflow/srv/CommissionGripperStep` | Commissioning only, signed `delta_counts`, nonzero magnitude at most 16; separate all-off-body and already-held-body admission. |
| `/follower/acknowledge_held_jaw` | Trigger | Explicit guarded held-jaw acknowledgement; not an automatic fault reset. |
| `/follower/calibration_ready` | Trigger | Opt-in supervised calibration observation; requires configured identity-bound target, never certifies collision clearance or writes measured HOME. |

Commissioning services retain fixed bounded profiles, fresh encoder/torque checks, restoration and stop handling. They do not establish a jaw gap/TCP calibration. Stable failed trajectory/jaw endpoints remain failed and latch admission; inspect the typed result and explicitly establish the required STOP/recovery state before another request. See [control contract](so101-control.md) and [gripper registration](so101-gripper-registration.md).

## Leader teleoperation

| Endpoint | Type | Direction/meaning |
|---|---|---|
| `/leader/teleop_pose` | `std_msgs/msg/String` JSON | Read-only leader publisher → follower; stamped identity/sequence-bound pose. |
| `/leader/joint_states` | `sensor_msgs/msg/JointState` | Measured leader joints for observation. |
| `/follower/joint_states` | `sensor_msgs/msg/JointState` | Measured follower joints, not requested targets. |
| `/follower/teleop_status` | `std_msgs/msg/String` JSON | Latched teleop mode/state and admission reason. |
| `/follower/teleop_absolute_mode` | SetBool | `true` selects absolute, `false` relative, only while disabled. Does not move the arm. |
| `/follower/teleop_enable` | SetBool | Explicit enable/disable; enabling captures a fresh admitted cohort. |

Relative mode maps leader changes from the enable-time origin onto the measured follower start. Absolute mode uses the calibrated leader mapping. Disable before changing mode. Stale/out-of-order poses, changed owner generation, faults, limits or an occupied owner reservation reject or stop motion. See [teleoperation](so101-ros-teleop.md) for mapping and operator procedure.

## Workflow and EV3

The workflow publishes `/station/status` and `/station/ev3/operator_status` as `std_msgs/msg/String` JSON. The latter supplies cached telemetry for operator clients. Long jobs return a UUID and report completion through the workflow snapshot; match that UUID rather than accepting any idle state.

| Service | Type | Request and purpose |
|---|---|---|
| `/station/status`, `/station/diagnostics` | Trigger | Cached workflow/device snapshot or read-only configuration/transport gaps. |
| `/station/stop_status` | Trigger | Compact correlated stop snapshot, excluding diagnostic history. |
| `/station/check_ready` | Trigger | Read-only admission job; does not wake EV3 or clear native faults. |
| `/station/run_cycles` | `so101_station_workflow/srv/RunCycles` | `cycles` 1..100; guarded sorting job, with independent pickup/lift/release verification. |
| `/station/stop`, `/station/cancel` | Trigger | Coordinate EV3 braking and follower disarm. |
| `/station/reset_fault` | Trigger | Explicit workflow reset only after worker termination and stop acknowledgements. |
| `/station/arm/prepare`, `/station/arm/home` | Trigger | Asynchronous arm-only wrapper jobs; no EV3 wake/reference changes. |
| `/station/ev3/connect` | Trigger | Explicit connection; no second direct EV3 client. |
| `/station/wake`, `/station/sleep` | Trigger | EV3 wake/power-save jobs. |
| `/station/ev3/home` | Trigger | Bounded touch-switch sorter homing. Does not invent a feeder reference. |
| `/station/ev3/eject` | Trigger | Bounded feeder operation through the existing workflow admission. |
| `/station/ev3/left_end`, `/station/ev3/right_end` | Trigger | Move to configured finite interior endpoints, rejecting absent/invalid configuration. |
| `/station/ev3/sorter_position` | `so101_station_workflow/srv/SorterPosition` | Absolute finite `target_deg`, strictly 0 < target < 550; fixed profile and phase-budget checks. Use explicit home for zero. |
| `/station/ev3/confirm_reference` | `so101_station_workflow/srv/ConfirmReference` | `name`, `reference`, `physically_verified`; only sorter/home and feeder/eject. Requires actual operator verification. |
| `/station/placement/select` | `so101_station_workflow/srv/SelectPlacementRegion` | `generation`, `mode`, integer `pixel_rectangle[4]`; pin/select feeder or goal, or clear. Served by the ROS twin state node. |
| `/station/select_destination` | `so101_station_workflow/srv/SelectDestination` | Idle-only `color` and selected-region UUID. |

Placement uses an exact pinned camera generation, expires stale frames and checks current tracking/calibration before motion. Pin modes are `pin_feeder`/`pin_goal`; selection modes are `feeder`/`goal`, plus `clear`. Do not reuse a selection from an unrelated generation. Qualified camera/base, tool transform, footprint and independent observation data remain required.

`/station/ev3/command` (`so101_station_workflow/srv/EV3Command`) is an internal serialized bridge, not a general motor-write API. Mutations require an internally minted private single-use capability bound to the exact active job and arguments. Knowing a public job UUID grants no mutation authority. Native references and native faults survive host workflow resets. Critical status retains original encoder acquisition clocks; cached or sleeping telemetry cannot prove readiness.

STOP acceptance is not STOP completion. Poll the exact `request_id` returned by the request until `pending:false`, `ev3_stopped:true` and `arm_disarmed:true`. The managed shutdown also independently verifies fresh matching all-six torque OFF. Missing acknowledgements retain the barrier and ownership record.

## Operator maintenance and reset

All `/station/operator/<command>` services use Trigger. Commands are:

```text
emergency_reset reset start base_release base_confirm
teleop teleop_relative teleop_absolute moveit damping zero_torque
restart_moveit restart_camera restart_depth restart_tags restart_all verify_restart
```

They adapt guarded resident operations and exact allowlisted native unit restarts. They never accept a shell command or arbitrary unit. `/station/operator/status` is `std_msgs/msg/String` JSON containing job/admission and observed subsystem state. `start` is an operator arm preparation operation; it is not a replacement for `./station start` deployment startup.

Reset/maintenance bind the current owner generation and exact unit/publisher identities. A changed generation, missing stop proof, absent configured unit, stale stream or invalid native graph fails the operation. Reset does not fabricate fresh calibration, EV3 references or physical acceptance. See [RViz operations](rviz-operation-panel.md) and [emergency reset](station-emergency-reset.md).

## Blockly programs and recording through ROS

The running physical Blockly application also exposes `/station/program/command`, type `so101_station_workflow/srv/StationCommand`. This is a general ROS entrypoint to the same bounded program/recording engine used by the web UI. It is available when that application runs, not merely when the base ROS deployment runs. Simulation can expose it with the server's explicit `--ros-api` option.

Requests contain `operation` and `arguments_json`; responses contain `success`, `result_json` and `message`. Arguments and results are JSON objects with a 65536-byte UTF-8 bound; duplicate fields and nonfinite values are rejected. Operations are:

| Operation | Arguments |
|---|---|
| `status`, `recordings`, `stop` | `{}` |
| `run` | `{"program": [...], "admission": {"session": "...", "epoch": 0}}` |
| `record_start` | `{"name": "stroke_name", "admission": {...}}` |
| `record_stop`, `recover` | `{"admission": {...}}` |
| `operator` | `{"command": "teleop_relative", "admission": {...}}`; command must be allowlisted |

Status exposes `admission={"session":"...","epoch":0}`; `{...}` above means that
exact object. All external mutations require it, except emergency `stop`, which
always accepts `{}`. STOP increments the epoch before cancellation; restart
changes the session. Per-operation cancellation combines application STOP with
transport shutdown and reaches the guarded ROS delivery check. Stale or uncertain
mutations are never refreshed and retried automatically. Read-only status, lists
and Python export require no stamp.

An admitted program runs asynchronously; inspect `status` for the exact program job/result/error. The interface preserves physical/instructor admission and the same cancellation, recovery and generation barriers as HTTP. A successful RPC is not a completed motion.

Recording is read-only acquisition of measured follower arm joints. Physical mode is required. Strokes have a fixed jaw, at least two fresh samples, at most 600 samples and 60 seconds. Capture provenance records arm, calibration, source hashes and owner generation. Durable binding excludes owner generation only; restart loads structure as revalidation-required, and each physical operation checks fresh live identity/cohort. Changed robot/calibration/source identity requires reteaching or explicit revalidation; no silent rebinding occurs. Replay uses the normal trajectory action, checks recorded identity, near-start pose and fixed jaw, and retains collision/motion limits. Use separate calibrated jaw commands for changing gap. Optional skipping of exact stationary pauses retains moving edges and rejects an expanded nominal curve-deviation bound.

Recordings are stored outside Git under `~/.local/share/exciting-science/blockly-recordings`. There is no independent always-running ROS recorder, arbitrary trajectory bypass or automatic authorization from a saved capture generation. See [durable recording checks](python-station-client.md#recording-restart-behavior).

## Blockly HTTP adapter

The browser server binds loopback at `127.0.0.1:8877`. Its authenticated routes are separate from native ROS names. The page receives the station token; host/origin/token checks apply. Do not expose the server as an unauthenticated network controller.

| HTTP route | Payload |
|---|---|
| `GET /api/status`, `GET /api/recordings` | No body |
| `POST /api/program/run` | `{"program": [...], "admission": {"session": "...", "epoch": 0}}` |
| `POST /api/program/stop` | `{}`; no admission stamp required |
| `POST /api/record/stop`, `/api/recover` | `{"admission": {...}}` |
| `POST /api/record/start` | `{"name": "stroke_name", "admission": {...}}` |
| `POST /api/instructor` | `{"command": "<allowlisted command>", "admission": {...}}` |
| `POST /api/simulation/color` | `{"color": "red", "admission": {...}}`; simulation only |
| `POST /api/python` | `{"program": [...]}`; read-only executable shared-client export |

Malformed/unsupported HTTP fields return 400 with an `error`; authentication failures return 403. Program, recording and operator exceptions remain failures, not success-shaped completion. The dedicated STOP listener accepts only program STOP. [Blockly client sources](../web/blockly-station/README.md) document the UI integration.

## Perception, planning and passive observation

The prepared native route publishes `/camera/color/image_raw`, `/camera/color/camera_info`, `/camera/aligned_depth_to_color/image_raw`, `/camera/aligned_depth_to_color/camera_info` and `/camera/depth/color/points` with standard Image/CameraInfo/PointCloud2 messages. Frame IDs, stamps and publisher identity matter. Derived ROCm paths are optional subsystem routes, not interchangeable evidence for native identity checks.

Public observation namespaces include `/so101_perception/tag_detections`, `/so101_perception/tag_observations`, `/so101_perception/charuco_world/status`, `/so101/objects`, `/so101/object_markers`, `/so101/grasp_candidates`, `/so101/grasp_contacts`, `/so101/grasp_status`, `/so101_scene/status`, map/virtual-camera output namespaces, `/tf`, `/tf_static`, and follower robot-description/planning-scene topics. See [perception](so101-perception.md), [objects](so101-objects.md), [scene](so101-scene.md), [map](so101-map.md) and [virtual camera](virtual-gripper-camera.md) for message schemas and configured names.

MoveIt supplies standard services/actions under `/follower`, including `/follower/check_state_validity` and planning-scene services. Display alignment reseed at `/so101_perception/live_tag_alignment/reseed` is a Trigger; it resets passive display alignment, not physical calibration. Optional subsystem interfaces are available only when their nodes are launched:

| Namespace | Interfaces |
|---|---|
| `/so101_perception_recovery` | Trigger `start_recovery`, `recovery_status` |
| `/so101_perception_supervisor` | Trigger `enable`, `disable`, `status`, `reset_fault`, `reconcile_consumers`, `adopt_completed_job`; String `status` topic |
| `/so101_charuco_world`, `/so101_colored_objects` | Trigger `reset_derived_source`, `derived_source_status`, `derived_source_health` for derived-source admission |
| `/so101/registration_diagnostic` | Trigger `trigger`, `reset`; SetBool `enable`; String `status`, `result` topics |
| `/so101_map` | Marker `occupied_surface`, String `status` topics |
| `/so101_scene` | String `attachment_request`, `attachment_status` topics |
| `/so101/synthetic_gripper_camera` | Image `image_raw`, `image_annotated`, `annotation_mask`, `image_scene`, `provenance_mask`; CameraInfo `camera_info`; String `status` |

Derived-source resets, recovery transactions and native resets have different admission contracts. Use their subsystem documentation and configured publisher identities; do not substitute one route for another. Inspect the selected graph with `ros2 service list -t`, `ros2 action list -t` and `ros2 topic list -t` in its actual wrapper environment.

Transport availability, simulated success and a visible twin do not establish physical sorting. Missing camera/base registration, jaw/TCP calibration, placement geometry, physical references or independent cycle evidence remain explicit unsupported gates; see [sorting acceptance](ros-sorting-acceptance.md).
