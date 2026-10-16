# Offline SO-101 gripper registration

`tools/fit_ros_gripper_registration.py` compares local wrist-roll mount hypotheses against saved RGB-D bundles while holding the recorded base transform and all body joints fixed. It applies the candidate only to the URDF joint from `wrist_roll` to `gripper_link`, then reports both observed-depth-to-CAD residuals on a per-snapshot population frozen from the union of candidate CAD visibility masks and CAD-frontface-to-depth overlap. The last bundle is the held-out pose by default; pass `--holdout-index` to change it.

For the current configured mount of +90 degrees, the default `0,90,180,-90` deltas test total mount rotations of +90, +180, +270, and 0 degrees. A candidate JSON file and held-out image overlay are written to the requested output directory. The overlay marks CAD samples whose projected depth agrees within 60 mm; it is a visual aid, not an acceptance criterion.

The report includes every detected AprilTag 25h9 observation (including repeated code ID 0), depth-derived provisional tag size, track association and planar pose branches. Size and movement are clues for assigning the ~19.4 mm moving gripper tag and ~36 mm stationary EV3-base tag; track IDs are temporal indices, never permanent identity. A constant `gripper_from_tag` transform has an absolute-roll gauge: tag observations alone cannot select one of the constant roll candidates. CAD face geometry and depth returns are required to compare absolute mounting orientation. The station's gripper joint state is an opening-percent proxy, so its current value does not calibrate jaw angle or servo mapping.

This utility is strictly an offline diagnostic. It does not invoke ROS, access hardware, change the live model, or apply a candidate. It does not jointly optimize global pose, body joint zeros, jaw geometry, or tag mounting. Treat all outputs as unqualified until independent review of held-out per-link metrics and overlays. It refuses a bundle set with fewer than three training poses, insufficient independent motion, or a held-out pose too close to training.

## Read-only capture and held-out comparison

`tools/capture_ros_robot_registration.py` records one settled snapshot from
already-running ROS topics. It does not start the camera or command arm motion.
It requires fresh color, aligned depth, CameraInfo, ChArUco tracking, joint
states, the expanded RSP description, and TF. It rejects RGB-D skew above
40 ms, stale data, and joint movement above 0.004 rad over the preceding
250 ms. Capture only after the arm has independently reached a stable pose.

Diagnostic captures remain available without arm-state or camera identity
arguments. They record absent/unmatched provenance explicitly and always set
`qualified: false`. For qualification input, require bound provenance and pin
the live RealSense serial. The strict mode reads (but does not command) the
existing ArmSession state file, the camera's `DeviceInfo` service and
`serial_no` parameter. It requires their identities and calibration hashes to
match the station config, the owner acquisition timestamp to be within 40 ms of
the color frame and no more than 80 ms old, and the five arm joints to agree
with the ROS radians within 0.01 rad. It also records the nearest ROS joint
state timestamp and camera identity query timestamps. The gripper ROS joint is
only a visual proxy: this provenance does not qualify jaw angle, camera/base
registration accuracy, pickup accuracy, or the fitted transform.

Run strict capture against an already-running station, using a new output
directory and the station's explicit pinned serial/state path:

```sh
ROS_DOMAIN_ID=0 ./ros2/conda/so101-conda.sh run \
  python tools/capture_ros_robot_registration.py \
  --output-directory /path/to/new-registration-bundle \
  --station-config /path/to/station.json \
  --state-file /path/to/current/arm-state.json \
  --camera-node /camera --expected-camera-serial PINNED_SERIAL \
  --require-bound-provenance --timeout 30
```

Each new bundle uses capture schema 2 while retaining the top-level fields
consumed by the offline fitters. The `provenance.qualification_inputs_bound`
flag means identity and capture timing were bound; it is not a physical
calibration qualification. Older bundles are not modified or retroactively
marked as identity-bound.

For a comparison, collect at least four snapshots: three distinct training
poses followed by a separate held-out pose. Use the same camera, board
placement, station, and effective robot-description hash throughout. Capture
the held-out pose after selecting the candidate training set, and do not use
its results to pick the roll hypothesis. For example, with the camera and
read-only ROS state already running, a ROS container can capture each pose
without passing through a serial device:

```sh
ros2/scripts/runtime.sh run -- bash -lc '
  set -e
  source /opt/ros/jazzy/setup.bash
  source /ws/install/setup.bash
  cd /ws
  python3 tools/capture_ros_robot_registration.py \
    --output-directory /ws/runtime/evidence/gripper-roll/train-a --timeout 30
'
```

Repeat that command with a new output directory at each settled pose; each
directory must be new. Keep the first three paths as training inputs and the
last path held out. Then run the offline comparison with those paths in order:

```sh
python tools/fit_ros_gripper_registration.py \
  runtime/evidence/gripper-roll/train-a \
  runtime/evidence/gripper-roll/train-b \
  runtime/evidence/gripper-roll/train-c \
  runtime/evidence/gripper-roll/heldout \
  --holdout-index 3 --output /tmp/gripper-roll-comparison --samples 10000
```

The report must retain its `qualified: false` status. A 180-degree correction
requires a consistent held-out improvement across per-link depth residuals and
CAD-to-depth visibility, a clearly separated competing hypothesis, and review
of the held-out overlay. This offline comparison is evidence for review; it
does not promote the result into RSP, MoveIt, RViz, or MuJoCo.
## Jaw-gap action calibration

The resident gripper action accepts jaw gaps in metres only when the bridge is
configured with an independently reviewed file using this schema:

```json
{
  "schema": "so101-jaw-gap-calibration-v1",
  "qualified": true,
  "closed_gap_m": 0.004,
  "open_gap_m": 0.075,
  "closed_percent": 0.0,
  "open_percent": 100.0
}
```

Replace the example gap values with measured values from the station's jaws.
The file must set `qualified` to `true` only after that calibration is reviewed.
The station-live launch passes its absolute path through
`gripper_jaw_calibration:=...`; without it, gripper action goals are rejected.
`max_effort` is required to be zero because the servo has no calibrated force
limit. See [live station startup](so101-live-station.md) for the launch command.

Example:

```sh
python tools/fit_ros_gripper_registration.py \
  evidence/baseline evidence/wrist-roll-plus evidence/elbow-plus evidence/holdout-combination \
  --output /tmp/gripper-registration --samples 10000
```
## Resident `GripperCommand` action

The resident trajectory bridge exposes the standard
`control_msgs/action/GripperCommand` action at
`/follower/gripper_controller/gripper_cmd`. Its `command.position` and result
`position` are jaw opening in metres. Internally, the bridge converts through a
qualified jaw-gap calibration into the station owner's native gripper percent
and then into the identity-matched raw encoder interval. It shares the existing
ArmSession command owner and admission lock; it never starts a second bus owner.

The bridge submits one resident JSON-lines command,
`{"command":"move_gripper","target_raw":ENCODER_COUNT}`. The resident measures
the starting pose and keeps all five body joints held while splitting the jaw
travel into legs of at most 650 encoder counts. The existing 150-count feedback
frame limit, native torque limits, and six-second deadline per leg still apply.
The action reserves the owner across every leg and budgets
`ceil(calibrated_raw_span / 636) * 6.4 + 2` seconds for the complete move; a
1494-count calibrated span therefore has a 21.2-second action budget. The
636-count progress allowance includes the ten-count arrival tolerance and the
four-count dwell allowance.

Closing stops at the first inferred contact, replaces the goal with the measured
pose, verifies the hold, and issues no further closing legs. Cancellation before
submission prevents motion. Cancellation during or between resident legs keeps
the owner reserved until its measured hold is acknowledged, with a two-second
acknowledgement deadline. A missing acknowledgement faults and reserves the
owner. Generic `move_plan` sequences retain their existing behavior.

The bridge requires the `gripper_jaw_calibration` parameter to name a JSON file
with this schema:

```json
{
  "schema": "so101-jaw-gap-calibration-v1",
  "qualified": true,
  "closed_gap_m": 0.004,
  "open_gap_m": 0.075,
  "closed_percent": 0.0,
  "open_percent": 100.0
}
```

`max_effort` must be zero because the servo has no calibrated force limit.
Contact inferred by the resident owner is provisional; pickup requires three
distinct, fresh geometric observations of the same object elevated with the
gripper. The scene lock is acknowledged before opening.

## Sorting grasp execution

The opt-in CLI is `tools/ros_plan_grasp.py --execute --track-id ID
--place-pose X Y Z QX QY QZ QW --execution-config PATH --output PATH`. The
execution config must use `so101-grasp-execution-config-v1` with
`qualified: true`, and name relative or absolute `jaw_calibration`,
`tcp_calibration`, and `camera_arm_calibration` files, explicit `touch_links`,
`open_gap_m`, `close_gap_m`, and the lowercase SHA256 of the effective
`robot_description`. `model_parameter_node` defaults to `/move_group`. The
camera-arm file must carry qualified `camera-arm-calibration-v1` provenance;
the TCP file must be qualified `so101-grasp-tcp-calibration-v1`, match the
requested tip link, and provide two independently calibrated rigid transforms
in SI: `tip_from_grasp` and `tcp_from_object`. It must declare
`grasp_frame_convention: "x_closing_z_approach"`. The grasp origin is the physical
jaw contact center, +X is the closing axis, and +Z points toward the object.
The commanded tool-link pose is `world_from_grasp @ inverse(tip_from_grasp)`;
pregrasp/lift move opposite the approach direction before this composition.
`tcp_from_object` separately describes the held object's pose relative to the
tool link and is used for pickup/release observation predictions. These frames
must not be silently assumed identical.
Motion also requires current `tracking`/`observation_valid` registration
status in ROS time. Camera identity is supplied by the station launch's pinned
camera serial and is not independently attested by the registration status
message.
The CLI checks those files, the current model digest, and all tool/contact link
names before it commands the initial open.

Scene ownership uses JSON requests on `/so101_scene/attachment_request` and
matching request ID acknowledgements on `/so101_scene/attachment_status`. The
scene updater owns the world-to-attached diff, limits temporary contact-matrix
allowance to the selected brick and configured touch links, suppresses stale
tracker updates and TTL removal while attached, and only releases ownership
after a successful detach diff. After lift and release, the CLI requires three
distinct, fresh geometric observations of the same track in the expected frame,
with current Charuco registration observations. A missing lift observation
stops before place and leaves the object attached in the planning scene. After
opening and detaching, the CLI retreats with a collision-checked commissioned
`--return-joint-pose` JSON map (five arm joint names to radians), or a
collision-checked 60 mm move along the TCP tool axis. Release is checked only
after that retreat. `pick_place_complete` is emitted only when pickup, lift,
and release evidence are all true. Software tests do not validate physical
calibration, contact, or hardware execution.
