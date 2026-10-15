# SO-ARM101 ROS leader teleoperation

## Integrated ROS launch

Build and source the ROS workspace, then start the follower resident owner,
MoveIt state publisher, and read-only leader ROS publisher together:

```bash
ros2 launch so101_station_control leader_follower_teleop.launch.py enable_hardware:=true
```

The launch starts the follower bus owner only once, through `trajectory_server`,
and the leader publisher only reads the calibrated leader. The combined launch
defaults `enable_hardware` to `false`; pass `true` explicitly to start the
follower hardware owner. It does not enable teleoperation on startup. Wait for
the follower owner and leader reader readiness messages, make sure the
measured follower pose is within the saved `measured-home.json` pose and its
station-configured tolerance (currently 2 degrees/percent), then explicitly
arm teleoperation:

```bash
ros2 service call /follower/teleop_enable std_srvs/srv/SetBool "{data: true}"
```

The bridge captures fresh leader and follower references when that service
acknowledges. Consequently the first mapped command equals the follower's
measured starting pose, and subsequent commands apply only the leader's change
from its captured reference. If the stream becomes stale, the resident owner
holds the measured position and the bridge disables the session; inspect the
status stream and issue a new explicit enable request to rearm:

```bash
ros2 topic echo /follower/teleop_status std_msgs/msg/String
```

The latched JSON status reports active/stopping/disabled state, stop reason,
leader source age, age at the last bridge receipt, and cumulative source
sequence gaps. A stale stop logs the measured age and unchanged 150 ms limit.
The control pose topic uses best-effort, depth-one QoS at both ends so delayed
history is discarded while source timestamps still enforce freshness. To stop motion,
disable first and then stop the ROS launch with Ctrl-C:

```bash
ros2 service call /follower/teleop_enable std_srvs/srv/SetBool "{data: false}"
```

Teleoperation enable checks the fresh owner identity, calibration hash, and all
six measured pose values against the saved home. If the follower is outside
tolerance, the service refuses to start and instructs the operator to call
`/follower/home` or align the arm manually before trying again. Home remains a
separate explicit request; enabling teleoperation never initiates a home move.

For a publisher-only start, use `leader_teleop.launch.py`; it never opens or
starts the follower owner. The same four leader arguments and station Python
and checkout overrides described below are accepted by the integrated launch.

The built-in defaults use Rig01's current station files. For a different
installation, override `station_python`, `station_root`, `station_config`,
`leader_port`, and `calibration_dir`. Startup remains unqualified until the
installed calibration hashes, exact serial identities, and both ready states
have been checked on the target station.

The ROS leader reader publishes calibrated leader state from the exact device
`/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AB9068765-if00` using
`hs_rig01_leader.json` in `/home/bjoern/git/soarm101-highschool-demos/.calibration`.
It runs LeRobot in the configured station Python environment because the ROS
Python environment does not provide that hardware SDK. The reader connects with
the calibration handshake disabled, verifies the saved calibration and all six
leader torque registers are already off, and fails closed if either check fails.
It only reads the leader. Shutdown disconnects without changing torque.

Build and source the ROS workspace, then start the reader:

```bash
ros2 launch so101_station_control leader_teleop.launch.py
```

Override `station_python`, `station_root`, `leader_port`, or `calibration_dir`
when the installation differs. The launch starts only the read-only leader
publisher. It does not enable follower teleoperation. The follower must already
be managed by the resident `trajectory_server` bridge with its hardware owner
ready.

`/leader/teleop_pose` is `std_msgs/msg/String` containing compact JSON schema 1:

```json
{"schema":1,"stamp_s":1234.5,"sequence":12,"units":{"body":"degrees","gripper":"percent"},"pose":{"shoulder_pan":0,"shoulder_lift":0,"elbow_flex":0,"wrist_flex":0,"wrist_roll":0,"gripper":50}}
```

`stamp_s` is the host monotonic acquisition time and `sequence` increases for
each leader sample. This timestamp is a same-host contract; remote ROS hosts
with independent monotonic clocks are unsupported. Body values use calibrated
physical degrees; gripper uses calibrated opening percent. The follower bridge
rejects malformed or older than 150 ms samples, captures fresh matched
references when teleoperation is enabled, and sends mapped follower poses
through its existing ArmSession owner. The mapping is
`follower_start + leader - leader_start`, using the per-joint calibrated
convention above. ArmSession applies the calibrated follower bounds and rate
limits. The bridge provides `std_srvs/SetBool` at `/follower/teleop_enable`;
enable fails while a trajectory action is active and reports the owner start
acknowledgement before returning success. Disable stops the session and holds
the measured pose. Do not publish a synthetic enable request at startup.

`/leader/joint_states` publishes the five body joints in radians. Its `gripper`
joint is a visual-only jaw estimate mapped from normalized percent into the
canonical `[-10, 100]` degree display range; it is not a measured angle and is
never commanded. The JSON pose topic is the teleoperation input contract.
Online teleoperation is joint-space control and does not run MoveIt collision
checks; keep the physical workspace clear and supervise the arm.

The leader ROS node does not own or open the follower serial port. Follower
commands continue through the one resident `ArmSession`, which enforces a
300 ms command watchdog. A lost leader stream therefore expires at the bridge
and resident owner rather than leaving a stale command queued for later use.

The bridge also exposes `/follower/home` as `std_srvs/Trigger`. It invokes the
existing measured-home routine through the same resident owner and succeeds
only after an owner acknowledgement with `home_complete`. The bridge requires
a fresh owner state and reserves exclusive control while homing, so active
trajectory and teleoperation operations are rejected. The call times out after
at most 90 seconds; it does not add a motion profile or a MoveIt collision
bypass to trajectory execution.

For a camera-confirmed calibration observation pose, the opt-in
`/follower/calibration_ready` Trigger is available only when
`calibration_endpoint_mode:=true` and `calibration_ready_pose_file` names an
absolute JSON file containing `arm_id`, `calibration_sha256`, and six integer
raw counts under `raw`. The file identity must match the fresh resident state.
The bridge creates linear sublegs no larger than 100 raw counts per servo and
sends them to the existing `move_plan` owner command; ArmSession still applies
its normal raw limits and transition checks. This explicit observation-pose
move does not run MoveIt and does not establish collision qualification. The
service reserves control exclusively and requires an `ok` owner reply with
`plan_complete` before it reports success.

## Explicit measured-endpoint calibration mode

For a calibration capture that needs the stable measured endpoint even when
the planned goal misses the normal tolerance, start `trajectory_server` with
`calibration_endpoint_mode:=true`. The setting defaults to false. In this mode
ArmSession still waits at least three seconds and only reports
`calibration_measured_endpoint_held` when the measured endpoint stays quiet,
torque remains on, and the maximum joint error is at most 10 degrees. Quiet
means no more than four raw counts of span per joint during the final 0.3
seconds. It retains the measured pose and reports `target_reached: false`;
this is a calibration measurement, not plan success or an endpoint-accuracy
claim.

The ROS action deliberately returns `GOAL_TOLERANCE_VIOLATED` (-5) and saves
the artifact status `calibration_measured_endpoint_held`. The planning client
accepts that one explicit result only when both `--execute` and `--calibration`
are supplied. Its metadata records `target_reached: false` and status
`calibration_measured_endpoint_held`, not
`executed_successfully`. Without `--calibration`, the same action result is a
failure. The mode does not change trajectory tolerances or motion profiles.
