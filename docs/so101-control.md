# Resident MoveIt trajectory control

ROS is the station control and measured-state interface. The ROS action bridge
keeps the Python `ArmSession` subprocess as the sole Feetech bus owner; it does
not open the serial port or import LeRobot. MoveIt sends complete
`FollowJointTrajectory` goals through that bridge. `/follower/joint_states`
publishes the measured five-axis arm state and visual jaw estimate, while the
runtime JSON file is private IPC between the bridge and its owner.

MoveIt point velocities and accelerations are preserved in the command audit,
but are not used directly: planner output may contain discontinuous
accelerations. The owner builds a C2 quintic through the requested positions
and timestamps, with zero velocity and acceleration at the endpoints. Before
any owner command, the bridge asks MoveIt's `/follower/check_state_validity`
service to validate every effective 20 Hz command state and a bounded
interpolation with no adjacent joint step above 0.01 rad. It saves the checked
path and original goal before sending anything. The owner independently
rebuilds the spline from fresh measured state and requires its SHA-256 to match
the collision-checked path before any profile or goal write. Missing validity
service, collision, path mismatch, or dynamics-limit violation rejects the
goal without an actuator command. The software limits are 0.125 rad/s,
0.2 rad/s², and 2 rad/s³. They are admission limits, not measured hardware
performance guarantees; unsafe timing is rejected rather than silently sped up.

Only the five arm joints are accepted, in any order. The first point must be
at time zero and agree with fresh measured state within one degree. Non-default
client path tolerance, goal tolerance, or goal-time tolerance is rejected
instead of being silently ignored. The action uses the station's fixed 2°
measured terminal tolerance. Arm trajectories hold the measured gripper raw
position in every synchronized write. Gripper execution uses a separate
controller described below; it is not a sixth arm-trajectory joint. Without a
qualified joint-angle calibration, its visual feedback retains the explicitly
unqualified -10° to 100° opening-percent proxy and MoveIt gripper execution is
rejected.

The action server must run on the host ROS 2 Jazzy environment. It launches the
configured station virtual-environment Python as a subprocess, with ROS,
conda, and injected Python/shared-library paths removed from the child's
environment. Set `station_root` to this checkout so the station package is
importable. The child takes the canonical station lock itself. Do not wrap
this launch in another process that already holds that same flock.

Build the ROS workspace with `ros2/scripts/so101.sh build`, then launch the
real planning stack explicitly with the station's commissioned config and
virtual environment:

```bash
ROS_DOMAIN_ID=42 ros2 launch so101_station_control moveit_resident.launch.py \
  enable_hardware:=true \
  station_python:=/absolute/path/to/stationvenv/bin/python \
  station_root:=/home/bjoern/git/teaching-exciting-science-2026-ros-arm-control \
  station_config:=/home/bjoern/.local/share/exciting-science/station/station.json
```

The launch includes `robot_state_publisher`, `move_group`, RViz, and the
ArmSession action bridge. It starts no `ros2_control` or mock hardware
controller. Hardware remains disabled unless `enable_hardware:=true` is
passed. The resident launch defaults to `startup_observe_only:=true`: the owner
reads measured encoders and torque without writing goals, torque, profiles, or
calibration registers. Fresh joint states and bridge APIs remain available when
the measured pose is outside the motion bounds. Genuine bus or serial startup
errors still fail visibly; observation does not authorize execution.

Support the arm and manually align it if the measured pose is outside the bounds.
Then use **Reset station (torque OFF)** for the configured ordered station
restart and fresh acquisition checks; see [Emergency station reset](station-emergency-reset.md).
It leaves torque OFF and does not rearm a profile or home. The existing
`/station/operator/reset` API retains its profile-reset behavior and requires a
fresh normal pose. Use **Saved home position** next to perform the measured home
sequence and any required profile rearm. After home
completion, use ordinary MoveIt planning and execution. **Verify restart / recover**
also performs an explicit verified STOP; a subsequent **Start** applies any
required profile rearm before homing. Start alone rejects observation mode.
Neither launching nor recovering observation automatically enables torque or
motion. Direct bridge users retain the internal `startup_observe_only=false`
default; the observation-first default belongs to this resident launch.

The live state JSON and ArmSession log default to `XDG_RUNTIME_DIR` or
`/dev/shm`; persistent paths are rejected for these high-rate runtime files.
The bridge writes the original goal and the collision-checked effective path
to `trajectory_log_dir` before motion, then appends the result after completion.
The owner buffers measured trajectory samples and writes them after motion, so
durable storage latency is outside the tick loop. Each action has one owner
and one active goal. Action cancellation commands a measured-pose hold while
preserving torque. The explicit `stop` command retains the station's existing
torque-off cleanup behavior; shutting down the action server sends `exit` and
therefore also disables torque and restores saved RAM settings.

`external_state_only:=true` is a temporary diagnostic mode for mirroring an
already-running resident. It starts no owner and rejects all action goals; the
primary path starts the ROS-owned resident instead. The pure trajectory and
fake-bus tests do not qualify the ROS action endpoint, collision-scene setup,
physical tracking, smoothness, cancellation, or replay. Those require separate
host and hardware acceptance.


### Continuous MoveIt gripper execution

The resident launch exposes `moveit_gripper_controller` to MoveIt's simple
controller manager as a `GripperCommand` handle for the revolute `gripper` joint.
Select the gripper planning group in RViz and use the Joints tab for intermediate
positions; `open` and `closed` are presets, not binary joint limits.

There are two deliberately separate action interfaces:

| Endpoint | Position, feedback and result units |
| --- | --- |
| `/follower/gripper_controller/gripper_cmd` | Calibrated jaw gap in metres |
| `/follower/moveit_gripper_controller/gripper_cmd` | Calibrated URDF joint angle in radians |

The second endpoint follows MoveIt's single revolute-joint controller-handle
convention. It converts radians to calibrated station percent and then into the
existing metric action execution path. Both reject nonzero `max_effort` because
physical force is not calibrated. A GripperCommand executes an endpoint with the
owner's bounded native legs; it does not promise FollowJointTrajectory timing.
Arm and gripper commands share one owner reservation and cannot run concurrently.

Supply both `gripper_jaw_calibration:=/absolute/gap.json` (existing qualified
`so101-jaw-gap-calibration-v1`) and
`gripper_joint_calibration:=/absolute/joint.json`. The latter must contain
physically measured and reviewed values, using this shape:

```json
{
  "schema": "so101-gripper-joint-calibration-v1",
  "qualified": false,
  "arm_id": "replace-with-measured-arm-identity",
  "calibration_sha256": "replace-with-64-character-calibration-sha256",
  "closed_joint_rad": -0.1,
  "open_joint_rad": 1.5,
  "closed_percent": 0.0,
  "open_percent": 100.0
}
```

The example is intentionally unqualified and cannot enable execution. Angles
outside the actual loaded URDF limits are rejected; calibration never stretches
or clamps the model. Joint feedback uses this same measured mapping once loaded.
`/follower/gripper_calibration_status` (`std_srvs/srv/Trigger`) reports which
calibrations are loaded without commanding hardware. Loading calibration is
necessary but does not assert physical station commissioning or pickup success.

Before an angular command is dispatched, the bridge requires fresh, torque-on,
normal owner state with teleoperation disabled. It checks every integer encoder
sample between the measured jaw start and endpoint against the complete MoveIt
collision scene, using qualified joint angles and the measured held body. The
sweep has a bounded deadline and supports cancellation before submission. The
owner checks the exact six-joint raw start and session identity again before any
profile or goal write, closing the preflight-to-dispatch race. For this bound
command only, gravity trim is disabled, all five body goals remain exactly the
certified values through movement, contact and cancellation holds, and body
drift over two encoder counts stops execution. Legacy metric commands retain
their existing native-leg behavior. Endpoint quantization selects the nearest
integer inside both calibrated and admitted raw intervals; it never clamps an
out-of-range angle into admission. Feedback uses the fresh source- and
generation-bound resident cache without additional snapshot-file reads.
Cancellation after
submission retains the existing measured-hold acknowledgement handshake. The
metric API remains compatible; this new collision sweep applies to the angular
MoveIt endpoint.
