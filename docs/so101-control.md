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
measured terminal tolerance. The gripper goal is never commanded; its measured
raw position is held in every synchronized position write. For visualization,
the measured normalized opening percent maps linearly to the URDF gripper range
of -10° to 100°. That proxy is explicitly unqualified and never enters the
five-axis command interface.

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
passed. The bridge requires verified torque-off startup; a startup failure
does not qualify the station for execution.

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
