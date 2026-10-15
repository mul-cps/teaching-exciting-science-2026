# Resident MoveIt trajectory control

The real action bridge keeps the existing Python `ArmSession` as the only
Feetech serial owner. A ROS 2 `FollowJointTrajectory` action server sends one
complete goal to that child process over JSON-lines stdin. It never opens the
serial port or imports LeRobot. The resident owner validates the whole goal
before writing, streams a C2 quintic position path at 20 Hz with the station's
native servo profile, publishes fresh measured state, and holds the terminal
pose until measured error is within tolerance. It does not stop at each MoveIt
waypoint.

MoveIt point velocities and accelerations are preserved in the command audit,
but are not used directly: planner output may contain discontinuous
accelerations. The owner builds a C2 quintic through the requested positions
and timestamps, with zero velocity and acceleration at the endpoints, then
checks interpolated position, velocity, and acceleration bounds before any
profile or goal write. Timing is retained; trajectories that exceed the
station software limits are rejected rather than silently sped up. The
current configured software limits are 0.125 rad/s and 0.2 rad/s². These are
software admission limits, not measured hardware performance guarantees.

Only the five arm joints are accepted, in any order. The first point must be
at time zero and agree with fresh measured state within one degree. The
gripper goal is never commanded; its measured raw position is held in every
synchronized position write. `/follower/joint_states` publishes the five
measured arm joints. It omits gripper because the station's normalized jaw
state has no qualified URDF radians mapping. MoveIt planning and visualization
therefore cover the arm only.

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

The live state JSON defaults to `XDG_RUNTIME_DIR` or `/dev/shm` so the 20 Hz
state refresh avoids slow durable storage on the feedback path. Set `state_file`
only to a similarly low-latency runtime filesystem. The durable log stores the
original goal and sampled command before motion; measured feedback is buffered
in memory during the run and appended after motion. Each action has one owner
and one active goal. Action cancellation commands a measured-pose hold while
preserving torque. The explicit `stop` command retains the station's existing
torque-off cleanup behavior; shutting down the action server sends `exit` and
therefore also disables torque and restores saved RAM settings.

These tests exercise trajectory math and the owner against a fake bus. The
real ROS action endpoint, station-specific mapping, physical tracking,
smoothness, cancellation, and persistent MoveIt replay still require separate
host/hardware acceptance.
