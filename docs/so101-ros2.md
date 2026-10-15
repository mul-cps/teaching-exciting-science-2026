# SO-ARM101 ROS 2 Jazzy stack

This integration pins the Apache-2.0 [`legalaspro/so101-ros-physical-ai`](https://github.com/legalaspro/so101-ros-physical-ai) stack at `58318c905a2c61289fa907de85cb8473322fbe68`, with its recursive Apache-2.0 Feetech ROS 2 driver submodule at `4c0fdbfe16c84c686f8ace09526c52d98d0110ca`. The upstream tree contains the SO-101 URDF/STL description, `ros2_control`, RViz, MoveIt 2, controller definitions, and the Feetech hardware plugin. The `upstream.repos` manifest pins both commits. Fetched source and STL files stay under ignored `ros2/.cache/upstream/`; they are not copied into this repository.

The supported runtime is ROS 2 Jazzy on Ubuntu 24.04 inside the rootless Podman container. The host is Fedora, so the container avoids relying on a native Fedora ROS/MoveIt binary build. Build and launch the mock stack:

```bash
ros2/scripts/so101.sh build
ros2/scripts/so101.sh rviz
```

The RViz command starts the upstream `follower_moveit_demo.launch.py` through a small overlay that explicitly passes `hardware_type:=mock`, disables cameras, and enables MoveIt RViz. It does not pass a serial device. While that process is running, use a second terminal:

```bash
ros2/scripts/so101.sh inspect
ros2/scripts/so101.sh plan-smoke
```

For the optional Jazzy `moveit_servo::ServoNode` interface, use `ros2/scripts/so101.sh servo`. It starts the mock MoveIt/RViz system, subscribes to `/follower/joint_states`, and routes Servo's `JointTrajectory` output to `/follower/arm_trajectory_controller/joint_trajectory`. No command publisher is started by this launch. Servo scales are deliberately low starting values and have not been validated for real actuation.

`plan-smoke` reads the mock current state, asks PickIK for a near-current `gripper_frame_link` pose while retaining the current orientation as its seed, then plans to the resulting five-joint state. Keeping the query near the current tool pose avoids demanding an arbitrary six-dimensional pose from a five-joint arm. It never calls a trajectory execution API. The resulting ROS `RobotTrajectory` is saved at `ros2/.cache/plan-only-smoke.yaml` for replay review. MoveIt config uses the actual upstream joint names: `shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_roll`; gripper commands use `gripper`. The arm controller is `follower/arm_trajectory_controller`; the gripper controller is `follower/gripper_controller`.

MoveIt planning limits arm motion to ±90° on each revolute joint, with provisional velocity 0.5 rad/s, acceleration 0.8 rad/s², and jerk 2.0 rad/s³ caps, scaled to 25% by default. The gripper has separate conservative provisional dynamics. These are planning-envelope values, not measured hardware ratings. The pinned OMPL pipeline applies Time-Optimal Trajectory Generation followed by Ruckig jerk-limited smoothing, as MoveIt documents for response-adapter order.

## Rig01 calibration identity

`ros2/config/rig01_joint_map.yaml` records the logical joint-to-servo-ID map and compatibility software-zero raw counts from `/home/bjoern/git/soarm101-highschool-demos/.calibration/hs_rig01_follower.json`, SHA-256 `bc688147848f386899f8f34eb09cdb285fcdb0fc8de576b78a50a8efaf7f424c`. The arm zeros are the range midpoints and use the existing `2π/4095` raw-count mapping. They are explicitly unqualified as physical URDF zeroes. The gripper is represented as normalized fraction from raw endpoints 1872–3366; conversion to physical gripper angle is unverified. The file omits homing offsets and is not supplied to the upstream `joint_config_file` parameter.

## Hardware gate

The pinned driver patch adds a default `read_only:=true` mode, suppresses torque/EEPROM writes and command interfaces in that mode, and reads measured positions using an explicitly separate `state_mapping_file`. The mapping file is never passed as `joint_config_file`. The patch and state conversion tests pass, and the driver builds in the Jazzy container. However, the Rig01 software-zero map is still unqualified as a physical URDF mapping, the gripper's angular state conversion remains unverified, and no physical state-only startup has been reviewed here. `ros2/scripts/so101.sh real` therefore remains fail-closed. No physical access or hardware startup is part of this integration.

## Scope and limitations

Upstream supports MoveIt 2 planning with OMPL and `pick_ik`; its checked-in `kinematics.yaml` selects `pick_ik/PickIkPlugin` with approximate solutions. The plan-only smoke checks a small, reachable target generated from a near-zero joint state. It does not qualify arbitrary full-pose IK or physical accuracy. Cameras and MuJoCo are not enabled in the supported mock launch.
