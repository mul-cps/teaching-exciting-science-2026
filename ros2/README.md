# ROS 2 station workspace

The native ROS stack exposes the same resident arm, gripper, EV3, perception and operator APIs to RViz, Blockly and custom clients. Start the prepared physical deployment from the repository root with `./station start`; see [startup and shutdown](../docs/station-startup.md). The source-pinned manifest controls the exact native units, wrapper and domain. No client should create a second serial owner.

## Build

```sh
ros2/conda/so101-conda.sh build
ros2/conda/so101-conda.sh realsense-cpu-build
```

`upstream.repos` pins the upstream MoveIt description and driver sources. `config/rig01_joint_map.yaml` maps joints to servo IDs; it is not an EEPROM configuration. Downloads and build products stay in ignored caches. Private machine configuration and calibration remain outside Git.

## Mock planning

```sh
ros2/conda/so101-conda.sh rviz
```

This explicitly selects mock control; it does not start physical hardware. Keep fixture graphs isolated from live domain 76. Domain 177 is reserved for explicitly configured software fixtures. The prepared hardware wrapper uses native ROS and Intel Vulkan; do not infer a hardware route from a mock controller manager.

## Packages and interfaces

- `so101_station_control`: sole resident follower bridge, trajectory/gripper actions and leader teleoperation
- `so101_station_workflow`: typed EV3 and sorting services
- `so101_station_operator`: RViz panels, maintenance and reset API
- `so101_station_perception`, `objects`, `map`, `scene`, `gpu`: camera observations, derived geometry, planning scene and rendering
- `so101_station_bringup`: compositions for managed hardware and mock planning

See [public ROS API](../docs/ros-station-api.md), [development](../docs/development.md), [environment details](../docs/so101-ros2.md) and [control contract](../docs/so101-control.md). Physical motion remains subject to measured calibration, fresh feedback, collision and commissioning gates.
