# ROS 2 Jazzy SO-ARM101 stack

Run `../ros2/scripts/so101.sh build`, then `../ros2/scripts/so101.sh rviz` to open the pinned upstream MoveIt/RViz stack with explicit mock `ros2_control`. In a second terminal, `../ros2/scripts/so101.sh inspect` shows the live graph and `../ros2/scripts/so101.sh plan-smoke` runs near-current PickIK, plans a small five-joint move, and saves its trajectory without executing it.

`upstream.repos` pins `legalaspro/so101-ros-physical-ai` and its Feetech driver submodule. `config/rig01_joint_map.yaml` is the read-only joint/servo-ID mapping; it is not a Feetech EEPROM configuration file. Source fetches, meshes, and build products remain in ignored caches. See [`docs/so101-ros2.md`](../docs/so101-ros2.md) for host/container setup, calibration provenance, controller names, and safety limitations.

Real hardware startup currently fails closed. The pinned driver's initialization writes EEPROM fields and changes torque state. A driver-level no-write mode and station hardware review are required before a real launch can be enabled.
