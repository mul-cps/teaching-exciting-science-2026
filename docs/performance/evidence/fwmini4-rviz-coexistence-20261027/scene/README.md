Temporary RViz coexistence visual scene, 2026-10-27.

The Image display consumes actual resident RGB8 from `/station/resident/frame`.
The custom FrameLease type adapter serializes this as sensor_msgs/Image for DDS
subscribers. No second camera pipeline is started. Robot poses, world cloud and
brick markers are deliberately SYNTHETIC visual load, never measured geometry.

Runtime contents:
- `rviz-coexistence.rviz`: derived station_live.rviz; Grid, TF, RobotModel,
  actual RGB Image, synthetic world PointCloud2 and brick MarkerArray. No station
  operations panels or MotionPlanning. Fixed charuco_board, requested 30 FPS.
  Requested main window 1600x900; RGB is DOCKED on the right, Views below it.
  Requested RGB dock 640x360. Kiosk/window manager can change actual sizes;
  record actual main and image drawable dimensions and screenshot before timing.
- `visual_scene.py`: one native rclpy node; 20,000 XYZRGB points, 320,000-byte
  immutable payload at 10 Hz; four markers at 10 Hz; six joint states at 30 Hz;
  one static charuco_board -> base_link transform. No Image publisher,
  subscriptions, device access, controller, trajectory/action or owner launch.
- `rsp.yaml` / `robot.urdf`: follower model, use_ros2_control=false. All six
  independent nonfixed joints are supplied, initial values are within URDF limits.
- `share/so101_description`: pinned description + mesh assets. URDF uses file://
  paths inside this bundle. Copy the WHOLE directory to the SAME absolute
  `/tmp/rviz-coexistence-scene-20261027` path on the target.

Root orchestration should source native /opt/ros/jazzy/setup.bash and launch
robot_state_publisher, visual_scene.py and native rviz2 as separate processes for
exact per-process CPU accounting, with ROS_DOMAIN_ID=179, ROS_LOCALHOST_ONLY=1,
QT_QPA_PLATFORM=xcb and the authorized private DISPLAY/XAUTHORITY. Example:

    ros2 run robot_state_publisher robot_state_publisher --ros-args -r __ns:=/follower --params-file /tmp/rviz-coexistence-scene-20261027/rsp.yaml
    python3 /tmp/rviz-coexistence-scene-20261027/visual_scene.py --profile vga
    rviz2 -d /tmp/rviz-coexistence-scene-20261027/rviz-coexistence.rviz

`--profile` is only an evidence label; select the real camera's vga/720p/1080p
profile independently. Use identical synthetic scene load for all three.
Do not call rviz_gpu_guard: it forces AMD radeonsi on this Intel target.
`run_scene.sh` is an optional native-environment launcher, not used for PID-bound
measurement. Never use station_live or moveit_resident launch for this scene.

CPU-only validation used local so101-jazzy Python and native ROS message types:

    /home/bjoern/miniforge3/envs/so101-jazzy/bin/python test_scene.py
    /home/bjoern/miniforge3/envs/so101-jazzy/bin/python benchmark_scene.py
    /home/bjoern/miniforge3/envs/so101-jazzy/bin/python -m py_compile *.py
    bash -n run_scene.sh

`test_scene.py` checks YAML, six exact URDF joints/limits, mesh paths, cloud size,
valid frames, no Image publisher, static hot-path allocation/point-loop guards,
and stable message/payload identities over 1,000 updates. `build_bundle.py`
produces Qt dock state using plain offscreen Qt Widgets, never a RenderPanel or
OpenGL context. `cpu-microbenchmark.json` separates helper updates and actual
rclpy serialization, binds source/payload hashes and reports bounded traced
Python memory. It excludes Node/executor/DDS publication and transport, RSP,
actual RGB compatibility copies, RViz, driver and GPU. ROS serialization allocates
its output buffers by design; only application-owned hot payloads are reused.
