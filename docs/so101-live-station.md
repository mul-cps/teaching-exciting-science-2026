# Live ROS station

ROS is the primary interface for measured joint states, camera topics, marker observations, TF, planning and trajectory execution. The lightweight demo viewer remains optional and can run without ROS. When ROS is active, its evidence export feeds the demo rather than opening another camera or arm controller.

The native RoboStack environment runs MoveIt, RViz and the official RealSense ROS driver. The LeRobot station process remains the sole arm-bus owner. See [Conda installation](so101-conda.md), [arm control](so101-control.md), and [camera/calibration](so101-perception.md).

From the checkout root, build once:

```bash
ros2/conda/so101-conda.sh create  # only if the dedicated environment is absent
ros2/conda/so101-conda.sh clean-build
```

`clean-build` recreates only this checkout's ROS `build`, `install`, and `log`
prefixes. The pinned upstream checkout remains in the ignored source cache.
The `so101_station_perception` runtime dependency installs its Python package,
including `charuco_plane`, into this overlay. The station venv remains the
resident follower owner's Python, the dedicated Conda prefix supplies ROS and
perception, and `web_twin_python` selects the separate viewer Python
environment. The wrapper removes inherited ROS and
Python paths before every command, chooses CycloneDDS by default when available,
and honors an explicit `RMW_IMPLEMENTATION` override.

For the integrated startup, use `ros2/conda/so101-conda.sh station-live`.
That launch includes one resident follower owner, one read-only leader
publisher, and one RealSense camera owner. The leader publisher does not
enable teleoperation. It does not authorize startup on the physical station;
commission the saved measured home first. Teleop's resident service continues
to reject a start unless fresh measured joint feedback is within the saved-home
tolerance.

For a temporary read-only mirror while an existing resident arm controller is running:

```bash
ROS_DOMAIN_ID=42 ros2/conda/so101-conda.sh run ros2 launch \
  so101_station_bringup station_live.launch.py \
  external_state_only:=true enable_hardware:=false \
  camera_arm_calibration:=/path/to/camera-arm-calibration.json \
  camera_target_calibration:=/path/to/camera-target-calibration.json
```

The default arm snapshot is `$XDG_RUNTIME_DIR/exciting-science/arm-state.json`. Set `state_file` to use another location. Only one process may own the RealSense SDK; stop a direct SDK recorder before starting this launch. The optional `evidence_folder` must be new or empty and records ROS camera data without opening a second SDK pipeline.

The scene shows measured arm states, camera data and marker observations. The combined launch explicitly attaches the supplied camera calibration estimate at `camera_link`, preserving the RealSense internal TF tree. Candidate status stays provisional; supplying an estimate does not qualify its physical accuracy. Set `attach_candidate:=false` to keep an unqualified estimate isolated in the diagnostic subtree. The gripper jaw angle is a labelled rendering approximation; arm trajectory commands contain only the five calibrated body axes.

The normal ROS-owned launch defaults to `external_state_only:=false enable_hardware:=true`; it connects without automatically moving the arm. Release the existing resident station lock before starting it. The ROS bridge then starts the station-venv child itself; do not run another serial owner alongside it. Incoming and effective trajectories, measured feedback and results are retained for replay review. See the control guide for execution validation and current limits.

RViz must use the active desktop's `DISPLAY` and `XAUTHORITY`; inherited values from an older login session can open a window on an inactive desktop. The lab currently has both `:0` and `:1` Xwayland sessions, so select the active session before launch rather than assuming `:0`.
