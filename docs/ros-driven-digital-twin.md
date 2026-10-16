# ROS-driven digital twin

`station_live.launch.py` starts the web twin by default (`use_web_twin:=false`
disables it; `web_twin_host`, `web_twin_port`, and `web_twin_python` configure
the viewer). It is a passive display of the same ROS inputs used by RViz:

- `/follower/robot_description`, published by the station's
  `robot_state_publisher`. The web process compiles this effective URDF in
  MuJoCo, including the runtime joint-zero and gripper-mount corrections.
- `/follower/joint_states`, the measured robot state in radians. The MuJoCo
  qpos values are copied by joint name, including the physical gripper angle.
  This path does not use the standalone demo's percentage jaw proxy.
- `/tf` and `/tf_static`, including `charuco_board` to `base_link`, camera,
  tool, and tracked object frames. The robot base pose is taken from TF; no
  camera registration or second joint calibration runs in ROS mode.
- `/camera/depth/color/points`, `/so101/object_markers`,
  `/so101/grasp_contacts`, and `/so101/grasp_candidates` for the scene view.
  The adapter subscribes to the existing cloud and markers and republishes
  nothing.

The ROS process writes atomic snapshots under `$XDG_RUNTIME_DIR` and the web
process reads them. Joint samples older than 500 ms, a missing/stale dynamic
`base_link` TF, or a joint-to-base-TF timestamp delta above 100 ms change the
viewer to `STALE / WAITING`. Cached geometry is hidden while stale. Static TF
edges do not expire. The diagnostics panel reports source, age, timestamp
delta, and fresh frame count.

The station's `charuco_board` frame is already right-handed, metres, and +Z up,
so the viewer uses that coordinate system directly. A camera point cloud stays
in its advertised ROS optical frame and is transformed to `charuco_board`
through the received TF graph; there is no second optical-axis conversion.
MuJoCo also uses metres and radians. Its renderer is passive: no MuJoCo state is
published back to ROS, and no arm command or camera SDK is opened.

Standalone demo mode remains available with `python -m workshop.digital_twin
--snapshot ... --calibration ...`. It uses its snapshot and calibration inputs
and is clearly separate from ROS mode, selected with `--ros-state-file`.

The station installer adds the pinned `digital-twin` extra to the station
environment and verifies the MuJoCo and Viser imports. The web viewer defaults
to `$HOME/.local/share/exciting-science/station/venv/bin/python`; the bridge
continues to use the native ROS/Jazzy interpreter. Preserve that split when
launching the web twin outside `station_live.launch.py`. To repair an existing
station installation without rerunning the hardware checks, install the same
project extra directly:

```sh
root=/path/to/teaching-exciting-science-2026
uv pip install --python "$HOME/.local/share/exciting-science/station/venv/bin/python" \
  "$root[digital-twin]"
```

Override the viewer Python with `SO101_TWIN_VIEWER_PYTHON` or the
`viewer_python` launch argument. The default web port is 8099.
