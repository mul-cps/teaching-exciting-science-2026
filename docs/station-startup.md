# Start the complete instructor station manually

Use this guide for the real SO-ARM101, MoveIt, RViz, RGB-D scene and LEGO EV3. The notebook launchers are separate: they observe the station and do not start its hardware services. These commands describe the source checked on 21 October 2026. Physical pick and place and automatic sorting are **not qualified by starting the scene**.

## Start and stop scripts

For the prepared fwmini4 bjoern login, run these in the checkout (native EV3 `cps-ev3-api.service` must already be active):

```bash
cd ~/git/teaching-exciting-science-2026-current-20261021
tools/start_station.sh --profile fwmini4 --bootstrap-owner --color-mask-backend vulkan
```

The script checks existing station processes, exact arm serial paths/calibration, current launch arguments, packages and the native brick service. `--bootstrap-owner` explicitly exports new passive real-device limits before launching. It starts one complete scene, its SSH tunnel, and performs one non-motion EV3 Connect. It never requests home or enables teleop. Logs stay in the foreground; Ctrl-C requests bounded coordinated STOP before closing its own process group. To stop from another terminal:

```bash
tools/stop_station.sh
```

Use `tools/start_station.sh --profile local --bootstrap-owner` on the local workstation **only after the restored stack is safely shut down**. Existing components cause refusal; use the additive recovery section below to preserve them. `--existing-tunnel` reuses a verified forward. `--background` writes `station-launch.log` under the runtime owner log directory; stop with the same `--record` path if customized. `--config`, `--contract`, `--state-file`, `--station-lock`, `--log-dir`, `--wrapper`, `--wrapper-command`, `--domain` and other options are listed by `--help`. The default record is `${XDG_RUNTIME_DIR:-/dev/shm}/exciting-science/managed-station.json`.

Shutdown checks the saved PID, Linux process start time, session and process group before any RPC or signal. A missing record makes stop a no-op; a stale identity is rejected for manual inspection. STOP RPCs are bounded and both station and follower paths are attempted independently. Missing acknowledgements remain explicit even while owned-process cleanup proceeds. This script only stops its managed launch group, including its SSH tunnel; it does not stop the pre-existing brick service. Inspect physical state if cleanup reports an unknown acknowledgement. Sources: [start](../tools/start_station.sh), [stop](../tools/stop_station.sh), [process ownership](../tools/station_process.py).

## Manual quick start on fwmini4 (bjoern login)

These five steps assume a cold station, the prepared current ROS overlay, identified connected devices and an already running native EV3 brick API. Keep each long-running command in its own terminal. Read the single-owner check below before starting.

1. Set the machine paths (repeat this block in each terminal):

   ```bash
   cd ~/git/teaching-exciting-science-2026-current-20261021
   export STATION_ROOT="$PWD"
   export STATION_CONFIG="$HOME/.local/share/exciting-science/station/station-fwmini4-20261022.json"
   export STATION_PYTHON="$HOME/.local/share/exciting-science/station/venv/bin/python"
   export STATION_STATE="${XDG_RUNTIME_DIR:-/dev/shm}/exciting-science/arm-state.json"
   export STATION_CONTRACT="$HOME/.local/share/exciting-science/station/joint-limits-current.json"
   export STATION_LOCK="${XDG_STATE_HOME:-$HOME/.local/state}/exciting-science/station.lock"
   station_ros() { "$HOME/.local/bin/station-fwmini4" ros-vulkan "$@"; }
   ```

2. Check existing processes and the prepared identities; stop here if another station owner exists:

   ```bash
   pgrep -af 'arm_session|trajectory_server|leader_teleop|station_api|ros_station_api|realsense2_camera|move_group|station_live.launch'
   ss -ltnp 'sport = :18766'
   python3 tools/prepare_workshop_pc.py station-preflight
   ```

   `pgrep` returning no matches is normal for a cold station. The preflight prints a command; it does not launch it. Check D435i serial **243322071783** separately. Follower is **5AB9068906**, leader **5AB9068765**. Never select an arbitrary `/dev/ttyUSB*` or `/dev/video*`.

3. Generate fresh real limits with no running follower owner, then let the passive bootstrap exit:

   ```bash
   "$STATION_PYTHON" tools/freeze_ros_joint_limits.py \
     --config "$STATION_CONFIG" --state-file "$STATION_STATE" \
     --output "$STATION_CONTRACT" --bootstrap-owner --lock "$STATION_LOCK" \
     --log-dir "${XDG_RUNTIME_DIR:-/dev/shm}/exciting-science/limit-export-logs"
   ```

4. Start the EV3 tunnel in a second terminal, only if port 18766 is not already forwarded:

   ```bash
   ssh -N -T -o ExitOnForwardFailure=yes -o BatchMode=yes \
     -o StrictHostKeyChecking=yes -o IdentitiesOnly=yes \
     -i "$HOME/.local/share/exciting-science/station/ssh/id_ev3" \
     -L 127.0.0.1:18766:127.0.0.1:8766 robot@192.168.0.1
   ```

5. Start the full scene in the first terminal:

   ```bash
   station_ros ros2 launch so101_station_bringup station_live.launch.py \
     station_root:="$STATION_ROOT" station_config:="$STATION_CONFIG" \
     station_python:="$STATION_PYTHON" state_file:="$STATION_STATE" \
     station_lock:="$STATION_LOCK" joint_limit_contract:="$STATION_CONTRACT" \
     calibration_dir:="$HOME/git/soarm101-highschool-demos/.calibration" \
     enable_hardware:=true external_state_only:=false rviz:=true \
     camera_repair_prefix:="$STATION_ROOT/ros2/.cache/realsense-cpu-texture-bounds/install" \
     camera_sdk_prefix:="$HOME/miniforge3/envs/so101-jazzy" \
     serial_no:=243322071783 color_mask_backend:=vulkan \
     use_station_api:=true use_operator_panel_api:=true \
     use_ev3_tunnel:=false web_twin_host:=127.0.0.1
   ```

   Leave this terminal open. Startup uses observe-only arm admission and does not enable teleop. The integrated launch disables the resident launch's nested reader and starts one read-only leader. Its explicit `joint_limit_contract` argument forwards the checked contract to the resident launch. Inspect current launch arguments after rebuilding when deploying another revision.

## Choose the machine and environment

On **fwmini4**, use the bjoern graphical login and the checkout above. The prepared `station-fwmini4` wrapper selects the station ROS environment, domain **177**, and localhost discovery; inspect `~/.local/bin/station-fwmini4 --help` for the installed wrapper. Keep the classroom Jupyter venv at `~/.local/share/exciting-science/workshop-20261022/venv` separate from the station hardware venv and ROS Conda environment. The copied calibration JSON and saved home retain their original identities; neither a historical saved home nor old feedback proves today's arm state. No old owner lease or live contract was transferred.

On the **local workstation**, use the same numbered sequence with this path block instead:

```bash
cd ~/git/teaching-exciting-science-2026
export STATION_ROOT="$PWD"
export STATION_CONFIG="$HOME/.local/share/exciting-science/station/station.json"
export STATION_PYTHON="$HOME/.local/share/exciting-science/station/venv/bin/python"
export STATION_STATE="${XDG_RUNTIME_DIR:-/dev/shm}/exciting-science/arm-state.json"
export STATION_CONTRACT="$HOME/.local/share/exciting-science/station/joint-limits-current.json"
export STATION_LOCK="${XDG_STATE_HOME:-$HOME/.local/state}/exciting-science/station.lock"
station_ros() { "$STATION_ROOT/ros2/conda/so101-conda.sh" run "$@"; }
```

The local wrapper defaults to domain **76**; all terminals must use the running station's domain/RMW/overlay. **The workstation currently has a restored custom stack: inspect and reuse it. Do not run the cold-start recipe alongside it.** `/station/operator/*` alone is the arm panel adapter, not the `/station/*` EV3 workflow API. Do not change domains to hide a duplicate owner: separate ROS domains do not separate USB ownership or the native EV3 session.

The shorthand local integrated launch is `ros2/conda/so101-conda.sh station-live` followed by the same explicit launch arguments above. `ros2/conda/so101-conda.sh rviz` launches an explicitly mocked stack and is not this hardware recipe.

### Install/build once before class

A current source checkout does not imply a current installed ROS overlay. On 21 October the local installed overlay exposed older launch arguments than source. With the existing stack safely shut down, build the reviewed checkout and its pinned camera repair:

```bash
ros2/conda/so101-conda.sh build
ros2/conda/so101-conda.sh realsense-cpu-build
ros2/conda/so101-conda.sh camera-preflight
station_ros ros2 launch so101_station_bringup station_live.launch.py --show-args
```

The launch argument list must match this checkout, including the workflow and operator API options. A build/preflight validates software, not connected devices. The desktop **Instructor robot and RViz** runs `python3 tools/prepare_workshop_pc.py station`; its source checks saved calibration/home/USB identities, but at this revision its generated command omits the fresh contract used here. Use the explicit recipe above for the full manual startup. [Prepared machine notes](fwmini4-workshop.md) cover notebooks and desktop launchers.

## Terminal order and what starts

The full launch is the single owner of this composition. Do not separately launch these same components afterward.

| Component | Started by the integrated scene |
| --- | --- |
| Follower owner and trajectory bridge | `moveit_resident.launch.py`; one station Python `arm_session`, measured `/follower/joint_states`, arm/gripper actions |
| MoveIt and robot TF | `/follower/move_group`, robot state publisher and GPU-guarded RViz; this is the resident bridge, not a mock ros2_control controller manager |
| Leader | One read-only calibrated leader publisher; follower teleop requires explicit enable |
| D435i | Pinned CPU-repaired RealSense process, RGB, depth aligned to color, camera info and point cloud |
| Perception | Depth-backed tags, ChArUco/world diagnostics, colored objects, contact/pregrasp observations, sparse surface map |
| Planning scene | Station scene integration into MoveIt |
| Arm operations | `so101_station_operator/operator_api`, `/station/operator/*` |
| EV3/workflow | Sole `so101_station_workflow/station_api`, `/station/*`; connection is explicit |
| Web twin | ROS web twin, open `http://127.0.0.1:8099` locally |

Sources: [integrated launch](../ros2/src/so101_station_bringup/launch/station_live.launch.py), [resident launch](../ros2/src/so101_station_control/launch/moveit_resident.launch.py), [perception](../ros2/src/so101_station_perception/launch/perception.launch.py), [objects](../ros2/src/so101_station_objects/launch/objects.launch.py), [scene](../ros2/src/so101_station_scene/launch/station_scene.launch.py), [web twin](../ros2/src/so101_station_bringup/launch/ros_web_twin.launch.py).

### Real joint limits and URDF

The export command reads a fresh observe-only owner generation and binds calibration, configuration, source hashes, actual firmware limits and admitted bounds. The resident launch validates that contract and applies its calibrated position limits to both robot-description URDF and MoveIt planning limits. Do not hand-edit the upstream URDF or copy an old contract to bypass a cohort mismatch. Motion rejection such as `planning joint limits lack calibrated owner evidence` or `portable model differs from owner limits` requires a fresh export and a planner relaunch with that contract.

With an already healthy idle owner, omit `--bootstrap-owner`, `--lock` and `--log-dir` to export its fresh state without opening a second bus owner. Restarting a planning stack is still an explicit stop/recovery operation. Do not bootstrap while another owner holds the station. Source: [exporter](../tools/freeze_ros_joint_limits.py), [contract validation](../workshop/station/ros_trajectory.py), [bridge admission](../ros2/src/so101_station_control/so101_station_control/trajectory_server.py).

## LEGO EV3: native daemon, tunnel, ROS owner

All three layers are needed. The brick uses medium feeder **A**, large sorter **D**, touch switch **S1**, color sensor **S3**. Its native API binds only brick `127.0.0.1:8766`; the PC uses the tunnel at `127.0.0.1:18766`.

For an already provisioned brick, inspect its existing service before starting anything:

```bash
ssh -o BatchMode=yes -o StrictHostKeyChecking=yes -o IdentitiesOnly=yes \
  -i "$HOME/.local/share/exciting-science/station/ssh/id_ev3" \
  robot@192.168.0.1 'systemctl --user status cps-ev3-api.service --no-pager'
```

If that installed service is inactive and the station is cleared for startup, start the existing service explicitly:

```bash
ssh -o BatchMode=yes -o StrictHostKeyChecking=yes -o IdentitiesOnly=yes \
  -i "$HOME/.local/share/exciting-science/station/ssh/id_ev3" \
  robot@192.168.0.1 'systemctl --user start cps-ev3-api.service'
```

The installer-defined command is `/usr/bin/flock -n /home/robot/cps-station/ev3-live-api.lock /usr/bin/brickrun -r -- /usr/bin/pybricks-micropython /home/robot/cps-station/ev3_native/live_api.py`, working directory `/home/robot/cps-station/ev3_native`. Starting initializes and brakes the motors; it does not home or reset references. Do not run a second daemon, diagnostic motor script or direct native client alongside the ROS owner. Do not run the installer as a routine startup command: it copies files and restarts the service. Source: [installer](../tools/install_ev3_live_api.sh), [native daemon](../tools/ev3_native/live_api.py), [protocol](ev3-live-api.md).

Keep the tunnel terminal open. Alternatively the integrated launch can own it with `use_ev3_tunnel:=true`; choose one method. The separate tunnel is clearer for startup diagnosis.

### Add missing APIs to a restored running stack

First inspect `station_ros ros2 node list` and `station_ros ros2 service list -t`. If the follower, leader, camera and MoveIt are already present, retain them. If `/station/status` and `/station/ev3/connect` are absent and no workflow owner exists, start **only** this in a new terminal:

```bash
export SO101_STATION_ROOT="$STATION_ROOT"
station_ros ros2 run so101_station_workflow station_api --ros-args \
  -p ev3_host:=127.0.0.1 -p ev3_port:=18766 \
  -p sorting_config:="$HOME/.local/share/exciting-science/station/sorting.json" \
  -p evidence_dir:="$HOME/.local/share/exciting-science/station/evidence/sorting" \
  -p ev3_left_end_deg:=-1.0 -p ev3_right_end_deg:=-1.0
```

This starts no arm, camera, leader or MoveIt. If only the arm operations adapter is missing, start `station_ros ros2 launch so101_station_operator operator.launch.py station_root:="$STATION_ROOT" station_config:="$STATION_CONFIG"` once. Load the installed `so101_station_perception/config/station_live.rviz`, or add **Panels → Add New Panel → so101_station_operator/Operations** and **so101_station_operator/EV3Operations** to the existing RViz. An absent MoveIt/camera/leader must be recovered through that deployment's existing launch terminal or configured user unit; do not guess unit names or start the entire integrated stack over surviving owners. [EV3 panel details](ev3-rviz-panel.md).

## Verify services before enabling motion

In a third terminal, repeat the machine path block, then run these non-motion checks:

```bash
station_ros ros2 node list
station_ros ros2 service list -t
station_ros ros2 action list -t
station_ros ros2 service call /follower/status std_srvs/srv/Trigger '{}'
station_ros ros2 service call /station/status std_srvs/srv/Trigger '{}'
station_ros ros2 service call /station/diagnostics std_srvs/srv/Trigger '{}'
station_ros ros2 service call /station/ev3/connect std_srvs/srv/Trigger '{}'
station_ros ros2 topic echo /station/ev3/operator_status std_msgs/msg/String --once
station_ros ros2 topic echo /camera/color/camera_info sensor_msgs/msg/CameraInfo --once
```

Connect reads status; it does not wake, home or move. Inspect successful replies and fresh current owner/calibration/source identity, admitted bounds and all six measured torque states. Expected arm action is `/follower/arm_trajectory_controller/follow_joint_trajectory` (`control_msgs/action/FollowJointTrajectory`); the default gripper action is `/follower/gripper_controller/gripper_cmd` (`control_msgs/action/GripperCommand`). Camera input topics are `/camera/color/image_raw`, `/camera/aligned_depth_to_color/image_raw`, `/camera/color/camera_info`; MoveIt monitors `/follower/monitored_planning_scene`. A visible model alone does not establish live feedback or correct camera registration.

`/station/status` is both a Trigger service and a `std_msgs/msg/String` topic. `/station/ev3/operator_status` and `/station/operator/status` are String topics. Commands below are Trigger services unless specified otherwise. Source: [workflow API](../tools/ros_station_api.py), [arm operations adapter](../ros2/src/so101_station_operator/scripts/operator_api), [default RViz scene](../ros2/src/so101_station_perception/config/station_live.rviz).

## Explicit operator actions

Support the arm before releasing torque. **ZERO TORQUE / STOP** remains independently available during other requests. **Reset (torque OFF)** reconnects only if disconnected, explicitly stops and rearms the configured profile when required; it is not a home motion. **MoveIt mode** disables teleop and returns the owner to idle admission. Plan first in MotionPlanning; Execute is a separate physical action subject to current bounds, collision and trajectory checks.

For leader control, select **MoveIt mode** to disable an active teleop session, choose **Teleop: relative** or **Teleop: absolute**, then explicitly choose **Leader teleop**. Relative mode anchors at the current measured pose. Absolute mode follows calibrated leader joint targets directly with native acceleration smoothing: align the leader to the follower before enabling. Neither mode requires a saved-home move; fresh calibrated leader/current follower state and normal admission are required. **Saved home position** is a separate checked physical move; automatic sorting retains its own home/readiness gates. [Teleop contract](so101-ros-teleop.md).

The new **Restart entire station** button (`/station/operator/emergency_reset`) performs a bounded STOP and passive owner reinitialization plus configured component recovery. **Restart MoveIt** (`/station/operator/restart_moveit`) explicitly STOPs before restarting the configured MoveGroup unit. These require exact deployment user units in both API and bridge allowlists; the consolidated launch's empty defaults do not configure them. Disabled/rejected buttons need deployment configuration, not a guessed unit name. A completed full reset leaves all six joints OFF and still reports `pick_place_ready=false`. Do not delete pending recovery journals after an unknown reply; inspect status and use **Verify restart / recover** for the recorded job. [Operations implementation](../ros2/src/so101_station_operator/scripts/operator_api).

EV3 actions in its RViz panel:

| Action | Endpoint | Physical effect/admission |
| --- | --- | --- |
| Connect | `/station/ev3/connect` | Connection and status only |
| Wake / Sleep | `/station/wake`, `/station/sleep` | Explicit power management; Sleep brakes motors |
| Conveyor home | `/station/ev3/home` | Bounded sorter D/S1 switch motion |
| Left / right end | `/station/ev3/left_end`, `/station/ev3/right_end` | Guarded configured endpoints; defaults `-1.0` mean unsupported until physical mapping |
| Eject one | `/station/ev3/eject` | One bounded LOAD/READY 209° → EJECT 0° cycle, only with established references |
| STOP | `/station/stop` | Coordinated EV3 brake and arm STOP |

Do not infer left/right from recorded interior targets 10°, 132°, 360°, 530°. Zero uses switch homing; 550° is an unqualified boundary and is rejected. The existing conservative six-second SorterPosition phase budget also rejects excessive distances. Feeder reference is a physical assertion, not an automatic homing operation; the panel never manufactures it. For exact custom request types and qualification see [EV3 panel](ev3-rviz-panel.md) and [station API](ros-station-api.md). Do not run automatic cycles merely because these services are available.

## Recovery and shutdown

1. Use the panel STOP, or `station_ros ros2 service call /station/stop std_srvs/srv/Trigger '{}'`. If the workflow API is absent, use `station_ros ros2 service call /follower/stop std_srvs/srv/Trigger '{}'` for the arm. Keep the physical stop accessible.
2. Read current status. Require terminal EV3 stop and arm disarm acknowledgements and fresh all-six OFF before touching hardware. A timeout is an unknown outcome, not successful stopping. The bridge's current STOP path includes its guarded owner fallback; inspect the returned summary and fresh state rather than treating a transport reply alone as proof.
3. Ctrl-C the workflow/full-launch terminal and allow cleanup to finish. **Workflow API shutdown calls the station STOP/disarm path when needed, including arm STOP**; closing an API terminal is a hardware safety action. Do not stop a workflow owner during somebody else's arm motion.
4. Close the SSH tunnel after the workflow has shut down. Close the twin/RViz/remaining launch terminals. Leave the native EV3 service in its established managed state unless an instructor intentionally shuts it down.
5. For restart, repeat the ownership check. Preserve fault/pending journals and inspect the exact failure before reconnecting. Never automatically kill owners, delete lock files, relax limits, replay timed-out motion or restore an old live contract.

Hardware-dependent steps in this guide require a supervised rehearsal on the exact connected station. Documentation verification used source inspection and domain-75 launch introspection only; it did not access USB, actuate arms/EV3, connect to the brick or qualify physical sorting.
