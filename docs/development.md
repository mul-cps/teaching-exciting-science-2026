# Development

Use [the project overview](../README.md) for daily teaching and operation, [startup](station-startup.md) for lifecycle commands and [the ROS API](ros-station-api.md) for client integration.

| Path | Responsibility |
|---|---|
| `station`, `scripts/` | Small operator entrypoints |
| `tools/station_cli.py`, `tools/station_process.py` | Managed lifecycle, ownership and cleanup |
| `tools/install_station_user_units.py` | Source-pinned native deployment recipe |
| `ros2/src/` | ROS packages, actions, services and panels |
| `workshop/station/`, `tools/ros_station_api.py` | EV3 session and asynchronous workflow |
| `workshop/blockly_*.py`, `tools/run_blockly_station.py`, `web/blockly-station/` | Bounded Blockly programs, recording, ROS clients and local UI |
| `notebooks/`, `exercises/`, `workshop/` | Classroom notebooks and Python simulation |
| `docs/` | Maintained operator and subsystem documentation |

Build the native ROS overlay with `ros2/conda/so101-conda.sh build`; see [ROS workspace](../ros2/README.md). Prepare classroom dependencies with `python3 tools/prepare_workshop_pc.py prepare`. New checkout paths require matching source-pinned unit installation. Credentials, calibration, configuration and evidence stay outside Git.

Use focused checks for the changed component. Python tests live in `tests/`, component tests under ROS package `test/` directories, and EV3 fake checks under `tools/ev3_native/tests/`. Test commands that launch ROS must explicitly select isolated domain 177; do not attach fixtures to live domain 76 or acquire physical devices. Software checks and simulation do not qualify physical operation.

General clients call the resident public ROS services/actions. Blockly's HTTP server is a loopback application adapter, not a replacement ROS controller. Preserve explicit hardware admission, owner-generation checks, fresh feedback, coordinated STOP and unknown-delivery barriers when extending either layer.
