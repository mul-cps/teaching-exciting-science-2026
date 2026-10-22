# Prepared classroom PC

Use the prepared instructor graphical login. The checked-out project, installed wrapper and source-pinned native manifest must identify the same deployment. Runtime machine state and dated preparation receipts belong in private evidence, not in startup instructions.

## Daily use

Follow [station startup and shutdown](station-startup.md) for all hardware operations:

```sh
./station doctor
./station start --physical --instructor
./station stop
```

For teaching activities without hardware:

```sh
./station notebook
./station sim
```

The application menu provides Student workshop, Instructor live lab, Instructor robot and RViz, Student Blockly simulation and Instructor Blockly PHYSICAL ROS. The notebook launchers do not start the station. Jupyter runs on loopback with authentication; retain its token. Remote viewing uses an SSH tunnel and the authenticated URL from the launch terminal.

## Preparation

```sh
python3 tools/prepare_workshop_pc.py prepare
```

This prepares the classroom Python environment and shortcuts. Initial package preparation requires internet; local notebooks, exercises, images and Blockly work offline afterward. Optional external teaching sites require internet. Classroom Python, station hardware Python, native ROS and Slang/Vulkan are separate environments.

The station configuration, deployment manifest, SSH identity and measured calibration live under private machine paths. The default manifest is `~/.local/share/exciting-science/station/fwmini4-units.json`; its exact wrapper is `~/.local/bin/station-fwmini4 ros-vulkan`. Build the ROS overlay and camera repair before installing a reviewed manifest:

```sh
ros2/conda/so101-conda.sh build
ros2/conda/so101-conda.sh realsense-cpu-build
python3 tools/install_station_user_units.py --help
```

Use the installer help to supply the exact private configuration and deployment parameters. Units are installed disabled; installation is not hardware startup. Live native services, RViz and physical Blockly share domain **76**. Domain **177** is reserved for explicitly isolated software fixtures; it must not silently become the live deployment domain.

The follower, leader and camera serial identities are listed in the canonical startup guide. A copied saved HOME is historical calibration data, not proof of today's physical state. Generate fresh passive limits through managed startup. Keep passwords, SSH keys, calibration JSON, measured homes and evidence out of Git.

Use `./station purge` to preview cleanup, and `./station purge --uninstall --yes` to remove verified project registrations after successful STOP. The [development guide](development.md) describes rebuilding. Physical display behavior, camera throughput, STOP, pickup and release need on-machine rehearsal and their own evidence.
