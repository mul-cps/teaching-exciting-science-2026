# Station startup and shutdown

Run commands from the repository root in the prepared instructor graphical login. The canonical entrypoint is `./station`; `scripts/start.sh`, `scripts/stop.sh`, `scripts/status.sh` and `scripts/purge.sh` delegate to the same commands.

## Prepared native station

The prepared `fwmini4` profile needs its exact installed manifest at `~/.local/share/exciting-science/station/fwmini4-units.json`, matching source pins and disabled project user services. Its roles are owner, camera, tags, movegroup, objects, scene, leader and apis. RViz uses a separate GUI role. [PC preparation](fwmini4-workshop.md) describes installation.

Connect follower **5AB9068906**, leader **5AB9068765** and D435i **243322071783**. Use serial identities, not arbitrary USB/video device numbers. The EV3 native `cps-ev3-api.service` must already run on the brick; starting the host station does not install brick firmware.

```sh
./station doctor
./station start
```

`doctor` checks local paths, calibration identity and expected arm devices. Startup additionally verifies the current overlay, manifest, user manager and EV3 readiness, passively exports real joint limits, starts the selected services and opens RViz. The manifest's source, wrapper, configuration, calibration and domain must agree; a different checkout requires an explicitly regenerated installation.

The wrapper `~/.local/bin/station-fwmini4 ros-vulkan` selects native ROS and the Intel Vulkan environment. Live domain is **76**, with localhost discovery. All physical clients use the manifest domain. **177** is for explicitly isolated software fixtures, including their own manifest. Changing domains does not separate USB ownership or the native EV3 session.

Startup uses observe-only admission and verifies coordinated STOP with fresh matching all-six torque OFF feedback. It does not request HOME, enable teleoperation, wake a sorting job or qualify physical motion. Use the operator panel to request subsequent operations explicitly. Keep support available when disarming a gravity-loaded arm.

```sh
./station start --physical --instructor   # full station + RViz + physical Blockly
./station notebook --live
./station status
```

Use the combined physical command instead of the ordinary `start` command. If the station is already running, `./station blockly --physical --instructor` opens only its Blockly client. Physical Blockly reuses the same ROS station. The live notebook observes it. `status` reports managed ownership records and native unit state; process existence is not a motion-readiness certificate. `./station sim` and `./station notebook` are classroom activities that do not start hardware.

## Shutdown

```sh
./station stop
```

The managed lifecycle requests station STOP and waits for the exact request UUID, `pending:false`, `ev3_stopped:true`, `arm_disarmed:true`, and independent fresh matching six-joint torque OFF observations before stopping owned services and tunnel. It validates unit/process identities before signaling. The brick's independently managed API remains separate.

A missing acknowledgement is an unknown outcome. The command retains ownership evidence and reports failure; inspect the physical station and logs before recovery. Do not replace this procedure with broad process killing. Closing a browser does not shut down hardware. Closing the managed RViz launcher requests its matching shutdown. Separately opened notebooks and other unmanaged applications require their own normal close operation.

## Purge and uninstall

```sh
./station purge
./station purge --yes
./station purge --runtime              # preview generated runtime prefixes
./station purge --runtime --yes        # STOP, then remove generated runtimes
./station purge --uninstall
./station purge --uninstall --yes
```

Purge defaults to a dry run. `--yes` first requires successful managed STOP, revalidates the deletion list, then removes recognized generated build/log/cache directories. `--uninstall` additionally selects exact matching project user-unit and desktop registrations. Changed registrations, symlink boundaries and generated directories containing tracked source are refused. Private credentials, configuration, calibration, saved homes, recordings and external evidence remain outside these purge targets. `--runtime` additionally selects only `ros2/.cache/{conda,realsense-cpu-texture-bounds,rclpy-publisher-gid-7.1.12,rmw-cyclonedds-gid-2.2.4}`; it also defaults to a dry run and excludes unknown/private cache prefixes. Rebuild removed overlays before the next startup.

## Native ROS manual fallback

Use this only for a cold station after the managed deployment is shut down. Never run the integrated launch beside the native units or another follower/camera owner. Prefer the managed command without the GUI when diagnosing:

```sh
./station start --headless
./station stop
```

For a local development workstation, `./station start --profile local --headless` selects the local ROS wrapper and managed integrated lifecycle; its private station paths and connected devices must already be prepared.

To inspect the native recipe without duplicating its deployment settings:

```sh
python3 - <<'PY_MANIFEST'
from pathlib import Path
from tools.install_station_user_units import load_manifest, component_command
manifest = load_manifest(Path.home() / '.local/share/exciting-science/station/fwmini4-units.json')
for role in ('owner', 'camera', 'tags', 'movegroup', 'objects', 'scene', 'leader', 'apis', 'rviz'):
    print(role, component_command(manifest, role))
PY_MANIFEST
```

These commands expose the exact native ROS launch arguments. Starting components individually bypasses lifecycle ownership; it is a developer recovery procedure and requires corresponding reviewed STOP and identity checks. `ros2/conda/so101-conda.sh rviz` starts the **mock** stack. It is not a real hardware fallback.

## Troubleshooting

| Symptom | Action |
|---|---|
| Missing device or calibration | Check the identified cables and private files reported by `doctor`; do not substitute another serial device. |
| Manifest/source mismatch | Stop the existing deployment, rebuild the reviewed checkout and explicitly regenerate matching units/manifest. |
| Existing owner or GUI record | Inspect `./station status`; use the matching managed stop. Preserve unknown ownership for investigation. |
| Active units without a lifecycle record | Inspect the reported exact units and original deployment; use reviewed recovery before starting again. |
| EV3 unavailable | Check the brick's native API and configured SSH identity/tunnel. Never open a second direct control session. |
| RViz/Vulkan failure | Check the prepared native wrapper, Intel Vulkan environment and graphical login. |
| Jaw command rejected | Inspect `/follower/gripper_calibration_status`; SI gap and MoveIt joint calibration are separate. |
| Sorting readiness rejected | Inspect `/station/diagnostics` and `/station/status`; restore real references and qualified calibration explicitly. |
| Shutdown acknowledgement missing | Keep the record, inspect physical state and logs, and follow [emergency reset](station-emergency-reset.md). |

See the [ROS API](ros-station-api.md) for exact service types, asynchronous jobs and reset semantics. Startup is not physical pick/place or automatic-sorting acceptance.
