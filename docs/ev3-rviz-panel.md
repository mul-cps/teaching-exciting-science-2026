# LEGO EV3 operations in RViz

The default live RViz config loads `so101_station_operator/EV3Operations`
(`so101_station_operator::EV3Panel`) beside Station operations. Build the
`so101_station_operator` package and source its install overlay on the machine
running RViz. Bringup `station_live.launch.py` already starts the existing
`so101_station_workflow/station_api` owner when `use_station_api:=true`.
No new motor driver, socket session owner or physical references are created.
Panel load and ROS subscription send no commands. Connect explicitly connects
and reads status without homing, waking, ejecting or resetting references.

The panel uses these existing owner services plus the narrow eject and endpoint
wrappers, all `std_srvs/srv/Trigger`:

| Button | Service | Behavior |
| --- | --- | --- |
| Connect EV3 | `/station/ev3/connect` | Explicit session connection and status |
| Conveyor home (switch) | `/station/ev3/home` | Existing bounded D/S1 switch home |
| Conveyor left end | `/station/ev3/left_end` | Fixed deployment endpoint through guarded SorterPosition |
| Conveyor right end | `/station/ev3/right_end` | Fixed deployment endpoint through guarded SorterPosition |
| Eject one brick | `/station/ev3/eject` | One native LOAD/READY 209° → EJECT 0° cycle |
| STOP station / EV3 | `/station/stop` | Existing station cancellation, EV3 brake and arm stop |
| Wake / Sleep | `/station/wake`, `/station/sleep` | Existing native power management |
| Reset inspected fault | `/station/reset_fault` | Existing explicit reset and STOP acknowledgement checks |

STOP remains available when status is stale, motion is busy or another command
awaits acknowledgement. A duplicate outstanding STOP is suppressed. Commands
are asynchronous in Qt, service replies expire after five seconds, and terminal
workflow jobs expire after thirty seconds. A timeout means outcome unknown;
inspect the owner status before reloading the panel. Requests are never retried.
An accepted job is displayed as pending until its exact workflow job ID reaches
idle or fault. The owner verifies native session, job identity and completion;
errors remain visible with their reason.

`/station/ev3/operator_status` is a depth-one compact projection published every
0.5 seconds. It includes connection, receipt freshness, native session, encoder
positions, references, S1 switch, S3 color, power, busy/fault, STOP pending and
per-action capability reasons. It excludes workflow event lists, result objects
and native timing histories. The GUI rejects frames larger than 8192 bytes and
expires status after two seconds. Motion admission still uses the existing
native telemetry guards; panel capability display does not replace admission.

## Endpoint configuration and feeder reference

`station_live.launch.py` exposes `ev3_left_end_deg` and `ev3_right_end_deg`
(double ROS parameters on `station_workflow`), each defaulting to `-1.0`, meaning
configuration required. The current repository records sorter interior positions
10°, 132°, 360° and 530°, but does not identify which physical endpoint is left
or right. Do not infer direction from those values. Set these parameters only
after mapping physical directions on this exact station. The panel has no raw
motor target entry. Both endpoint wrappers use the existing conservative
`SorterPosition` path, including its same-session stationary telemetry, home
reference, six-second phase budget and terminal position checks. Zero must use
switch homing; 550° is an unqualified guard boundary and is rejected. Distances
outside the existing six-second budget are rejected with the existing reason.

There is no existing native feeder homing operation. Eject requires the current
session's physically established `feeder/eject` reference and `sorter/home`.
The existing `/station/ev3/confirm_reference` service requires explicit
`physically_verified: true`; the panel never establishes this assertion.
The eject wrapper additionally requires advancing fresh stopped telemetry via
`require_fresh_stationary`, then uses the existing native bounded cycle and
terminal job checks. Component feeder stroke calibration is recorded in
`docs/ev3-feeder-calibration-2026-10-14.json`; complete sorting and final landing
remain separate acceptance criteria.

## Local verification without hardware

Use stock CycloneDDS, `ROS_DOMAIN_ID=75`, `QT_QPA_PLATFORM=offscreen` and the
conda ROS wrapper. Build `so101_station_operator` with `BUILD_TESTING=ON`, source
`ros2/install/local_setup.bash`, then run
`ros2/build/so101_station_operator/ev3_panel_load_check`. The plugin factory test
checks nine actions, only STOP enabled at construction, no startup service calls,
and an acknowledged STOP against a fake service while telemetry is stale/busy.
Run `python -m unittest discover -s tests -p test_ev3_operator.py` for projection
and capability checks. These checks do not actuate physical hardware.

## Add the owner to an already running arm/camera stack

Start only the existing workflow executable after building and sourcing
`so101_station_workflow` and `so101_station_operator`:

```bash
export SO101_STATION_ROOT=/absolute/path/to/teaching-exciting-science-2026
ros2 run so101_station_workflow station_api --ros-args \
  -p ev3_host:=127.0.0.1 -p ev3_port:=18766 \
  -p evidence_dir:=/absolute/path/to/ev3-panel/evidence \
  -p ev3_left_end_deg:=-1.0 -p ev3_right_end_deg:=-1.0
```

Use the running stack's ROS domain and RMW configuration. This executable owns
one EV3 session and uses the existing `/follower/stop`; it starts no follower,
leader, camera or MoveIt process. Keep one workflow API owner per station. The
native brick API listens on brick localhost 8766, reached by the existing SSH
forward on host localhost 18766. Endpoint mapping and sorting configuration are
not needed for read-only connection/status; missing sorting configuration
continues to reject sorting. Startup does not automatically connect. Explicitly
call `/station/ev3/connect` or press Connect EV3 for status. **Owner shutdown
retains its existing station STOP/disarm behavior**, so shutdown is an actuator
safety action rather than a read-only API operation.

For an N100 installation, deploy the same source revision (the workflow wrapper
imports Python helpers from `SO101_STATION_ROOT`), build those two ROS packages,
source the resulting overlay, then load the installed
`so101_station_perception/config/station_live.rviz` config or add the
`so101_station_operator/EV3Operations` panel manually. Rebuild/install
`so101_station_perception` if deploying the updated default RViz config. Keep
existing robot/camera services running and start only the workflow owner if it
is absent. No brick firmware installation is required by this panel change.
