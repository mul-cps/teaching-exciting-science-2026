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
wrappers, `std_srvs/srv/Trigger` except the typed reference service:

| Button | Service | Behavior |
| --- | --- | --- |
| Connect EV3 | `/station/ev3/connect` | Explicit session connection and status |
| Conveyor home (switch) | `/station/ev3/home` | Existing bounded D/S1 switch home |
| Conveyor left end | `/station/ev3/left_end` | Fixed deployment endpoint through guarded SorterPosition |
| Conveyor right end | `/station/ev3/right_end` | Fixed deployment endpoint through guarded SorterPosition |
| Record feeder eject reference | `/station/ev3/confirm_reference` (`ConfirmReference`) | Explicit physical EJECT assertion after default-No confirmation |
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
(double ROS parameters on `station_workflow`). The integrated launch uses **left
10° and right 530°** for this station: camera and encoder checks on 2026-10-21
confirmed that increasing motor D moves the carriage toward the follower arm.
The standalone API retains unconfigured `-1.0` defaults, so pass the two values
explicitly when launching it alone. Recheck direction if the mechanism is rebuilt
or reversed. The [commissioning record](ev3-panel-commissioning-2026-10-21.json)
lists actual targets, measured results and workflow IDs. The panel has no raw
motor target entry. Both endpoint wrappers share the existing conservative single-phase sorter
implementation. Each endpoint runs at most three phases of at most 200° under
one workflow job and a 29-second overall deadline. Every phase independently
requires advancing same-session stationary telemetry and the home reference,
uses the unchanged six-second phase budget, and verifies its exact native job ID
and terminal position. Cancellation and deadlines are checked between phases.
The original typed `SorterPosition` service remains a single phase. Zero must use
switch homing; 550° is an unqualified guard boundary and is rejected. Typed single-phase distances outside the existing six-second budget continue
to be rejected with the existing reason. Endpoint routes stop after any failed
phase and never continue to the next target.

Power-save is a connected idle state. Explicit Home, left/right or Eject clicks
wake the brick before checking fresh motion telemetry. Panel load, connection
and status polling do not wake it or move motors. Feeder-reference confirmation
uses the native foreground command's existing wake behavior.

There is no existing native feeder homing operation. Eject requires the current
session's physically established `feeder/eject` reference and `sorter/home`.
The existing `/station/ev3/confirm_reference` typed service requires explicit
`physically_verified: true`. The Record feeder eject reference button asks the
operator to manually position the stopped pusher fully EJECTED and physically
verify that pose. The dialog defaults to No. Declining sends no request; Yes
sends exactly `name: feeder`, `reference: eject`, `physically_verified: true`
through the existing service. This records the current position as zero and
does not move or home the feeder. Connect, Wake and panel load never establish
this reference. The status area explicitly explains a missing feeder reference.
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
checks ten actions, only STOP enabled at construction, no startup service calls,
an acknowledged STOP against a fake service while telemetry is stale/busy,
zero reference requests after declining the dialog, and the exact typed physical
acknowledgement after accepting it.
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
  -p ev3_left_end_deg:=10.0 -p ev3_right_end_deg:=530.0
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
