# ROS station operation

The integrated launch starts the resident workflow API beside the existing single follower and camera owners. It never moves a motor or connects to the EV3 automatically. A sole `EV3LiveClient` in this resident API owns the TCP session; workflow workers call its ROS service, and status clients read its cached status. Do not run a second direct EV3 client beside it.

After installation/build (`tools/install_station.sh`, then `ros2/conda/so101-conda.sh build`), the cold-start command is:

```sh
ros2/conda/so101-conda.sh station-live use_ev3_tunnel:=true
```

An existing SSH forward can be reused by leaving `use_ev3_tunnel` false. The optional launch-managed tunnel uses the installed station SSH identity, strict host-key checking and localhost forwarding. `ev3_remote`, `ev3_ssh_key`, `ev3_api_port`, `sorting_config` and `sorting_evidence_dir` are launch arguments. The EV3 native daemon must already be installed through the identified-device installer; startup does not deploy firmware. Domain 76/local discovery and the GPU-only RViz guard remain the wrapper defaults. Tests use isolated domain 75.

Ordinary operation uses typed ROS-only commands, with no configuration-file edits:

```sh
ros2/scripts/station-action.sh connect
ros2/scripts/station-action.sh diagnostics
ros2/scripts/station-action.sh status
ros2/scripts/station-action.sh wake
ros2/scripts/station-action.sh ev3-home
# Only after physically checking the indicated reference:
ros2/scripts/station-action.sh confirm-reference feeder --physically-verified
ros2/scripts/station-action.sh arm-home
ros2/scripts/station-action.sh check-ready
ros2/scripts/station-action.sh run 1
ros2/scripts/station-action.sh run 20
ros2/scripts/station-action.sh cancel
ros2/scripts/station-action.sh stop
ros2/scripts/station-action.sh reset-fault
ros2/scripts/station-action.sh sleep
```

`wake` and `sleep` control EV3 power state. `ev3-home` runs the bounded native touch-switch sorter homing job. It never invents a feeder reference. `confirm-reference` forwards only sorter/home or feeder/eject, with an explicit operator physical acknowledgement; the native sorter additionally checks the touch switch. `arm-home` uses the resident measured-home/recovery service and preserves its qualification gates.

`stop` and `cancel` request cancellation and EV3 braking, and independently request `/follower/stop` (torque off/disarm). Their immediate response states that the arm acknowledgement is pending. Inspect `/station/status`: `stop.pending` must become false, with `ev3_stopped` and `arm_disarmed` true. Missing or rejected acknowledgements block fault reset and further sorting. Normal action cancellation during failed cycles still follows the action server's existing measured hold semantics. Stop/disarm can require physical support of the arm.

Every accepted long operation returns a UUID job ID. Follow `/station/status` (`std_msgs/String` JSON) or call the status service for completion, result, latched fault and the last 100 events. Evidence is appended under `sorting_evidence_dir`; each sorting UUID has a `cycles.jsonl` and grasp artifacts. An interrupted or failed job latches a fault and never admits another brick. Inspect the evidence, establish a known physical state, then call explicit reset. An accepted service request is not completion evidence.

## Moving placement selection

Select a region using the passive twin's camera interface, or use the equivalent typed ROS API. Read the exact current camera `generation` from the twin/placement observation; stale generations are rejected:

```sh
ros2/scripts/station-action.sh select-anchor GENERATION X0 Y0 X1 Y1
ros2/scripts/station-action.sh select-area GENERATION X0 Y0 X1 Y1
ros2/scripts/station-action.sh clear-selection GENERATION
```

Read the returned stable UUID, then bind a color:

```sh
ros2/scripts/station-action.sh select-destination red SELECTION_UUID
```

Bindings stay fixed throughout a running job. The executor resolves the selected RGB-D pose using a server-loaded qualified `camera-arm-calibration-v1` file, pinned selection ID, local receive age and acquisition timestamp. The qualified grasp execution configuration must also supply `placement_goal_from_tip`, a calibrated rigid transform from selected-region axes to the commanded tool link, including the release height and orientation. The same qualified execution file must include `placement_footprint_m` (two goal-axis lengths of the conservative carried-brick envelope about goal center), `placement_tool_footprint_m` (two goal-axis lengths about tool origin), positive `placement_clearance_m` and positive `placement_registration_uncertainty_m`. The observed brick XY diagonal must fit both commissioned carried-envelope dimensions, conservatively covering arbitrary in-plane rotation. Four current corners must form a centered planar rectangle consistent with the reported extent. Both carried and tool footprints must fit after shrinking the area by clearance, registration uncertainty and three times combined tracking/depth-plane uncertainty. Tiny, malformed or inconsistent areas are rejected before dispensing and again before placement. These are commissioning data, not browser qualification flags. Missing registration, tool transform, tracking, freshness or a changed UUID blocks motion. The executor resolves again before place, replans up to three times if the goal moves during planning, and rechecks immediately before submission. Further movement aborts without release. It composes TF at acquisition time into the observation frame for independent release verification.

## ROS interfaces

| Endpoint | Type | Meaning |
|---|---|---|
| `/station/status`, `/station/diagnostics` | `std_srvs/srv/Trigger` | Read cached job/device state or config/transport gaps; never moves hardware |
| `/station/check_ready` | `std_srvs/srv/Trigger` | Start read-only admission job; result appears in status |
| `/station/run_cycles` | `so101_station_workflow/srv/RunCycles` | `cycles` in 1..100; returns admitted UUID |
| `/station/stop`, `/station/cancel` | `std_srvs/srv/Trigger` | Request stop/disarm; terminal acknowledgement appears in status |
| `/station/reset_fault` | `std_srvs/srv/Trigger` | Explicit reset after worker termination and stop acknowledgement |
| `/station/ev3/connect` | `std_srvs/srv/Trigger` | Explicit connect/reconnect; no automatic retry of unknown delivery |
| `/station/wake`, `/station/sleep`, `/station/ev3/home` | `std_srvs/srv/Trigger` | Bounded EV3 lifecycle jobs |
| `/station/arm/home` | `std_srvs/srv/Trigger` | Qualified resident measured home/recovery |
| `/station/ev3/confirm_reference` | `so101_station_workflow/srv/ConfirmReference` | `name`, `reference`, `physically_verified` |
| `/station/placement/select` | `so101_station_workflow/srv/SelectPlacementRegion` | Exact `generation`, `mode` feeder/goal/clear, four integer pixel coordinates; returns selection UUID |
| `/station/select_destination` | `so101_station_workflow/srv/SelectDestination` | `color`, `selection_id`; idle only |
| `/station/ev3/command` | `so101_station_workflow/srv/EV3Command` | Internal serialized bridge; exact private single-use capability and active-job UUID required for changes; public UUID never grants eject |

The bridge service is an implementation API; ordinary users use the typed services/quick commands. It permits only status/stop/eject/home/wake/power-save operations, requires the active job UUID plus a private, single-use operation capability bound to the exact operation and arguments for all changes, and rejects reference confirmation outside the explicit acknowledgement service. The capability is minted only by the internal proxy and never published in status; knowing a public job UUID cannot authorize dispensing or other mutation during readiness, arm homing or sorting. Cancellation or a pending stop revokes all queued mutations, while internal cleanup stop remains available. External stop requests use `/station/stop` to coordinate the arm and EV3. It never opens another arm serial port or camera SDK. Automatic operation still requires the resident teleop-disabled readiness gate.

## Physical gates

These APIs do not qualify the rig. Current missing gates include feeder reference, qualified camera/base registration, jaw/TCP calibration and placement tool transform, reachable clear arm/home state, collision/scene admission, independent pickup/lift/release evidence and the physical endurance/failure matrix in `ros-sorting-acceptance.md`. Synthetic ROS tests prove orchestration and transport contracts only. Missing commissioning data is reported as a gate and cannot be substituted with a synthetic confirmation.
