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
ros2/scripts/station-action.sh sorter-position 200
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

`wake` and `sleep` control EV3 power state. An explicitly requested `run` wakes the EV3 through the same private ROS bridge before admission; read-only `check-ready` never wakes or clears a native fault. Readiness requires exact same-session sorter/home and feeder/eject references, native connected/awake/idle status, `last_error` exactly null, clean previous cleanup and finite stopped encoder/speed telemetry for both axes. The feeder must also be measured at its eject endpoint within the existing native three-degree terminal tolerance (including signed undershoot); a retained reference with a loaded or intermediate angle cannot admit another brick. Both native encoder timestamps must advance within a bounded two-second observation window; cached sleeping or frozen data cannot pass. The adapter repeats these gates after resident readiness, immediately before each dispense, and before creating a grasp execution client. Restarting the host API or resetting its workflow fault cannot clear a retained native motion fault. Establish the proper physical recovery/reference state explicitly before another run. `ev3-home` runs the bounded native touch-switch sorter homing job. It never invents a feeder reference. `confirm-reference` forwards only sorter/home or feeder/eject, with an explicit operator physical acknowledgement; the native sorter additionally checks the touch switch. `arm-home` uses the resident measured-home/recovery service and preserves its qualification gates.

`stop` and `cancel` request cancellation and EV3 braking, and independently request `/follower/stop` (torque off/disarm). Their immediate response states that the arm acknowledgement is pending. Inspect `/station/status`: `stop.pending` must become false, with `ev3_stopped` and `arm_disarmed` true. Missing or rejected acknowledgements block fault reset and further sorting. Normal action cancellation during failed cycles still follows the action server's existing measured hold semantics. Stop/disarm can require physical support of the arm.

Every accepted long operation returns a UUID job ID. Follow `/station/status` (`std_msgs/String` JSON) or call the status service for completion, result, latched fault and the last 100 events. Evidence is appended under `sorting_evidence_dir`; each sorting UUID has a `cycles.jsonl` and grasp artifacts. An interrupted or failed job latches a fault and never admits another brick. Inspect the evidence, establish a known physical state, then call explicit reset. An accepted service request is not completion evidence.

## Bounded sorter-axis commissioning

`sorter-position TARGET_DEGREES` positions only the sorter/vertical feeder axis. Its typed request has no speed or motor selector. It uses the existing native `motor_target` path with fixed **50 degrees/second, 400 degrees/second² acceleration and 30% actuation**. Targets are absolute within native **0..550 degrees**; this does not reset or confirm any reference, dispense a brick, wake the device or move the follower arm.

Each job requires the connected session's exact sorter `home` reference, awake/idle/no-fault status, stationary measured speed, finite encoder telemetry, bounded receipt latency and two advancing native encoder samples. Run `wake` and measured `ev3-home` explicitly when needed. A trapezoid travel estimate plus one second of settling/latency reserve must fit the native six-second phase watchdog. At the fixed profile a 0→550 request is rejected before dispatch; traverse the axis with separate bounded absolute requests such as **0→200→400→550**, waiting for each terminal result. Reverse similarly in bounded steps. A high speed must not be inferred from this API or substituted to make a long step fit.

Critical reads use the native read-only `critical_status` operation: both encoders and speeds are read with their actual acquisition timestamps, without color/LCD/beep work or waking a sleeping device. The host suppresses duplicate background native queries during an active workflow and publishes the worker’s latest observation instead. Logs separate total ROS round-trip, native-call and non-backend time and retain encoder/speed timestamps. One response exceeding the unchanged 0.75-second gate may be discarded and retried once within the same absolute phase deadline; it is never admitted as fresh, and a second slow response, cancellation or deadline expiry faults the job.

Fresh session/reference/encoder observations, exact native job identity, clean completion and measured endpoint within three degrees are required for success. Evidence records native control and target tolerances before/after motion, the accepted/terminal native job and measured encoders. A native controller position tolerance above three degrees (or unavailable tolerance telemetry) is explicitly flagged as exceeding the terminal gate; it never relaxes the three-degree acceptance check and may cause a legitimate job fault. The fixed profile and exact fake endpoints do not qualify native controller timing or physical accuracy. Cancellation, timeout, stale telemetry or native failure brakes the EV3 and latches a workflow fault. No new sorter or sorting job is admitted until explicit reset and any coordinated stop/disarm acknowledgements complete. This is a manual commissioning API; it does not qualify physical tracking, collision clearance or automatic sorting. Establish a clear supervised workspace before the operator requests movement.

## Moving placement selection

Select a region using the passive twin's camera interface, or use the equivalent typed ROS API. Pin a frame for human selection first. `pin-anchor` and `pin-area` accept an optional exact generation; omitting it requests the current frame. The returned `message` JSON contains the pinned `generation`, selection mode and expiry (30 seconds). Use that generation in the next rectangle command; expired or unrelated frames are rejected:

```sh
ros2/scripts/station-action.sh pin-anchor
# Read generation from the response, then choose feeder pixels:
ros2/scripts/station-action.sh select-anchor GENERATION X0 Y0 X1 Y1
ros2/scripts/station-action.sh pin-area
# Read the new pinned generation, then choose usable-area pixels:
ros2/scripts/station-action.sh select-area GENERATION X0 Y0 X1 Y1
```

The selection commands are:

```sh
ros2/scripts/station-action.sh select-anchor GENERATION X0 Y0 X1 Y1
ros2/scripts/station-action.sh select-area GENERATION X0 Y0 X1 Y1
ros2/scripts/station-action.sh clear-selection GENERATION
```

Read the returned stable UUID, then bind a color:

```sh
ros2/scripts/station-action.sh select-destination red SELECTION_UUID
```

Bindings stay fixed throughout a running job. The executor resolves the selected RGB-D pose using a server-loaded qualified `camera-arm-calibration-v1` file, pinned selection ID, local receive age and acquisition timestamp. The qualified grasp execution configuration must also supply `placement_goal_from_tip`, a calibrated rigid transform from selected-region axes to the commanded tool link, including the release height and orientation. The same qualified execution file must include `placement_footprint_m` (two goal-axis lengths of the conservative carried-brick envelope about goal center), `placement_tool_footprint_m` (two goal-axis lengths about tool origin), positive `placement_clearance_m` and positive `placement_registration_uncertainty_m`. The observed brick XY diagonal must fit both commissioned carried-envelope dimensions, conservatively covering arbitrary in-plane rotation. Four current corners must form an ordered convex planar quadrilateral centered on the selected goal pose, with top/left edge lengths consistent with the reported extent. Perspective backprojection of an image rectangle may produce a trapezoid on a tilted surface; this remains usable. Every corner of both carried and tool footprints must lie inside all four inward edge halfspaces after shrinking the area by clearance, registration uncertainty and three times combined tracking/depth-plane uncertainty. Tiny, malformed or inconsistent areas are rejected before dispensing and again before placement. These are commissioning data, not browser qualification flags. Missing registration, tool transform, tracking, freshness or a changed UUID blocks motion. The executor resolves again before place, replans up to three times if the goal moves during planning, and rechecks immediately before submission. Further movement aborts without release. It composes TF at acquisition time into the observation frame for independent release verification.

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
| `/station/ev3/sorter_position` | `so101_station_workflow/srv/SorterPosition` | Absolute `target_deg`, fixed conservative profile; returns bounded job UUID |
| `/station/ev3/confirm_reference` | `so101_station_workflow/srv/ConfirmReference` | `name`, `reference`, `physically_verified` |
| `/station/placement/select` | `so101_station_workflow/srv/SelectPlacementRegion` | Exact `generation`, `mode` pin_feeder/pin_goal/feeder/goal/clear, four integer pixel coordinates; returns selection UUID |
| `/station/select_destination` | `so101_station_workflow/srv/SelectDestination` | `color`, `selection_id`; idle only |
| `/station/ev3/command` | `so101_station_workflow/srv/EV3Command` | Internal serialized bridge; exact private single-use capability and active-job UUID required for changes; public UUID never grants eject |

The bridge service is an implementation API; ordinary users use the typed services/quick commands. It permits only status/critical-status/stop/eject/home/wake/power-save operations and internally constructed sorter targets, requires the active job UUID plus a private, single-use operation capability bound to the exact operation and arguments for all changes, and rejects reference confirmation outside the explicit acknowledgement service. The capability is minted only by the internal proxy and never published in status; knowing a public job UUID cannot authorize dispensing or other mutation during readiness, arm homing or sorting. Cancellation or a pending stop revokes all queued mutations, while internal cleanup stop remains available. External stop requests use `/station/stop` to coordinate the arm and EV3. It never opens another arm serial port or camera SDK. Automatic operation still requires the resident teleop-disabled readiness gate.

## Physical gates

These APIs do not qualify the rig. Current missing gates include feeder reference, qualified camera/base registration, jaw/TCP calibration and placement tool transform, reachable clear arm/home state, collision/scene admission, independent pickup/lift/release evidence and the physical endurance/failure matrix in `ros-sorting-acceptance.md`. Synthetic ROS tests prove orchestration and transport contracts only. Missing commissioning data is reported as a gate and cannot be substituted with a synthetic confirmation.
