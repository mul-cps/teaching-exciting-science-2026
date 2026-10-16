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

`sorter-position TARGET_DEGREES` positions only the sorter/vertical feeder axis. Its typed request has no speed or motor selector. It uses the existing native `motor_target` path with fixed **50 degrees/second, 400 degrees/second² acceleration and 30% actuation**. The ROS API accepts only finite absolute interior targets **0 < target < 550 degrees**. Exact 0° is rejected before creating a workflow job: use explicit `ev3-home` for switch-verified zeroing. Exact 550° is rejected because upper-bound stopping is unqualified. Native bounds remain **0..550 degrees**; this does not reset or confirm any reference, dispense a brick, wake the device or move the follower arm.

Each job requires the connected session's exact sorter `home` reference, awake/idle/no-fault status, stationary measured speed, finite encoder telemetry, bounded receipt latency and two advancing native encoder samples. Run `wake` and measured `ev3-home` explicitly when needed. A trapezoid travel estimate plus one second of settling/latency reserve must fit the native six-second phase watchdog. A long interior step such as 0→549 is rejected by the phase budget before dispatch. Use separate bounded interior requests and wait for each terminal result; return to the lower reference through explicit `ev3-home`. Interior admission does not qualify near-bound stopping distance or imply a certified inset margin. A high speed must not be inferred from this API or substituted to make a long step fit.

Critical reads use the native read-only `critical_status` operation: both encoders and speeds are read with their actual acquisition timestamps, without color/LCD/beep work or waking a sleeping device. The host suppresses duplicate background native queries during an active workflow and publishes the worker’s latest observation instead. Logs separate total ROS round-trip, native-call and non-backend time and retain encoder/speed timestamps. One response exceeding the unchanged 0.75-second gate may be discarded and retried once within the same absolute phase deadline; it is never admitted as fresh, and a second slow response, cancellation or deadline expiry faults the job.

Fresh session/reference/encoder observations, exact native job identity, clean completion and measured endpoint within three degrees are required for success. Evidence records native control and target tolerances before/after motion, the accepted/terminal native job and measured encoders. A native controller position tolerance above three degrees (or unavailable tolerance telemetry) is explicitly flagged as exceeding the terminal gate; it never relaxes the three-degree acceptance check and may cause a legitimate job fault. The fixed profile and exact fake endpoints do not qualify native controller timing or physical accuracy. Cancellation, timeout, stale telemetry or native failure brakes the EV3 and latches a workflow fault. No new sorter or sorting job is admitted until explicit reset and any coordinated stop/disarm acknowledgements complete. After a failed position request, inspect the recorded fault and coordinated stop acknowledgements, explicitly reset the host workflow fault, and explicitly request `ev3-home` when lower-switch recovery is appropriate. Host fault reset never resets the native reference; homing resets the encoder only after the native switch and quiet-settling checks. A failed position is never automatically retried or converted into homing.

This is a manual commissioning API; it does not qualify physical tracking, collision clearance or automatic sorting. Establish a clear supervised workspace before the operator requests movement.

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

## Bounded jaw commissioning (raw counts)

After rebuilding the workflow interfaces and refreshing the resident follower
owner, the calibration prerequisite is ROS-callable without a jaw gap model:

```sh
ros2/scripts/station-action.sh gripper-step 8
# Equivalent: /follower/commission_gripper_step
# so101_station_workflow/srv/CommissionGripperStep {delta_counts: 8}
```

This is one relative encoder step, not an SI jaw-gap command. The owner derives
the absolute target from fresh measured jaw position and requires both start and
target inside the existing jaw firmware/calibration/configuration intersection.
Only nonzero signed steps of at most 16 counts are accepted. The first physical
trial is +8 counts. Raw Goal_Velocity=1, Acceleration=1 and Torque_Limit=50 are
fixed with readback; their physical velocity/force units remain unqualified.
The complete operation has a two-second deadline, with no retry or escalation.
Failure to arrive within one count and remain there for 200 ms is a failure.

Admission requires all six measured torque states off and a 200 ms stable passive
body baseline. During the step, all five body encoders and torque states are
sampled with a maximum 100 ms feedback gap; any body drift exceeding two counts,
nonzero body torque, unreadable state, excessive jaw excursion or cancellation
aborts. A passive body may be outside ordinary motion bounds only in this
component operation. It receives no profile, goal or torque writes. Ordinary
home, recovery, teleoperation and trajectory bounds are unchanged.

On the observed rig, writing the bounded jaw goal changes measured jaw torque
from 0 to 1 before a separate enable command. The audited Python driver chain
contains no extra torque-enable write; this observation does not establish the
firmware cause. The goal write is therefore the explicitly authorized energizing
operation, after verified low-profile readback and fresh all-off body/jaw guards.
Immediately after exact goal readback, the owner requires jaw torque 1 and body
torque 0; a jaw that remains off fails closed. It sends no redundant enable and
does not retry the goal.

Only gripper RAM is written. Success, failure, `/follower/stop`, EOF and later
idle/shutdown disable only the gripper. Saved jaw RAM is restored only after
measured jaw torque-off; unknown/nonzero body torque prevents an all-off
acknowledgement and is never corrected by writing the body. The gripper-only
write scope persists after completion until a fresh normal in-bounds, all-off
admission succeeds. `/follower/stop` cancels the bounded operation; normal
trajectory cancellation still retains its existing hold behavior.

The owner state file exposes `write_scope`, `component_profile_registers`,
`component_restore_pending` and `component_command` alongside all six raw
encoders and torque states. The command record contains an owner-generated
identity, signed step, derived absolute target, fixed raw profile, start/deadline
and outcome; it remains available after cleanup.
Guarded commissioning samples preserve separate position/torque acquisition
timestamps. The owner JSONL log records each verified profile readback and
restoration. The typed service returns measured final raw counts and torque
states only after completion with both encoder and torque acquisition stamps
within 100 ms. Process teardown closes the transport explicitly without the
normal all-servo stop path. Failures latch bridge admission until explicit
verified stop. This produces experimental evidence for camera jaw-gap/TCP
calibration; it does not mark calibration qualified or enable automatic sorting.

A component control fault deliberately terminates the resident owner after scoped
cleanup; a later stop request can therefore report owner-exited rather than a
new correlated stop acknowledgement. Earlier all-off log samples prove only
those acquisition times. Reconnect the owner read-only and obtain fresh six-joint
torque/encoder observations before authorizing another experiment. The raw
observation log is written before guard rejection, and single-register command
logs preserve intended value, actual scalar readback/type and write/read timing.
These diagnostics distinguish the rejected observation from subsequent cleanup;
they do not convert unexpected torque into an admitted state or retry a command.

## Recovering a terminated follower owner using ROS

```sh
ros2/scripts/station-action.sh arm-connect
ros2/scripts/station-action.sh stop
# Wait for the coordinated station stop to confirm both EV3 and follower.
ros2/scripts/station-action.sh status
ros2/scripts/station-action.sh reset-fault
```

`arm-connect` calls `/follower/reconnect` (`std_srvs/srv/Trigger`) and waits for
an explicitly requested replacement of a dead/disconnected owner. It never
kills or replaces a live owner, including an owner with stale telemetry. Active
actions, teleoperation, pending control callbacks and concurrent stop/reconnect
block replacement. Existing bridge/automatic-cycle faults remain latched and
measured-home completion is invalidated.

The replacement opens in observe-only mode under the same canonical ownership
lock and exact configured USB/calibration identity. Startup and all implicit
idle/error/EOF/close paths perform no servo writes. New-generation state must
report matching runtime hashes of the imported `arm.py` and `arm_session.py`,
exact identity/calibration and all six encoder/torque values acquired within
100 ms. Hashes are computed once per import/generation, never on telemetry ticks.

Reconnect success requires all six torque states off. If fresh telemetry shows
any torque on, reconnect returns false/not ready **and keeps the observe-only
owner alive** for explicit stop. Observe-only mode rejects motion, calibration
steps and teleoperation. Only an explicit `/follower/stop` (also used by the
coordinated station stop above) authorizes disarming writes and fresh verification;
only successful verification may leave observe-only mode. Unknown identity,
source or stale feedback prevents that stop admission. Reconnect itself never
homes, moves, changes calibration/qualification, retries a command, or resets a
fault. After explicit stop/reset, all ordinary motion and physical qualification
gates still apply. A direct commissioning failure can be recovered with
`arm-connect` followed by an explicit `/follower/stop`; automatic station faults
also require the coordinated station stop and explicit station fault reset.

## Camera recovery runtime: stage 1 owner integration

The production owner now has a server-configured factory for the existing
`recover_camera` operation. Missing/disabled configuration installs no recovery
authority. The actual rig remains disabled/unqualified. `/follower/recover_camera`
is an explicit Trigger operation; successful recovery preserves the measured
six-joint torque-on hold and leaves ordinary home invalid. `/follower/status`
reports cached acquisition timestamps and source/generation identity.

Commissioning installs a hash-pinned `so101-camera-recovery-runtime-v1` manifest
under `arm.camera_recovery` in the fixed server station configuration. Motion
commands cannot supply a manifest path, target, policy, clearance or qualification
flag. The manifest binds the effective URDF/material hash, qualified explicit
joint mapping, physical arm calibration SHA, qualified base/camera registration,
camera serial/frame, calibrated image dimensions/intrinsics/distortion, positive
geometry/depth uncertainties and explicit contact/support declarations. Model
and material preparation occurs once per runtime; no qualification is inferred
from CAD, convex enclosures or offline diagnostics.

Each owner runtime generates a private random32hex nonce once and publishes
`camera_recovery_cache_generation` and its distinct
`camera_recovery_runtime_started_monotonic` in source-bound owner telemetry.
The fixed RAM cache is
`<owner-state-directory>/camera-recovery/<runtime-nonce>`. A replacement runtime
reads only its new directory. The trusted camera collector publishes an immutable `<generation>.npz` containing only
`depth_m`, followed by atomic `latest.json`. The record uses schema
`recovery-depth-acquisition-v1`, a 32-character hexadecimal generation, payload
SHA, `depth_stamp_ns`, actual `depth_stamp_monotonic`, historically paired
`acquisition_arm_raw` and `acquisition_arm_stamp_monotonic`, original
`acquisition_owner_started_monotonic`, exact `acquisition_owner_source_sha256`,
camera/arm identities and calibrated intrinsics/distortion. The owner's original
bridge Popen/start stamp and later runtime-construction stamp are distinct;
acquisition reads must not predate this runtime. Source pins cover `arm.py`,
`arm_session.py`, `recovery_types.py`, `recovery_runtime.py`,
`camera_recovery.py` and the lightweight shared `recovery_runtime_config.py`
loader, computed only at startup. This is a selected source scope, not the whole
transitive dependency graph. Cache payload size and decompressed array
size are bounded. Missing, malformed, mismatched, stale or unknown observations
cannot grant clearance. This component opens no camera SDK or second motor bus.

Preparation builds `DepthRecoveryScene` outside the owner tick. Each prepared
acquisition has its own mutable geometry LRU while sharing immutable model
materials. Its depth timestamp, historical arm pose/stamp, support transforms
and certified physical self-interior remain unchanged for its entire lifetime.
`BoundRecoveryScene` adds a **current** actual owner raw pose/read timestamp;
it does not replace either acquisition timestamp. Changing current poses are
checked as candidate/swept volumes against the original scene. Only certified
material at the acquisition pose can be excluded from depth obstacles; current
pose geometry is never carved out of the image. Unknown, occluded and out-of-view
space still rejects, including continuous remaining-path clearance below10mm.

Provider lookup is nonblocking and rejects work exceeding50ms. The complete
bound proof has a250ms budget; original acquisition age and the existing
camera/encoder pairing limits remain independently enforced. New bindings do
not extend an old scene's lifetime. The owner rechecks exact encoders, cancellation
and proof/read clocks after the last queued-input pump before its first admitted
goal attempt. Before that attempt, failures/cancellation use fresh verified
disarm and RAM restoration with **zero goal writes**. A partially applied or
unacknowledged attempted goal counts as dispatched; subsequent failure retains
the existing measured-hold behavior. Failure to verify disarm is an error.

Hardware-free tests cover factory→RAM cache→real depth oracle→owner→fake motor
re-entry, changed current poses with immutable acquisition carving/timestamps,
missing/unqualified/stale/mismatched/occluded inputs, over-budget checks and
pre-dispatch cleanup. The worker may prewarm model-only voxel material once, with a separate2s cap
and no scene authority. It drops the triggering cohort and requires a strictly
new depth header after prewarm finishes. Each acquired scene gets a private LRU
copy sharing only immutable geometry. A diagnostic with checked-in SO101 geometry
measured cold preparation rejection at251ms, model-only prewarm478ms, then fresh
warm preparation16ms; the synthetic pose still rejected unresolved self contact.
These timings establish neither physical geometry qualification nor free space.

The cache protocol accepts only `source_clock="system"` epoch-UTC image headers.
A bracketed local wall/monotonic sample independently derives exposure monotonic
time; future/older-than250ms headers, clock sampling over5ms, and declared mapping
inconsistency over20ms reject. Original header, declared mapping and clock bracket
uncertainty remain immutable provenance. The20ms consistency bound never makes an
old/future exposure fresh. Simulated/non-epoch source clocks are unsupported.

Normal startup fails on invalid optional recovery artifacts. Passive reconnect
instead disables this authority with a logged reason while preserving exact arm
identity/calibration checks, zero implicit writes and explicit verified stop.
The recovery policy type is shared by imported and actual `python -m` owners.


### ROS depth cache producer and stationary recapture

The bridge owns one permanent subscription pair for existing
`/camera/aligned_depth_to_color/image_raw` and `/camera/color/camera_info`.
It opens no camera SDK and forwards only admitted current-owner cached samples.
The collector keeps at most8 immutable samples/400ms and one pending depth cohort;
callbacks do no filesystem access, hashing, conversion or blocking lock waits.
One worker converts/serializes/hash-checks metric depth and publishes the pointer
atomically. Camera frame, dimensions, intrinsics and distortion must exactly
match the pinned registration; depth/CameraInfo headers must differ by at most40ms.

`16UC1` uses0.001 meters per count because the installed RealSense ROS4.58.4
publisher explicitly normalizes depth to millimeters before publishing (see
[matching driver source](https://github.com/realsenseai/realsense-ros/blob/4.58.4/realsense2_camera/src/base_realsense_node.cpp)).
`32FC1` is metric float depth. Row stride and endianness are honored. Other
encodings reject; zero, NaN and other invalid depths remain unknown to the oracle.

Both actual owner read clocks must bracket the independently derived image
exposure, agree with the original UTC bracket, and lie within100ms of exposure.
All six raw encoders must be identical in both acquisitions. No interpolation,
receipt-time pose, caller-provided qualification or moving-pose approximation
is permitted. The resident recovery now uses an explicit stationary-recapture
phase: certify and command one waypoint of at most2 counts per active joint,
verify the exact already-commanded pose with all six torques1 before the original
motion certificate expires, then hold that existing goal while acquiring a new
stationary cohort. No Goal, torque-enable or profile writes occur during wait.
The original motion proof may expire only in this verified hold; it is never
reused to authorize the next movement. A new cohort must retain its original
exposure strictly after hold entry, exact held acquisition raw and the same
camera/owner/runtime generation, then pass a fresh full proof and final
raw/cancel/health check. Staged authority accepts only the concrete full-volume
per-joint displacement-envelope oracle, not a caller capability flag.

Same-nonce typed health is serialized by the collector worker and validated by
the runtime background preparer; owner queries use only the cached result.
Only explicit future-bracket/moving-bracket transitions, same-context identity
refresh awaiting a genuinely post-confirmation exposure, or evaluation of a ready
cohort may sustain stationary waiting. Missing/stale/malformed health, unknown
clearance, preparation faults, identity changes or a depth/info stream gap over
250ms abort. Runtime geometry faults override producer-ready status. Wait is
capped at500ms and the whole operation at6s. Pose, torque or hold-verification
failure disarms; camera/timeout failure may retain only the verified stationary
hold and latches failure. Stop/disarm takes precedence. There are no automatic
recovery retries or fault resets.

A successful wait retains its immutable exact held pose, entry and original
500ms deadline through proof and terminal settling. A known same-generation
identity refresh may resume only this same verified hold, with fresh exact raw,
all-six torque-on and Goal readback; it resets quiet-sample authority, never the
deadline. No Goal/enable/profile is written by the continuation. Unknown or real
geometry/proof failures remain fatal even if refresh happens concurrently.
New motion and terminal success require a genuinely post-confirmation,
post-hold cohort and fresh full proof. Cached ready health cannot promote an
older prepared oracle while preparation catches up. Already submitted immutable
cohorts retain their own original live lease; newly received delayed exposures
cannot borrow a newer confirmation. The final exact commanded waypoint needs no
redundant terminal Goal: success verifies its original actual position/torque
clocks, Goal readback, proof, health and cancellation after logging too.

Wait polls are deliberately scheduled at up to40Hz, with20ms read-budget and5ms
scheduler reserve, to enforce the unchanged50ms actual-acquisition interval and
sample-age limits. Each poll reads six-position, torque and current Goal
(up to120 read operations/s; about60 per500ms wait, plus cleanup reads).
This is additional phase-specific bus work; the observational publication helper
itself adds no bus reads. See the [reviewed phase contract](superpowers/plans/2026-10-16-stationary-recapture.md).
The full synthetic2993→2973 external ROS fixture also completed with15Hz
acquisitions and measured80ms transport delay:11 Goals (one initial hold and ten
certified waypoints), with no writes in ten stationary waits. This verifies the
software continuation path; no redundant terminal hold Goal is required.
Native ROS/fake-bus completion proves software integration, not physical
recovery. Actual station calibration/contact qualification remains disabled,
and preparation at new physical poses is not yet qualified.

Close/invalidate are nonblocking with zero filesystem I/O or joins. Old callbacks
become inert. Late writes/deletes remain confined to their old runtime nonce;
replacement owners never consume them. At most two writer threads may exist per
bridge process; if both are stuck, further acquisition is unavailable. Successful
publication retains at most3 immutable cohorts and cleans failed-publication
orphans. Worker-side best-effort cleanup keeps two prior validated nonce
directories; a stalled old writer can recreate stale non-authoritative data, so
this is not an instantaneous global disk-count guarantee. Unrelated names are
preserved, and old-file races fail closed.

The supported camera identity path is RealSense-specific. The bridge asynchronously
observes the fixed trusted camera node's `DeviceInfo.serial_number` and
`serial_no` parameter; both must match the pinned registration. Both depth and
CameraInfo must have exactly one publisher on that node, with pinned publisher
GIDs. The1s identity lease starts when requests are sent and refreshes every0.5s;
response arrival cannot mint a new observation time. Records retain actual and
configured serials, explicit device/stream generation, request-start and actual
confirmation timestamps. Frames must be acquired strictly after their identity
confirmation. GID changes, parameter serial mismatch, header rollback or a stream
gap over250ms revoke identity/history and require fresh queries and a new cohort.
Missing or stale identity never falls back to copying the expected serial into
an asserted observation. Simulation/standalone configurations remain usable with
recovery disabled; they do not gain RealSense qualification by supplying a flag.

### Isolated native publisher identity support

The pinned Conda rclpy7.1.12 omits publisher GIDs from Python message metadata;
Cyclone RMW2.2.4 additionally reports a publication handle instead of its RTPS
GUID. The reproducible isolated builds are:

```bash
ROS_DOMAIN_ID=75 ros2/conda/build-rclpy-publisher-gid.sh
ROS_DOMAIN_ID=75 ros2/conda/build-rmw-cyclonedds-publisher-gid.sh
```

They pin upstream revisions and patch bytes, build into separate `ros2/.cache`
prefixes, and verify that installed Conda binaries remain unchanged. The wrapper
validates both provenance markers before prepending the isolated Python/native
paths after environment/workspace activation. The patched backend has the
explicit filename and implementation identifier `rmw_cyclonedds_cpp_so101_gid`;
this avoids Conda RPATH resolving the identically named stock backend first.
CDR, introspection type support and Cyclone DDS wire behavior remain unchanged.
An explicit non-Cyclone implementation selection is preserved. Existing live
processes are unchanged by building an overlay.

The native patch looks up the actual matched publication GUID and frees the
returned endpoint metadata. Missing/disappeared endpoint metadata yields an
unavailable zero GID, never a handle or graph attribution fallback. Tests cover
typed/serialized messages, recreated publishers, and stock↔patched processes;
`/proc/self/maps` proves the loaded binary paths. This transport proof does not
qualify camera registration, geometry contacts, or continuous recovery.

The selected owner source map now pins six modules: `arm`, `arm_session`,
`recovery_runtime`, `recovery_types`, `recovery_runtime_config`, and
`camera_recovery`. This is an explicit selected source scope, not a claim to
pin every transitive dependency. Hashes are cached once per owner generation.
The trusted bound scene keeps original acquisition-pair synchronization≤100ms
separate from the exact current owner observation≤50ms; original depth/paired
pose250ms and identity-request1s expiry remain unchanged. Generic adapters
cannot assert alternate acquisition timestamps to bypass synchronization.
Active recovery publishes observational samples from existing position/torque
reads before proof; initial/setup samples retain cached torque's original clocks.
This observational publication adds no bus reads or goal/enable writes and does
not imply motion admission; the separate recapture phase polling budget is
documented above.

During wait, cached owner telemetry exposes `camera_recovery_phase`
(`stationary_recapture`), `camera_recovery_held_raw`, the original
`camera_recovery_hold_stamp_monotonic`, and
`camera_recovery_wait_deadline_monotonic`. These are observational fields, not
motion permission. An older owner without these fields reports phase support as
unavailable rather than fabricating progress.

A successful same-generation refresh of a still-valid actual camera lease may
report `awaiting_post_confirmation_exposure`. This permits stationary waiting
only: an older delayed frame remains ineligible as a scene, and cached old scenes
cannot authorize a new Goal while this health state is known. The refresh marker
is tied to the new request start and clears on every context invalidation. New
motion requires a genuinely post-confirmation/post-hold acquisition; neither the
1s request lease nor any image/pose250ms deadline is extended. A refresh arriving
after proof but before dispatch rejects that write and conservatively fails held;
there is no generic retry or reset of the original500ms/6s deadlines.
