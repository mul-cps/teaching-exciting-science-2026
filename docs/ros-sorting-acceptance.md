# ROS sorting station acceptance — 2026-10-16

The station is **not physically accepted**. The implementation branch is
`feat/sorting-station-20261016`; the original dirty checkout is preserved.
The [plan](superpowers/plans/2026-10-16-ros-sorting-station.md) defines the
release requirements; the [progress record](superpowers/plans/2026-10-16-ros-sorting-progress.md)
records commands, limitations and decisions.

## Overnight checkpoint — 2026-10-20, 00:20 UTC

The main acceptance target remains autonomous ROS-only brick pick and place,
followed by twenty consecutive successful physical sorts. No physical sorting
cycle or arm motion has been performed during this overnight run.

The follower owner exited on October 19 at 20:48:30 UTC after an endpoint-hold
health failure. Its final historical readback had all six torque bits off;
there is no fresh owner telemetry. The running camera and RViz do not qualify
the arm pose, home, camera/base registration, TCP or jaw measurements.

The frozen software matrix at `4072395` passed: native ROS domain75,
3019 passed/64 skipped; station vision, 2323 passed/94 skipped; isolated
domain177, 181 passed/zero skips. These stages overlap and must not be summed.
An earlier 180-pass/one-failure DDS run is retained; only its outdated fake
reconciliation response was corrected. All 706 tracked hashes stayed fixed.

The first actual consumer-reconciliation job completed its private-worker reset
and two advancing IMAGE-admission proofs per consumer. Supervisor adoption then
latched with `derived lineage: source receipts expired/future`, preserving the
owned job and its one charged attempt. Deterministic replay reproduced a
retained-history bug; the exact offending live row was not captured. The fix is
verified in source and is not deployed to the current supervisor. Explicit
resolution of that already-completed job is still being implemented. No automatic outage
test has run, and perception recovery grants no arm execution authority.

Evidence is under `2026-10-19-sorting/overnight/final-4072395/` and
`2026-10-19-sorting/automatic-perception-supervisor/completed-adoption/` in the
station evidence directory. The table and hardware description below retain
their original checkpoint evidence; statements about a running owner or fresh
elbow telemetry are historical, not the current live state.

## Original acceptance checkpoint

| Requirement | Current evidence | Status |
| --- | --- | --- |
| Software suite | Final combined: 1595 passed, 84 skipped, 6 subtests (32.55s); actual station interpreter: 264 passed (6.15s) | Current scoped software gates passed; physical acceptance separate |
| Native ROS checks | Final current-source broad run: 606 passed, 1 EGL renderer skip (40.70s). Eight recovery cases passed together (14.75s), including original and15Hz/80ms-delay full trajectories; independent delayed/loss replay passed | Scoped checks passed; two historical API timeouts remain unexplained, no physical execution |
| Clean package installation | Latest canonical build: all 19 packages passed; earlier all 17 packages built in a fresh integration prefix; separate startup worker verified a non-symlink install and installed plane-module import | Passed |
| MoveIt planning | Mock start state valid, no collision; seven-point trajectory saved | Planning only; physical scene not qualified |
| Single station startup | One camera launch and one resident follower owner; calibrated jaw path forwarded through both launch layers | Implemented; live cold starts pending |
| Calibration | Training-only tag hand-eye fit has submillimeter held-out tag agreement, but wrist-roll/tag-attachment gauge remains; CAD supports a diagnostic offset near -95 degrees. Current depth residuals do not qualify camera/base/TCP/jaw geometry | Not accepted |
| Plane residual ≤3 mm / pickup position error ≤5 mm | No current held-out physical measurements prove these bounds | Pending |
| Passive twin | Live viewer renders correct root-frame TF/marker semantics; raw RSP/MoveIt hashes match; bridge interruption visibly marks STALE and restart restores FRESH | Stationary live display and stale/reconnect checked; physical registration remains separate |
| Twin error ≤1 mm / ≤0.5 degrees | One fresh synchronized stationary sample: tool disagreement1.24e-16m/0degrees; all-link/root-geometry nonidentity-transform regressions pass | Stationary mathematical parity passed; moving physical qualification pending |
| Teleop ≥100 Hz for 30 minutes | ACK wakeup fix retained uncapped profile and one-inflight admission; real bridge/production owner with FakeBus measured 108–109 Hz at 8.5 ms added ACK delay. No qualifying physical endurance run | Pending |
| Twenty consecutive physical sorting cycles, ≥5 per color | No physical sorting cycle executed | Pending |
| Failure recovery and reconnect | Software tests cover stop/ownership paths; transport failure observed while torque was last verified off | Physical behavior unqualified |
| Camera-guided automatic arm recovery | Trusted ROS identity/cache/factory and stationary recapture are implemented. Final synthetic2993-to-2973: ideal2.048s,80ms delay2.399s;11Goals,0EEPROM,zero writes in all waits,homefalse. Independent delayed replay2.600s. Physical registration remains unqualified | Software path validated with simulated hardware; physical recovery pending |
| ROS isolation and rendering | Full station and twin restarted on domain76; wrapper defaults76/rejects0; localhost discovery; actual RViz amdgpu counter increments | Live checked; rebuilt container image remains separate |
| Native EV3 commissioning | Exact identity and updated source hash checked; current-session switch reference established; latest critical-query target12 completed measured13 in737ms. Return0 hit encoder-1 and correctly faulted the unchanged0degree soft bound; coordinated stop confirmed | Explicit reset plus switch-home recovered measured0/touchtrue in240ms; exact-boundary position requests reject before job mutation; feeder reference/ejection and broader qualification pending |
| ROS workflow and quick actions | Reviewed resident API, private one-use EV3 command admission, bounded cycles/cancel/stop; root combined native subset43passed; live typed status/diagnostics and sole EV3 connection work on domain76 | API connected idle after explicit switch-home recovery; failed return evidence and terminal stop acknowledged; commissioning config absent; physical cycles pending |
| Acknowledged arm stop | New bridge/sole owner started; live `/follower/stop` succeeded with fresh six-servo torque-off readback, unchanged raw positions | Idle protocol checked; arm was already torque off, moving-stop qualification pending |
| Interactive moving placement area | Browser8101 upright with matching pixel coordinates; asynchronous frozen-frame replay tested. Correct LEGO anchor retained across80outbound/53return frames; motion only~1pixel. A goal picked on the fixed belt must not be bound to that anchor | Correct moving feeder goal and calibrated dynamic placement remain unqualified |
| ROS sorter-axis positioning | Latest critical-query target12 completed measured13 in737ms. Return0 correctly faulted at encoder-1 under the unchanged soft limit; explicit switch-home restored measured0. Exact boundary requests reject before mutation | Limited physical trial evidence; full positioning and feeder qualification pending |
| Three successful cold starts | Three camera-only starts passed after reconnection; complete station cold-start gate not exercised | Pending |
| Release artifacts and publication | Local camera/USB logs retained; implementation branch not a release | Pending |

## Hardware boundary at the original checkpoint

The D435i is SDK serial `243322071783`, firmware `5.17.0.10`. After the
recorded USB outage, three native ROS camera starts each delivered approximately
200 paired frames over15seconds with zero saved-pair timestamp skew. The live
camera, RViz and sole follower owner are running. These camera-only cycles do
not prove full station cold starts or camera-to-arm registration.

Fresh read-only arm telemetry still reports elbow raw2993/91.56degrees,
outside its configured90degree bound, with all six torque states zero. The
recovery owner derives a bounded inward target but production lacks a qualified
camera scene provider. The trusted factory and ROS API exist in source, while the live owner still runs the earlier passive-reconnect build. No body recovery or grasp cycle has been sent. Physical
camera/base/TCP/jaw qualification remains a prerequisite for execution.

Full-view color tracking now separates observation, support and pickup admission.
Contact/pregrasp previews outside pickup bounds do not authorize execution.
Depth support, original per-child shape evidence and three-frame acquisition
confirmation reject transient floating fragments without a fixed workspace
exclusion. Earlier30-second capture `tracking-confirmed-live/` preserved all eight
real bricks with zero extras in150/150tracking reports. Receipt age was below
0.327seconds. The later domain76 sample has9colored surfaces, including near hardware; all ghosts are not eliminated. This verifies the earlier stationary reproduced scene; live dynamic,
occlusion and semantic generalization remain unqualified.

Evidence is under
`~/.local/share/exciting-science/station/evidence/2026-10-16-sorting/`, including
`README.md`, camera probe logs, kernel events and
`rgbd_pair_ros_driver/{color.png,depth_raw.npy,depth_aligned.npy,info.json}`.
Native startup/build/plan artifacts are under
`/tmp/sorting-startup-validation-20261016/`; retain them with the release
evidence before clearing temporary storage.

## Sorting entry point

The resident ROS workflow consumes the already-running station and owns the
persistent EV3 API connection. Its sorting adapter opens neither a camera SDK
nor an arm serial port. Use the [typed ROS quick actions](ros-station-api.md);
the resident API is the sole EV3 client. Sorting requires a
`so101-ros-sorting-v1` JSON configuration containing:

- `enabled`, `commissioned`, and `automatic_start`, all explicitly true only
  after physical commissioning;
- `station_config` and `grasp_execution_config` paths, resolved relative to the
  sorting configuration;
- `frame_id`, six ordered `pickup_bounds_m` values (`xmin,xmax,ymin,ymax,zmin,zmax`),
  and a measured `clear_joint_pose_rad` map for all five arm joints;
- `destinations`, mapping commissioned colors to XYZ plus XYZW place poses.

The grasp execution configuration and calibrated jaw/TCP/camera/model gates
are described in [gripper registration](so101-gripper-registration.md).
The EV3 must already have valid physical references in the current daemon
session. The read-only `/follower/sorting_ready` query requires measured-home
completion in the current resident-owner session.

Once these prerequisites are physically verified:

```sh
ros2/scripts/station-action.sh check-ready
ros2/scripts/station-action.sh run 1
ros2/scripts/station-action.sh status
```

Every failure prevents another dispense. A cycle completes only after distinct
fresh observations verify pickup/lift and the brick remaining at its destination
after the tool retreats, followed by measured clearance. Command acceptance,
successful planning, and jaw movement do not establish a completed physical sort.


### Camera and model based calibration status

Missing measurements may be derived from pinned source CAD and bounded physical
experiments compared with synchronized camera data. Keep nominal estimates, fit
uncertainty and separate held-out validation explicit; external metrology is not
assumed available. Current STEP contact geometry is nominal only.

The gripper-only commissioning API has software coverage and review. The first
two +8-count trials exposed goal-write energization and correctly faulted with all
five body joints off. After correcting the write/enable expectation, one bounded
+8 trial succeeded (raw 1936 to 1945), followed by a +16 trial that reached 1959
but failed its unchanged two-second deadline. Cleanup verified all six torque
bits off and restored the RAM profile. No automatic retry or force/deadline
increase followed. Small image changes were insufficient to identify physical
jaw scale. Fresh passive camera/owner captures bind serial, calibration and
acquisition time; they remain explicitly unqualified. These experiments do not
qualify jaw gap, TCP, camera recovery or physical sorting. See the progress log
and evidence directories `gripper-component-step-8-8c80fce`,
`gripper-component-step-16-8c80fce`, and `camera-current-visibility-check`.


### Live recovery deployment checkpoint

Source79d4234 is deployed in the follower bridge/serial owner on ROS domain76
with all six source pins matched and native publisher identity enabled. A single
unconfigured recovery request rejected without position changes; explicit stop
then verified all six torque bits off and cleared the resulting owner fault.
Camera recovery remains not configured; station commissioning and automatic
start remain disabled. No physical recovery, home or sort is qualified by this
checkpoint. Evidence: `recovery-live-79d4234/RESULTS.md` and `final-status.json`.


### ROS arm preparation software checkpoint

The isolated preparation package adds zero-argument `/follower/prepare`,
`/station/arm/prepare`, `arm-prepare`, and `arm-stop`. One follower reservation
spans optional qualified camera recovery, measured home and readiness. Explicit
preparation does not connect or wake EV3. Automatic sorting reuses preparation
inside its existing station job and retains its independent EV3/reference gates.

Source-pinned native recovery:9cases passed18.90s, including the complete original
2993-to2973 target,15Hz/80ms camera transport, six negative cases, and genuine
identity renewal before the first write. The latter retains original clocks,
all-six torque-off/unchanged raw and zero writes during acquisition.

Resident station API composition:1case passed21.33s using actual ROS, bridge and
station-interpreter owner processes with an explicitly synthetic bus/model/jaw
and temporary qualified configuration. Job
`9b730042-1b80-46eb-8f9a-f2dcc948293a` completed recovery then measured home once,
reported actual raw2000/normal bounds/readiness, and made no EV3 connection.
Cleanup verified all torque off; disconnected-EV3 shutdown diagnostics remain
recorded separately and do not qualify full station stop.

Evidence: `arm-prepare-bounds/native-complete-nine*`,
`PREDISPATCH-NATIVE-AUDIT.json`, and
`arm-prepare-integration/native-owner-prepare*`. Final source validation passed: system1678tests/89skips/6subtests, station
interpreter536tests, focusedAPI230tests and broad native658tests/3skips.
The independent clean prefix built19packages and passed import smoke. Native
skips are the EGL renderer and two optional local-overlay wrapper checks; actual
publisher-GID/interop checks passed against the validated read-only overlays.
One earlier broad unit failed its positive fake-home setup before NaN injection;
the same-order diagnostic replay passed, and the cause remains unproven. This is
not claimed as a production fault fix. Source/index evidence is saved alongside
the final logs and import/dependency provenance. This is software evidence; actual arm recovery,
home, gripping, placing and physical acceptance remain unqualified.


## Disarmed deployment of preparation software 63465aa

- Published source `63465aaf92214acf5dca5ba677ae0272ec100ec2` is deployed. The previous API, bridge and sole serial owner exited before replacement. New bridge272577/owner272595/API285928 were verified from actual process mappings and environment: ROS domain76, localhost discovery, isolated publisher-GID rclpy/Cyclone backend. All six owner source hashes match.
- EV3 reconnect, the new `arm-stop` action and coordinated station stop succeeded. Final workflow is idle with no fault, both stop acknowledgements true and no pending stop. Sorter remains home0/touchtrue; feeder reference remains unknown.
- One explicit `arm-prepare` job `11d86e8e-7644-4a45-9a2c-7fe977c2fc14` correctly rejected disabled commissioning before owner command delivery. Explicit operator fault reset cleared this deliberate negative check; preparation was not retried.
- Actual positions stayed shoulder_pan1996, shoulder_lift1143, elbow_flex2993, wrist_flex2826, wrist_roll2076, gripper1959; all six torque bits remain off. Owner is fresh, fault-free, with no completed home. Elbow91.56044degrees is classified bounded_reentry_required; classification grants no motion authority.
- Station enabled/commissioned/automatic_start remain false and camera recovery remains absent/not_configured. This disarmed software deployment does not qualify physical recovery, home, gripping or sorting, and does not count as a full station cold-start qualification.
- Evidence: `deploy-63465aa/RESULTS.md`, `final-live-validation.json`, `old-process-exit-check.json` and `disabled-prepare-check.json` under the 2026-10-16-sorting evidence directory. Calibration continues with frozen TRAIN selections and independent scoring; no candidate is promoted yet.


## Read-only jaw diagnostics: qualified software and live deployment

- Published isolatedc945396/integrated3fe99d9 adds fixed zero-argument `/follower/gripper_diagnostics` and `gripper-diagnostics`. The resident owner reads12fixed jaw registers plus original pre/post six-joint position/torque guards, without diagnostic register writes, automatic connect/recovery or motion authority. Explicit stop/exit/transport-loss safety cleanup retains its separate existing authority.
- Final identical six-file source validation: system1709passed/91skipped/6subtests29.29s; station568passed10.58s; native695passed/4skipped79.89s, including38diagnostic cases plus real ROS prepare/recovery regressions. Native skips are EGL opt-in, two optional local GID builds and absent description checkout; runtime pinned GID imports passed. One timer-jump teardown error line is retained; pytest exited0. Earlier mixed-source native history is explicitly nonqualifying.
- Independent lifecycle/schema/performance review found and fixed malformed-data write escapes, stopped-idle rejection, serialization-success ordering and numeric metadata validation. Final ownerc3b0875b/bridge3976e962 accepted. FakeBus diagnostic p95.655ms/16reads/0writes; six periodic methods remain AST-identical and retain two idle reads. Final source before/after/root-index hashes match.
- Root verified coordinated station stop (both acknowledgements), then old bridge272577/owner1946294/API285928 exited before source integration. New bridge3634174/owner3634214/API3646748 use actual domain76/localhost/isolated native GID libraries; all six actual owner source pins match. Existing symlink installation serves the exact validated Python bytes; no interface or native dependency changed.
- The live diagnostic succeeded AFTER explicit verified stop:16reads in9.535ms, unchanged exact six raw maps and all torque bits zero. Measured raw values: CW/CCWdeadzone1/1; P/D/I16/32/0; Minimum_Startup_Force driverword16; maximum/current RAM torque limits500/500; load/current/status/moving0. This does not explain the four-count plateau or qualify any force/tolerance change. Installed two-byte startup-word interpretation remains distinct from the current generic vendor table's separate addresses24/25; no physical units conversion or EEPROM change was made.
- Existing EV3 unit/source/machine identity were verified remotely and ONLY the missing SSH forward restored (PID2601027, local18766→remote8766). New resident API reconnect and coordinated stop succeed; final workflowidle/faultnone/bothstopACKtrue/pendingfalse. Power-save cached EV3 measurements are not fresh physical-reference qualification. Station flags remain all false; camera recovery remains unconfigured; raw gripper remains1955 and elbow2993.
- Evidence: `2026-10-16-sorting/jaw-diagnostics-software/` (runs actually performedOctober19), `2026-10-19-sorting/jaw-diagnostics-review/`, `jaw-diagnostics-deploy/final-live-validation.json`, and `ev3-reconnect/RESTORATION.md`. Primary reference for the unresolved register-table interpretation: https://docs.waveshare.com/Memory_Map_Explanation/ST_Servo_Memory_Map_Explanation . All calibration, actual recovery/home and sorting acceptance gates remain open.
