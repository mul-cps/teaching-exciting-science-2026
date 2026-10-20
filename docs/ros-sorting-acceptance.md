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


## Overnight checkpoint — 2026-10-20 01:10 UTC

The main acceptance target remains autonomous ROS-only pick/place followed by
twenty consecutive successful physical sorts. No physical cycle occurred overnight.
The arm owner exited at 20:48:30 UTC after the unchanged 100 ms endpoint-hold
health guard. Its final six torque-off readings are historical; telemetry is stale.
There has been no owner restart, arm STOP, torque command, or calibration motion.
Fresh supported recovery and the physical calibration/home/grasp gates remain open.

- The independently sealed matrix at STAGED `e5ba970` passed: native75
  **3045 passed / 64 skipped**, station **2349 passed / 94 skipped**, isolated177
  **229 passed / zero skipped**. All 712 tracked hashes matched before/after and
  at the audit. The first two stages share 2390 case IDs; do not sum the counts.
- Same-job late local adoption is implemented, reviewed, privately built and
  deployed. Actual acceptance `463b8c87b9fb4f82bfcd2252a63ff329` resolved the
  already-completed job `ae1222a1bcab47c0bfed06b4d180a5a2` using fresh native and
  selected-image gates. It preserved the original attempt and deadline, did not
  reset the worker again, and remained disabled until explicit enable. Historical
  completion attestation does not replay old monotonic clocks as current proof.
  The retained transaction marker requires fresh confirmation after a later restart.
- A direct metadata-only observation captured genuinely ARMED source health:
  5.041921 seconds since stability began and 83.731 ms original coverage remaining
  at the check. This is an expired historical witness; current coverage is
  intermittent. No receipt/header lease or five-second criterion was enlarged.
- The one preregistered live perception trial ended **NOT_STARTED** after 6.575 ms:
  callback metadata was a dictionary while the harness expected an object. There
  was no baseline, process pause, watchdog, fault or new recovery job. Actual
  producer/worker identities and journal bytes stayed unchanged. The compatibility
  repair is separate software work, with no second live trial tonight and no
  automatic recovery qualification claimed.
- The reviewed idle telemetry patch remains staged, not deployed to the arm. It
  removes snapshot/log I/O before idle health checks and adds timing diagnostics.
  Archived data cannot separate the observed post-read stall from scheduling or
  publication cost; this is not a demonstrated physical fix.

Current camera configuration is native RGB 1920x1080 at 30 Hz and depth 1280x720
at 30 Hz. Prior bounded metadata observations measured roughly 29.98 Hz native
Info and 25.87 Hz derived Info; these are acquisition proxies under mixed workload,
not a controlled performance comparison or a pixel/geometry qualification. Main
ROS domain remains 76 with localhost discovery.

Evidence under `2026-10-19-sorting/`: `overnight/final-e5ba970/INDEPENDENT-SEALED-MATRIX-AUDIT.json`,
`automatic-perception-supervisor/INDEPENDENT-ACTUAL-LATE-LOCAL-ADOPTION-AUDIT.json`,
`automatic-perception-supervisor/actual-outage-qualification/INDEPENDENT-HISTORICAL-ARMED-PREREQUISITE-AUDIT.json`,
`automatic-perception-supervisor/actual-outage-qualification/INDEPENDENT-ACTUAL-TRIAL-NEGATIVE-AUDIT.json`,
and `overnight/INDEPENDENT-OWNER-TIMING-PATH-AUDIT.json`.

## Overnight checkpoint — 2026-10-20 03:10 UTC

The physical goal remains unachieved: one verified autonomous ROS-only pick/place cycle, then twenty successful sorts with at least five per color. **Physical cycles: zero.** The arm owner exited on October 19 at 20:48:30 UTC; its final six torque-off readings are historical. No overnight owner restart or physical motion is recorded. Fresh support, bounded recovery/home, calibration and registration, jaw/contact/TCP, grasp/release and placement gates remain open. MoveGroup’s captured optical transform differs from the pinned native calibration by 4.997 mm/0.403°; planner cache correctness and action quiescence require verification before motion.

Historical-adoption ACK and default-off, explicitly enabled private perception repair were conditionally deployed to perception only. The full matrix remains **negative**: native75 3044 passed/64 skipped/1 failed; station 2349 passed/94 skipped; isolated177 314 passed. All 727 tracked source pins matched before/after the sealed run. Native75 and station share 2390 case IDs; counts must not be added. The unchanged teleop case measured 98.972 Hz against its unchanged 100 Hz gate. The fixed twelve-trial AB/BA diagnostic passed 108.856–111.374 Hz but did not reproduce or explain that failure; its causal result is inconclusive and the original gate remains open. No control or threshold change followed.

One opt-in policy repair handled the existing natural fault `96f0…`: coordinator job `2807a4ce…` completed, and the **supervisor adopted** generation `9298…`, pin `6dae…` and new consumer epochs. One attempt was charged and the fault was durably deduplicated; no same-fault retry occurred. Replacement worker 2778568 was observed under the unchanged producer parent and the original worker identity was absent. Independent waitpid/reaping was not proved. Native camera and consumer processes were not restarted; their graph context was retained.

The observer’s full result remains **NEGATIVE**. All 136 retained coordinator replies reported `ready_now=false`; three of four frozen IMAGE witnesses had expired original receipt leases by the first completed reply’s take. Its later original-deadline error concerns missing FIRST current-completion evidence, not failed supervisor adoption. Both older witnesses were already expired before the last noncompleted reply: higher polling alone is not an established remedy. The exact coordinator internal completion timestamp was not exported.

Separately, nine historical SOURCE-ARMED callbacks had five-second stability and positive original lease margins. They do not replace current-at-take IMAGE evidence, establish semantic detection, grant present readiness, or qualify physical motion. The earlier controlled trial remains NOT_STARTED with no intervention and was not retried. The staged owner telemetry change is not deployed or proved to explain the owner exit.

Evidence under `2026-10-19-sorting/overnight/`: `final-94cf077/INDEPENDENT-SEALED-NEGATIVE-AUDIT.json`; `teleop-ack-wakeup/INDEPENDENT-SEALED-ACTUAL-AB-AUDIT.json`; `private-availability-repair/natural-repair-observation/INDEPENDENT-SEALED-ACTUAL-NATURAL-REPAIR-AUDIT.json` and `READONLY-COMPLETION-HANDOFF-ANALYSIS.json`; `movegroup-tf-audit/OFFLINE-OPTICAL-RELATIVE-AUDIT.json`.

## Overnight checkpoint — 2026-10-20 03:55 UTC

**Physical cycles remain zero.** One verified ROS-only pick/place cycle, then twenty successful sorts with at least five per color, remain unachieved. Owner 2070720 exited October 19 at 20:48:30 UTC; its final torque-off readings are stale. Fresh support/recovery/home, calibration/registration, jaw/contact/TCP, collision clearance, grasp/release and placement remain prerequisites. MoveGroup’s captured optical transform differs from native calibration by 4.997 mm/0.403°; planner cache correctness and quiescence remain unverified.

Completion-evidence source is integrated at LIVE `c636062` / STAGED `4351b82`; eleven reviewed feature paths match. Independent combined software verification passed **398 cases, zero skips**, including semantically overlapping copies of five regressions. Do not add author/earlier counts. Construction-time exposure is READ_ONLY and default false; legacy responses bypass event work. Historical completion evidence and current readiness remain distinct.

ROOT built an **inactive** private prefix: two packages in 2.20s, with 91 source and 85 installed-file checks and verified new-prefix cold imports. No-init/Node claims are pinned-helper attestations; no runtime environment snapshot was exported. **No live completion event, coordinator deployment or new recovery trial was qualified.**

The prior natural-fault repair completed and was adopted, but its CURRENT-IMAGE observer qualification remains negative. Nine historical five-second SOURCE-ARMED samples do not replace IMAGE or physical evidence. Segment03’s final 89.17s observed 25.513Hz derived metadata, producer ready438/438, board tracking434/434, and objects tracking339/427. These are received-message fractions, not uptime, piece identity or a controlled comparison; TF sampling covered only the first five seconds.

The full matrix remains negative on unchanged teleop **98.972Hz <100Hz**; the fixed AB/BA diagnostic was causally inconclusive. No threshold weakening or same-fault/controlled-trial retry occurred.

## Overnight checkpoint — 2026-10-20 04:40 UTC

**Perception is currently unavailable; physical cycles remain zero.** The supervisor latched after its latest automatic repair failed. No manual reset, retry, latch clear, policy re-enable, native-camera restart or arm action followed.

Two additional natural faults occurred after the earlier successful repair. Job `3bc040f1…` completed and was adopted as generation `7ad60c33…`. Subsequent job `39320f2c…` reserved generation `d7b37155…` but failed during producer warmup: source activity was lost before the required two new publications. The coordinator rejected this permanent reset failure before capture/rebinding, and the supervisor disabled itself without retry. Two charged attempts remain in the current hourly window; three historical faults are durably handled. The failed generation must not be confused with the last adopted generation.

Read-only snapshots found all six actor process identities unchanged and producer child 3202485 present. Process presence is not health. The producer’s one-shot callback probe still describes the earlier 01:15 fault, so it cannot explain these new faults. Native Info continuity does not prove image payload delivery. SDK, DDS and scheduling causes remain unresolved.

The preceding full post-repair hour is historical: derived metadata averaged 25.7333Hz; objects tracked in 14792/17136 received statuses and reported unavailable in 2310. These are message fractions, not current availability or physical qualification. Recorded-scene CPU analysis and whole-worker diagnostics likewise do not establish live failure causality.

Completion-event software remains integrated, independently reviewed and built in an inactive prefix. Optional object timing diagnostics are under isolated development; neither feature has been deployed to the running actors. Physical prerequisites and the unchanged negative 100Hz control gate remain open.

Evidence: `overnight/natural-private-fault-0434/INDEPENDENT-NEW-FAULT-AUDIT.json`, `ROOT-CURRENT-PRODUCER-PROC-SNAPSHOT.json`; `overnight/passive-night-segments/pipeline-night-segment04.INDEPENDENT-AUDIT.json`.
