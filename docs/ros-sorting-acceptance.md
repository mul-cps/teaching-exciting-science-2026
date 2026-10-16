# ROS sorting station acceptance — 2026-10-16

The station is **not physically accepted**. The implementation branch is
`feat/sorting-station-20261016`; the original dirty checkout is preserved.
The [plan](superpowers/plans/2026-10-16-ros-sorting-station.md) defines the
release requirements; the [progress record](superpowers/plans/2026-10-16-ros-sorting-progress.md)
records commands, limitations and decisions.

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

## Current hardware boundary

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
