# Sorting station implementation evidence

Plan: [Complete and verify the ROS sorting station](2026-10-16-ros-sorting-station.md).

## 2026-10-16 baseline

- Integration branch: `feat/sorting-station-20261016`, base `2ffe8b2`. Original dirty checkout preserved; patch saved outside Git.
- Independent worktrees own startup/packaging, calibration/twin, and grasp execution. Integration owns EV3-to-ROS orchestration and acceptance evidence.
- Clean branch software baseline: `python -m pytest -q` -> 797 passed, 29 skipped in 13.29 s. Original `.venv` has no pytest; system Python used.
- USB enumeration: EV3 `0694:0005`, D435i `8086:0b3a`, follower serial `5AB9068906`, leader serial `5AB9068765`. Enumeration does not prove motor/camera operation.
- No resident ROS/controller process or live API tunnel found. Station config has `enabled=false`, `commissioned=false`, `automatic_start=false`.
- Saved measured home dated October 14 is a torque-off measurement with `geometry_commissioned=false` and `grasp_commissioned=false`. Historical invalid-home notes are superseded by this file, but live arrival remains unverified.

## Interface and requirement review

| Tasks | Producer / consumer | Finding |
| --- | --- | --- |
| Startup / grasp | Single resident owner / arm and gripper actions | Must share one admission lock and serial owner. |
| Calibration / grasp | Qualified camera, TCP and jaw mapping / execution | No physical qualification can be inferred from fit convergence. |
| Grasp / orchestration | Explicit execution result / next EV3 dispense | Next brick requires verified pickup, lift, placement and release, never planning success. |
| Startup / twin | Effective robot description and timestamped ROS state / passive viewer | Keep one camera and no viewer actuation. |
| Calibration | Held-out measurements / <=5 mm position and <=3 mm plane gates | Existing local reports explicitly unqualified. |
| Teleop | Measured home, uncapped velocity, 100 Hz / 30-minute test | Preserve uncapped teleop; validate rate separately from trajectory cadence. |
| Release | Test and physical artifacts / acceptance register | 20 real cycles, recovery and three cold starts remain pending. |

Ruling: implement the existing plan directly, keeping its full physical gates; no separate design spec was referenced. The saved plan is the authority.

## Implementation wave 1 and live diagnostics

- Integrated startup, calibrated SI gripper action, scene/execution prototype, optimizer convergence rejection, acquisition-time cloud transform regressions and MoveIt start-state diagnostics.
- Startup worker: clean symlink and non-symlink builds both passed 17 packages; native control tests 10/10, native perception/control pytest 75 passed, mock MoveIt start state valid/no collision with seven-point trajectory. Artifacts: `/tmp/sorting-startup-validation-20261016`.
- Root added finite single-brick orchestration and distinct-image stable target selection; no next dispense after unverified pickup/lift/release. Adapter requires explicit automatic commissioning, measured-home readiness query, fresh localization, clear pickup ROI and a commissioned clear joint pose. Child client stops before final action cancellations.
- Camera-recovery planner added with regression tests: at most 32 counts inward to two counts inside admitted bounds; firmware bounds are immutable; complete acquisition-time collision oracle with >=10 mm clearance and qualified matching model/calibration required for all <=2-count path samples. **Planning module only: live collision-scene adapter, owner admission/execution and continuous camera watchdog are not yet integrated or validated.**
- Root focused tests: 32 passed (sorting engine, adapter, camera recovery planner). Initial full suite after integration: 816 passed,31 skipped,2 failed because commission CLI tests used the live station lock. Tests now use isolated temporary lock files; focused station tests pass. Full rerun pending review fixes.
- Arm read-only connection succeeded; last measured elbow raw2993 /91.56deg versus configured90deg; all torque states verified zero. No arm motion was sent.
- Camera SDK identified D435i serial243322071783 firmware5.17.0.10. Native ROS captured a paired RGB8/Z16 frame at zero timestamp skew, visually inspected. Saved in `~/.local/share/exciting-science/station/evidence/2026-10-16-sorting/rgbd_pair_ros_driver`. Repeat starts failed with UVC -71/USB re-enumeration. This is not a stable camera gate.
- At10:37:55 the separate USB5 hub subtree carrying both arm adapters and EV3 disconnected. `/dev/serial/by-id` disappeared; resident arm process exited with serial write timeout. Last torque sample was off. No reconnect or motion replay was attempted. User notified to check hub cable/power.
- Independent review found execution blockers including native parameter/YAML handling, unbound NumPy, target revalidation race, repeated-frame proof, cancellation ordering/contracts, absent clear return, tracker ownership and unresolved scene transactions. Fix workers are active; execution remains unqualified.

Ruling: the user requested autonomous camera-guided recovery rather than manual elbow repositioning; retain original sorting acceptance gates and add recovery as a prerequisite. A planner alone does not satisfy this requirement.
Ruling: do not infer successful recovery from encoder-only motion, stale images or a vision callback returning a fabricated clearance. Hardware transport and a qualified scene must be available before the live path is admitted.

## Review corrections and remaining work

- Root recovery planner now binds the camera serial and exact encoder snapshot as well as model/calibration hashes; mismatched source/sample regression tests failed before the fix and pass afterward.
- Root adapter now terminates/reaps the goal-producing client before final action cancellations, attempts both cancellations even when child shutdown fails, and waits for terminal GoalStatus for acknowledged cancelling goals. Regression tests cover late-submit ordering, process-exit race, forced-shutdown timeout and cancellation-accepted-without-terminal-state.
- Root integration clean-build: all 17 packages passed. Native ROS control, scene, map, gripper and grasp subset: 45 passed; RCL logged a timer callback cleanup diagnostic after tests. This is not physical execution evidence.
- Independent scene fixes are in progress with native regression tests; execute CLI fixes are in a separate worker branch. Full suite remains pending until these are integrated.
- Camera-guided automatic recovery requirement is broader than the small bound-reentry planner: general recovery planning, complete acquired collision scene, owner-side path verification, live vision watchdog, and physical recovery trials still remain. Do not present the bounded helper as completed autonomous recovery.
- Physical sorting, calibration accuracy, 30-minute teleop, recovery experiments and three cold starts remain unverified. The USB hub outage still prevents arm/EV3 access.

## Integrated verification checkpoint

- HEAD includes scene ownership fix91d4fbd, grasp cancellation/evidence0395cbd, canonical-ID/atomic-submit/localization fixesa12edf6, and jaw launch forwarding1ada560.
- Latest root full suite: `python -m pytest -q` ->865passed45skipped in12.01s. Native ROS control/scene/objects/map/gripper/grasp/ownership subset ->84passed in1.38s.
- Integration clean-build rerun ->17packages passed in19.9s; log `/tmp/sorting-integration-clean-build-20261016.log`.
- Final registration gate moved into common action submission after discovery, with regression tests for registration expiring during discovery and execution deadline expiry; no goal is sent in either case.
- Sorting adapter forwards its operation deadline to the grasp child and writes the commissioned clear-joint map as an explicit evidence file.
- User again authorized running the robot; root attempted a fresh resident connection. It exited with `arm persistent USB identity absent or ambiguous`. Both arm adapters and EV3 remained absent in current enumeration. No motion sent.
- Goal remains active. Do not mark completion or claim physical validation.

## Latest software checkpoint

- Full suite after all current code changes:880passed45skipped in15.89s. Native control/scene/objects/map/gripper/grasp/geometry/ownership subset:99passed in1.91s.
- Real native GripperCommand round trips now exercise real ArmSession with FakeBus, not an owner that accepts impossible legs. Full range previously failed the650-count per-leg budget. Dedicated `move_gripper` command now splits the range, holds all body joints, stops at first inferred contact and confirms cancellation; existing per-leg/per-frame/firmware limits remain. Commit60fbefd.
- Corrected tracked grasp pose geometry: candidate +Z is approach toward the object, so pregrasp/lift move opposite that direction. Qualified `tip_from_grasp` in the declared `x_closing_z_approach` frame is inverted/composed into all pickup link targets; `tcp_from_object` remains separate. A rotated/nonzero-offset regression proves physical contact60mm pregrasp and80mm lift heights, and explicit tool-place poses remain unchanged. Missing/ambiguous grasp-frame calibration rejects execution.
- Latest live connection attempt still failed `arm persistent USB identity absent or ambiguous`; EV3 SSH also timed out. Last read-only arm state and logs copied into the durable local evidence folder.
- Remaining work: restore physical connections and repeat reliable RGB-D cold starts; qualify camera/base/jaw/TCP/model from held-out data; integrate and validate live camera-guided recovery; perform actual grasp/sort/recovery/twin/teleop/cold-start acceptance. All original gates remain required.

## Continuous recovery collision proof

- Recovery planning now requires a continuous swept-volume oracle for every adjacent sample, in addition to complete-robot pose clearance. Two-count encoder sampling alone does not prove geometric clearance between poses. Missing, unknown or insufficient swept-volume proof rejects the plan.
- Focused planner regressions: `python -m pytest -q tests/test_camera_recovery.py` ->14 passed. Three new cases failed before the fix: a pose-only adapter, an unknown swept segment, and a segment oracle never being called. Every segment now contributes to the returned minimum clearance.
- A real URDF/depth collision adapter and native grasp CLI subprocess harness are in progress. These tests do not qualify live recovery or physical sorting.
- Latest user-authorized run preflight still found no `/dev/serial/by-id` and no EV3 USB device. D435i remains enumerated; no actuator commands were sent.

## Reconnection and native execution checkpoint

- After the user requested another connection check, both expected serial identities returned (`5AB9068906`, `5AB9068765`), along with EV3 USB and `usb0`. SSH using the station EV3 key confirmed machine-id `0a8ffb6d314b403ab9cb5dc4ba3f599d`; the persistent live API returned idle/power-save status with both motor references still unset.
- A new resident read-only connection succeeded and shut down on stdin EOF. All six torque bits were zero; elbow remains raw2993/91.56deg. Saved state: local evidence `reconnected-arm-state.json`. No goal/profile/motion was sent.
- Fresh native ROS RGB-D start delivered201color/205depth/200aligned frames over15seconds, saved a zero-skew paired image and exited cleanly. Evidence: local `rgbd_reconnected/`. Visual inspection confirms the station scene; this does not qualify camera/base registration or repeated cold starts.
- New actual CLI subprocess tests exercise native ROS services/actions end to end: complete synthetic pick/place; lost localization before any arm goal; interruption during lift with terminal canceled goal and retained scene attachment. Native result:3passed in5.80seconds. Synthetic calibration records are test fixtures only.
- SIGINT previously shut down the ROS context before the CLI could confirm action cancellation. Execution now handles SIGINT/SIGTERM through Python interruption, keeps ROS alive during cancellation, and shuts it down afterward. The native interruption regression passes.
- Recovery now checks encoder acquisition freshness throughout collision evaluation as well as camera freshness. Focused camera/grasp geometry tests:34passed2skipped; camera planner alone15passed. Physical acceptance gates and live recovery integration remain incomplete.

## Repeated camera startup evidence

- Three sequential real native ROS camera starts after USB reconnection each ran15seconds and exited cleanly. Color/depth/aligned counts were201/205/200,201/206/201 and201/203/200. Each saved pair had zero color/depth timestamp skew. Durable local manifest: `rgbd-reconnection-cycles.json` beside the three `rgbd_reconnected*` capture folders.
- These are camera-driver start/stop cycles, not the plan's three complete station cold starts. No robot motion or EV3 dispensing occurred during them.
- Root independently ran current depth-scene/planner tests:43passed in0.42seconds. The new adapter remains under development; representative real-CAD checks exposed unresolved self-contact and unknown clearance, so no live recovery admission is claimed.
