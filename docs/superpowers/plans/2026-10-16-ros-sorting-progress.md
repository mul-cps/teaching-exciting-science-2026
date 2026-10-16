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
