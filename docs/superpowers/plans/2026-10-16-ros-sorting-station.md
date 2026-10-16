# Complete and verify the ROS sorting station

## Outcome

Deliver reliable EV3 dispensing and colored-brick pick-and-place, with RViz and the MuJoCo web twin displaying the same authoritative ROS state.

Default release target: the attached rig using RealSense and EV3dev. Preserve standalone simulation and alternate firmware configurations, with their validation status stated separately.

## Implementation

1. **Restore reproducible startup**
   - Preserve the existing dirty changes and integrate verified work through isolated worktrees.
   - Rebuild the ROS overlay and verify installation in a clean prefix, including the new plane module and both Python environments.
   - Use one station launch, CycloneDDS by default, one camera owner, and one follower owner.
   - Require measured-home completion before teleop or automatic operation. Retain the requested uncapped teleop velocity profile.
   - User follow-up: implement automatic camera-guided recovery from a recoverable displaced arm pose, with fresh RGB-D and encoder observations, calibrated model/registration, checked collision clearance and continuous cancellation/watchdog monitoring. Keep ordinary operating bounds and firmware limits intact; unavailable vision or actuator transport must produce an explicit recovery failure.

2. **Finish calibration and viewer consistency**
   - Fit the measured ChArUco depth plane; constrain the arm and LEGO mounting geometry to that plane.
   - Resolve the gripper’s additional 180° mounting correction against held-out depth observations, then save one effective model for RSP, MoveIt, RViz, and MuJoCo.
   - Integrate the cached global optimizer with known tag associations; reject ambiguous fits.
   - Fix static-TF timestamp handling and transform clouds at acquisition time.
   - Replace the legacy viewers with the passive ROS-driven twin. Treat mathematical model parity and physical registration accuracy as separate checks.

3. **Complete MoveIt grasp execution**
   - Diagnose the reported planning failure using native responses, MoveIt logs, start-state validity, and the actual planning scene.
   - Calibrate jaw opening and TCP geometry. Add a standard ROS gripper action backed by the existing resident controller; retain native percentage units internally.
   - Extend the existing grasp tool with explicit execution: lock target → open → approach → close → verify pickup → lift → place → release.
   - Serialize tracked-object ownership through attachment and detachment so tracking cannot recreate or expire an attached brick.
   - Revalidate target and scene before motion. Cancellation, lost localization, or failed pickup stops the cycle without dispensing another brick.

4. **Connect and package the complete workflow**
   - Verify EV3 identity, sensors, homing, feeder endpoints, and the persistent API.
   - Dispense one brick, confirm a stable detection, then run the grasp sequence to a reachable clear color destination.
   - Provide one documented installation/startup path, current diagnostics, and a truthful acceptance register.
   - Use parallel Luna workers for calibration/twin, grasp execution, and startup/testing; serialize physical experiments through the sole controller.

## Verification gates

- **Software:** full suite, native ROS tests, package builds, and clean-install smoke pass. Current baseline: 802 tests passed, 28 skipped; 69 control tests and 16 perception tests passed.
- **Calibration:** held-out pickup-position error ≤5 mm; support-plane residual ≤3 mm; mounting correction validated across multiple wrist poses.
- **Twin:** effective-model hash matches ROS; tool-transform disagreement ≤1 mm and ≤0.5° at synchronized samples; stale data is visibly identified.
- **Teleop:** 30-minute run without unexplained stops, sustained ≥100 Hz control updates, and verified disconnect/watchdog behavior.
- **Sorting:** 20 consecutive successful physical cycles, at least five per color, with recorded pickup, lift, and release evidence. No hidden retries.
- **Recovery:** verify missing brick, failed grip, feeder jam, camera/localization loss, cancellation, and device reconnect.
- **Release:** three successful cold starts, saved stills/videos and trajectories, updated documentation, and verified commits pushed to the requested repository.

## Completion rule

Do not mark the goal complete until the physical sorting and live twin gates pass. Record unavailable or failed checks explicitly and continue from the remaining gate.

Source: Codex session `01a12fe0-e9e3-7462-8979-aec0d147bee3`, assistant plan recorded `2026-10-16T07:36:55.248Z` (09:36 Vienna).
