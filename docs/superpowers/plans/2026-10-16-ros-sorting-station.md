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


## Recovery replanning: stationary camera recapture

The native external-producer test demonstrated that continuous encoder motion
cannot supply identical-raw samples bracketing each camera exposure. The current
original-scene expiry therefore stops recovery before the normal-range target.
Implement explicit stationary recapture between certified movement segments:

1. Certify at most a two-count waypoint using the existing immutable acquisition,
   reach it, and verify its exact commanded hold before that proof expires.
2. Enter a separate stationary-only wait, at most 500 ms and within the unchanged
   six-second operation deadline. Issue no Goal, torque-enable or profile writes.
   Continue actual fresh encoder/torque reads, exact held-pose checks and immediate
   stop/cancel handling. This phase grants no movement authority after proof expiry.
   Retain its original immutable hold context through candidate proof and final
   settled verification; a normal same-generation identity renewal may resume
   only that same verified hold within its original deadline. Never reset the
   wait, carry old quiet-sample authority, or retry a motor write.
3. Require typed cached producer health. Only explicitly transitional bracket
   unavailability may permit waiting. Missing/stale/malformed health, identity or
   generation changes, stream loss, unknown geometry and proof faults abort.
   Filesystem and geometry preparation stay outside the owner critical path.
4. Continue only from a strictly post-hold original camera exposure with identical
   held-raw bracketing, matching identities and a fresh full clearance proof.
   Preserve the 100 ms pairing, 50 ms current binding and 250 ms acquisition/proof
   bounds. Never refresh old timestamps or reuse an expired motion proof.
5. Verify the complete original synthetic trajectory and stop, cancellation,
   camera loss, pose drift, identity change and timeout cases independently.
   This software gate does not qualify real calibration or physical recovery.

The original physical acceptance gates above remain required.


## Confirmed remaining ROS orchestration gap

The resident API currently exposes recovery and measured home as separate ROS
operations; sorting run does not compose them. After the staged recovery package
passes its software gates, implement one serialized arm-preparation sequence
shared by an explicit arm preparation API and authorized sorting-run preflight:

- Hold one follower reservation across recovery, measured home and readiness;
  direct motion/reconnect clients must not interleave, and stop retains priority.
  Use cached owner-computed typed bounds classification rather than duplicate
  calibration math in the station coordinator.
- Read fresh owner status and enforce trusted configuration, calibration,
  identity, fault, stop and observe-only admission gates.
- For a qualified bounded soft excursion, request recovery once and verify its
  fresh held success and invalid home state. Do not reset faults or retry.
- Request measured home once, then verify fresh sorting readiness before any
  dispense. Already-ready arms should not repeat preparation motion.
- Cancellation and stop use the existing coordinated stop path. Report phase
  and failure through cached ROS status. Keep EV3 references and feeder
  qualification as separate mandatory station gates.

This closes an actual missing composition; API availability alone does not prove
an autonomous startup or sorting workflow.
