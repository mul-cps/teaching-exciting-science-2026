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

## Geometry and live visual checkpoint

- Geometry adapter commits01deaa2/20de3a1 add effective collision geometry, calibrated joint FK, conservative projected-depth volumes and continuous segment proof. Dedicated tests include real SO101 CAD and rendered current-self occlusion. Hull enclosures of concave/unwatertight meshes cannot supply physical self interior; current SO101 triangle-soup STL representation supplies zero certified interior cells. Single-view physical recovery remains unqualified.
- Latest root full suite before the subsequent plane-sampling change:915passed46skipped13.52seconds. Native ROS subset, including actual CLI subprocess tests:102passed6.96seconds; a nonfatal RCL timer cleanup diagnostic remains.
- Opened the integrated live station/RViz at the user's request, runtime state `/dev/shm/so101-sorting-20261016/rviz-arm-state.json`; all torque bits verified zero. ROS domain is0 (unset), not the earlier assumed73. Keep synthetic native tests isolated in domain75. Overlay/base registration and visual gripper angle remain provisional.
- First saved frames failed the existing board depth sample/coverage gates. Found an actual sampler defect: its stride was based on the whole image even for a small projected board, discarding usable board pixels. Sampling now uses the clipped board region, preserving the2000-point cap and unchanged80-inlier/8-region/residual gates. A small-board regression failed with24samples before the fix; six native plane tests pass after it. The sparse-coverage fixture now confines data to two board columns so denser valid sampling still exercises actual insufficient coverage.
- A warmed30-second capture saved a late exact-synchronized RGB/depth/aligned triple. Current plane diagnostic:13corners,0.216px reprojection RMS,1967/2000inliers,10coverage regions,0.513mmmedian/1.736mmp95 plane residual. First frames still reject insufficient coverage. Evidence: local `rgbd_warmed/` and `board-plane-roi-sampling-diagnostics.json`. This is static board evidence only, not held-out pickup accuracy or qualified camera-arm registration.
- User requested background repair of colored-brick tracking beyond the pickup bounds. Assigned tracker observation/support/pickup separation and dynamic sequence tests; execution bounds must remain independent and unchanged.

## Owner recovery admission and tracking proof boundary

- Commit812f79f adds resident `recover_camera` with trusted in-process scene provider/policy injection. Commands cannot supply target/path/clearance/policy. Owner derives the bounded inward reentry, binds the current calibration and encoder acquisition, checks continuously swept remaining geometry, and holds on expired/mismatched vision, drift, unexpected motion, cancellation or deadline. Normal home/sag/firmware limits are unchanged. Production CLI has no provider configured, so this is not live recovery admission.
- Root independently verified113focused owner/session/planner tests in0.37seconds. Native actual grasp CLI/proof tests in isolated ROSdomain75:17passed5.68seconds, with existing nonfatal RCL timer teardown messages.
- Broad tracking adds visible-surface positions outside the pickup support region. Such a centroid is not an object-center measurement. New failing-before-fix regressions showed that the old lift/release predicates accepted those records. They now reject explicit visible-surface-only geometry for center-based proof. Supported geometric box estimates retain the existing globally calibrated execution gates.
- Remaining proof work: independent lifted/released rigid-object localization outside the support/pickup ROI. Tracking continuity alone must not be reported as verified pickup or release. Physical sorting, live recovery, calibration, teleop, twin and full cold-start gates remain open.
- Live read-only state remained fresh with all six torque bits zero. RViz, camera and measured-state owner remain running. Tracking workspace changes are under final review before replacing only the object-perception process.

## Workspace tracking applied live

- Committed/pusheda15a581 separates full valid camera observation from support/pickup ROIs, adds bounded time-aware association/ambiguity handling and explicit expiry. A straddling observed component cannot produce a truncated in-ROI pickup fit. Dedicated scoped native tests42passed; both object and station launch expose the new arguments.
- Replaced only the object tracker (oldPID445117) after verifying fresh torque-off state. New process runs in execsession55619; live log `/dev/shm/so101-sorting-20261016/workspace-tracking.log`. RViz, camera, MoveIt and resident owner were left running. No motion sent.
- Fifteen-second domain0status recording:66fresh reports, objects beyond oldX/Y/Zbounds retained as visible-surface observations, with no corresponding grasp proposals outside pickup admission. Before/after and per-track evidence saved locally as `tracking-before-workspace-update.json`, `tracking-workspace-live.json`, `tracking-workspace-live-summary.json`. A colored-component detection is not independent physical brick identity validation.
- Root current full suite962passed46skipped17.89seconds; native tracking/objects/plane/actualCLI/proof/ownership subset64passed6.47seconds.
- Live residuals: blue tracks show fragmentation and red pickup eligibility flickers despite small position changes. Do not claim full dynamic acceptance. Review also identified a remaining table-plane projection in visible footprint geometry that can discard an otherwise visible lifted/vertical brick; follow-up now replaces this with3Dsurface axes and rotated sequence tests. Pickup gates remain unchanged.

## Preview eligibility and conveyor false-positive diagnosis

- Commit a10eb46 replaces table-XY visible footprint projection with3Dprincipal surface axes; a6x4cm synthetic face rotates0–90degrees while retaining its track. The6.5cm component ceiling stays unchanged. Supported full-component geometry outside pickup bounds can publish contact/pregrasp previews; records distinguish inside/outside pickup eligibility and remain unvalidated by IK/collision.
- Execution consumers now require explicit `pickup_eligible:true`: CLI before opening/moving and on target revalidation; automatic sort selector requires both candidate and object admission. Missing/false/string/integer eligibility cannot authorize execution. Geometric preview selection remains available without execution. An actual native CLI regression proves preview-only records cause no gripper commands, arm goals, scene requests or plans.
- Fixed a native test-fixture race: its resident-state refresh wrote JSON in place, briefly exposing invalid/empty state to the bridge. It now uses atomic replace, matching the real owner's contract. A cross-OpenCV preview fixture sat near the55mm jaw threshold; the synthetic cube now has comfortable margin, with production tolerance unchanged.
- Current full software suite971passed46skipped17.20seconds; native tracking/preview/CLI/consumer subset72passed6.93seconds. Existing nonfatal RCL timer cleanup diagnostics remain.
- Restarted only object perception with reviewed preview code, execsession35594; camera/RViz/owner stayed running. Five-second read-only probe received23tracking reports with explicit eligibility fields. No actuator motion sent.
- User reported floating nonexistent objects around the LEGO conveyor. Background validator work captured15exact synchronized RGB/raw/aligned-depth frames and50statuses before the preview restart, under local `tracking-ghost-diagnosis/`. Evidence identifies a large blue background/glass component with21.9percent valid depth and1.57mextent; dropping invalid pixels before splitting creates plausible-looking floating islands. Eight real bricks have full depth support in this capture. Rejection rules are being evaluated against real captures, touching groups and dynamic/tilted objects; the ghost fix is not yet applied or qualified.


## Conveyor ghost rejection and live processing freshness

- Canonical depth validation now keeps original color support before depth splitting, cuts depth discontinuities, and rejects insufficient local depth support. Development replay preserved112/112 real brick observations while reducing82 nonbrick fragments to13; coherent colored hardware can remain. No workspace exclusion was added.
- The first live implementation failed freshness (median0.667s), so its stale empty outputs are not positive evidence. Bounding local mask computations restored identical replay results with analyzer median0.124s; subsequent44/44 live reports were tracking (median age0.220s).
- Commits719c965/a19cea9 add one running/one replaceable pending analysis job and resolve strong mutual-nearest matches before unresolved ambiguity. A stale chair track previously vetoed clear matches to both real yellows. Latest51/51 reports tracked at median0.190s/max0.260s, with both real yellow references present50/51reports. This is stationary correspondence evidence, not full dynamic qualification.
- New module packaging initially prevented tracker restart; rebuilding the canonical colcon symlink overlay resolved the missing module. Future restarts preflight installed imports. Only perception was restarted; camera, RViz and measured-state owner stayed running, and all observed torque bits remained zero.
- Commit9616ee7 adds final-child silhouette validation at0.45 rectangular fill. Captured contour replay preserves320/320 real brick contours across three captures. Separately,334 fresh real observations had fill>=0.620 while29 chair patches had fill<=0.407. Coherent red hardware can still pass. This evidence does not establish semantic LEGO identity or unseen-view performance.

- Final-child shape integrationce1bba1 is live after installed import preflight and tracker-only restart (execsession65697). Root independent focused suite48passed; worker native/system suite60passed each. Fresh12-second capture `tracking-shape-live/` contains15synchronized frames and60/60tracking reports: all eight actual table bricks present in all60reports,55reports exactly8objects,5reports a ninth yellow chair fragment. The latter has final-child rectangular fill0.613 after fragmentation, so shape filtering alone is insufficient. Background investigation continues; do not claim all ghosts removed.

- Root full suite after final-child integration:1008passed46skipped in17.34seconds. Acceptance register refreshed to distinguish restored live transport and camera-only repeated starts from still-open full-station/physical gates.


## Original-pixel evidence and passive twin qualification

- Morphological closing and external-contour fill could manufacture a convex-looking child from sparse chair color. Commits50d09b0/1a2b4f8 preserve the original per-child color/depth mask and require original boundary/pixel-cell support at the unchanged0.45threshold. Replay retains432/432 actual brick observations across four captures. Root65focused tests passed after boundary implementation; earlier missing-API/comb/real-chair regressions failed as expected before their fixes.
- Replaced only perception again after installed API preflight and fresh torque-off check (execsession51470). New15-second `tracking-observed-live/` capture has19matched frames and76/76tracking reports, actual8bricks in everyreport;74reports exactly8objects and two singleton extras (blueimageedge, yellowchair). Receipt age median0.2005s/p950.2160s/max0.2684s. Blue revealed the observed-evidence branch could relax the existing reconstructed contour gate; all evidence must intersect. The remaining yellow singleton motivates temporal acquisition confirmation. These are still active repairs, not completed ghost qualification.
- Commit82f6018 fixes plain-dict object snapshots, package mesh URI joining and false stale detection from comparing fresh joints to a timeless static anchor. Reproducible live read-only twin diagnostics record identical raw RSP/MoveIt model SHA d8b4c027e7ee6e2ac52721345f4b0b65cb8ffc146ce57f02ed80932590fac6a8,6fresh dynamic arm TFs and30/30fresh observations, with MuJoCo tool transform error1.24e-16m/0degrees. This qualifies mathematical parity only; physical registration remains false.
- Actual viewer-only launch then failed because stationvenv lacked viser. Root stopped only that twin launch; original station/RViz/camera/owner stayed running. Canonical installer/dependency/default-Python repair and graceful bridge shutdown are being completed before another live viewer start.

- Commitc10b26c intersects all three silhouette gates (reconstructed contour, original boundary, original pixel-cell density); original evidence cannot relax the earlier contour gate. Five-capture full analyzer replay retains576/576 real observations. Raw sampled shape-live/observed-live frames emit no extras; earlier captures retain14coherent colored surfaces. A singleton yellow in status between saved RGBD samples passes all three shape checks, so ROS acquisition probation is being implemented with3consecutive unambiguous acquired frames and a bounded pending pool.
- Second viewer-only start succeeded through imports/server creation but failed rendering800x640 against MuJoCo default640wide framebuffer. Only the twin launch was stopped; bridge exited cleanly with the repaired shutdown. A real rendering smoke and explicit framebuffer sizing are required before viewer acceptance.
- Recovery mesh audit preserves exact triangle positions through vertex welding and records source-unit geometry plus explicit URDF instance scales. All13meshes/17instances enumerate3,541,005candidate triangle pairs (~11.6s); this is not an intersection proof. Twelve meshes are watertight after welding, but winding/topology do not certify material interior, and nested jaw cavities must not be filled by shell unions. Production collision/recovery admission remains unchanged and fail-closed. Root8focused audit tests passed; audit artifact is under local evidence2026-10-16/recovery-mesh-audit/.


## Live ghost suppression verification

- Commit6b2a9f6 requires three consecutive independent unambiguous observations before ROS publishes a new track. Pending candidates are bounded64; failed/ambiguous/missing acquisitions reset probation, while established/owned identities retain their prior motion and ownership behavior. Review caught and fixed ambiguous-frame evidence retention and rejected-future-timestamp highwater poisoning before rollout. Native focused113passed; offline replay retains8real/0extra in all74post-confirmation reports.
- Root verified installed API, fresh torque-off state and then replaced only perception (execsession14312). Live ROS parameter `acquisition_min_frames` is3. Thirty-second capture `tracking-confirmed-live/` saved38paired RGBD frames and150statuses. Every report was tracking with exactly8real table bricks and0extras; all1200published object observations were confirmed with3acquisitions. Receipt ages median0.2757s/p950.2905s/max0.3265s with passive twin also running. `root-summary.json` uses stationary eight-brick image annotations only, never a runtime ROI. This passes the reproduced stationary floating-object check; dynamic/occlusion/semantic generalization remains unqualified.
- The passive viewer now starts, renders and visibly reportsFRESH after installer/environment/framebuffer fixes0a6124. Browser inspection exposed further preexisting display defects: parent-local TFs drawn as world poses, all marker kinds drawn as gold spheres, and MuJoCo world-attached root-link geometry left at its original pose. Current failing regressions prove these issues; fullsuite during that TDDstage1055passed50skipped3failed. Tool-only mathematical parity does not prove complete visual model parity. Repairs continue before final twin acceptance.


## Final software and stationary twin checkpoint

- Commitse4ad720/25d60db correct root-frame TF/object composition, metric marker boxes/text/pose markers, world-attached MuJoCo base geometry, empty-root-geom handling and weak model baseline caches. Root full suite1061passed50skipped in18.51seconds; independent native domain75 tracking/validation/worker subset94passed; stationvenv EGL/twin/dependency subset19passed. `uv pip check` confirms106compatible installed packages.
- Root fresh collection/evaluation artifacts `root-live-twin-inputs.json` and `root-live-twin-parity.json` confirm identical raw RSP/MoveIt hashes,6/6fresh arm TFs,0stampdelta and tool disagreement1.241e-16m/0degrees at the current stationary measured pose. Physical registration is explicitly false. Browser inspection verifies corrected root-frame axes, actual marker types and useful initial framing.
- Tested actual passive-bridge loss by stopping only twin childPID869701. Browser visibly changed toSTALE/WAITING (`stale_bridge_snapshot`) and hid cloud/axes/markers while identifying cached image. Clean twin-only restart restoredFRESH and live cloud/markers. Final twin execsession34939,viewerPID968132,bridgePID968134,URL http://127.0.0.1:8099. OriginalRViz/camera/owner stayed running throughout; owner remained fresh torqueoff.
- Live read-only `/follower/sorting_ready` currently returnsfalse: fresh matching owner, completed measuredhome in this session, qualifiedjawcalibration, idletrajectory and disabledteleoperation are required. Elbow is still91.56degrees beyondconfigured90. No actuator motion was sent. Qualifiedcamera/base/jaw/TCP physical data, actual recovery/sorting/teleop/endurance and fullstationcoldstarts remain open; this software/stationarycheckpoint does not complete the plan.


## GPU rendering and bounded depth follow-up

- A manual RViz restart omitted the launch-local host DRM preload and logged radeonsi initialization errors. Restoring the host DRM libraries and AMD hardware driver settings removed those errors. Kernel amdgpu counters for live RViz PID2584324 increased by44,924,072ns graphics and2,353,070ns compute over2seconds; evidence is `evidence/2026-10-16-sorting/rviz-gpu/counters.json`. This proves GPU work by this process, not a controlled performance benchmark.
- A shared fail-closed RViz GPU launcher is being integrated for resident, perception and mock/Servo launch paths. CPU fallback must produce an explicit startup error.
- The wider848x480 RGB profile exposes the provisional tool projection while retaining640x480 native depth at15Hz. Strict registration capture now binds camera serial/firmware, nearest resident raw sample, ROS joints and frozen station hashes; all physical calibration remains unqualified.
- Far-depth detector commit39dd21a defaults to1.2m and skips expensive geometry for wholly distant components while preserving original color support. Paired six-capture replay retains720/720 real observations at both2.0m and1.2m; modest median124.10→120.99ms is host-load-dependent evidence. Near colored machinery remains a semantic identity limitation. Shared producer/detector limit forwarding is under integration; do not treat this as live producer clipping yet.
- Offline CGAL prototype c4f16c9 adds exact topology/interior-cell proofs with pinned build/source, immutable sealed execution and bounded resources. Twelve frozen meshes prove topology; the known motor-holder membrane is rejected. This does not qualify recovery motion or physical registration.
- No actuator motion was sent; actual recovery, physical calibration and supervised sort acceptance remain open.

- GPU guard a918c20 is installed and used by resident/perception/mock/Servo launch paths. Packaging was corrected to the ament `lib/so101_station_gpu` executable directory and verified through `ros2 pkg executables` and `get_executable_path`. Actual guarded RViz on domain76 (PID2972225) reports Radeon8060S and added40,086,608ns graphics work in2seconds; `rviz-gpu/domain76-gpu.json` and station log record the proof.
- At user request, cleanly stopped all prior station ROS processes on domain0, verified the old camera/owner processes exited, and restarted the full station and passive twin on domain76 with local-only discovery. The new owner is fresh with all6torque bits0; no motion was sent. Process inventory records all active station ROS participants on76.
- Producer clip_distance1.2 and detector max_observation_depth_m1.2 are live. An8-second read-only sample received121pointclouds, maxZ1.197m, zero points beyond1.2m, and40tracking reports. Latest report contains9colored surfaces/3grasp previews, so near hardware semantics remain unresolved. This is transport/range validation, not automated-sort acceptance.

- Domain defaults/guards6d93879 and isolated-test fix5626436 verified: actual unset-domain wrapper reports76/LOCALHOST; explicit0exits2 before invoking ROS. Root native camera/GPU/domain subset42passed. Root full system suite1140passed53skipped in23.80s (tests isolated on domain75). Five affected ROS packages rebuilt successfully. Container entrypoint/command tests pass; the revised image has not been rebuilt. `/follower/sorting_ready` remains false for the documented physical prerequisites.
