# Emergency station reset

Support the arm before selecting **Restart entire station** in the RViz operator panel, or call:

```bash
ros2 service call /station/operator/emergency_reset std_srvs/srv/Trigger '{}'
```

The separate **Reset (torque OFF)** button calls the existing `/station/operator/reset`, which remains available with its previous owner/profile behavior. The station reset performs no homing, arming, calibration, planning or execution. It restarts only the configured station recipe. The operator adapter never opens the servo bus.

The reset stops the current owner, explicitly reinitializes the controller into a passive owner through `/follower/reset_owner`, then verifies STOP against the new owner. A fresh latched fault observer first uses the guarded `/follower/reconnect` path. After guarded explicit exit, the bridge waits up to eight seconds for the retiring owner process to finish shutdown before starting a replacement; timeout never kills the owner or starts a replacement. One maintenance token excludes goals during the ordered camera/depth (deduplicated), tags, MoveGroup, objects, SceneUpdater and optional read-only leader jobs, coordinated perception recovery, and fresh acquisition acceptance. The bridge holds that reservation for at most 120 seconds; the API stops waiting after 110 seconds. STOP remains available through its independent request path and cancels the reset. No automatic retry follows an unknown acknowledgement.

Acceptance requires advancing actual RGB and depth image payload stamps, tags and object source stamps (an empty workspace is valid), a new SceneUpdater instance with current applied world state, a responsive MoveIt scene service whose complete finite joint values agree within 0.01 rad with advancing independent six-joint feedback bracketing the RPC, and a configured leader's fresh read-only initialization. Scene application stamps describe the exact asynchronous request that completed. An empty detection message cannot bypass pending scene removal. The MoveIt `colorbrick_*` IDs must match the SceneUpdater's applied current cohort. A final fresh owner identity and measured six-OFF check runs while the exact maintenance reservation is held. Its release acknowledgement is the last backend operation on successful reset; no later dependent proof can fail after releasing the goal barrier.

`/station/operator/status` reports `station_reset.phase`, its bounded deadline while running, and `initialized_all_off` separately from `pick_place_ready`. Reset success means initialized with torque OFF. Calibration, joint bounds, saved home, motion profile rearm and physical pick/place acceptance remain separate motion blockers. Reset does not alter joint bounds or calibration to obtain a successful result.

Configure `enable_service_restarts:=true` and exact `camera_unit`, `depth_unit`, `tags_unit`, `movegroup_unit`, `objects_unit`, `scene_unit` parameters on `operator.launch.py`. Set `leader_required:=true` and `leader_readonly_unit` when a leader is part of the station. Camera/depth/tags must also appear in the bridge's existing `operator_service_units`; the additional recipe units must appear in `operator_station_reset_units`. The latter are admitted only by an explicit station-reset reservation. Both sides reject arbitrary commands and critical arm/bridge/RViz/teleop/EV3 units. Use exact running service names from the commissioned station configuration; defaults are empty and reject an incomplete recipe.

A failed acquisition, identity or pre-release deadline check retains the reservation until explicit STOP/retry or expiry, when the reset barrier stays latched. If the release acknowledgement is lost or late, the server may already have released the token: the API reports an unknown outcome, attempts an explicit STOP with a separate two-second bound, and never claims that the reservation remains held. A failed or timed-out systemd job retains the existing durable unknown-outcome journal; **Verify restart / recover** must resolve its exact job before retry. Neither reset nor STOP blindly removes that evidence.

If the bridge itself is unavailable, the ROS reset cannot progress or prove torque OFF. The bounded failure is intentional: use the station's configured manual bridge-unit recovery command from its service deployment recipe, then repeat reset. For example, only after verifying the exact deployed bridge unit and ensuring it is the sole serial owner:

```bash
systemctl --user restart -- YOUR_CONFIGURED_PASSIVE_BRIDGE_UNIT.service
```

The arm/bridge unit is deliberately excluded from ROS restart slots. Do not substitute an active torque-on unit. The deployment recipe must name the passive bridge unit explicitly; this placeholder is not a runnable station-specific recovery configuration.

The 2026-10-21 station deployment uses this concrete recipe (deployment must preserve each unit's existing launch arguments and safety settings):

| Slot | Exact unit |
| --- | --- |
| camera and depth | `so101-rocm-rgbd-recovery-20261020.service` |
| tags | `so101-marker-provisional-body-20261020.service` |
| MoveGroup | `so101-movegroup-limit-v2-20261021.service` |
| objects | `so101-colored-objects-fastpath-20261021.service` |
| SceneUpdater | `so101-scene-stable-20261021.service` |
| read-only leader | `so101-leader-readonly.service` |

For that deployment, the manual bridge-unavailable fallback is:

```bash
systemctl --user restart -- so101-arm-passive-canonical-20261021.service
```

This command is an explicit manual recovery action; the ROS reset does not invoke it. Verify fresh six-OFF through the restored bridge before proceeding. Service installation and live station qualification are separate from the software tests.

When the display alignment node is configured, also pass `alignment_reseed_service:=/so101_perception/live_tag_alignment/reseed`. After fresh perception/scene acquisition, the reset invokes that existing display-only reseed once and requires two advancing admitted alignment exposure stamps, fresh independent camera identity, and source context matching the current tag producer. Qualification stays false and canonical control TF stays unchanged. If tags are not visible or alignment remains unavailable, reset reports that blocker and retains its admission barrier; it never fabricates an aligned model. The 2026-10-21 display unit is `so101-alignment-current-reference-20261020.service`; reseeding its existing endpoint avoids restarting an unrelated caller or fitting new calibration.

Emergency entry always attempts the independent STOP service first, including while a normal home command is waiting, the unit recipe is invalid, or an acknowledgement/journal remains unknown. It then waits at most three seconds for the normal operator command to quiesce before starting the recipe. An unresolved command or unknown outcome prevents the recipe; its uncertainty remains recorded. A duplicate emergency request attempts STOP and returns the existing reset's in-progress result without overlapping restarts. The RViz emergency button remains usable while another panel command is pending.

The operator API has no RGB/depth Image DDS subscriptions while idle. Only the post-coordinator reset proof window creates those two best-effort subscriptions with queue depth one, then destroys them immediately after image acquisition acceptance or on any failed/interrupted proof. Each activation starts new counters and a source-time watermark; callbacks queued from a retired generation are ignored. CameraInfo, tags, joint and scene metadata monitoring remain lightweight permanent subscriptions.


A successful coordinator job is historical source adoption. Its original `ready_now` lease is deliberately nonrenewable; an expired lease does not undo completion and polling cannot restore it. The operator records the exact successful job's producer generation, pin and consumer epochs, then requires new acquisition after completion and current checked marker/object admission leases matching that identity. Fresh tag source context must match the same adopted marker epoch, pin and generation. Failed, unknown and mismatched jobs never become successful by observing unrelated frames. Ordinary restart success also checks these current admissions and labels coordinator completion as historical adoption; it does not claim detector or physical qualification.

MoveIt's standard `GetPlanningScene` response may leave the serialized robot joint header stamp at zero. That stamp is not used as evidence of freshness. The reset instead retains only the latest bounded six-joint measured cohort from `/follower/joint_states`, requires an advancing fresh measurement after the scene request, and checks all six returned positions against both sides of that bracket within 0.01 rad. The request and post-request measured bracket have a shared two-second deadline, with unchanged owner identity, maintenance token, STOP handling and six-OFF checks. Missing, nonfinite, disagreeing or nonadvancing feedback fails acceptance. World object IDs must still match the current applied scene cohort exactly.


The panel places **Restart entire station**, **Restart MoveIt** and **ZERO TORQUE / STOP** in separate full-width rows at the top. Secondary controls scroll when the dock is short. A second click cannot submit a duplicate pending restart. STOP remains independently available. After either restart succeeds, the panel disables and reenables enabled MoveIt MotionPlanning displays using RViz's display lifecycle so their model and planning request are recreated. It explicitly reports when no enabled MotionPlanning display was available for that reload.

**Restart MoveIt** calls `/station/operator/restart_moveit`. It first requests STOP and requires fresh measured six-OFF and idle state. It takes the existing exact station-reset maintenance reservation (the purpose needed to admit the configured MoveGroup and SceneUpdater units), restarts only `movegroup_unit` and `scene_unit`, and uses the existing manager journal and job completion validation. It does not restart the follower owner, camera, tags, objects producer or leader, and does not call the perception recovery coordinator. The SceneUpdater is reinitialized to repopulate the fresh MoveGroup scene. Success requires its new current applied/empty instance, matching current world IDs, unchanged owner identity, and the same independent measured-joint bracket used for complete station reset. A failed proof retains the barrier; unknown manager outcomes require **Verify restart / recover**. The normal idle metadata monitoring remains unchanged; this path creates no Image subscriptions.

“Fresh” or “at zero” here means a new planning request anchored at the arm's **current measured pose**, with torque OFF. Neither restart commands home motion, resets encoders, changes calibration nor substitutes zero joint angles. Support the arm during torque release. The reset does not promise to hold an unsupported arm physically against gravity. Existing home, profile, calibration and physical acceptance gates still apply before later motion.
