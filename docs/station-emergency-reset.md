# Emergency station reset

Support the arm before selecting **Reset station (torque OFF)** in the RViz operator panel, or call:

```bash
ros2 service call /station/operator/emergency_reset std_srvs/srv/Trigger '{}'
```

The existing `/station/operator/reset` remains available with its previous owner/profile behavior. The station reset performs no homing, arming, calibration, planning or execution. It restarts only the configured station recipe. The operator adapter never opens the servo bus.

The reset stops the current owner, explicitly reinitializes the controller into a passive owner through `/follower/reset_owner`, then verifies STOP against the new owner. A fresh latched fault observer first uses the guarded `/follower/reconnect` path. One maintenance token excludes goals during the ordered camera/depth (deduplicated), tags, MoveGroup, objects, SceneUpdater and optional read-only leader jobs, coordinated perception recovery, and fresh acquisition acceptance. The bridge holds that reservation for at most 120 seconds; the API stops waiting after 110 seconds. STOP remains available through its independent request path and cancels the reset. No automatic retry follows an unknown acknowledgement.

Acceptance requires advancing actual RGB and depth image payload stamps, tags and object source stamps (an empty workspace is valid), a new SceneUpdater instance with current applied world state, a responsive MoveIt scene service with fresh six-joint feedback, and a configured leader's fresh read-only initialization. Scene application stamps describe the exact asynchronous request that completed. An empty detection message cannot bypass pending scene removal. The MoveIt `colorbrick_*` IDs must match the SceneUpdater's applied current cohort. A final fresh owner identity and measured six-OFF check runs while the exact maintenance reservation is held. Its release acknowledgement is the last backend operation on successful reset; no later dependent proof can fail after releasing the goal barrier.

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
