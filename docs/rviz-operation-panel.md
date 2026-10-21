# RViz station operations

Build `so101_station_operator`, source its install prefix in the existing GPU-guarded RViz launch, then select **Panels → Add New Panel → so101_station_operator/Operations**. Save the existing RViz configuration. `config/panel.rviz` contains the panel entry for merging into an existing configuration; it is not a standalone station scene.

Start its ROS adapter alongside the resident bridge:

```sh
ros2 launch so101_station_operator operator.launch.py \
  station_config:=/absolute/path/to/station.json \
  station_root:=/absolute/path/to/repo
```

Use the station wrapper so ROS domain 76 and the station environment are retained. `station_live.launch.py` starts the adapter by default; `use_operator_panel_api:=false` disables it. The adapter never opens serial, changes encoder offsets, starts a second follower owner or changes the MoveIt velocity settings. UI requests are asynchronous and serialize conflicting operations. Status reports cached measured owner mode, all six torque states, stop latch, fault and metadata freshness. Camera/depth health uses **advancing CameraInfo stamps** and tags use the exact `so101-tag-detections-v1` schema and **advancing JSON source stamps** (including valid empty observation lists); it does not subscribe to or copy full-resolution images. These indicators do not prove image contents or registration accuracy. A successful request is acknowledged by the resident service; unavailable and timed-out requests are not shown as success. An unknown command outcome blocks further commands except STOP and external-job verification. A timed-out motion acknowledgement requires owner inspection before adapter recovery; Verify restart cannot clear an unknown motion outcome. Unsupported buttons are disabled with a tooltip naming the missing service, leader readiness or exact restart configuration. STOP is always available, including during another pending request, without a confirmation dialog. Each outstanding panel RPC retains its own future, so a STOP cannot discard a delayed home/restart reply.

| Panel / Trigger service | Prerequisite and exact resident path | Expected state / domain87 fake result | Remaining live check |
| --- | --- | --- | --- |
| Reset (`reset`) | `/follower/status`, `/stop`, `/rearm_motion_profile`; `/reconnect` only if disconnected | Fresh all-six OFF, profile rearmed; passed | Repeated same-owner P24 Reset cycles |
| Saved home position (`start`) | Fresh idle owner, rearm if required, `/follower/home` | Fresh `home_completed_this_session=true`; passed after Reset | Checked saved-home motion after Reset |
| Release for manual setup (`base_release`) | Priority `/follower/stop` + fresh owner | All six OFF; passed | Supported arm release |
| Record manual reference (`base_confirm`) | Released same owner/calibration/source; `/follower/status` | In-memory six-joint raw/pose observation, all OFF; passed away from old home | Observe manual pose and source identity |
| Leader teleop (`teleop`) | `teleop_leader_ready=true`; `/follower/teleop_enable` true | Fresh active teleop; stale/absent calibrated leader disables button; both passed | Real calibrated leader stream and motion |
| MoveIt mode (`moveit`) | `/follower/teleop_enable` false, rearm if required | Fresh idle owner; passed including after STOP | Plan/execute under calibrated-limit admission |
| ZERO TORQUE / STOP (`zero_torque`) | `/follower/stop`; independent priority path | Fresh all-six OFF; passed during delayed home and unknown ACK | Physical prompt disarm |
| Gentle damping (`damping`) | Exact native torque value 2 firmware capability and physical qualification; `/follower/damping_enable` | Disabled with reason when unsupported; passed, no damping RPC dispatched | Exact firmware and physical backdrive qualification remain open |
| Restart camera/depth/tags/all (`restart_camera`, etc.) | Exact units configured on API and bridge; all OFF; `/follower/operator_maintenance_enable` | Correlated manager jobs, one correlated source-recovery job with current readiness, then fresh advancing requested streams; fake passed; duplicate unit aliases once | Real user-unit restart and stream recovery |
| Verify restart (`verify_restart`) | Fresh all OFF; exact pending token via `/follower/operator_maintenance_enable`, then `/follower/stop` | Unknown/failed-job barrier resolves only after explicit verification; passed | Actual durable job completion proof |

## Manual setup reference and fixed home

Support the arm, choose **Release for manual setup**, move it manually, then choose **Record manual reference**. Recording accepts any complete fresh measured pose within the resident owner's admitted state; it does not demand that every joint match the old measured home. The observation includes all six raw encoder positions and body/gripper pose, current owner generation, arm ID, calibration hash and owner source hashes. It remains in adapter memory and the status payload, and is cleared by the next release/STOP. A changed owner/source/calibration cannot confirm an earlier release.

This is an observation for manual setup, not a new trajectory destination. **Saved home position** always uses the existing `/follower/home` checked measured-home path. Returning to a newly recorded observation would require an explicit checked backend motion API and is not advertised by this panel. Neither recording nor release rewrites encoder zeros, model offsets, base TF, camera extrinsics or saved home. Confirmation changes no resident admission state. Reconnect never kills a live owner or silently clears its fault.

## Restart configuration

Configure exact camera, depth and tag **user** units through launch arguments `camera_unit`, `depth_unit`, `tags_unit`, then explicitly enable `enable_service_restarts:=true`. Defaults are empty; no guessed unit names are invoked. Names must match `so101-*.service`; requests carry no shell, executable, unit or arbitrary argument. Only the three configured slots can be selected. **Restart perception** means the configured units, in camera → depth → tags order, with the same exact unit restarted only once; it never restarts the arm bridge, MoveIt, RViz or the EV3.

The bridge independently needs the same exact `operator_service_units` allowlist. The integrated launch forwards `operator_camera_unit`, `operator_depth_unit`, `operator_tags_unit` to both bridge and adapter; `operator_service_restarts:=true` enables public restart commands. Units whose names contain arm/bridge/MoveIt/RViz/teleop/EV3 are rejected. The backend accepts no arbitrary shell or executable. Its only mutating OS command is `systemctl --user restart -- <allowlisted-unit>`.

Every device/service reset requires fresh idle owner state and verified all-six torque OFF. The adapter reserves `/follower/operator_maintenance_enable` with an exact UUID token. The bridge blocks conflicting motion for up to 60 seconds; the adapter's complete sequence has a 50-second budget. Before every restart the adapter checks that its token still owns the reservation and torque remains OFF. The backend rechecks the admission atomically before journaling and dispatch. STOP can revoke a reservation; a revoked client never releases another client's token.

**Unknown external jobs override the ordinary lease.** Before submitting any OS restart, the bridge durably writes a token/unit/source/calibration/owner-generation/request-time journal. The OS work runs outside the hardware callback so STOP can disarm promptly. Success requires actual systemd `Job=0`, a terminal `ActiveState`/`SubState`, and a new invocation plus state-transition timestamp after the request for an active unit. An inactive/failed terminal state is recorded as failure, not success. A CLI timeout does not prove that the manager job stopped: unresolved outcomes retain the journal and admission barrier across lease expiry, adapter crashes and bridge restarts. STOP still disables and verifies all six joints, but does not clear an unresolved external-job barrier.

Choose **Verify restart / recover** after an unknown result. It performs read-only systemd queries and resolves only the exact pending token/unit/cohort after actual completion proof. It does not cancel jobs or resubmit a restart as proof. Its final explicit STOP clears the now-resolved motion barrier and leaves torque OFF. A manager job still running, an unchanged old active invocation, invalid pending evidence or a changed allowlist/source cohort remains blocked. The durable default journal is `~/.local/share/exciting-science/station/operator-maintenance-pending.json`; `operator_maintenance_journal` changes it. Do not delete pending evidence. Unit activity remains distinct from camera/depth metadata freshness and registration qualification. After all requested unit jobs complete, the adapter calls `/so101_perception_recovery/start_recovery` (`std_srvs/srv/Trigger`) exactly once. The existing coordinator explicitly resets the producer, captures fresh source pins, adopts and reads back `derived_source_cohort`, resets both marker and colored-object consumers, and checks independent source-admission witnesses. The adapter polls `/so101_perception_recovery/recovery_status`, requires the exact accepted `job_id`, `state=completed`, `operation_success=true` and `ready_now=true`. A completed job with temporarily expired current receipt leases is polled again within the same deadline; no new reset or recovery job is dispatched. Failed, busy, mismatched and unavailable outcomes are rejected. Both coordinator services must exist before any unit restart is dispatched; otherwise the buttons are disabled with the missing-service reason.

The coordinator's absolute job budget is 40 seconds. The adapter requires at least that much of its 50-second maintenance budget before dispatching recovery, and continues checking its exact reservation plus fresh all-six OFF throughout. Unknown coordinator acknowledgements or mismatched job identity retain the reservation and block further commands pending inspection; unknown systemd jobs retain their independent durable barrier. A successful restart button additionally requires new advancing source metadata **after recovery** for every requested slot within the bounded sequence. Missing streams report failure even if the unit is active. The status `streams` object includes publisher counts, source age and parse/stamp failure reasons; receipt of malformed or repeated tag JSON does not refresh health.

When there is no external job, an abandoned healthy all-OFF maintenance lease can expire normally. After a known terminal job, the adapter releases only its own token. No failed or partially completed restart is automatically retried.

## Named motion profile rearm

`/follower/rearm_motion_profile` is a Trigger service that changes no torque state. For a configured named P24 profile, a repeated same-owner trial requires the retained hash of the actual prior restoration archive, matching source/USB/calibration/owner identity, no pending PID/damping journal and an exact current restored PID/lock snapshot. Only then may the owner reset its restoration-attempt flag and create a **new UUID journal before applying P24** through the existing verified startup transaction. It never clears an unrestored record or bypasses normal-motion admission.

A new observe-only P24 generation can read without applying P24. After explicit STOP exits read-only mode, its first explicit rearm may create a new initial trial only from an exact six-joint P16/I0/D32/Lock0 baseline, normal fresh all-OFF state, valid source/USB/calibration/model identities and no pending PID/damping journal. This is a new trial, not a claim of prior same-owner restoration. Subsequent cycles require the genuine retained restoration receipt. Reset, Start and MoveIt/teleop transitions call rearm only when the owner reports it required. Normal existing motion code sets a fresh measured goal before torque ON. Pending old journals always require their specific frozen-cohort restore-only procedure first.

## Backdrivable damping: software implemented, hardware qualification pending

The requested mode is free manual movement with gentle resistance, not a position hold. The [Waveshare memory map](https://docs.waveshare.com/Memory_Map_Explanation/ST_Servo_Memory_Map_Explanation) explicitly documents **Torque switch address 40 value 2 as damping output**, with **Torque limit address 48 controlling its magnitude**. This is separate from the four operating modes. The owner and bridge accept torque value 2 only during the explicit native-damping phase; ordinary motion keeps its binary torque checks. Exact installed firmware support still requires qualification.

The isolated owner implementation requires idle all-six torque OFF, valid source/calibration/USB/motor identities, normal admitted bounds and exact configured firmware versions. It journals original RAM torque limits and lock readbacks before any change. It sets a finite raw torque limit (default **25/1000**, allowed **1..50/1000**), verifies readback, and requests native torque-switch value 2 with exact all-six readback. It writes no goals, PID, lock or EEPROM registers. Damping is not gravity compensation; support the arm.

The lease expires after **60 seconds** regardless of status polling. The panel shows remaining time and an explicit support acknowledgement is required to enable or renew it. Health guards require fresh feedback within 100 ms, existing per-sample movement bounds, normal bounds, current below 150 raw units, temperature below 50°C and zero servo status faults. STOP, lease expiry, EOF and shutdown disable all six joints, verify OFF, restore original torque limits, verify original lock/limits and archive/clear the journal. A damping → MoveIt/start/teleop transition first uses verified STOP/restoration; existing motion admission then sets a fresh measured goal. No raw torque-1 transition is added.

Configuration under `arm` is default off:

```json
{
  "enable_native_damping": false,
  "native_damping_torque_limit": 25,
  "native_damping_firmware": {},
  "restore_native_damping_only": false
}
```

`/follower/damping_inventory` is a passive Trigger API that returns the six exact `[major, minor]` firmware versions. An enabled candidate requires `native_damping_firmware` to map **all six joint names** to their exact versions; an empty map rejects entry even if the enable flag is true. This configuration is an explicit capability declaration, not evidence of physical qualification. The service acknowledgement always retains `physically_qualified: false`.

A pending `.native-damping-pending.json` beside the calibration file blocks ordinary startup. After interrupted restoration use a new explicit **restore-only** owner with `restore_native_damping_only: true`, the same frozen source cohort, USB identity and calibration. It validates journal/firmware identity, explicitly establishes all-six OFF, restores and verifies original limits/lock, archives the restoration receipt and clears pending only after success. This owner admits read/STOP/exit only; no manual journal deletion is required. An unreadable/disconnected bus cannot claim successful restoration.

## Qualification boundary

When owner status is not fresh, the panel labels torque, stop latch and fault **STALE / last known**, explicitly marks current values unknown, and displays mode `unknown`. Old cached torque values are not current measurements.

Offline compilation does not qualify physical mode switching, restart health or damping. The existing owner remains the final authority for motion and stop admission. Manual panel commands do not qualify picking, placement or registration accuracy.

## Focused fake verification (no hardware)

Build this package into a private prefix and source it. These checks require domain 87 and use only fake ROS services; they import no arm owner, open no devices and invoke no systemd operations:

```sh
ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1 ros2/conda/so101-conda.sh run bash -c '
  source /tmp/so101-operator-recovery-install/local_setup.bash
  python ros2/src/so101_station_operator/test/test_operator_fake_ros.py
  QT_QPA_PLATFORM=offscreen /tmp/so101-operator-recovery-build/so101_station_operator/operator_panel_load_check
'
```

The Python check constructs the actual API node (including ROS entity property names), calls every public command through real ROS RPC, tests Reset/home/rearm, leader readiness, MoveIt, manual observation, restart aliases and failure barrier, delayed replies with concurrent STOP, unknown motion ACK retention, and fresh/repeated/malformed/stale tag metadata. The C++ check loads the actual plugin through pluginlib and constructs all 13 buttons with Qt offscreen. These are software qualification; every live check in the matrix remains separate.
