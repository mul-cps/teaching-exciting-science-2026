# RViz station operations

Build `so101_station_operator`, source its install prefix in the existing GPU-guarded RViz launch, then select **Panels → Add New Panel → so101_station_operator/Operations**. Save the existing RViz configuration. `config/panel.rviz` contains the panel entry for merging into an existing configuration; it is not a standalone station scene.

Start its ROS adapter alongside the resident bridge:

```sh
ros2 launch so101_station_operator operator.launch.py \
  station_config:=/absolute/path/to/station.json \
  station_root:=/absolute/path/to/repo
```

Use the station wrapper so ROS domain 76 and the station environment are retained. `station_live.launch.py` starts the adapter by default; `use_operator_panel_api:=false` disables it. The adapter never opens serial, changes encoder offsets, starts a second follower owner or changes the MoveIt velocity settings. UI requests are asynchronous and serialize conflicting operations. Status reports cached measured owner mode, all six torque states, stop latch, fault and metadata freshness. Camera/depth health uses **advancing CameraInfo stamps** and tags use detection JSON receipt; it does not subscribe to or copy full-resolution images. These indicators do not prove image contents or registration accuracy. A successful request is acknowledged by the resident service; unavailable and timed-out requests are not shown as success. An unknown command outcome blocks further commands except STOP until the adapter is explicitly restarted after inspection.

| Panel / `/station/operator/` Trigger service | Resident operation |
| --- | --- |
| Reset, torque OFF (`reset`) | Dead-owner reconnect if needed, verified STOP, then named-profile rearm if required; remains torque OFF |
| Start position (`start`) | `/follower/home`, existing measured-home admission |
| Manual base: release (`base_release`) | `/follower/stop`, then verifies fresh six-joint torque OFF |
| Manual base: confirm (`base_confirm`) | Read-only comparison to identity-bound measured-home file and configured tolerance |
| Leader teleop (`teleop`) | `/follower/teleop_enable` true; existing leader/home/owner checks |
| MoveIt mode (`moveit`) | `/follower/teleop_enable` false, then fresh idle confirmation |
| Zero torque (`zero_torque`) | Priority `/follower/stop`; fresh all-six OFF acknowledgement required |
| Gentle damping / renew 60 s (`damping`) | `/follower/damping_enable`; default disabled until exact firmware capability configured |
| Restart camera/depth/tags/all (`restart_camera`, etc.) | Explicit configured user-unit allowlist; disabled by default |
| Verify restart / recover (`verify_restart`) | Read-only manager job and actual unit-transition proof, then explicit STOP; never retries a restart |

## Manual base alignment

Support the arm, choose **Manual base: release**, manually place the arm at the saved measured start, then choose **Manual base: confirm**. Confirmation checks all six joints, current owner generation, arm/calibration identity, file schema and tolerance. It leaves torque OFF and does not rewrite encoder zeros, model offsets, camera extrinsics or the saved home. A confirmation outside tolerance fails with measured joint errors. Stopping invalidates the resident home provenance; successful correlated STOP clears the resident barrier/fault through its existing logic. Confirmation itself changes no resident admission state. Reconnect never kills a live owner or silently clears its fault.

## Restart configuration

Configure exact camera, depth and tag **user** units through launch arguments `camera_unit`, `depth_unit`, `tags_unit`, then explicitly enable `enable_service_restarts:=true`. Defaults are empty; no guessed unit names are invoked. Names must match `so101-*.service`; requests carry no shell, executable, unit or arbitrary argument. Only the three configured slots can be selected. **Restart perception** means these three units, in camera → depth → tags order; it never restarts the arm bridge, MoveIt, RViz or the EV3.

The bridge independently needs the same exact `operator_service_units` allowlist. The integrated launch forwards `operator_camera_unit`, `operator_depth_unit`, `operator_tags_unit` to both bridge and adapter; `operator_service_restarts:=true` enables public restart commands. Units whose names contain arm/bridge/MoveIt/RViz/teleop/EV3 are rejected. The backend accepts no arbitrary shell or executable. Its only mutating OS command is `systemctl --user restart -- <allowlisted-unit>`.

Every device/service reset requires fresh idle owner state and verified all-six torque OFF. The adapter reserves `/follower/operator_maintenance_enable` with an exact UUID token. The bridge blocks conflicting motion for up to 60 seconds; the adapter's complete sequence has a 50-second budget. Before every restart the adapter checks that its token still owns the reservation and torque remains OFF. The backend rechecks the admission atomically before journaling and dispatch. STOP can revoke a reservation; a revoked client never releases another client's token.

**Unknown external jobs override the ordinary lease.** Before submitting any OS restart, the bridge durably writes a token/unit/source/calibration/owner-generation/request-time journal. The OS work runs outside the hardware callback so STOP can disarm promptly. Success requires actual systemd `Job=0`, a terminal `ActiveState`/`SubState`, and a new invocation plus state-transition timestamp after the request for an active unit. An inactive/failed terminal state is recorded as failure, not success. A CLI timeout does not prove that the manager job stopped: unresolved outcomes retain the journal and admission barrier across lease expiry, adapter crashes and bridge restarts. STOP still disables and verifies all six joints, but does not clear an unresolved external-job barrier.

Choose **Verify restart / recover** after an unknown result. It performs read-only systemd queries and resolves only the exact pending token/unit/cohort after actual completion proof. It does not cancel jobs or resubmit a restart as proof. Its final explicit STOP clears the now-resolved motion barrier and leaves torque OFF. A manager job still running, an unchanged old active invocation, invalid pending evidence or a changed allowlist/source cohort remains blocked. The durable default journal is `~/.local/share/exciting-science/station/operator-maintenance-pending.json`; `operator_maintenance_journal` changes it. Do not delete pending evidence. Unit activity remains distinct from camera/depth metadata freshness and registration qualification.

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

Offline compilation does not qualify physical mode switching, restart health or damping. The existing owner remains the final authority for motion and stop admission. Manual panel commands do not qualify picking, placement or registration accuracy.
