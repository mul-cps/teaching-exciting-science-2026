# Explicit ROS perception-source recovery

This optional node repairs source admission after an explicitly enabled
producer reset. It does not reset the native camera, fit or apply calibration,
change TF, contact arm/EV3 services, detect a successful grasp or authorize motion.
All schemas remain `qualified=false` and `execution_enabled=false`.

## Configuration and calls

The launch default is `enabled:=false`, so no node or background job is created.
Enable explicitly with immutable startup parameters:

- `artifact_directory`: existing trusted directory for new UUID-named source
  artifact/evidence files; no file is overwritten.
- `expected_source_cohort`: reviewed camera-only source artifact from the same
  producer implementation/configuration/runtime. Its startup bytes are hashed
  and rechecked per job. Calibration, code, interpreter, runtime, serial, camera
  node, topics and optical frames must remain exactly equal. Only native/output
  GIDs, their derived source generation and producer generation may rotate.
- `expected_serial`: explicit native camera serial, default `243322071783`.

The producer must already be enabled with `allow_explicit_reset:=true`. Consumers
must already use the explicitly configured private derived path. Fixed endpoints
are producer `/so101_rocm_rgbd` and consumers `/so101_charuco_world` and
`/so101_colored_objects`. Requests cannot supply paths, PIDs, code or endpoints.

```bash
ROS_DOMAIN_ID=76 ROS_LOCALHOST_ONLY=1 ros2 service call \
  /so101_perception_recovery/start_recovery std_srvs/srv/Trigger '{}'
ROS_DOMAIN_ID=76 ROS_LOCALHOST_ONLY=1 ros2 service call \
  /so101_perception_recovery/recovery_status std_srvs/srv/Trigger '{}'
```

`start_recovery.success=true` means one asynchronous job was accepted, with JSON
`job_id`, `state`, absolute monotonic deadline and `ready_now=false`. It does not
mean recovered. One job may be active and accepted jobs have a 30-second cooldown;
there is no automatic retry. `recovery_status.success=true` means status retrieval
succeeded. Its bounded JSON carries `operation_success` and dynamically checked
`ready_now` separately. A completed operation remains recorded as successful;
its readiness expires with the ORIGINAL evidence, even while newer camera frames
continue to arrive. This status is never persistent motion permission.

## Sequence and evidence

1. Under ONE request/job 40-second absolute budget, observe two advancing native
   depth/color CameraInfo pairs acquired after job start. Bind the actual four
   native GIDs to one camera. Nonfuture headers <=500 ms, receipts <=250 ms and
   pair differences <=40 ms remain mandatory; duplicates cannot renew receipts.
   A native camera outage fails explicitly and needs separate camera-only
   supervision. No camera reset/SDK/device open is attempted here.
2. Dispatch producer reset exactly ONCE. Require its accepted reset ID, different
   reserved producer generation, matched current producer status and two actual
   fresh new-generation lineages. Heartbeat/reset ACK alone is insufficient.
3. Run the independent metadata-only CameraSourceCapture on a PRIVATE
   SingleThreadedExecutor using the parent's ROS context and
   `use_global_arguments=False`. It independently verifies native DeviceInfo,
   serial, seven actual GIDs, exact CameraInfo/optical TF and exclusive complete
   artifact publication. No global/nested executor spinning occurs.
4. Read both old consumer epochs, set and read back the exact new artifact path,
   reset each consumer once, and require two advancing sequences AND both
   original acquisition headers for each new epoch/pin/generation. Each status
   must match an actual retained lineage. Poll both current proofs; an expired
   or cleared/newest invalid source cannot leave old readiness usable.
5. Check original leases, current producer reset/generation, seven graph
   endpoints, immutable files and both consumer proofs around final
   serialization. Partial rebind fails without rollback or repeated resets.
   The artifact is source provenance, not physical geometry qualification.

Each RPC has a <=2-second subdeadline including response access/JSON processing,
within the same 40-second job budget. Late or unknown completion fails without
adoption. Native/FS calls cannot be forcibly interrupted in Python; these normal
operation budgets do not guarantee termination of wedged native calls. Shutdown
cancels the job and waits 2 seconds; if the non-daemon thread remains active, the
node/context are retained and an external launch/caller supervisor is required.
The coordinator never discovers or kills processes.

## Bounds and validation scope

No image/cloud payload subscriptions or copies are added. Native metadata retains
four scalar records per Info stream; lineage retains at most eight statements
(each input <=64KiB before Python object overhead), one producer heartbeat
(<=64KiB), two proofs/witnesses per consumer and bounded source artifacts
(<=128KiB). The temporary pinner additionally retains its existing four native
Info references/stream and eight statements for at most 4 seconds. DDS buffers and
Python object overhead are additional. After completion only the original short
readiness window is observed; there is no background recovery or proof renewal.

Tests include adversarial pure metadata/budget checks and an actual isolated
DDS177 fixture using fake producer/consumers but the REAL private-executor pinner,
DeviceInfo RPC, exact static optical TF and seven actual publisher GIDs. It proves
one reset, exclusive source capture, both consumer rebinds, advancing admission
and readiness expiry without any image/cloud subscriptions. It is not live GPU,
physical camera, calibration, object tracking or sorting qualification.

### Heartbeat ordering during an accepted reset

A producer's cold owned-worker setup can temporarily pause its heartbeat even
when native metadata and new-generation lineage callbacks arrive independently.
After this job has explicitly received an accepted reset response, an expired
heartbeat from the unchanged actual endpoint is temporal unavailability: the
coordinator waits within the SAME absolute40-second job budget. It never adopts
that expired heartbeat, renews its receipt or starts consumer rebind from it.
A fresh matched producer reset status and two genuinely advancing original
lineages are still required before source capture/rebind. Endpoint/GID mismatch,
missing/malformed status and future receipt clocks remain errors; later
completed-status queries also continue to return ready_now=false for expired
heartbeats. This is not a retry of reset or an extension of any evidence lease.

The isolated DDS regression deliberately publishes fresh native metadata and
lineage while withholding heartbeat publication for450ms. It reproduces the old
failure and verifies one reset, independent source capture, consumer rebind and
original readiness expiry with the corrected waiting behavior. This synthetic
ordering result does not establish the cause of a particular live endpoint or
scheduling failure.
