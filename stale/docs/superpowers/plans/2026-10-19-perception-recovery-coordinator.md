# Bounded ROS perception recovery

The live camera restart and producer fault exposed an operational gap: recovery
requires a producer reset, independent fresh source pins and consumer reloads.
This change makes that sequence callable through ROS. It never qualifies a
physical calibration, enables sorting, or calls the arm or EV3.

## Contract

- Default off. `start_recovery` accepts one asynchronous job; `recovery_status`
  reports its ID and state. Acceptance is not readiness. No retry loop.
- Fixed producer and two perception consumers. No request-supplied paths, PIDs,
  imports or service names. Trusted immutable startup configuration selects the
  evidence directory and expected camera serial.
- Observe two advancing, nonfuture native CameraInfo pairs before resetting the
  producer. Missing camera source fails explicitly. Version one does not reset
  or terminate the native camera; its existing ROS reset remains separate.
- Call the explicitly enabled, producer-owned reset once. Match its accepted
  reset ID and reserved new generation in subsequent status and actual lineage.
  Unknown/late RPC completion fails the job without a blind retry.
- Reuse the independent metadata source pinner with a private executor, original
  leases, seven actual GIDs, exact metadata/TF and independent DeviceInfo.
  Publish a new immutable artifact per job; never overwrite an operator file.
- Set each consumer's artifact parameter, explicitly reset it, then obtain two
  advancing fresh admission proofs for the expected generation and pin hash.
  Admission proves source use; it does not assert detected bricks or a grasp.
- Consumer status retains only scalar metadata, not another image pair. The
  current newest lineage must remain valid at admission and status/commit.
  Rejected or cleared newest input cannot leave an old proof usable.
- One absolute 40-second job budget, individual RPC bounds, 30-second cooldown.
  Native/filesystem calls cannot be forcibly preempted in Python; unknown
  completion stays failed and requires external process supervision.

## Implementation order and qualification

1. Tests for metadata-only consumer proofs, newest-input revocation and expiry;
   implement the cold status API without changing direct-camera operation.
2. Add optional pinner context/executor injection; retain standalone defaults.
3. Pure recovery state/proof helpers, then default-off ROS coordinator, launch
   and script. Exercise actual synthetic DDS with fake producer/consumers.
4. Independent static/performance review; exact source freeze and installed
   artifact checks before root-controlled live GPU recovery qualification.
5. Stage station quick actions/API forwarding separately from the held owner's
   active source directory. Physical commissioning gates remain explicit.

Ownership: root owns this plan, coordinator and shared admission status files.
The gripper agent owns the separate producer-reset implementation. The Intel
agent owns object CPU optimization. Preserve their edits and the held owner.
