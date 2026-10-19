# Recovery coordinator implementation plan

Goal: explicit default-off ROS-only source recovery, not calibration or motion.
Ownership: new `recovery_core.py`, `perception_recovery.py`, script, launch,
dedicated tests/docs only. Root supplies shared admission/pinner prerequisites
and CMake integration. No camera SDK, process control, actuator calls or retries.

1. Pure bounded native CameraInfo pairs and consumer proof validation: two
   independent advancing original acquisitions, exact GIDs and original leases.
   One40-second deadline covers all stages, <=2-second RPCs, no renewed receipts.
2. Optional ROS node with fixed producer/two consumer endpoints and immutable
   trusted configuration. Main executor handles metadata and RPC completions;
   one owned job thread performs the sequence without nested/global spinning.
3. Preflight fresh native pairs, reset producer exactly once, require matched
   accepted reset ID/new generation plus fresh actual new-generation lineage.
4. Capture immutable source artifact with the independent pinner in a private
   executor on the same context; require exact generation and expected pins.
5. Snapshot consumer epochs, set/read back exact artifact path, reset each and
   require two advancing fresh source admission proofs for new epoch/pin/gen.
   Match proofs to actual retained lineage; newest bad evidence revokes oldproof.
6. Final check both current proofs, latest lineage, seven graph endpoints and
   original expiries after serialization. Completion records operation_success;
   ready_now expires with original evidence and never grants motion permission.
7. Adversarial pure tests, actual isolated DDS fake producer/consumers +real
   pinner/DeviceInfo/TF, bounded metadata benchmarks and independent review.

Default off creates no launched node. Enabled node collects metadata only.
Native source unhealthy fails explicitly; camera-only supervision is external.
Cancellation/timeout checks bound normal Python work, not wedged native/FS calls.
Shutdown retains context/node if a job cannot join within a finite cleanup wait;
external launch/caller process supervision is required for a wedged thread.
