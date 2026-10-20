# Optional historical source-completion evidence

The recovery coordinator can expose versioned historical completion evidence. It is metadata only: `qualified=false`, `execution_enabled=false`. It does not qualify detections, calibration, picking or motion.

## Exposure and compatibility

`expose_completion_evidence` is a construction-time, read-only bool, default `false`. False keeps the exact legacy status keys and bypasses witness capture, sidecars and event hashing. Set it only in the explicit recovery leaf launch:

```sh
ros2 launch so101_station_perception perception_recovery.launch.py \
  enabled:=true expose_completion_evidence:=true \
  artifact_directory:=/trusted/existing/directory \
  expected_source_cohort:=/trusted/reviewed/source.json \
  expected_serial:=243322071783
```

This starts an available coordinator; it does not call recovery. Supervisor/policy/station configuration is unchanged. The opt-in captures only future owned jobs; runtime changes are rejected. Restart loses resident completion evidence. No old event is reconstructed from a source artifact or historical response.

The Trigger `/so101_perception_recovery/recovery_status` retains root schema `so101-perception-recovery-v1`, RPC-success semantics and existing fields. Opt-in adds:

- `completion_event`: `so101-source-admission-completion-event-v1`, or null with bounded `completion_event_unavailable`.
- `current_readiness`: `so101-current-source-readiness-v1`, describing the server's current check and ORIGINAL limits.

The frozen original natural observer compares all historical fields except `ready_now`/`readiness_reason`; dynamic `current_readiness` is incompatible with that comparison. Leave that observer frozen. Use the new verifier for the new explicitly approved historical-event contract. Strict legacy consumers should keep the default off.

## Two distinct facts

**Completion event:** two advancing actual IMAGE admissions per canonical consumer were valid when the coordinator consumed each one. The latest witness per consumer was still current at completion validation and the final commit gate. Earlier historical witnesses may already be expired at the event. Each event retains original proof/receipt/header/source/identity/local-lineage limits and actual consumption clocks, boot/domain, coordinator instance, owned job/reset/generation/pin/artifact identity, original40s budget and canonical digest. Duplicate cohorts cannot renew or reconstruct evidence. The digest detects accidental mutation; it is not a cryptographic trust signature.

`event_checked_*` is the recorded validation snapshot before canonicalization. The subsequent final original-clock gate is attested by the pinned coordinator code, not a separately exported exact commit instant. No ROS/monotonic subtraction is performed.

**Current readiness:** `ready_now` and `current_readiness.ready` are checked using the existing producer/graph/TF/artifact/current latest IMAGE gates. The effective original monotonic deadline includes source/consumer receipt, identity, local lineage receipt and active original job budget. Header validity remains a separate ROS deadline. A true server check may expire in transit; callers must compare original limits against ACTUAL receipt/take clock pairs. Historical event retrieval cannot renew them. Later readiness may be false while historical `operation_success` remains true.

Event absence, invalid evidence or the event8KiB/status16KiB cap cannot change an otherwise valid legacy operation outcome. The extension emits eventnull/reason if it fits; if extensions do not fit, it returns the bounded legacy-only response. A new verifier treats missing evidence as unavailable, never success. No truncation or bound increase.

## Offline verifier

`tools/verify_completion_event.py` reads bounded JSON files only; no ROS init, service calls or devices. Supply a trusted independently bound expected context and the ORIGINAL caller take clocks, not filemtime or verification-time clocks:

```sh
python tools/verify_completion_event.py --response captured-response.json \
  --expected-context owned-context.json \
  --take-monotonic ORIGINAL_CLOCK_MONOTONIC --take-ros-ns ORIGINAL_NODE_ROS_NS
```

Expected context keys are exported as `completion_event.CONTEXT`: boot/monotonic-domain, coordinator instance/job/reset/generation/source pin/artifact/baseline hashes and original job/native-start clocks. It must come from the trusted owned context; copying identities blindly out of an untrusted response is not independent binding.

Outputs distinguish historical validity, observed-within-original-job-budget, reported server-ready and actual current readiness. Verification alone always says `observation_qualified=false`, `qualified=false`, `execution_enabled=false`. Late delivery after40s remains negative observation evidence even if a historical event is well formed. The frozen actual negative fixture has no event/original coordinator consumption clock pairs and cannot be upgraded retroactively.

## Software evidence

Tests use `tools/test_perception_supervisor.sh` (hard177/local1 before imports). Synthetic DDS uses fake producer/consumers and the real metadata pinner; no camera payloads, SDK, GPU, native camera or arm. Scope is software metadata qualification only.

The CPU benchmark `tools/benchmark_completion_event.py --samples 1000 --dds-evidence captured-dds.json --output report.json` measures the full fixed4-witness capture/hash/event/deepcopy/JSON/parse/verify boundary with typical and a large bounded metadata stress vector; exact8KiB/16KiB boundary cases are tested separately (not a mathematical maximum across arbitrary-size positive integers), and an optional actual-DDS legacy response template. It excludes actual ROS graph, scheduler, transport, native process and device latency. Independent source/performance/native review is required before deployment.
