# Optional automatic perception supervision

This default-off node is a metadata-only client of the existing perception recovery coordinator. It never controls the arm, native camera, SDK, processes, or producer reset directly. A successful recovery qualifies current camera metadata admission only; it does not qualify calibration, physical grasping, motion, or sorting.

## Enable explicitly

A reviewed camera-only source artifact, existing trusted journal directory, and the coordinator's existing recovery-artifact directory are required. After installation:

```sh
ros2 launch so101_station_perception perception_supervisor.launch.py \
  enabled:=true expected_source_cohort:=/trusted/source.json \
  expected_serial:=243322071783 journal_directory:=/trusted/supervisor \
  recovery_artifact_directory:=/trusted/recovery
ros2 service call /so101_perception_supervisor/enable std_srvs/srv/Trigger '{}'
```

Launch starts the node with supervision disabled. `enabled:=true` permits the optional node to start; the explicit `/enable` request begins observation. All path/serial launch parameters are typed strings, including leading-zero serials and YAML-looking paths. Existing domain policy applies; never use domain0.

Fixed Trigger APIs:

| Service | Meaning of success |
| --- | --- |
| `~/enable` | Observation accepted, after baseline/graph/producer checks; no immediate reset |
| `~/disable` | Future automatic dispatch disabled; an accepted coordinator job continues |
| `~/status` | Current bounded metadata status returned |
| `~/reset_fault` | Asynchronous terminal-coordinator check accepted; no quota reset or implicit enable |

Status topic `~/status` and service JSON use schema `so101-perception-supervisor-v1`, `qualified=false`, `execution_enabled=false`. Fields include `enabled`, `state`, `latched`, `job`, `attempts_last_hour`, `stable_since`, bounded `events`, `reset_check_pending`, `health_reason`, `current_producer_generation`, `current_pin_sha256`. `/disable` adds `accepted_job_continues`; `/reset_fault` adds `reset_check_accepted`. RPC acceptance is distinct from eventual recovery completion and current health.

## Automatic authority and bounds

Only the two original producer receipt-gap/lost checks emit typed transient fault codes. Fault ID, original clocks, generation and node-code SHA are latched at first failure. Matching error text is insufficient. Unknown, calibration, source replacement, graph, TF, code and reset failures cannot authorize automatic recovery.

Before one automatic dispatch, the supervisor must have observed five continuous seconds of current healthy producer and both consumer admissions. A new typed outage then requires two advancing native CameraInfo pairs acquired after the fault, original receipt/age/pair leases, actual seven fixed graph GIDs, native K/D/profile/frame, independent DeviceInfo serial and exact optical TF. Old derived lineage is not freshened while the producer is unavailable. Original selected pairs/Info receipts, identity request/completion and the two-second preflight budget are checked again after all graph/TF/result access and service-readiness access, immediately before dispatch. No witness is reread or renewed. The sole dispatch target is `/so101_perception_recovery/start_recovery`; the coordinator independently verifies recovery.

The supervisor reserves its attempt in an exclusive, fsynced/read-back journal before sending the start RPC. Limits are three attempts per rolling hour and thirty seconds between attempts. Clock rollback/discontinuity, boot change, unknown in-flight completion, journal ambiguity or changed baseline fails closed. Existing journal files require the complete outer/policy schema and typed quota/handled/in-flight/latch/clock fields; omitted authority never defaults to an empty budget. Explicit first run without a journal is supported. Disable and manual fault reset preserve quotas and handled outages. Existing journal bytes must not be removed to bypass a limit. Boot/baseline changes require an explicit reviewed bootstrap, not automatic journal replacement.

Only the supervisor's own exact job/reset IDs, new private producer generation, immutable artifact hash, unchanged complete source/code/calibration/graph pin and two new consumer epochs may be adopted. No arbitrary file, PID or service comes from a ROS request. All seven GIDs remain unchanged automatically. Actual fresh lineage and both consumer admissions must remain healthy for five seconds after completion; the thirty-second cooldown also applies. Historical coordinator `ready_now` expiry does not cause an outage.

Failed, partial, late or unknown jobs latch without retry. An accepted job is not canceled by disable or shutdown. Shutdown records the unknown/in-flight outcome without claiming cancellation. An unresolved in-flight journal cannot be erased through `/reset_fault`; explicit operator reconciliation is required. Consumer/heartbeat expiry can inhibit/latch observation but never independently authorize recovery. No image/cloud payload subscriptions exist: at most eight raw lineage strings, eight parsed ledger entries and four references per native Info stream are retained; events/handled history are bounded128; public status exposes the latest64 events.

## Source/bootstrap and qualification

Adding producer fault metadata changes `producer_node_sha256`. A **new root-reviewed source bootstrap** is required before enabling this code against a station. The supervisor does not adopt unknown source hashes. Installation/build and actual deployment remain separate steps. No real GPU worker, SDK, native camera, arm or live source is opened by this implementation's synthetic tests.

Native tests must use the fixed isolated launcher:

```sh
SO101_TEST_ROS_WRAPPER=/path/to/prepared/ros2/conda/so101-conda.sh \
  tools/test_perception_supervisor.sh -q tests/test_supervisor_policy.py \
  tests/test_supervisor_journal.py tests/test_perception_supervisor_ros.py \
  tests/test_supervisor_static.py tests/test_supervisor_launch.py \
  tests/test_perception_supervisor_dds.py
```

The optional wrapper override is for an isolated author worktree with no local cache. The default discovers this repository's `ros2/conda/so101-conda.sh`. The launcher forces domain177/localhost before the wrapper and asserts them again before pytest; actual ROS fixtures assert domain177 before initialization. Never invoke these fake-source tests through a bare station wrapper. A prior development invocation in domain76 contaminated real graph/static-TF caches; that test interval is invalid live evidence. Cleanup and fresh source/TF re-attestation are root-owned, with no relaxation of source guards.


## Original health coverage and completion guards

The fixed supervisor timer alone starts at most one consumer-status request per
consumer per200ms, with one pending per endpoint and no catchup. The timer is
reanchored after its bounded callback so sub-millisecond timer phase jitter
cannot turn the exact start-rate cap into alternate-tick polling. A future
completion records its original completion clocks and signals a serialized ROS
guard; that guard consumes bounded metadata immediately and starts no RPCs.

Known past expiry is an unavailable health observation: discard it, reset the
stability window and await a new rate-capped request. Schema/context/epoch/GID,
calibration and future/nonfinite-clock contradictions remain fatal. The current
job, durable quota and dispatch preflight retain their existing strict gates.
Original response, consumer check, header, source and identity clocks are never
renewed when cached or consumed.

Five seconds means continuous *original lease coverage*, not five seconds of
sparse successful samples. A fresh decision after the previous coverage end
resets the window. Proofs with90ms remaining sampled200ms apart cannot establish
that coverage; the supervisor correctly stays warming and does not recover or
qualify the station. Higher polling rates or changed stability semantics require
separate evidence/review. Status includes bounded `health_observation` coverage
end/gap, request/guard counters, and original RPC/proof clocks for diagnosis.
