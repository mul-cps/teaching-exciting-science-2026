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

Before one automatic dispatch, the supervisor must have observed five continuous seconds of current healthy producer and both consumer SOURCE-BINDING health proofs. A new typed outage then requires two advancing native CameraInfo pairs acquired after the fault, original receipt/age/pair leases, actual seven fixed graph GIDs, native K/D/profile/frame, independent DeviceInfo serial and exact optical TF. Old derived lineage is not freshened while the producer is unavailable. Original selected pairs/Info receipts, identity request/completion and the two-second preflight budget are checked again after all graph/TF/result access and service-readiness access, immediately before dispatch. No witness is reread or renewed. The sole dispatch target is `/so101_perception_recovery/start_recovery`; the coordinator independently verifies recovery.

The supervisor reserves its attempt in an exclusive, fsynced/read-back journal before sending the start RPC. Limits are three attempts per rolling hour and thirty seconds between attempts. Clock rollback/discontinuity, boot change, unknown in-flight completion, journal ambiguity or changed baseline fails closed. Existing journal files require the complete outer/policy schema and typed quota/handled/in-flight/latch/clock fields; omitted authority never defaults to an empty budget. Explicit first run without a journal is supported. Disable and manual fault reset preserve quotas and handled outages. Existing journal bytes must not be removed to bypass a limit. Boot/baseline changes require an explicit reviewed bootstrap, not automatic journal replacement.

Only the supervisor's own exact job/reset IDs, new private producer generation, immutable artifact hash, unchanged complete source/code/calibration/graph pin and two new consumer epochs may be adopted. No arbitrary file, PID or service comes from a ROS request. All seven GIDs remain unchanged automatically. Actual fresh lineage and both consumer SOURCE-BINDING health proofs must remain continuously healthy for five seconds after completion; the thirty-second cooldown also applies. Historical coordinator `ready_now` expiry does not cause an outage.

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

## Separate source binding and selected-image admission

Supervisor warmup/rearm calls only each consumer's `~/derived_source_health`
Trigger API with schema `so101-derived-source-binding-health-v1`. This is the
explicit source-health acceptance amendment after the recorded 5/20/50Hz matrix
showed selected-image proofs could not establish continuous coverage. The old
`~/derived_source_status` API remains selected-image admission; the coordinator
still requires two actual advancing IMAGE proofs after recovery. Metadata source
health cannot substitute those proofs.

A source-health proof always declares `image_admitted=false`,
`processing_progress=false`, `qualified=false`, and `execution_enabled=false`.
It may be healthy while analysis is stalled; it establishes no detections or
physical readiness. `binding_invalidated` must be a strict boolean: true is a
persistent/context contradiction and fatal; false unavailable proofs merely
inhibit/reset stability. No reason strings grant authority.

Both proofs pin the consumer epoch, reviewed source pin, private generation,
latest exact original lineage and seven actual publisher GIDs. They explicitly
require independently received native CameraInfos, local optical TF at the exact
original depth timestamp, and producer-attested original DeviceInfo identity
(no new consumer DeviceInfo RPC). All four original header stamps and seven
original receipt deadlines are retained. Receipt expiry is their exact minimum;
identity expiry stays separate; effective expiry is the minimum of both. The
supervisor also checks its own original lineage/RPC receipts, current producer
heartbeat and all original 500ms headers. No checked timestamp renews evidence.

The same 5Hz request cap, completion-only guard and strict five-second continuous
coverage remain. Short proofs and actual gaps correctly prevent arming; installing
the new API does not establish actual-station coverage or automatic recovery.
The recorded synthetic metadata-only DDS fixture is replayed at its ORIGINAL
client-take clocks, never retimestamped as live qualification.

## Bounded historical eligibility across an outage notification

A producer can report its typed fault after current leases have already expired.
Current health immediately becomes unavailable/warming. A separate **volatile
pre-outage eligibility history** can authorize only the existing native preflight;
it is never current readiness or a source/motion proof. A token is minted only
while genuinely armed after five continuous seconds of source binding, recording
its original verified ROS/monotonic clocks, exact source pin/consumer epochs and
original coverage end. Its fixed deadline is coverage end plus **two seconds**.
Short healthy blips cannot renew it; a new full five-second qualification can.

The typed fault must first occur after the corresponding last verified healthy
ROS and monotonic observations and arrive before that original deadline. Both
arrival orders (lease expiry first or typed status first) retain one-use semantics.
All original postfault native/Info/identity/header/TF/graph checks still apply.
The native preflight and final actual start RPC are bounded by the minimum of
the existing two-second preflight budget and the ORIGINAL history deadline;
there is no two-plus-two-second extension. The coordinator still requires actual
selected-image witnesses after its own reset.

Disable, explicit fault reset, restart, rejected lineage, source/context/epoch or
structural/clock contradiction clear history. Nothing is journaled as historical
authority; quota and handled-outage journals remain unchanged. ROS clocks cannot
roll back against the original pair; ROS/monotonic and existing UTC/monotonic
continuity reuse the existing two-second sanity bound. This is **not** a data-age
tolerance: receipts remain250ms and original headers500ms. Status exposes only
`pre_outage_history_available` and its original clocks/deadlines separately from
current health; all qualification/execution flags remain false.

## Explicit same-boot consumer reconciliation

`/so101_perception_supervisor/reconcile_consumers` (`std_srvs/srv/Trigger`) and
`station-action.sh perception-supervisor-reconcile-consumers` request ONE
operator-owned reconciliation, only while supervision is explicitly disabled,
terminal and unlatched. No request arguments, arbitrary artifact paths or
endpoints exist. Acceptance returns a `reconciliation_id` and ORIGINAL two-second
preflight deadline; it is not a coordinator job ID, readiness or completed repair.
Read supervisor status for bounded `operator_reconciliation` progress and the
existing owned `job`. A pending operation blocks enable and another reconcile.

This repairs consumer replacement epochs on the SAME boot with the SAME seven
source publisher GIDs, producer/code/calibration/serial and immutable baseline.
The producer must currently be healthy. After the request, two advancing native
Info pairs and a NEW independent DeviceInfo request/completion are required; all
original receipt/header/TF/graph gates remain. The two-second budget starts at
Trigger ENTRY and includes admission, file/graph access, reply serialization,
queueing, durable reservation and the final actual coordinator start RPC. It is
never extended or borrowed from automatic pre-outage history.

Before that sole fixed coordinator RPC, an `operator_reconciliation` identity is
reserved and fsynced/read back in the SAME journal, charging the SAME three/hour
and thirty-second cooldown. No typed fault or automatic eligibility is forged.
Unknown/late/reserved outcomes remain charged and latched; old handled outages
are preserved. Restart with an unresolved manual reservation stays latched.
Disable before reservation cancels only read-only preflight; disabling an already
reserved/accepted job never claims cancellation.

The coordinator still resets the private producer once, captures its exact new
private generation/artifact and requires two actual IMAGE admission proofs after
consumer rebind. The supervisor adopts only its OWN exact job/reset/artifact/hash
and new epochs. Successful reconciliation remains DISABLED; explicit enable and
new five-second continuous source health are required afterward. Current image/
source/calibration/motion qualification flags remain false.

If an ordinary terminal latch exists, inspect it and complete the existing
`perception-supervisor-reset-fault` API first. That never clears quotas or adopts
epochs. Unknown in-flight jobs require separate resolution, not a reconcile.
Producer ROS process/GID replacement, changed code/calibration, OS reboot and
monotonic boundaries are OUT OF SCOPE and fail closed. Do not edit the immutable
baseline, delete/change journal directories or invoke bare coordinator recovery
expecting it to become an automatically adopted supervisor job.

### Separate source-health sampling cadence

The read-only source-binding health starter runs at a fixed100ms cadence, with
one pending RPC per consumer. Each start is gated again after service readiness;
the next due time is anchored after actual admission. The health timer resets
after its callback and never catches up missed intervals. Completion guards
consume existing responses only; they cannot chain RPCs.

Control/status, native preflight, quota/journal transitions and coordinator job
monitoring retain their separate200ms cadence. Disabled, closing, latched,
resetting, native-preflight and owned-job phases start no health RPCs. Close
cancels the new timer before pending-future cleanup.

This changes sampling only. All original250ms receipt/500ms header/identity
leases, strict five-second continuous overlap, original two-second pre-outage
history and dispatch deadlines, quota and cooldown remain unchanged. Diagnostics
report both periods and the scalar refresh callback count. A healthy sample or
receipt-only matrix is not full five-second qualification or image progress.
