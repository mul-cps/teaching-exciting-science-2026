# Late local adoption API

Use this explicit API only when the existing supervisor owns an accepted job,
the still-resident coordinator reports that exact job completed, and local
adoption failed. It performs no reset or new recovery attempt. It always leaves
supervision disabled. Service acceptance is pending, not completion/readiness.

```bash
ros2 service call /so101_perception_supervisor/adopt_completed_job std_srvs/srv/Trigger '{}'
python tools/ros_station_action.py perception-supervisor-adopt-completed-job
```

Inspect `/so101_perception_supervisor/status`: `late_local_adoption` has schema
`so101-late-local-adoption-status-v1`, operator `adoption_id`, same owned `job_id`,
state pending/completed/failed and local `operation_success`. Completion includes
artifact SHA, private generation, pin SHA and both consumer epochs. All physical
qualification/execution/readiness flags remain false; explicit enable and a new
five-second current source-health window remain necessary.

The following reviewed contract is normative. Tests and synthetic DDS demonstrate
software behavior; they do not qualify physical motion or a real recovery.

# Explicit late local adoption of an already completed owned job — implemented contract

## Scope and fixed API

Add explicit Trigger `/so101_perception_supervisor/adopt_completed_job` and fixed
quick action `perception-supervisor-adopt-completed-job` (eighth route). No
arguments, paths, reset calls or automatic retry. Availability requires the
supervisor DISABLED/latched, same boot and its existing durable journal containing
one accepted owned job. Preserve the original accepted job ID, reservation,
original coordinator deadline, quota/handled history and original immutable
bootstrap. Unknown dispatch, failed/interleaved coordinator, missing original
journal, changed native/derived GIDs/code/calibration or coordinator restart with
lost completion evidence reject without journal changes.

The existing coordinator job deadline remains historical and is never extended.
The API performs a distinct **local adoption**, not a second recovery or a claim
that historical readiness remains current. No new quota attempt, typed outage,
pre-outage history or automatic eligibility is created. Remain disabled after
success; explicit enable and a new genuine five-second source-health window are
still necessary.

## Entry and historical attribution

Capture fresh monotonic/ROS clocks at Trigger entry; create a volatile operator
adoption UUID and an original two-second deadline. Reject another pending intent.
Return only accepted/pending identity, disabled, ready_now=false,
operation_success=false, qualified=false, execution_enabled=false. Read-only
collection may continue only through this original deadline; no renewal.

Request the fixed coordinator recovery_status and require a fresh bounded RPC
response: schema completed/operation_success=true; exact journal-owned job ID and
original deadline; valid reset UUID; new private generation; exact configured
artifact path and SHA; pin hash; same complete seven GIDs/code/calibration/serial
as the original baseline, with private generation alone changed. Require both
configured consumers' two advancing historical IMAGE witnesses and latest proof
matching completed epoch/gen/pin, false qualification/execution and valid original
header/check/expiry relationships. The coordinator's still-resident completed
operation is the attestation that original receipt gates passed at completion;
its public witnesses do NOT retain their original validator monotonic check time,
so independently reconstructing original receipt validity from that JSON is not
possible. No arbitrary historical validation clock is substituted. If root
requires independent receipt replay instead of this existing coordinator
attestation, the old job lacks that field and must remain unadoptable; this is an
explicit design decision, not an invented timestamp.

## Fresh current admission (all POST-entry)

Collect two advancing native Info acquisition pairs after entry, independently
request/complete DeviceInfo after entry, verify exact serial/current native four
GIDs and optical TF at the original acquisition timestamps; keep existing
250ms receipt, 500ms header, 40ms pair and one-second identity limits. Check all
seven actual publisher GIDs/code/calibration against the prospective completed
pin. Require currently ready healthy producer with its exact completed private
generation and current fully validated original lineage under that pin.

Poll only fixed two derived_source_status IMAGE APIs, never source-health APIs.
Reuse ConsumerProgress with old journal epochs and entry ROS start; additionally
pin its new epoch to the exact completed epoch. Require TWO advancing actual
post-entry IMAGE witnesses per consumer, exact source lineage/gen/pin and all
original current leases at the final decision. New generation lineage collection
is explicit prospective validation, never implicit old-context adoption. No
consumer reset/rebind, start_recovery or reset_source RPC is allowed.

After all filesystem/graph/TF/result/materialization accesses, repeat original
scalar native/identity/image/lineage clocks and original entry deadline with
fresh actual clocks. Any future/schema/context contradiction latches; ordinary
missing/expired evidence cannot succeed and expires the operator intent within
two seconds. No health proof substitutes for an image witness.

## Atomic local commit

Build candidate policy/journal state separately, retaining quota/handled/clock
accounting and original job provenance in a bounded adoption result; clear only
the exact owned completed job/latch and set disabled. Journal.write must atomically
replace and read back this complete candidate with new artifact identity/epochs
BEFORE swapping in-memory pin/ledger/policy. A write/readback ambiguity retains
failure/disabled status and cannot claim adoption; original completed job
provenance stays available for review. No empty/recreated journal and no new
quota counter are permitted. Restart restores the adopted artifact and disabled
state through the existing journal path.

## Evidence and required tests

Keep actual first failure and historical coordinator success separate. First
land the narrow past-only retained-lineage skip with deterministic RED→GREEN;
late-adoption implementation is a separate commit after independent design ACK.

Test exact original deadline unchanged/preserved quota1/handled, same owned job
only, coordinator unknown/failed/interleaved, missing or changed artifact,
future/malformed historical proofs, source/GID/code/cal/epoch changes,
post-entry native/identity/image requirements, no reset/start RPC, no new attempt,
entry deadline crossing after access/materialization, journal ambiguity,
remaining disabled, restarted journal restoration, and all eight CLI routes.
Synthetic DDS must use strict177/local1 and real coordinator fixture semantics;
no hardware/SDK/GPU/actuator calls. Independent static/performance/native review
before root-only deployment or actual adoption.

## Normative transaction amendment (supersedes atomic-commit paragraph above)

No generic transaction framework. Use ONE fixed exclusive file beside the
existing journal: `late-local-adoption.json`, maximum65536bytes, strict duplicate-
free JSON. Schema `so101-late-local-adoption-transaction-v1`; exact fields:
`schema`, `boot_id`, `baseline_sha256`, `adoption_id`, `owned_job`,
`original_journal`, `original_journal_sha256`, `prospective_artifact`,
`prospective_artifact_sha256`, `prospective_pin_sha256`, `entry_monotonic`,
`entry_ros_ns`, `deadline_monotonic`, `phase`. owned_job is the exact existing
accepted discriminated policy job, not a new coordinator request. Original
journal is the complete bounded existing v1 journal record with quota/handled/
clock/job/adopted provenance preserved; its canonical bytes must match its hash.
Phase is `prepared` or `committed_local_success`; neither is enable authority.
Any malformed/conflicting marker is UNKNOWN, never treated as absent. All marker
writes use exclusive ownership, temp+fsync+replace+directoryfsync+readback under
the existing lifetime lock. Existing no-marker v1 journals remain unchanged.

Before writing a candidate journal, persist and read back `prepared` marker.
Recheck fresh original native/current IMAGE/prospective-lineage gates and entry
budget after that IO. Then write the complete candidate journal atomically using
existing strict v1 schema. The candidate retains attempt/handled accounting;
marker retains exact original owned-job provenance even if disk journal changes.
After write/readback, recheck fresh original gates and entry deadline BEFORE any
memory adoption/success. If expired or readback failed, API cannot succeed: while
holding lock reread disk; restore exact original accepted-job journal ONLY if
actual bytes match the known original or our candidate. Never overwrite an
unknown concurrent/different journal. Preserve marker; latch UNKNOWN on any
compensation/readback ambiguity. _fail/close never delete/overwrite marker.

On fully verified local success, persist marker phase `committed_local_success`
with original provenance intact, then repeat post-IO freshness/deadline checks.
If that final marker IO overshoots, compensate as above and retain marker.
The marker is deliberately NEVER silently deleted. This conservative plan makes
ANY restart with this marker require explicit SAME-job confirmation again,
even after a prior local API success: validate marker BEFORE automatic adopted-
artifact restoration, reconstruct the original accepted owned-job policy from
its verified snapshot, remain DISABLED+latched, expose disk/marker ambiguity,
and permit only the same fixed adoption API to collect NEW fresh proof. It adds
no reset/attempt, and can only proceed while the pinned coordinator still owns
that completed operation. This conservative restart-confirmation limitation is
explicit; it avoids an unbounded final unlink/IO silently hiding an uncertain
transaction. ROOT must accept this limitation before implementation. It can be
superseded later by a separately reviewed durable resolution protocol, never by
journal deletion or inferred authority.

| Stage/failure | Durable record | Runtime/result/restart |
| --- | --- | --- |
| Before prepared marker | Original accepted job/quota | Reject, no adoption |
| Marker write/readback ambiguous | Marker or disk unknown | Latch; preserve exact job provenance; no success |
| Candidate write/readback/lease expiry | Prepared marker + known/unknown journal | Compare actual bytes; compensate only known own state; API false |
| Compensation ambiguous | Marker retained + actual disk evidence | UNKNOWN latch; no enable/reset/retry; explicit SAME-job resolution only |
| Final marker IO exceeds entry budget | Marker retains original job; candidate may exist | API false; same compensation policy |
| Successful local adoption | Candidate journal + committed_local_success marker | New in-memory pin/epochs; job/latch cleared; DISABLED; no history minted |
| Any restart with marker | Both records retained | Validate marker first; restore original owned-job provenance/UNKNOWN latch; explicit fresh SAME-job confirmation required |

Fresh 2s budget is a monitored soft bound around nonpreemptible filesystem/native
calls, never hard preemption. Observed post-IO expiry always returns false and
retains an explicit unresolved record; no physical/action authority exists here.

## Normative immediate IMAGE consumption

Fixed200ms control callback alone starts the two IMAGE status RPCs; one pending
per consumer, no catchup or start chaining. Future completion triggers a separate
same-group consume-only operator guard. It consumes at original actual clocks,
validates corresponding prospective lineage and ConsumerProgress immediately,
and starts NO RPC/job/reset. First witness must be current when consumed; it is
not rejuvenated. At final commit, latest second proof per consumer plus current
native/identity/producer/lineage must still be current under original leases.
All guard/error/close paths are bounded by the same ENTRY2s deadline. No source-
health proof substitutes for IMAGE progress and no existing automatic guard rate
or coordinator deadline changes.

The resident coordinator attestation is established by the current unique fixed
service endpoint and non-restorable completed runtime state matching exact
unpredictable owned job/reset/gen/artifact/witnesses. Existing APIs did not pin a
historical service GID or retain validator monotonic clocks; this design does not
claim either. If runtime completed state is lost/interleaved, reject.

### Accounting correction and exact candidate pinning

The normative marker exact field set additionally includes `candidate_journal`
and `candidate_journal_sha256`, the complete bounded prospective v1 journal and
canonical hash. Both original and candidate snapshots together must fit the
65536byte marker limit; reject before any commit if not. Build candidate only
after fresh context/proof gates; marker writing consumes the same ENTRY budget.

Restart handling must compare actual journal with original/candidate known
transaction states BEFORE any reconstruction. It must NEVER rewind newer
quota, handled IDs, policy clocks, jobs or adopted identity from a stale original
snapshot. If actual journal reflects later legitimate activity (for example a
successful late local adoption followed by explicit enable and another recovery),
preserve that LATEST durable accounting and latch UNKNOWN. Reject same-old-job
reconstruction; retain both marker snapshots as provenance. This deliberately
conservative limitation requires explicit later resolution, not deletion or
blind rollback. Tests cover restart after a subsequent legitimate attempt and
prove quota/handled/clock/job accounting never decreases.

### Approved paired-clock compensation descendant rule

Original marker snapshot/hash never changes. Compensation restores its exact
owned job/accounting/epochs/adopted context but carries the LATEST actual observed
validated `(last_mono,last_utc)` PAIR and a nonempty bounded UNKNOWN/failure latch.
Validate nonrollback and the existing two-second wall/monotonic continuity bound
between original, actual disk and current observed pairs. Never coordinate-max
or synthesize a pair. Startup also recognizes this strict original-descendant
state: only the clock PAIR and nonempty failure latch may differ; ALL other
original fields must be identical. Inconsistent/future pairs or any other
activity preserve latest accounting and UNKNOWN, rejecting old reconstruction.
Tests include compensation clock carry, inconsistent pair rejection and later
legitimate activity/no rewind. ROOT and independent reviewer approved this
narrow accounting amendment.

The immediate same-group IMAGE completion guard may also perform LOCAL final
admission and the atomic metadata commit once every current gate passes. It
starts zero RPCs, resets, native/DeviceInfo requests or jobs. The fixed control
callback remains the sole RPC starter. Waiting solely for its next200ms tick
would recreate the known IMAGE-proof lease expiry problem.


## Late-only buffered native acquisitions

Only the late-local operator path selects two independently advancing pairs from
its existing four depth and four color Info records in one 200 ms control
callback. Both original stamps and first unique receipts must be post-entry;
each selected pair receives independent K/D/profile/frame and exact depth-stamp
TF validation. No record is reused across pairs. The control callback refreshes
these real acquisitions while retaining the original independent DeviceInfo
request/receipt. Ordinary automatic recovery and consumer reconciliation retain
their original `_native_proof` implementation unchanged. No subscriptions,
DeviceInfo rate, control cadence, clock tolerance or freshness limit changes.
The completion guard starts no RPC or native action and commits only after both
IMAGE responses have been consumed with no IMAGE request in flight.

Completed public `late_local_adoption` includes
`transaction_phase="committed_local_success"`; this local metadata result keeps
`enabled`, `ready_now`, `qualified` and `execution_enabled` false. A restart with
any retained transaction marker requires explicit fresh same-job confirmation;
later journal accounting is never rewound to the marker's original snapshot.
