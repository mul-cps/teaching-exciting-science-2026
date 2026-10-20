# Historical transaction finalization, then opt-in private availability repair

ROOT design decision under the user's overnight autonomous instruction: implement A1 (pinned startup receipt migration plus future explicit-enable finalization), then B as separately reviewed commits. This design authorizes isolated software work and qualification. ROOT alone decides any deployment or response to the existing natural fault. No controlled outage trial is repeated; no arm or native-camera action is authorized.

## Current source/evidence

Pinned source LIVE b1dae717 / STAGED e5ba970. Module paths below use `ros2/src/so101_station_perception/so101_station_perception/` unless otherwise stated; launch/test/docs paths relative to repo root.

- `supervisor_policy.py:11–24`: `transient_fault` accepts ONLY
  `private_source_receipt_gap` / `private_source_stream_lost`, exact fault UUID,
  current private generation/node SHA, nonfuture original first-observation
  clocks and terminal producer reset. Old first-observation time is allowed;
  the current producer status receipt and identity are checked independently.
- `perception_supervisor.py:690–694`: unavailable producer is ignored while
  warming without current pre-outage eligibility. `535–544` always claims that
  history; `551–566` and `_native_proof/_native_final` use its deadline.
- `supervisor_policy.py:97–108`: outage dedup precedes native verification;
  quota is durably reserved before dispatch. `29–52` has strict existing
  automatic/operator-reconciliation journal discriminants. `130–144` completes
  to warming/disabled and preserves attempts/handled/inflight accounting.
- `late_adoption.py:108–127`: ANY retained marker currently triggers UNKNOWN,
  even `committed_local_success`; only exact candidate or strict original
  failure descendant is recognized. `supervisor_journal.py:49–50` invokes this
  BEFORE supervisor adopted-artifact restoration.
- Actual ROOT late-local receipt:
  `automatic-perception-supervisor/ROOT-late-local-actual-completed-status-once.json`,
  SHA `56610c3d4291fe02dd1342bfe266a11fa3a35cbddea88e70e96e94991922f4f0`.
  Acceptance SHA `fc22d9a47c25b646a42bda6ec219982ed28b63331d52817e1992015d95177e7e`.
  Explicit enable SHA `9a91da104c8428aabf866e610d61dc06982dfd590e4bdaa4c8db8c3831249636`.
  Exact adoption `463b8c87b9fb4f82bfcd2252a63ff329`, job
  `ae1222a1bcab47c0bfed06b4d180a5a2`, generation
  `107e01f1e9c64ba590c12270eedf8cf4`, pin
  `327fff7b40729040f85c33b2068f9142cb2f7dfd22a01d8299d357df5fa12816`.
- Natural fault mono325410.809473416 / 01:15:03UTC: typed private stream loss,
  last depth activity254.169ms; no intervention. Last usable history expired
  about85s earlier. ROOT hour audit reports later native callback counts resumed
  +5218depth/+5219RGB/+5218depthInfo/+5221colorInfo while published305245 stayed
  frozen under the permanent private latch, same8GIDs/generation. This supports
  a software availability opportunity, not attribution of SDK vs DDS cause.

## A. Explicit historical ACK/finalization (recommended prerequisite)

### Authority boundary

Acknowledges ONLY that one exact local metadata transaction actually completed.
It is NOT new source/image/native readiness, calibration, stable5s history,
permission to enable, a new baseline or permission to clear a real unknown job.
Startup remains DISABLED. No source/motor/camera/consumer reset calls.

### Minimal current migration

Add optional explicit read-only startup confirmation input (completed CLI receipt
path+expected SHA, and the acceptance path+SHA for exact pairing). ROOT supplies
trusted bounded receipts for this migration. Resolve strictly beneath the trusted existing journal
directory, cap each input128KiB, reject duplicate/nonfinite JSON, require
exact successful fixed CLI command/service wrapper/raw-vs-parsed status.
No implicit search for newest artifact. All launch path/SHA parameters typed str.

Under the existing lifetime journal flock, BEFORE `load_restart_marker` and
before adopted-artifact restore:

1. Strictly validate existing same-boot/same-bootstrap journal and original
   paired wall/mono continuity. Do not create an empty journal/default quota.
2. Read bounded committed marker; require exact known schema, adoption/job,
   candidate hash, prospective artifact SHA/pin/gen/epochs. Merely having the
   committed phase is insufficient: it was written BEFORE the final freshness
   gate, so the explicit actual successful receipt is indispensable.
3. Validate ROOT actual acceptance and completed status refer to that exact
   adoption/job. Require completed `late_local_adoption` operation_success=true,
   phase committed_local_success, all nested authority flags false, exact
   artifact/gen/pin/epochs and resident-coordinator historical-attestation scope.
   This trusts ROOT's immutable successful runtime receipt; it does not recreate
   missing historical validator monotonic clocks or claim current IMAGE health.
4. Require a STRICT CANDIDATE DESCENDANT journal, not original compensation:
   same outer schema/boot/bootstrap/adopted/epochs; same policy schema/handled;
   inflight=None and latched=None; paired nondecreasing clocks with existing2s
   wall/mono continuity. Attempts may ONLY drop original candidate entries aged
   >=3600s by the ACTUAL journal last_utc. No new/insertion/reordered attempts or
   changed handled/job/artifact/epochs permitted. Preserve exact latest bytes.
   This accommodates old.close() clock advance and legitimate aged attempt1→[].
5. Re-read exact current journal hash and marker hash under the same lock before
   durable ACK. Unknown/later activity rejects untouched; NEVER reconstruct the
   old accepted job or rewind latest accounting for this ACK path.

### Durable archive / crash table

Use fixed controlled journal filenames keyed by the exact marker SHA H, not
arbitrary paths or a general transaction framework:
`late-local-adoption-committed-H.json` (exact original marker bytes) and
`late-local-adoption-ack-H.json` (strict bounded ACK schema).

Archive original marker bytes exclusively, fsync/read back, then atomically commit
and fsync ACK containing H, candidate hash, exact adoption/job/gen/pin/epochs,
bootstrap+boot, actual receipt digests and strict current-journal descendant hash /
accounting snapshot. Cap total ACK+embedded required evidence explicitly (e.g.
256KiB); reject oversize rather than abbreviate provenance. No journal rewrite.
After durable verified ACK, remove the active marker ONLY as part of this named
archive operation and fsync directory. Keep both archive and ACK indefinitely.

- Crash before ACK: old unresolved-marker rule remains; orphan archive grants
  no authority and original marker/journal remain intact.
- Crash after durable ACK before active-marker removal: new loader recognizes
  ONLY exact matching committed H+ACK+archive+strict candidate descendant and
  skips UNKNOWN/reconstruction. It must finish idempotent retirement of the redundant active marker under the
  lock before normal policy activity. Cleanup failure stays disabled/UNKNOWN.
  Unmatched/malformed ACK remains
  UNKNOWN, preserving latest journal.
- Crash after active removal: ordinary constructor restores the already-adopted
  persisted artifact and starts DISABLED. ACK/archive remain audit evidence.
- FS ambiguity: do not rewrite journal or infer current readiness. Report
  historical-finalization unknown/failed; later startup may recognize ONLY a
  completely validated durable ACK. OS writes are nonpreemptible; no hard
  storage wall-time guarantee. Historical completion is a durable fact, not a
  short-lived data lease renewed by the archive write.

For current startup, existing trusted actual CLI completion already attests the
then-resident coordinator operation. If ROOT instead requires a NEW current
coordinator RPC during ACK, that needs a two-phase disabled startup/admin node;
see alternative below. No claim of captured historical coordinator service GID.

### Future own-runtime finalization

Recommend existing explicit `enable` finalize an unambiguously successful own
runtime late-local result BEFORE further policy activity. Require live
`late_result.completed` + committed marker + exact candidate descendant; archive
with the same helper, then proceed existing explicit enable to warming. Failure
leaves disabled/latched, no reset or quota change. No ninth service is necessary.
Alternatively a fixed Trigger `finalize_local_adoption` may perform historical
ACK while remaining disabled; same validator, never a path/hash argument on ROS.
Do not auto-ACK merely from marker phase or arbitrary external status JSON.

### Alternatives / ROOT decisions

1. Recommended: explicit pinned startup receipt migration + future explicit
   enable finalization, as above. Pure file validation occurs under current lock
   before old UNKNOWN reconstruction; smallest change, no new coordinator RPC.
2. More elaborate: cold metadata-only CLI takes the same lifetime lock while
   supervisor stopped, validates receipts and one fresh fixed coordinator status
   attestation, writes the same archive. This proves current residency but adds
   ROS/lifecycle code. It must not open SDK/camera/arm or call start/reset.
3. Reject: erase marker/journal, infer success from committed phase alone, trust
   arbitrary new source artifact, or globally suppress UNKNOWN restoration.

## B. Distinct opt-in unarmed private availability repair

New launch/node boolean `allow_private_availability_repair`, default false. It is
separate from stable5s source-health/history and explicit supervisor enable.
Existing armed/history route is preferred and its default path unchanged.

When enabled, warming, terminal/no job/reset/reconciliation/local transaction,
and no latch, a fresh unique resident producer heartbeat may start the new route
ONLY for the two positively typed private-availability fault codes. Bind exact
same immutable fault UUID, installed producer node SHA, current persisted pin,
private generation, bootstrap/code/calibration, same7GIDs and known consumer
contexts. No automatic GID/epoch/code/calibration/new-baseline adoption.

The first-observed fault may be old; current heartbeat must be original<=250ms,
nonfuture and exact resident identity. Start a NEW explicit repair intent clock
pair on the decision, original deadline ENTRY+2s. This is not a renewed fault
clock or forged history_verified=True. Never mint history/stable/ready from it.

Persist dedup of the same fault before collection; quota remains same3/hour and
30s cooldown. Use a NEW STRICT journal origin `private_availability_repair` with
outage UUID, preserving existing two variants unchanged. Reserve durably BEFORE
fixed coordinator start RPC; unknown/late RPC, persistence or generation errors
latch/no retry. Any already-inflight/unknown job blocks this route.

Within original ENTRY2s, require POSTENTRY two advancing actual native Info pairs,
original first-unique receipts, same4 native GIDs, original250ms/500ms/40ms bounds,
independent DeviceInfo request+completion/serial, exact K/D/profile/frame and
exact optical TF. Keep native and metadata starter cadence200ms; no new payload
subscriptions or native SDK open. If bounded buffered native pair selection is
reused, scope it ONLY to this explicit new route; do not silently change existing
armed automatic native behavior. `_native_final(authority=...)` currently selects
operator contexts and cannot be reused with fake late/reconciliation fields;
use a distinct repair context/guard and preserve default armed path.

Also obtain fresh fixed consumer source-binding metadata replies: exact known
consumer epoch/gen/pin, machine `binding_invalidated=false`, valid schema/current
RPC receipt. Ordinary expired/unavailable proof is allowed only as negative
availability metadata, NOT health. Structural/context/future/schema/unknown
binding errors block repair. Current healthy derived IMAGE/lineage is NOT required
before resetting a permanently latched private producer; native evidence is
independent. Unknown initial consumer context still requires explicit bootstrap.

Require coordinator fresh terminal status; after all readiness/graph/FS/result
access, recheck original intent2s, original native/identity receipts/headers,
current producer/fault/context and consumer metadata scalars immediately before
ONE fixed coordinator start. No new unbounded retries/RPC chains.

Reuse unchanged owned40s coordinator completion, exact new private generation /
artifact pin attribution, two advancing actual IMAGE proofs per consumer, and
existing fresh prospective-lineage adoption. Failed repair latches; success
returns warming with no5s/history authority. If SDK/raw payload does not resume,
coordinator qualification fails safely—fresh Info alone is not raw-image proof.
Subsequent new typed fault may repair only under remaining quota/cooldown and all
new independent preflight gates. No arm, native-camera restart or calibration
permission follows from this availability feature.

## Narrow files and required review

ACK commit: `late_adoption.py`, `supervisor_journal.py` (optional explicit migration
input before marker load), `perception_supervisor.py` (typed parameters/future
explicit-enable archive), leaf/station launch forwarding, tests and docs only.
No arbitrary journal writer API or generic transaction library.

Repair commit: `supervisor_policy.py` strict new origin/intent, supervisor fixed
begin/preflight/dispatch guards, one launch bool, dedicated pure/nativeDDS tests /
docs. Preserve default armed/reconciliation/late paths and source/motion limits.

Tests: real successful receipt vs committed-but-failed marker; candidate clock
advance/natural aged quota removal; mutation/unknown job/latch/later activity;
crashes before/after ACK/archive; startup remains disabled. For repair: no5s
history, old typed event with fresh status, all POSTENTRY native/consumer gates,
late service readiness/FS/final clock stalls, dedup/quota/unknown dispatch, strict
new-origin restart, unchanged armed path, one real synthetic DDS coordinator job
with2actual IMAGE witnesses. Independent static/performance/native review before
ROOT-only build/cutover/actual natural-outage qualification. No implementation yet.

## Final ROOT decisions / builder review

- Candidate attempts equal the candidate list minus its exact aged prefix at the actual persisted `last_utc`; no arbitrary subset pruning. Preserve latest paired clocks/accounting bytes.
- Archive/ACK retries are idempotent only for identical validated bytes. A conflicting file or failed compare/readback remains UNKNOWN. Once the active marker is retired, old archives must not restrict later legitimate journal activity.
- Existing explicit enable finalizes its own successful historical result before any live-source check or subsequent policy activity. Historical ACK never enables policy or supplies current native/IMAGE health. No ninth quick-action route is required.
- The fresh terminal coordinator reply may contain an old completed job with `ready_now=false`; old IMAGE expiry does not prevent a new preflight.
- New private-repair binding metadata requires the exact known consumer epoch/gen/pin and explicit `binding_invalidated=false`, correct schema/false admission/qualification flags, and original current RPC clocks. Missing/expired IMAGE/source-health proof is negative availability, not health. Reject malformed, contradictory or future numeric evidence. Human reason strings are retained diagnostics and never authority.
- Default-off repair is deliberately bounded: preflight/unknown/persistence/structural failures latch; no automatic repeat of a handled fault. This does not promise arbitrary camera/OS/hardware recovery.
- The new repair must share fault dedup with the original armed path; keep all original armed, reconciliation, late-adoption and motion bounds.
- Review of this architecture by an independent builder identified no remaining architectural blocker. Native/GPU/live validation requires separate evidence after source review.

### Exact confirmation input/lifecycle decisions

Read-only string parameters: `confirmed_adoption_acceptance_artifact`, `confirmed_adoption_acceptance_sha256`, `confirmed_adoption_completed_artifact`, `confirmed_adoption_completed_sha256`; all empty by default, all four required together. Resolve input files strictly beneath the existing trusted journal directory (ROOT copies exact immutable receipt bytes there before deployment). Each input is capped at128KiB with strict duplicate/nonfinite JSON rejection. No newest-file search or arbitrary external path.

A matching durable ACK encountered with its redundant active marker MUST finish idempotent retirement under the journal lock before normal policy activity. If cleanup cannot be confirmed, remain disabled/UNKNOWN. Do not leave an active old marker while permitting new jobs. Once active marker is absent, ordinary latest journal authority proceeds; historical ACK is not a constraint on later legitimate completed jobs. Existing stale confirmation parameters must never apply an old journal state to new activity.

Implementation is authorized under the overnight instruction after ROOT plan review, with Task A and B independently reviewed and source-pinned in that order. This authorizes no live action by workers.

### Bringup wiring clarification

Source inspection found no existing supervisor include in the aggregate launches. Task A adds a real `perception_supervisor_enabled=false` include in `perception.launch.py` and forwarding in `station_live.launch.py`, including the leaf's existing baseline/journal/artifact/serial inputs plus the four confirmation strings. Aggregate argument names use the `perception_supervisor_` prefix; leaf/node confirmation names remain the exact names above. Defaults do not start another supervisor or change any camera/consumer/motion behavior. The launch flag only spawns an available supervisor whose policy still starts DISABLED. Document the separately launched recovery coordinator dependency; do not add coordinator wiring in this task. Task B forwards its new default-false repair boolean through the same include.
