# Shared complete-report admission implementation plan

> **For agentic workers:** Use superpowers:subagent-driven-development, task by task. The full workshop goal remains active after this prerequisite.

**Goal:** Give retained CPU and explicitly admitted compact native reports one executable stateful completion/publication path, without admitting current native component counts as observations.

**Architecture:** Extract the executor-owned completion sink and stage its tracker/cache/high-water changes until final source admission. An opt-in compact adapter owns one pending report and a trusted local provider; it never analyzes images. Retain the existing five ROS outputs and single tracker. Production native admission remains unavailable until complete numerical geometry, original-clock/source proof and final-report capacities are qualified.

**Tech stack:** Existing Python/NumPy, ROS 2 messages/executor and pytest; no new dependencies, C++ operators, services or hardware actions.

**Spec:** [Approved native geometry integration, sections 5–6](2026-10-23-native-geometry-integration.md#5-implement-the-complete-estimator-and-publication-contract), within the original complete workshop plan. This document records implementation choices by the supervisor, not additional user requirements.

## Global constraints

- Keep numerical estimators, tracker association/acquisition rules and physical thresholds unchanged. Successful CPU messages, IDs, types, field absence, order and original stamps must match the frozen retained baseline.
- Fix rejected-observation state advancement: final rejection must not commit its support cache, result high-water, new IDs or acquisition observations. Existing unavailable handling may invalidate pending acquisition and clear expired markers once, retaining the original stamp.
- One tracker, one support cache and the existing Detection3DArray/markers/grasps/contacts/status publisher. Five DDS publishes are not an atomic transport transaction.
- Only the default serialized executor group may complete reports, query attachment TF, track or publish. Native producer threads only validate/copy bounded compact fields and offer a result.
- Native source choice is immutable per node instance. No CPU analyzer fallback, image subscription processing, CPU 5 Hz scheduling limit or mixing of source-owned caches in native mode.
- A packet is data, never authority. The node owns a trusted provider that binds producer/session, original acquisition identities and clock, calibration/config/transform epochs, numerical report capability and admitted final-report resources.
- Current resident timestamp is receipt time, not exposure time. Current 4096 preparation records do not bound final contour/object reports. Do not invent a production object/byte cap or claim either missing proof is satisfied.
- Dense images, masks, points, contours, memberships and residual vectors cannot enter native compact ingress. Reject unknown fields before coercion; own approved small vectors so producers cannot mutate pending evidence.
- Native provider must fence final commit and publication against revocation. A bool/check-only proof cannot qualify external STOP behavior.
- No GPU/new camera execution in this unit. Separate resource review, actual benchmark and static guards precede native-path runtime qualification. Simulation/synthetic ingress is not physical/native-producer acceptance.

## Review focus

1. Source expires during attachment TF or message construction: discard staged observation state, publish no fresh observation, preserve unavailable invalidation.
2. Producer mutates offered arrays or hides a dense value in metadata: own fixed-size copies and reject undeclared/oversized data before conversion.
3. Same calibration but new producer/session, changed transform/config/clock, duplicated or restamped acquisition: trusted admission rejects, with no new acquisition.
4. Notification failure, close/invalidate or concurrent replacement: at most one pending result, no old-result resurrection or timer publication fallback.
5. Rejected report after a confirmed/attached track: preserve identity/ownership, old nested arrays and expected positions at the original stamp; subsequent accepted IDs/messages stay equal to CPU.

## Task 1: Shared completion with staged state

**Files:** Modify `ros2/src/so101_station_objects/so101_station_objects/objects.py` only in imports, `_consume_analysis`, new `_complete_report` and `_publish_report`; add `tests/test_complete_report_admission.py`; adapt existing derived/result/timing source guards in their owning test files. Do not change `StableObjectTracker.update` or geometry functions.

**Interfaces:** `_complete_report(self, job, report, error=None)` consumes the existing CPU completion job unchanged. `_consume_analysis` retains worker/fault/timing receipt and delegates. `_publish_report(self, report, stamp, derived_token=None, *, commit=None) -> bool` retains message construction and final source check; after successful admission it invokes a private precomputed state commit once, immediately before the existing five publishes. Rejection returns False before the callback.

- [x] Write failing CPU tests for the missing common sink and late source expiry after attachment lookup and during materialization. Pin zero rejected cache/high-water/ID/acquisition advancement, unchanged old track dictionaries/arrays/ownership and pending invalidation. Include successful measured/cached/empty reports and a three-independent-stamp acquisition sequence.
- [x] Run the tests with the pinned CPU Python; retain the expected RED result before production edits.
- [x] Extract completion in the existing order. Use `copy.copy(self.tracker)` and `dict(report)` only: current tracker replaces its track mapping before writes and reads old nested values. Stage cache/high-water locally; an internal closure commits only precomputed references/scalars at final admission. Keep source checks around attachment lookup and after all materialization.
- [x] Preserve exact tracker/geometry AST guards. Extend publisher/source-location guards only for the reviewed extraction, bool returns and private commit boundary; retain historical fingerprints and explicit delta evidence. Existing fake-publisher tests must honor the commit hook or exercise the real publisher.
- [x] Verify focused CPU tests and accepted frozen baseline data. Independent resource worker measures staging/actual sink costs and guards the no-nested-write/no-deepcopy assumption before ROS qualification. Then replay the five real-message retained batches through the actual new sink/publisher, matching all 25 messages and resulting accepted state.
- [x] Independent semantic/resource review; supervisor commits verified source and tests. No worker stages or commits.

## Task 2: Closed compact report and bounded ingress

**Files:** Create `ros2/src/so101_station_objects/so101_station_objects/native_complete_report.py` and `tests/test_native_complete_report.py`.

**Interfaces:** Frozen `NativeIdentity` binds the exact producer/session/frame/clock/calibration/configuration/transform identity. Frozen `ReportLimits(max_objects, max_tracks)` is supplied by the trusted provider and refers only to its admitted complete-report scope. `NativeCompleteReport(identity, report)` carries data. `AdmittedNativeReport(identity, report, admission)` owns the validated report and provider handle. `NativeCompleteIngress(provider, notify_ready)` exposes `offer(packet) -> bool` (accepted), `take() -> AdmittedNativeReport | None`, `take_fault() -> str | None`, `invalidate()` and `close()`; replacement/rejection/notification-failure counters are separate. The provider exposes pinned limits/backend status, `admit(identity, owned_report) -> handle`, `check(handle)` and `publish_if_current(handle, action) -> bool`; handles are created and retained locally, never deserialized from packets. Notification faults latch for executor-side unavailable handling.

- [x] Write/observe failing tests for complete supported, visible-only, cached-plane and rejected reports, exact optional-key absence, 1 grasp/object, 5 rejected samples, all retained validation metrics and fixed vector/matrix types. Use retained CPU fixture projections only in tests; expected outputs come from the frozen baseline.
- [x] Implement closed field-specific validation from the retained report schema. Omit/reject support dense diagnostics and unused contours; preserve accepted numeric/type/order/absence semantics. Reject extra tracking/acquisition/candidate IDs, nonfinite/wrong-shape values, custom containers/coercions, nested dense payloads and true qualification/execution claims. Backend metadata must equal the provider's pinned truthful status.
- [x] Require provider-owned complete-report limits; reject missing provider/capability/clock proof and all current count-only/Observed-only payloads. Synthetic limits admit only the frozen fixture population; they do not establish a live production range.
- [x] Own fixed-size copies before offering. Keep at most one pending result, reject overflow as a whole, and fence invalidation/close/notification failure so old work cannot reappear. No analyzer, TF, tracker, ROS publisher, GPU wait/readback or frame-sized copy in offer.
- [x] Verify mutation, replacement, false-proof, epoch/clock/config/transform, duplicate/restamp, lifecycle and notification failure tests. Independent review; supervisor commits after actual results.

## Task 3: Explicit node integration and fenced publication

**Files:** Node factory/init/destroy and completion glue in `objects.py`; create `tests/test_native_complete_report_ros.py`; extend the existing fresh-scheduler/derived/result/timing tests without reducing coverage.

**Interfaces:** An explicit trusted constructor argument selects native mode; omission retains CPU mode. Native guard consumes `NativeCompleteIngress.take()` and calls Task 1's `_complete_report`. Native jobs carry internal `native_admission` handle plus trusted original `stamp`/`ros_stamp`; packets cannot supply completion callbacks or tracker fields. Add keyword-only `native_admission=None` to `_publish_report`; provider `publish_if_current` encloses the already-materialized state commit plus five publishes, invokes the action at most once, and does no TF/materialization under its fence.

- [x] Write/observe failing real-node integration tests, but qualify the new path only after this task's independent resource pass. No live source/camera/control service is needed.
- [x] Preserve CPU input scheduling and guards. Native mode does not process CPU image jobs or inherit its 5 Hz start cap. Keep one native pending report and serialized completion; close safely before destroying guards. Fault timers may report unavailability, never publish an old result.
- [x] Compare actual common sink plus original five-message publisher for CPU and synthetic native, including supported grasp/contact, visible-only, rejected diagnostics, cached-plane absence, three independent acquisitions and original stamps. Match the frozen successful CPU baseline; any truthful backend-only difference must be explicit in expected fixture data.
- [x] Exercise missing/forged/expired admission, producer/session/config/transform/clock changes, duplicate/restamp, stale/future stamp, expired registration, expiry in TF/materialization/final fence, and callback-at-most-once. Assert no rejected observation state commit or fresh output. Verify locked/attached identity across occlusion, same-color ambiguity and timestamped gripper expectations.
- [x] A separate resource worker reviews/refines the combined data-path diff, adds benchmark/static guards under `tools/performance/` and `tests/`, and measures actual staging/compact offer/materialization/common sink with empty/typical/admitted-fixture-max reports and tracks, 20/30 Hz offers, replacements, CPU time/latency/allocations/freshness. Guard dense conversion, analyzer calls, deepcopy, per-pixel work, waits/readback, unchecked shape conversion, tracker nested mutation and unbounded pending storage. Keep source capacities explicit; no global bounded-tracker or zero-allocation claim.
- [x] Run isolated native ROS fixtures with the repository launcher (DDS177/localhost1) after resource approval. Skipped ROS tests are not acceptance. Independently review source/test deltas; supervisor commits verified integration.

## Task 4: Resource, full regression and checkpoint evidence

**Files:** Retain focused benchmark/static tests under `tools/performance/` and `tests/`; add a dated `docs/performance/` readout and machine receipt. Reseal only current v2 source metadata and derived manifests under `tests/fixtures/native_geometry/` after the final code freeze.

- [ ] Consolidate the independent Task 1 and Task 3 resource receipts, benchmark/static guards and exact admitted fixture capacities. Repeat measurements only for changed code or unresolved findings.
- [ ] Node-only changes still alter the whole-file `objects.py` hash in oracle provenance. Replay the unchanged twelve v2 inputs, prove numerical reports/ordered pieces/arrays unchanged, and reseal only v2 procedural/matrix/observed source identities plus derived manifest hashes. Keep every v1 byte and all numerical thresholds unchanged; audit the metadata-only diff explicitly.
- [ ] After freeze, run focused CPU and isolated real ROS lanes, then canonical pinned CPU suite once. Preserve failures, exact commands/artifacts and source/runtime pins; re-run only for changed code or unresolved findings.
- [ ] Independently review the whole checkpoint, preserve raw evidence durably, commit/push the coherent feature branch and verify remote SHA plus unchanged clean shared main.

Current baseline is source `443cd8f82ae19dbb1468088dbf33d9bed6851aaa`. Read-only CPU baseline capture is `/tmp/complete-report-publication-baseline-20261027/`: five batches/25 real messages, with original jobs/reports/types/state/CDR/runtime pins. Its initial fixture failure is retained. Architecture evidence is `/tmp/complete-report-contract-20261027/design-audit.md`. These private artifacts must be preserved with the checkpoint; they are not proof of live native admission.

Complete native geometry operators, truthful clock/source mapping, admitted final-report production budgets, live performance/deployment/control/video qualification, workshop review/rehearsal, physical commissioning/three cycles and final main release all remain required afterward.
