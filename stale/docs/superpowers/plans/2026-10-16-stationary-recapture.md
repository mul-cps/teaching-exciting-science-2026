# Bounded stationary camera recapture

The previous six-source native test reaches2985 toward2973, then correctly holds because a moving image has unequal encoder brackets. This phase change keeps that rejection and all100/50/250ms gates.

## Authority phases

1. A fresh immutable qualified acquisition certifies the whole remaining path. Issue at most one2count waypoint. Track its original certificate deadline; no successive waypoint is issued before recapture.
2. Existing20Hz position/goal/torque reads must show the exact already-commanded waypoint, all six torque1, firmware and32count envelope, before the original certificate expires. Keeping the already-commanded verified target is the stationary hold; do not write a new measured goal or profile. If no exact arrival before expiry, abort with existing hold/stop semantics.
3. Hold wait may outlive that old proof, maximum500ms capped by six-second operation. No Goal/enable/profile writes. Continue actual six-position/torque reads at≤50ms, cancel/stop pumps, exact heldraw/allTQ1. Pose or torque loss disarms; explicit stop takes precedence. Missing/invalid identity, stream, unknown geometry or timeout produces failed-held fault.
4. Only a newly acquired original depth header strictly after hold entry, exact held acquisition raw, identical measured brackets, unchanged camera/owner/runtime generation, original100ms synchronization and250ms age may become a new scene. Reprove full remaining path using a fresh exact current read≤50ms and final raw/cancel check before the next2count write. Never rebind an old scene into posthold authority.

## Typed health prerequisite

Collector worker writes bounded atomic `health.json` inside its runtime nonce directory. Fields: schema, nonce/runtime start, exact selected source map, arm/calibration identity and owner start, actual camera identity including original request/confirmation clocks/generation, actual depth/info header+receipt clocks, worker-observed monotonic clock, explicit state enum. Allowed transitional states are `pending_future_bracket`, `nonstationary_bracket` and the narrowly validated `awaiting_post_confirmation_exposure`; these permit stationary waiting only. `ready` allows evaluation of a new cohort. All other states are unavailable/fatal for wait. Invalid identity or generation synchronously revokes cached callback state; worker publishes invalid status without touching motor/lifecycle locks. Late writes stay confined to old nonce.

Runtime background loop validates and caches health; provider access never reads files or constructs geometry. Health may authorize stationary waiting only. It never authorizes motion, extends an image/pose timestamp, or grants clearance. Missing/stale/malformed health fails. Request-start lease1s and actual depth+info header/receipt age250ms are checked independently; current encoder/torque acquisition age50ms stays separate.

## Verification

Keep generic/unbound provider behavior unchanged. Add deterministic phase tests for complete original2993→2973 with immutable posthold cohorts; no writes during wait; old-cohort rejection; camera/identity loss, streamgap, pose/torque change, stop/cancel,500ms wait and6s total timeout. Preserve ordered bus operations outside phase-required reads; no critical-path filesystem or geometry preparation. Independently review safety/performance, then run actual ROS→collector→factory→oracle→FakeBus full original trajectory. This is not physical qualification; actual registration/model contacts remain disabled/unqualified.

## Read budget and implementation refinements

Stationary wait deliberately schedules raw reads every25ms nominally, reserving20ms bounded bus-read time and5ms scheduler margin before the unchanged50ms actual-acquisition interval limit. It reads raw, torque and current Goal each wait poll (up to40Hz/120 read operations per second, at most about20polls/60reads per500ms phase), plus explicit error cleanup reads. This extra phase read budget does not change the idle2-read telemetry budget. Actual read intervals and ages govern admission; a50.001ms interval or stale sample disarms.

Preparation/hash/geometry faults independently override producer-ready health until a genuinely newer successful cohort is prepared. Unknown clearance never becomes a wait transition. The actual scene used by the dispatch proof must match the pinned camera generation and both original exposure lower-bounds must exceed hold entry; readiness-candidate metadata alone is insufficient. Final prewrite raw matching also rechecks cached generation/health without I/O.500ms/6s phase deadlines and cancellation are rechecked after provider access, not only at loop entry.

Staged authority requires the exact concrete `DepthRecoveryScene` acquisition on the actual checked `BoundRecoveryScene`, not a subclass/caller capability flag. Its continuous certificate uses the sum of absolute per-joint displacement bounds over the full effective volume; thus it covers the entire independent per-joint interval, including asymmetric20/4count motion clamped to2/2. This stronger concrete contract is tested with the real oracle. Generic line-only adapters do not gain staged authority, and redundant geometry queries are not added to the control path.


## Same-context refresh continuation

Retain the immutable exact hold context through successful candidate evaluation,
proof and final quiet settling. Clear it immediately before the next Goal attempt
or only after final success verification. A dedicated internal transition may
resume that same context only for validated same-generation identity renewal;
its original500ms deadline and overall6s deadline never reset. Every retained
sample remains exactly the held six raw values/all-six torque-on, with actual
acquisition interval and sample ages<=50ms. Resume verifies current Goal too,
without Goal/enable/profile writes. Unknown, geometry and preparation faults
cannot be reclassified as refresh. Final success may reuse the already-commanded
exact target, with fresh actual position/torque/Goal and post-log proof/health/
cancel/deadline checks, instead of another hold Goal. A ready health record from
a new confirmation does not authorize an older prepared scene or missing scene;
only genuine same-context refresh provenance permits continued stationary wait.

The unchanged external ROS15Hz/80ms-delay synthetic2993→2973 fixture completed
successfully in the final marker-first eight-case replay in2.399s (ideal transport
2.048s), with11 Goals (initial hold plus ten certified waypoints), ten original
hold contexts, zero Goal writes within waits and no EEPROM writes. All eight
native cases passed together in14.75s. Independent delayed replay completed in
2.600s with11entries across10unchanged original contexts and no wait writes.
The older2.550s/2.169s replay is retained as historical checkpoint evidence.

Final software gates:1595system tests passed/84skipped/6subtests;264actual
station-interpreter tests;606broad native tests/1EGL-renderer skip;19packages
built. This is software evidence only. Actual rig calibration, geometry,
contact, clearance and recovery qualification remain separate unfulfilled gates.
