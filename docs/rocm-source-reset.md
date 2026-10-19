# Explicit projection-source reset

This diagnostic-only API is disabled by default. Launch the already optional
producer with `enabled:=true allow_explicit_reset:=true` to enable it. Both
parameters are immutable; enabling reset does not enable camera/arm control.

`/so101_rocm_rgbd/reset_source` uses `std_srvs/srv/Trigger`. Empty requests cannot
select paths, code, PIDs, IDs or generations. Response `success=true` means only
**accepted, not ready**. `message` is bounded JSON:

```json
{"accepted":true,"accepted_reset_id":"<uuid>","new_producer_generation":"<uuid>","producer_generation":"<same-new-uuid>","state":"resetting","ready":false,"reason":null}
```

A disabled, closed, busy, preparing, cooldown, missing/ambiguous graph or changed
startup-pin request is rejected before touching the worker. Every accepted reset
has a 30-second cooldown. A permanently failed reset cannot be retried in this
process; an external operator/supervisor must resolve it. Unknown RPC delivery
must not be adopted/retried automatically: the caller cannot supply a correlated
request ID; the accepted reset ID returned by this response is authoritative.

Acceptance rotates producer generation and revokes publication before cancelling
old identity RPC futures and clearing source, buffers, headers, receipts, ledger,
output bindings and old results. A one-shot deferred callback in the same default
mutually-exclusive group retires **only its owned worker**, using the existing
stable pidfd process-group cleanup. A known child returncode is required before
creating one replacement. No camera SDK, USB reset, arm/controller or arbitrary
process operation is performed. Graph presence before reset is not stream health;
changed native publisher GIDs can only be adopted through this explicit operation
and subsequent ordinary full source attestation.

Configuration, producer code, fixed six projection modules, interpreter and native
runtime-library byte hashes are compared to startup pins before acceptance and
around retirement/construction. Replacement uses the same startup factory,
configuration snapshot and resolved paths. Any cleanup/construction/pin failure
stays unavailable; a late or rejected replacement is retired and its owned handle
retained even if cleanup is unknown. No blind replacement or reset loop occurs.

One parent deadline is set at Trigger request entry plus 2 seconds, including
acceptance hashing/graph checks and deferred queue delay. It is rechecked after
blocking checks and before resource changes; expired work cannot begin retirement
or launch a replacement. Pre-acceptance timeout rejects without worker changes;
post-acceptance timeout permanently fails closed. Necessary retirement of an
already-constructed late replacement still runs after the parent deadline. Existing owned
cleanup waits at most .5s after TERM and .5s after KILL. Blocking filesystem/native
calls/Popen cannot be hard-interrupted by Python: this is not a hard process-exit
or stuck-OS-call guarantee; external process supervision remains necessary. The
separate child cold-prewarm watchdog remains 30 seconds. A failed startup before
two new published cohorts permanently marks `reset_failed`.

The normal 5 Hz producer status includes `reset` (null before any accepted request),
with `accepted_reset_id`, `new_producer_generation`, `producer_generation`,
`state`, `reason`, `published_new_cohorts`, `ready` and `cooldown_remaining_s`.
`ready=true` requires at least two actually published independent new-generation
cohorts and current normal original-stamp/receipt/identity health; no fresh receipt
or stamp is synthesized. The coordinator must independently validate those two
lineages, match the returned reset ID/generation, capture new source pins and
explicitly reload consumers. Heartbeat/RPC success alone is insufficient.

All 40 ms pairing, 500 ms original exposure, 250 ms source receipt, 1 s identity, exact
CameraInfo/TF and seven-endpoint identity checks remain unchanged. Consumers keep
rejecting old generations until explicitly repinned. Geometry, physical sorting,
model/marker calibration and motion remain unqualified and unauthorized by this
API.
