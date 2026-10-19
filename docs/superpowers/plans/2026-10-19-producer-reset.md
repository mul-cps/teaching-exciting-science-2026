# Explicit producer source reset implementation plan

**Goal:** A default-off ROS Trigger resets only the producer's owned projection
worker and source-generation state, without camera, arm or automatic recovery.

**Owned files:** `rocm_rgbd.py`, `rocm_rgbd.launch.py`, a new focused reset test,
and this documentation. Root owns coordinator and package registration.

1. Add immutable `allow_explicit_reset=false` and `~/reset_source` Trigger.
   Reject disabled/closed/busy/30-second-cooldown/permanently-failed requests.
   Validate startup-pinned configuration/node/core/Python/library bytes and the
   unique four-endpoint native graph before acceptance, without treating graph
   presence as stream health. Changed native GIDs are allowed only through this
   explicit reset and subsequent full ordinary source attestation.
2. On acceptance reserve reset UUID and new producer UUID, revoke old publication
   eligibility immediately, cancel pending identity RPC futures, and schedule one
   default-callback-group deferred job. Reply bounded JSON: accepted, not ready.
3. One request-entry +2-second parent deadline covers hash/graph/queue work, with
   checks after each stage and before retirement/construction. Deferred job rechecks startup pins, retires the owned worker through existing
   pidfd cleanup, requires a known reaped child returncode, rechecks pins, then
   constructs exactly one fresh worker using the startup factory/configuration
   and paths. Unknown cleanup, changed pins or construction error permanently
   latches reset_failed; no blind retry. No arbitrary PID/path/import input.
4. Clear old source/buffer/receipt/header/ledger/identity/client/output bindings
   and last result. Preserve lifetime counters. Reacquire ordinary graph,
   DeviceInfo, serial/profile/TF and freshness evidence with original limits.
   Status reports reset ID/generation and readiness only after two new published
   independent cohorts. RPC success means accepted, never recovered.
5. Red→green CPU fakeworker tests and synthetic native DDS tests prove disabled
   rejection, generation rotation, old results/RPC exclusion, busy/cooldown,
   source graph failures, immutable pins, cleanup failure and fresh readiness.
   Static/performance review by an independent author precedes GPU validation.

Normal owned worker cleanup waits at most .5s TERM + .5s KILL under the existing
implementation. Parent file hashing/Popen/native OS calls cannot be hard
interrupted by Python; a wedged call still needs an external process supervisor.
The asynchronous child cold-prewarm watchdog remains30s. This API never resets
camera SDK/USB or actuators, changes input limits, or automatically retries.
