# Idle owner telemetry isolation

The owner `serve` loop acquires the original six position/torque observations,
constructs their state, and runs the existing held-state acquisition and gap
checks **before** offering an idle snapshot. Every existing 100 ms bound remains
unchanged. The idle snapshot and sample JSONL record are written by one thread.
There is one immutable pending sample; slow storage coalesces idle logs. Observed
position and torque clocks are never refreshed to publication time. Logs are not
an exhaustive 20 Hz sample history.

Before a non-STOP command enters its existing acquisition/admission method, a
publication barrier cancels pending older idle state and waits at most 50 ms for
an in-flight sink. Failure rejects command admission without actuation. This is
not a freshness allowance: any existing original-clock/provider/command deadline
still applies. Synchronous command snapshots and records keep their existing
`_record` hooks and fail-closed semantics. A sequence floor under the single
publication gate prevents popped/queued older idle state from replacing a later
command snapshot. No additional idle samples are offered during a command.

STOP, exit, EOF, idle timeout and sample/command error disarm the physical bus
before telemetry barrier/logging waits. Failure to publish the all-off snapshot
still produces an error; a successful all-off acknowledgement is not invented.
The writer never accesses the bus. No EEPROM, priority, recovery policy, limits,
encoder calibration, or hardware configurations change.

A fixed 256-record in-memory ring stores wall and thread CPU times for original
acquisition/build, offer, wait, health checks, barrier, snapshot and sample-log
stages. At most two lanes and bounded nesting describe currently running stages,
so a blocked sink is visible before it returns. Sample errors capture this
metadata before cleanup or error logging. This is diagnostic data, not authority.

The writer is created only during `serve`. Its shutdown join is 50 ms. If a native
or filesystem sink remains stuck, resources/log stream are retained and a clear
cleanup fault is raised; Python cannot kill the non-daemon thread. External
process supervision is required for a hard process-exit guarantee. Physical
disarm precedes that join. No owner restart/deployment or hardware qualification
is part of the software tests.

`idle_telemetry.py` is included in the existing startup/cold-admission owner-source
hash set. Hashing is not added to the idle or motion tick.
