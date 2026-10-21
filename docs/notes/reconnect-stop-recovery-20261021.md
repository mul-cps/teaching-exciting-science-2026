# Reconnect STOP recovery, 2026-10-21

The live owner log `arm-session-67c90083a1e94f3fb7e46f2586258510.jsonl`
records passive connection at monotonic 477472.451, followed by
`shutdown(reason="command_stop")` at 477472.472. The owner remains alive and
continues sampling in normal mode. `ArmSession.handle(stop)` explicitly changes
observe-only mode to normal before verified torque-off shutdown; shutdown,
idle timeout and EOF do not make that change themselves. This proves delivery
of STOP, but the existing log does not identify its sender.

The bridge has no owner command writer queue. Writes go directly to the current
child stdin under the writer lock. Public STOP and reconnect share a lifecycle
lock. Consequently a queued writer leak or a specific operator request cannot
be claimed from this log alone. New low-volume STOP/exit delivery logs record
the request ID, owner generation and calling bridge method for a future capture.

A reproducible secondary failure stranded this state: reconnect waits for
`observe_only=True`, while its explicit STOP path also insists on that mode.
After a delivered STOP changes mode, startup times out and every later STOP
fails before sending a command. The repair fails passive startup promptly when
fresh, source-pinned normal-mode telemetry appears, and lets a later explicit
STOP verify either boolean owner mode. Identity, source, generation, encoder
and torque freshness, six-joint readback and correlated all-off acknowledgment
remain required. A native fault observation survives STOP. Home completion is
revoked. No joint limits, profile, sag allowance or motion admission changes.

Non-hardware reproduction uses the native ROS fixture on domain 75 with stock
CycloneDDS. Both added normal-mode STOP cases fail on integration HEAD 7f8de82
with `new observe-only owner state missing`, then pass with the repair. The
focused reconnect and stop suite passes 72 cases with one unrelated existing
fixture failure excluded: `test_external_stop_cannot_report_trajectory_success`
lacks `Goal.goal_id` and fails identically on 7f8de82 before command delivery.
The new startup test verifies prompt failure without sleeping or owner writes.
Python compilation and `git diff --check` pass. No hardware or live domain 76
interaction was performed. The originating STOP request remains unidentified.
