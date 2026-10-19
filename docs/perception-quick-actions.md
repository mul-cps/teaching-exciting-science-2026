# Explicit perception quick actions

Use the existing ROS wrapper with a nonzero domain (default76, localhost default1):

```sh
ros2/scripts/station-action.sh perception-recover
ros2/scripts/station-action.sh perception-status
ros2/scripts/station-action.sh perception-supervisor-enable
ros2/scripts/station-action.sh perception-supervisor-status
ros2/scripts/station-action.sh perception-supervisor-disable
ros2/scripts/station-action.sh perception-supervisor-reset-fault
```

Each command makes exactly one fixed Trigger request. No command polls status,
enables supervision implicitly, starts a server, resets the camera or accesses
arm/EV3 APIs. The optional coordinator/supervisor must already be running.
`--help` works without ROS installed. These new commands reject domain0 (including
`00`/`+0`) and invalid domains before importing/initializing ROS; explicit domain
1..232 and explicit localhost settings are preserved. Existing arm/EV3 actions
keep their prior parsing, output, endpoints and deadlines.

| CLI action | Fixed absolute service |
| --- | --- |
| `perception-recover` | `/so101_perception_recovery/start_recovery` |
| `perception-status` | `/so101_perception_recovery/recovery_status` |
| `perception-supervisor-enable` | `/so101_perception_supervisor/enable` |
| `perception-supervisor-disable` | `/so101_perception_supervisor/disable` |
| `perception-supervisor-status` | `/so101_perception_supervisor/status` |
| `perception-supervisor-reset-fault` | `/so101_perception_supervisor/reset_fault` |

JSON output retains `success`/`message` and adds `rpc_success`, `command`, `service`,
the complete parsed `status`, and separate `operation_success`/`ready_now` flags.
Exit0 means only that the server accepted the RPC or returned status successfully;
it does **not** mean recovered, ready for motion, or physically qualified. Recovery
start returns an accepted job ID with completion/readiness false. Ask for status
explicitly later. A completed recovery can retain `operation_success=true` while
its original source proof expires and `ready_now=false`; readiness is the server's
reported observation at its check time, never persistent client/motion permission.

Supervisor enable only enables its configured diagnostic/recovery supervision.
Disable may return `accepted_job_continues=true`: it does not cancel an already
accepted coordinator job. Supervisor reset-fault only accepts an asynchronous
terminal check (`reset_check_accepted`/`reset_check_pending`), does not immediately
claim the latch cleared, and never enables supervision. Inspect supervisor status
explicitly. The CLI never infers readiness from `enabled`, state, or RPC success.

All output remains `qualified=false` and `execution_enabled=false`; a server reply
claiming true authority is rejected, not silently masked. Malformed, nonobject,
nonfinite, duplicate-key or >16KiB diagnostic JSON is rejected. No arbitrary
service names, paths or JSON requests are accepted for these commands.

One absolute2s budget spans ROS setup, discovery, request, response access and
serialization. There are no retries and no late response adoption. A timeout
means outcome unknown: inspect status explicitly before considering another
request. Native ROS initialization/cleanup, filesystem/pipe output and OS calls
cannot be forcibly interrupted by Python; the elapsed guard rejects late returns,
not a hard real-time process-exit guarantee. Tests use fake services and isolated
DDS177 only; no physical producer/controller calls are part of qualification.
