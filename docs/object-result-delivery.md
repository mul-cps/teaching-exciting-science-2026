# Event-driven object analysis delivery

`LatestOnlyAnalysisWorker` optionally notifies after a completed result is stored,
**outside** its analysis condition lock. The object node uses a ROS guard
condition in its existing default mutually exclusive callback group. All ROS,
tracker, publication, original-header and final source-admission checks continue
on the executor. Analysis remains one active/one latest pending/one result and
starts at no more than the configured 5 Hz maximum.

The 50 ms timer remains for source faults, marker/tracking invalidation and
bounded notification-error reporting when no result exists. It does not routinely
poll completed results. A notification failure is latched, bounded to 512
characters and disables further wakeups; the cold fault monitor discards its
result and reports unavailable. It never publishes a timer fallback as fresh.

Close revokes future notifications and results, joins any in-flight notification,
then joins analysis under one caller-provided deadline. `False` means node/guard/
context resources must be retained for external supervision; Python cannot kill
stuck native code. The node checks this before destroying ROS resources.

CPU prerequisite fixture: real unchanged worker with synthetic 112 ms work,
211 ms deterministic ingress (within the 5 Hz maximum), isolated executor and
no callback competition. A post-unlock guard prototype reduced mean dispatch
latency by 24.53 ms (17.85% mean completion latency reduction). This establishes
polling headroom, not loaded station performance. The implementation still needs
independent review and root-owned live qualification. No camera, GPU, arm or
calibration APIs are involved.

No geometry functions, report fields, 3-acquisition/ghost admission, source GID
binding, 250 ms receipt lease, 500 ms original-header TTL or acquisition clocks
change. Earlier proposals to remove metrics or reuse geometry are not included.
