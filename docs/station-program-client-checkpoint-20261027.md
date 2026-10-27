# Shared station program client checkpoint — 2026-10-27

A real **no-ROS Python → Conda wrapper → ROS client** test reproduced a cancellation defect: the outer caller raised `InterruptedError`, while its newly admitted simulation job finished the wait and executed the forbidden tail. Signalling the wrapper Bash PID had left the ROS descendant running. The first failing receipt remains retained; neither fake-transport tests nor direct in-process ROS cancellation exposed this path.

The fix starts an owned wrapper session/process group and signals that group. The ROS child emits one bounded, tagged terminal receipt only after the shared client's STOP acknowledgment passes its existing verification. The outer client requires that unique receipt, exact Boolean mode/fields, and exit 1; missing, malformed, duplicate or ambiguous responses remain **STOP unverified**. Repeated interrupts are deferred during bounded cleanup. The leader stays unreaped through the final group signal, preventing cleanup against a reused PGID. Grace covers the current 5s RPC + 8s STOP + 2s close, with margin, then bounded TERM/KILL/reap. `/station/program/command` and its generated `StationCommand` ROS schema are unchanged.

**Source commit `a16aa86c48f3af72b5459353a48d110ded3537c7`:** all 11 frozen source/generated hashes match its immutable Git blobs. Historical run receipts predate this commit and bind the same bytes. The [machine readout](station-program-client-checkpoint-20261027.json) retains the hashes and canonical CPU receipt.

| Check | Result | Boundary |
|---|---|---|
| Direct native ROS scripts | Both passed | Real generated interfaces; synthetic backend, domain 177 |
| Actual wrapper fixture | Four modes passed in 6.085s | Real `-I -S` no-ROS outer, Conda, generated ROS client |
| Canonical CPU | 4527 passed, 133 skipped, 48 subtests; 248.35s | Skips remain pending |
| Focused CPU | 84 passed, 21.12s | Subprocess/receipt and existing client/API/control tests |
| Independent review | 35 tests, 43 process trials, 4 static mutants passed | Counts overlap; do not add them to 84 |
| Learner materials | 116 hashes verified | Only client ZIP member and its two receipt entries changed |
| Fresh kernels | Three notebooks, 25 canonical code cells passed | No ROS; explicit unavailable-wrapper and routing variants also passed |

The wrapper fixture checks normal trace parity, executable Python export, cancellation synchronized on a **new job ID and wait marker**, and physical-mode mismatch before mutation. Cancellation preserves the existing `failed/program cancelled` terminal state, verifies a new simulation STOP, excludes the tail, and leaves no active job. Outer process groups are empty and actual ROS child PIDs disappear. Separate independent tests cover post-spawn setup failure, a surviving closed-pipe descendant, and final signal-before-reap ownership.

Actual Conda/ROS run/export wall costs were 1.233/1.274s; reaped-child CPU was 0.810/.844s. Cancellation trigger-to-outer-reap was 0.376s. These include process startup, not station processing. Independent synthetic child-process measurements were separate: 30 normal trials had 18.74ms median wall, 10 verified-cancel trials 105.94ms. They are uncontrolled host costs, not zero-allocation, FPS, realtime or hardware performance claims. The selector and output chunks allocate per invocation; the change adds no per-frame processing.

The [history](performance/evidence/station-program-client-20261027/history.json) preserves the native RED, independent setup/ambiguous-exit failures, and an intermediate native-green candidate whose general STOP contract was incomplete. The final source was independently approved. Generated workflow interfaces were built only in an isolated temporary prefix. The learner ZIP changed only `workshop/blockly_client.py`; no private configuration or hardware adapter was added. All 116 material hashes verify, and unchanged notebook/layout sources were checked without rerendering.

This is **simulation/native-interface evidence**, not physical STOP or recovery verification, three physical cycles, station acceptance, deployment or a complete release. Those gates remain pending; [overall qualification stays false](station-program-client-checkpoint-20261027.json).

Selected small receipts and source hashes are in the [inventory](performance/evidence/station-program-client-20261027/readout-inventory.json). A private 158,371-byte raw archive retains reproduction scripts, source snapshots and logs; its hash and member inventory are recorded separately. No interface ELF/build tree, private configuration, credentials or notebook images are checked in. The retained [native command receipt](performance/evidence/station-program-client-20261027/wrapper-final.json) and [notebook receipt](performance/evidence/station-program-client-20261027/notebook-proof.json) give exact commands and runtime identities.
