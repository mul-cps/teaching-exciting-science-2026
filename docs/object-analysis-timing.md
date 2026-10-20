# Optional object analysis timing observations

`ros2 launch so101_station_objects objects.launch.py analysis_timing_diagnostics:=true`
creates a **read-only startup** flag, separate `~/analysis_timing` String publisher,
and bounded metadata recorder. Default false creates no publisher/timer/recorder
or per-job metadata envelope. Detection reports/status bytes and the public
worker `(job, report, error)` take API remain unchanged in both modes. No
thresholds, ROIs, transforms, source leases or processing rates are changed.
This launch argument is forwarded only by the objects leaf launch.

This is an observation, never source qualification, readiness, execution
permission or a source of replacement timestamps. No new image subscriptions,
pixel copies, graph queries or TF lookups are introduced. OpenCV version/thread
count is observed at startup after existing `setNumThreads(1)`; the worker is a
same-process Python thread, not IPC or a new interpreter. These startup values
are historical configuration observations, not live thread/latency attestation.

## Fixed packet and row schema

Schema: `so101-object-analysis-timing-v1`; maximum8192 UTF-8 bytes, at most2Hz
end-anchored/no catchup. Each packet drains up to4 scalar rows from a64-row ring.
Rows maximum1536 encoded bytes. Encoded reason text is ASCII20 (at most122 JSON
bytes even for controls). Counters saturate at signed64 with explicit overflow.
Malformed/oversize metadata increments invalid/loss indicators; it never enters
an admission/error path. The publisher serializes outside worker/acquisition
locks on the normal mutually exclusive ROS callback group.

Rows:

```
[id, outcome, site, reason, producer_generation, producer_sequence, consumer_epoch,
 headers4, source_receipts4, identity4, token_lineage_receipt, consumer_receipts4,
 clocks22, actual_analyze_rgbd_invocations]
```

Outcome0=rejection,1=published. Empty context/null clocks mean unavailable;
they are never invented. Headers and producer receipts use depth,color,
depth_info,color_info order. Identity contains original request/confirmed/expiry
monotonic and confirmed ROS ns. Consumer receipts use color,aligned,color_info,
depth_info order. Arrays contain scalars only; tokens/images/reports are not
retained. ROS ns and monotonic seconds are separate clock domains.

Clock slots0..13: process entry; existing board-TF request/return;
pre-submit check entry; worker submit/start; whole `_analyze_job` entry/return;
result slot-store entry; take; attachment-TF interval start/end;
pre-publication-check entry; after all existing publications.
Slots14..21 are entry/return pairs around the existing process-entry,
pre-submit, result-take, and pre-tracker checks. The final successful check has
**no injected work between it and the original publication sequence**. Its
combined interval ends after publication and must not be called graph-only.

Source-check sites1..5 are process_entry,pre_submit,result_take,pre_tracker,
pre_publish. Other sites:6 process header,7 process registration,8 pair,
9 processing access/copy error,10 result stamp,11 result registration,
12 analyzer error,13 post-TF header,14 post-TF registration,15 cold source fault,
17 result notification fault. Sites15/17 do not emit acquisition-associated
terminals; notification loss is counted separately. Latest five-site rejection summaries are historical
and carry no current health implication.

Original deadline components can be reconstructed from these immutable scalar
origins: four producer/consumer receipt limits250ms; exposure headers500ms;
identity request+1s and original expiry/confirmed-ROS check. The actual helper
also checks its current newest ledger; that row and internal checked-clock
margins are NOT exported here. Token scalar timing cannot identify which newest
ledger row failed. No diagnostic field refreshes any original deadline.

## Loss and performance interpretation

Terminals cover taken/completed or synchronous rejected acquisitions only.
Pending replacements, overwritten results, failed notifications and shutdown
loss have separate aggregate counters. Ring overflow drops oldest rows with an
explicit counter. Pending and invalid/overflow/loss counters constrain only the retained terminal
buffer. `locally_drained_without_known_loss` means that buffer is drained with no
known producer-local loss; it does NOT describe active jobs, complete acquisitions
or transport delivery. `coverage_scope` explicitly declares this partial subset. Shutdown counts are retained locally; final ROS delivery is not
guaranteed during teardown. Do not infer a complete latency distribution from
sampled/dropped rows. Repeated row completion is idempotent.

`whole_analyze_job` includes support-cache fallback; its actual analyzer-call
counter is1/2 (possibly0 in a synthetic mocked worker). Fallback's existing
`support_plane.plane_source` and age remain unchanged in original reports.
Worker queue wait includes the existing200ms start-rate scheduling. Result-take
includes real executor scheduling; no worker synchronization interval is IPC.

Dedicated tests cover caps, no payload retention, original report/algorithm AST
parity, default-off compatibility, worker API/lifecycle and native domain177
String/guard delivery. `tools/benchmark_object_timing.py` compares the frozen
original worker with disabled/enabled modes and measures actual monotonic clock-capture hooks plus scalar recorder/packet
cost. It deliberately excludes image analysis/TF/graph and makes no live
availability claim. A separate reviewer must ACK static/native/performance
before integration. Any live objects restart changes consumer epoch: source
reconciliation/deployment needs separate approval; this feature authorizes none.

Run the dedicated native tests from the repository root:

```bash
tools/test_perception_supervisor.sh -q \
  tests/test_object_timing_diagnostic.py \
  tests/test_object_timing_diagnostic_ros.py
```

This launcher fixes domain177/localhost before imports. All11 ROS cases must
pass without skips in the prepared native runtime. Ordinary CPU CI explicitly
skips these optional cases when `rclpy` cannot be found; an available ROS runtime
in any other domain still fails before importing/initializing ROS. There is no
automatic test partitioning: future overnight matrix protocols must explicitly
add `test_object_timing_diagnostic_ros.py` to both `native177_includes` and
`native75_excludes` (the station exclusion list derives from the former).
Historical sealed matrices are unchanged and do not establish this coverage.
