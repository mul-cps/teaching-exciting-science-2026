# Private availability repair implementation plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development or superpowers:executing-plans task by task.

**Goal:** Recover an authenticated resident private GPU availability latch without requiring expired pre-outage health, while preserving physical and freshness gates.

**Architecture:** First close a proved historical adoption transaction using a bounded durable ACK. Then add a separate, default-off private repair intent with fresh independent native preflight and the existing sole coordinator. ROOT owns deployment and any response to the existing natural fault.

**Tech Stack:** Python, ROS2 Jazzy/rclpy, std_srvs Trigger, existing journal/native/proof helpers. No new dependency.

**Spec:** ../specs/2026-10-20-private-availability-repair-design.md

## Global constraints

- Worktree `/home/bjoern/git/teaching-exciting-science-2026-private-availability-repair`, branch `feat/private-availability-repair-20261020`, base15bc703. Preserve all other checkouts/edits.
- No arm, bus, native-camera control, ROS76 test publishers, live recovery calls or controlled-outage rerun by workers. Actual DDS tests require domain177/local1 before ROS imports; CPU suites use existing isolated wrappers.
- Original250ms receipts/500ms headers, POSTENTRY2s preflight, independent identity bounds, stable5s health, three attempts/hour and30s cooldown, owned40s completion, and two actual IMAGE witnesses per consumer remain unchanged.
- Historical ACK does not write or rewind the journal, grant current health, enable policy or clear unknown jobs.
- All data-path changes receive independent static/performance review before native/live deployment. Preserve failures and source pins.

## Review focus

- A committed marker may have been written before the last freshness gate: require an actual successful receipt/runtime completion.
- Clock advancement and aged quota pruning must preserve the exact legitimate candidate descendant.
- Crash/IO/conflicting archive files remain idempotent or UNKNOWN without accounting rollback.
- Old resident faults require a fresh heartbeat/context and new POSTENTRY proof; never renew original fault clocks or forge history.
- Late discovery/FS/service access, disable, wrong generation/epoch/GID, quota and unknown jobs must block dispatch.

### Task A: Acknowledge proved historical adoption

**Files:** create `ros2/src/so101_station_perception/so101_station_perception/adoption_ack.py`; modify `supervisor_journal.py`, `late_adoption.py`, `perception_supervisor.py` in that package; add a real default-off supervisor include and forward its existing baseline/journal/artifact/serial plus new read-only inputs through `ros2/src/so101_station_perception/launch/{perception_supervisor,perception}.launch.py` and `ros2/src/so101_station_bringup/launch/station_live.launch.py`; dedicated `tests/test_supervisor_adoption_ack*.py`; docs.

**Interface:** `acknowledge_receipts(journal, acceptance_path, acceptance_sha256, completed_path, completed_sha256)` performs the explicit startup migration under the existing lock before `load_restart_marker`. `acknowledge_runtime(journal, result)` handles only the node's own successful late result before existing explicit enable. Both return bounded historical ACK metadata and preserve actual journal bytes. Controlled H-keyed archive/ACK files are validated on restart if an active marker remains; absent active marker does not resurrect old state.

- [ ] Write/run RED cases for success, failed-after-commit marker, exact aged prefix/clocks, all authority mutations, conflicting archive/CAS/readback, crashes before/after ACK/retirement, disabled startup, and later legitimate activity after retirement.
- [ ] Implement strict bounded receipt parsing/descendant validation and durable archive protocol from the spec. No newest-file discovery or arbitrary journal edit interface.
- [ ] Add the four exact typed read-only confirmation parameters from the spec; incomplete groups reject and paths resolve strictly below the journal directory. Existing explicit enable finalizes successful runtime state before policy activity.
- [ ] Run pure/native regressions and bounded FS/validation benchmarks; freeze source pins. Independent review required before Task B edits to shared modules.
- [ ] Commit Task A after review; ROOT integrates only after complete qualification.

### Task B: Separate private availability repair intent

**Files:** create focused pure helper `private_availability.py` only if it reduces the supervisor's state/validator complexity; modify package `perception_supervisor.py`/`supervisor_policy.py`, the same launch forwarding files, dedicated `tests/test_supervisor_private_availability*.py`, and docs. No quick-action command changes.

**Interface:** read-only boolean `allow_private_availability_repair=false`. A distinct internal intent/origin owns a fixed ENTRY+2s and exact fault/context. The strict persisted job variant `origin=private_availability_repair` supports dispatch_unknown/accepted with the same dedup/quota and owned deadline. It never calls `outage(history_verified=True)` or fabricates operator/late contexts.

- [ ] Write/run RED cases for an old resident typed fault and current heartbeat without health history; default-off/no enable; wrong types/contexts/future clocks; post-entry native/identity/two consumer metadata/terminal coordinator; disable/deadline/FS/readiness stalls; dedup across routes/quota/unknown restart; unchanged original route.
- [ ] Implement begin/preflight/reserve/dispatch through fixed clients at the existing200ms control cadence. One pending per client, no callback chaining, no extra payload/SDK opens. Recheck all original clocks/context after access/IO and immediately before one start RPC.
- [ ] Reuse existing owned40s/new-generation/artifact/two actual IMAGE/adoption completion; success clears old history and returns warming, all qualification/execution flags false. Failure latches without repeating the fault.
- [ ] Add one real synthetic DDS177 episode exercising native/binding proof, exactly one coordinator start and completion; no actual device or live76 publisher. Run the full existing supervisor suite and a CPU/performance/static guard pass.
- [ ] Freeze pins, obtain independent review, then commit.

### ROOT integration and natural-fault qualification

- [ ] Cherry reviewed commits to LIVE/STAGED, freeze/run complete sequential native75/station/177 matrix; retain every failure and pin all tracked source.
- [ ] Build a NEW immutable perception prefix; verify all installed modules/entry and exact-launch import paths. Prepare a pinned historical receipt bundle and exact owned supervisor/observer cutover only. Preserve journal/old marker/ACK evidence; no arm/producer/native-camera/consumer ROS restart.
- [ ] Start DISABLED with explicit migration inputs and opt-in repair. Validate preserved accounting/adoption/current context; explicit enable is ROOT's separate action.
- [ ] Preregister observation of the existing natural fault: no injected outage, no repeated controlled trial. Require one attributable attempt, original native preflight, old worker reaping/new generation, owned40s completion/two actual IMAGE and journal adoption. Report any failure without manual retry/reset.
- [ ] Observe subsequent SOURCE health/5s arming separately; do not treat frozen IMAGE completion as current later. Continue until06UTC and report physical gates still open.
