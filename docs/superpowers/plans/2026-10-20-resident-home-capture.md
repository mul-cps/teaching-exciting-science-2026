# Resident measured-home capture implementation plan

**Goal:** Capture a measured-home artifact through the existing resident owner and ROS, without opening another bus or changing motor registers.

**Architecture:** Add a finite `capture_home` owner command and `/follower/capture_home` Trigger service. Capture uses only the configured home path and keeps both home caches untouched. `/station/arm/capture_home` and `station-action.sh arm-capture-home` forward explicitly. Existing `/follower/home` remains the only cache reload/motion operation.

## Constraints and failure semantics

- No live calls, SDK connections, motion, STOP or deployment during development. Root owns integration/deployment.
- Require initialized non-observing idle owner, no failed hold/teleop/component/profile restoration/recovery, all six torque bits off, normal admitted raw/body bounds and unchanged arm/calibration/source/session identity.
- Use bounded three exact-equal position/torque snapshots: two before staging and one after file staging immediately before atomic replace. Each register acquisition <=100 ms; whole operation <=1 s before commit. STOP cancellation is pumped between reads and before commit; capture itself never writes registers.
- Stage candidate and exact-byte prior backup with fsync before the final acquisition/guard. Read/write/guard failure before atomic replace leaves the configured home unchanged. Atomic replace is the commit point; filesystem/native calls cannot be forcibly preempted, and uncertain service delivery is never automatically retried.
- Artifact records measured schema/units/raw/body identity, original acquisition clocks, source/session, previous byte hash and backup path, all unqualified flags. No qualification or cache promotion.
- Correlated command ID and same-generation reservation on ROS boundary; unknown/late/malformed completion latches motion unavailable until explicit verified STOP. Capture failures do not themselves trigger motor writes.

## Tasks

- [x] Add focused failing owner/artifact tests (all-off after STOP, no register/connect writes, stale/drift/identity/bounds/recovery/failed hold rejection, atomic failure preserves old bytes, caches unchanged).
- [x] Implement startup/explicit-request-only exact-byte staging helper in existing hashed `arm.py`; implement owner capture with the existing bus and cancellation pump. No tick/data-path changes.
- [x] Add service registration/reservation and strict correlated completion validation; cover mocked/native boundary and STOP/late response behavior.
- [x] Add station forwarding, quick action, documentation and forwarding tests; retain existing physical gates.
- [x] Run focused fake/native tests, cold-path benchmark/static guards; freeze source and request independent author-separated review from workflow agent (pending ACK). Root commits and deploys separately.
