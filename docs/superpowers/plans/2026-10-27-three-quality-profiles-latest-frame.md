# Three quality profiles and bounded 30 Hz acquisition

User intent: realtime 480p, medium 720p and high quality 1080p; every profile must accept the 30 Hz camera base rate with frame skipping and no accumulating queue. This is acquisition/admission cadence, separately measured from fresh compute outputs. Repeated results never count as new detections.

Runtime source starts at 5344d665; reviewed evidence commits through 4612f38. Work only in feat/beginner-vulkan-20261022 isolated worktree. Preserve main, existing owner and calibrations.

## Public interface

- Canonical names: realtime (640x480 RGB/depth), medium (1280x720 RGB/depth), high_quality (1920x1080 RGB,1280x720 depth). All request native RGB8/Z16 at30Hz, with no resizing. Default realtime.
- Preserve vga/720p/1080p aliases; report requested and canonical profile names.
- All profiles use one latest-native policy. Skipping is adaptive to actual worker availability; do not impose arbitrary device-specific compute FPS caps.

## Ownership and data flow

1. Existing cold source/calibration/SDK/exact-geometry admission remains before normal threads.
2. A dedicated capture/admission thread polls the single existing SDK pipeline on a 2ms monotonic service loop. It owns normal camera polling; validates structural/calibration/source IDs and timestamps for EVERY received cohort before any discard. Advance observed watermarks even for duplicate/partially advanced rejected pairs. Preserve receipt, original IDs/timestamps/domain/generation unchanged.
3. One replaceable native-cohort mailbox holds SDK handles and preallocated metadata. One compute worker exclusively owns RawCaptureBridge, buffer acquisition, plane copies, vendor CPU alignment, CPU Core/Vulkan submit/collect and frame DDS publication. It takes the newest pending cohort only when idle. No additional materialized pending frame while native pending exists; offer->dispatch is immediate.
4. Keep three GPU slots. No image copy, alignment, GPU waits or DDS publication on capture; no heavy work or SDK handle destruction under mailbox locks. Capture cannot block on worker/publication: use bounded nonblocking intake handoff, count any contention discard explicitly. Preallocate reusable metadata and reserve serial capacity at startup.
5. Publication invalidation precedes failure/STOP. Give each admission a fresh internal session token, independently of calibration generation; carry it with every frame. Capture revokes only its exact token using atomic CAS, without gate locking or DDS calls. Publication checks the token under its serialization lock immediately before committing; an already-committed publication may finish, with UNKNOWN finalized last on the same measurement writer. Old-session callbacks cannot publish, revoke or finalize a replacement session. This defines local commit order and cannot retract queued network samples or promise ordering across topics. Stop capture and worker, join capture before SDK stop, release retained SDK cohorts and safely drain worker/in-flight fences before freeing resources. Main owns SDK start/stop exactly once; never overlap poll/stop. Preserve existing external timeout limitation for a hung GPU driver.
6. ROS executor remains available for status/measurement callbacks. Cross-thread fields and publication gate must have explicit synchronization; no data races or stale-generation publication.

## Observability

Normal counters start after cold admission, not bridge startup seed. Distinguish polled/unique received cohorts, duplicate rejection, pending replacements, contention/no-slot skips, selected/submitted/computed/published results and active/pending occupancy. Give precise counter invariants and snapshots. Source-ID gaps from SDK are distinct from intentional application skips. Report measured acquisition unique rate, compute/output unique rate, capture service gap/duration/deadline misses over33.333ms, source receipt age, skip policy, requested/canonical profile and failure reason. Existing safety fields remain unqualified/false. SDK documentation guarantees latest nonblocking polling; do not claim numeric SDK queue capacity without source proof.

## Implementation and verification

- Tests first: canonical names/default/aliases/native extents; SDK-free bounded mailbox/lifecycle/admission tests. Simulate300 inputs over10s at30Hz with80/180ms compute and blocked publication/ROS executor; every input validated, pending<=1/active<=1/slots3; superseded inputs never copied; dispatched frame newest available and provenance original. Cover duplicate watermark regression, malformed skipped input, no-slot late consumers, no-arrival completion, STOP between compute/publish, constructor failure and shutdown.
- Wire launch/node/profile mapping, capture/worker and status collector. Preserve Core, shaders, geometry thresholds, exact cold checks and historical evidence unchanged.
- Mandatory separate resource review before GPU: no hot heap/device allocations, no capture-side waits/copies/serialization, bounded retained SDK handles, static guards plus measured host microbench (mailbox/admission/worker handoff and synthetic80/180ms overload). Verify thread lifetime/locking and source hashes independently.
- Run focused CPU tests then canonical tools/test_cpu.py in pinned environment; source before/after pins. Native build on fwmini4 in isolated prefix, strict Vulkan and optimized CPU all three profiles with actual RViz~30fps. Measure actual unique acquisition near30Hz, outputs/age/skips/occupancy, capture deadline misses, CPU/RSS and allocations. Preserve failures, distinguish short performance checks from ten-minute full-stack/control/physical qualification. Never relabel old output as fresh to satisfy30Hz.
- Review, commit/push coherent checkpoint and verify remote SHA. Full workshop/main/deployment/physical goal remains active.
