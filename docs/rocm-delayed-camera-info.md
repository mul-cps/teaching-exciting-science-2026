# Delayed CameraInfo admission

The diagnostic ROCm RGB-D producer can receive an image before its matching
CameraInfo. `_info` checks the nearest retained Info's immutable frame, dimensions,
K/D, distortion model and actual publisher GID first. A temporal difference greater
than the existing 40 ms pairing limit then returns unavailable (`None`), allowing
`_admit` to wait/drop that cohort instead of permanently invalidating the producer.
No timestamp or receipt is renewed. The four-entry buffers and all existing
original-exposure (500 ms), source-receipt (250 ms), identity (1 s), future and
publication checks remain unchanged. A true immutable mismatch still raises and
latches through the existing caller. No executor, worker, reset or recovery behavior
changes.

## Evidence and scope

The root-executed diagnostic first-fault capture at
`2026-10-19T20:07:25.900970Z` recorded a color image exactly 100,065,918 ns newer
than the nearest colorInfo. All four receipts were younger than 100 ms. The old
combined temporal/calibration condition latched `source CameraInfo
calibration/profile/cohort changed`. This temporal mismatch alone explains the
branch; no actual K/D mutation was recorded. Maximum instrumented tick duration
was 8.53 ms, so this incident does not establish the separately suspected 250 ms
callback-blocking cause.

The captured fault is under station evidence
`overnight/live-gpu-shadow/producer-probe-first-fault.json`, SHA256
`4c028f27d8cf97f950466ad2bb03d9ae4d8fdd024fef0a3c1557997dbcbd0231`.
`tests/test_ros_rocm_rgbd.py` replays that integer-nanosecond metadata through the
actual admission methods using small native ROS message objects and a CPU fake
worker. It checks waiting, eventual admission with distinct Info/image stamps,
unchanged original receipts, the exact 40 ms boundary, immutable mutations and
future/stale Info. The existing synthetic DDS test still verifies source identity,
original-stamp publication and publisher replacement. This is software admission
coverage, not live GPU, physical geometry or sorting qualification.
