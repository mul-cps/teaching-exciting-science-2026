# Exact RGB-D object diagnostics

These optional tools neither change detection thresholds nor classify targets.
Capture is ROS read-only, bounded to10 seconds, with four native/derived image
references per stream in the existing admission collector plus one candidate.
No cloud, joint/owner, actuator or camera SDK subscription/API is used. Four
aligned CameraInfo metadata references are additional. No background executor
threads are created by the tool.

From this pinned checkout, using the station ROS wrapper:

```
python tools/capture_object_diagnostic.py --source /absolute/current-camera-source.json --output /absolute/NEW-capture --timeout 10
python tools/replay_object_diagnostic.py /absolute/NEW-capture --output /absolute/NEW-replay
```

Capture requires the existing opt-in derived pipeline and object parameter
service. It verifies exact original RGB/depth and native Info stamps through
the reviewed source-pinned collector, current seven publisher GIDs, all original
250ms receipt/500ms exposure/40ms pairing bounds, exact optical TF and exact
camera_from_board at RGB time. Aligned Info additionally needs matching original
depth stamp/color calibration, a unique actual received GID and the same producer
node. Parameters are read through GetParameters; observed parameter changes
revoke capture. Files are exclusive; completion JSON is written last, NPZ arrays
read back exactly and both file hashes reported. Acquisition checks precede
filesystem persistence; saved timestamps are never refreshed into live evidence.
A missing cohort or parameter service fails with a finite timeout. Native
imports/filesystem calls cannot be forcibly preempted by this loop.

Replay requires the exact recorded object-source hashes, uses saved parameter
values, and emits complete validation/silhouette/visibility/rejection events,
local support masks, original color masks and every raw parent bounding box.
The first-five production rejection sample cap does not truncate replay events;
a10,000-event limit fails explicitly. No classifier/threshold is changed. The
instrumented report must exactly equal a second uninstrumented analysis report.

Replay describes single-acquisition geometry. Live track IDs, temporal three-frame
confirmation, registration readiness, attachment masking and the live cached
support-plane fallback are not reproduced. Failed fresh support-plane fitting
is reported rather than silently introducing a different support plane.
Everything is diagnostic/unqualified; no output is an execution approval.
