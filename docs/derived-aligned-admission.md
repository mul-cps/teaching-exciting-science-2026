# Optional exact derived aligned-image admission

The direct camera path remains the default. This diagnostic opt-in does not
qualify geometry, promote calibration, change TF, or change any physical owner,
recovery, safety or motion consumer. Source statements are local trusted ROS
provenance, not cryptographic signatures or independent pixel verification.

## Explicit source selection

Prepare a fresh source artifact with `registration_capture_sources`' existing
`--allow-derived-source` option after the producer is qualified separately. Its
actual native DeviceInfo request must bind the four native source endpoint GIDs
before and after RPC. Refresh old six-endpoint artifacts: the contract now also
requires the derived aligned endpoint and `implementation.producer_node_sha256`.
The six core module hash names remain fixed. A failed physical owner hold does
not authorize a registration fit, but it does not prevent read-only perception
from using independently pinned camera provenance.

For the marker and object **leaf** launch files, explicitly pass all three:

```
allow_derived_source:=true
derived_source_cohort:=/absolute/path/to/source.json
depth_topic:=/so101_perception/rocm/aligned_depth
```

Native RGB and native color CameraInfo topics stay unchanged. The station launch
forwards equivalent `marker_*` and `objects_*` parameters (depth selection uses
`marker_depth_topic` / `objects_depth_topic`). These default false/empty/direct.
Recovery depth and CameraInfo inputs remain direct and are not forwarded through
these switches. Marker opt-in requires the shared marker pipeline, not standalone
ChArUco or tags. Do not run two canonical marker pipelines: each broadcasts the
same board/camera/base frame edges. Root/operator must perform a sequential A/B
leaf cutover, retaining the same calibration seed and checking graph uniqueness.

## Admission and bounds

Both nodes require unique actual graph endpoints for four native images/Info
and derived cloud, lineage and aligned depth. Received MessageInfo GIDs bind
native RGB, both native Info streams, derived aligned depth and lineage. The
cloud graph endpoint is also checked. Node names alone never prove identity.
Full immutable implementation/calibration/source pins and both generations are
compared, including the producer node code hash. Any generation, graph, profile
or calibration replacement latches unavailable. `~/reset_derived_source` reloads
the explicitly named artifact and clears observations; it never learns new pins.
Old in-flight tokens cannot survive a reset, even if the artifact is unchanged.

Original RGB/depth/Info timestamps use exact integer nanoseconds. Approximate
pairing cannot substitute another exposure. Original age <=500 ms, native source
receipt lease <=250 ms, DeviceInfo request lease <=1 s, and pair delta <=40 ms
are unchanged. Final graph, native CameraInfo and exact stamped optical TF are
rechecked before board pose/tag/object commits. Failure never renews board
`last_valid_stamp`/`pose_estimate` or object acquisition. The old marker TF can
remain a documented last-known display transform, not a fresh observation.

There are four raw message references per stream and eight bounded (64 KiB)
lineage statements. No adapter image decoding/copies or extra image subscriptions
are added: the existing three image/Info subscriptions are replaced in opt-in
mode; only native depth Info and lineage metadata are added. At 1920x1080 RGB8
and aligned Z16, four pairs retain at most **41,472,000 bytes** of image backing
storage; including the existing two active/pending pairs gives **62,208,000
bytes**, plus at most 512 KiB lineage and small metadata. These are references,
not zero-copy DDS claims. ROS serialization and existing CV/worker copies are
additional. A stale reference may remain until replacement/reset/close, but is
never admitted after its original lease expires; reference counts stay bounded.

Marker scheduling retains one active/one latest pending at <=5 Hz. Objects use a
10 ms metadata drain to select the newest complete exact cohort at configured
<=5 Hz **before** decoding/copying pixels; the existing analysis worker remains
one active/one pending. No extra application queue is added. The original
250 ms source lease remains enforced after expensive analysis. If analysis cannot
finish within it, output is unavailable; thresholds are not relaxed. Three
independent object acquisitions, full-workspace observation defaults, pickup ROI,
far-depth and ghost rejection remain unchanged.

## Verification scope

Pure tests cover exact matching, future input without high-water poisoning,
bounded references, first receipt retention, no old-statement fallback, token
expiry/reset and source replacement latching. Native API fakes exercise optional
adapters and latest selection before decode. An isolated domain177 UUID-topic
DDS fixture binds seven actual graph GIDs and tests exact matching, missing and
future cohorts, expiry and same-node aligned publisher replacement. Its tiny
synthetic pixels/fake optical TF are not geometry qualification. The archived
native CPU producer fixture is emitted by actual DDS/DeviceInfo fakes, not a
physical camera or GPU proof.

Live GPU A/B, sustained tracking/confirmation and physical scene correctness
remain deployment gates. The metadata-only CPU baseline is
`overnight/pipeline-5min-cpu-baseline.json`: 300 s, 8755 RGB/8661 depth unique
native Info, 2203 aligned Info; largest RGB gap 1.026 s and aligned gap 1.427 s.
Marker callbacks 920, median board 35.60 ms/completion 337.53 ms/queue 16.06 ms.
Objects: 463 tracking, 333 registration unavailable, 1 stale; at most three
consecutive tracking reports, nested `acquisition.confirmed_count` max **0**,
objects max0, pending max13. This was concurrent software work, not an isolated
performance guarantee. Compare actual unique exposures, ages/gaps, board tracking
versus held, and independent confirmed tracks; heartbeat rates are insufficient.
