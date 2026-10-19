# Read-only derived source-binding health

Each explicitly configured derived-image adapter exposes Trigger
`~/derived_source_health`, distinct from `~/derived_source_status`. Direct camera
mode does not construct the adapter and gains no service or work.

The new schema is **`so101-derived-source-binding-health-v1`**. Successful RPCs mean
only current source binding. Every response explicitly reports
`image_admitted=false`, `processing_progress=false`, `qualified=false` and
`execution_enabled=false`. It can be ready with no received RGB/aligned images and
with a stalled analyzer. It is **not** a generic analyzer recovery permission,
selected-image admission, detection/grasp proof, or physical qualification.
Typed producer-fault dispatch must remain separate from analyzer progress.
The coordinator's two advancing selected-image proofs and the original
`derived_source_status` schema/implementation remain unchanged.

## Checks and provenance

The cold service uses only the newest current validated lineage entry. It requires
exact original native depth/color CameraInfo records from the existing four-record
metadata buffers, with their actual GIDs and **first unique receive times**. It
never reconstructs CameraInfo from reported calibration, uses a nearest cohort,
or falls back to an older complete lineage. Missing/evicted matching metadata
makes the call unavailable. All seven actual graph endpoints, immutable pins,
producer generation, consumer reset epoch and exact original-depth-time optical
TF are checked. Existing shared validators compare actual Info K/D/profile/frame
and actual TF against the pinned producer calibration.

The complete checks run before and after response JSON serialization; expired,
cleared, closed, reset or replaced latest evidence cannot publish a cached healthy
response. Missing TF and ordinary past expiry are unavailable. Existing persistent
invalidation is used for contradictory graph/calibration/pin information; recovery
requires explicit reviewed repin/reset. Such invalidation may clear observations,
as existing image checks already do. Successful queries do not take/pop an image
token, change emitted sequence, update `last_proof`, change receipts or retain
another image reference. No subscriptions, threads, timers, camera RPC, GPU or
pixel processing are added.

`camera_info_validation=independent_native_info` means independently **received
metadata** from native publishers, not independently measured physical calibration.
`identity_validation=producer_attested_device_info_no_new_rpc` identifies the
existing producer-attested identity lease. `optical_tf_validation=local_exact_depth_timestamp`
identifies the exact local TF query. Source-file hashes are local provenance,
not cryptographic security or independent pixel evidence.

An Info-only immutable K/D/profile/frame guard also runs before the existing
collector discards duplicate Info headers. Otherwise a same-GID/same-stamp
contradictory calibration could be silently discarded while an earlier good Info
remained. Valid duplicates still retain their first receive time and original
record; the collector's receipt/high-water/queue ownership is unchanged. This
checks K length nine and matching pinned D length (at most eight) before any
array materialization, and adds bounded small-array metadata work on the two Info streams only, no extra
image-path work. Actual contradictions now fail closed for image consumers too;
valid-input image-admission behavior and the image status implementation are
unchanged.

## Success fields and clocks

Common fields: strict boolean `binding_invalidated`, `state=ready`, `consumer_epoch`, `source_pin_sha256`,
`producer_generation`, `sequence`, `depth_stamp_ns`, `color_stamp_ns`,
`checked_ros_ns`, `checked_monotonic`, the four false flags and provenance labels.
`binding_invalidated` is true for the existing persistent ledger invalidation latch,
or when the captured core/consumer epoch was explicitly replaced during the call.
An old context cannot become healthy by polling; explicit reset establishes a new
epoch. Ordinary expiry, missing exact Info and temporarily missing TF do not set
this flag. Future/malformed data still fails closed without converting an expiry
into a permanent fault. Clients must reject missing/nonboolean flags, never infer
structural classes from reason text, and never implicitly reset.

Unavailable responses retain common source identity and add a bounded reason;
clients must require Trigger success and ready state before using deadlines.

- `receipt_deadlines_monotonic` has exactly seven original deadline components:
  `consumer_lineage`, `source_depth`, `source_color`, `source_depth_info`,
  `source_color_info`, `consumer_depth_info`, `consumer_color_info`.
  Each is the original unique receipt plus the existing 250 ms lease.
- `original_receipt_expiry_monotonic` is their minimum, **excluding identity**.
- `original_identity_expiry_monotonic` is the existing attested identity expiry.
- `effective_health_expiry_monotonic` is the minimum of receipt and identity expiry.
- `original_header_stamps_ns` contains the four original depth/color/image-Info
  integer ROS timestamps; `original_header_expiry_ros_ns` is their minimum plus
  the existing 500 ms acquisition lease.

Checked times are observations, never lease renewals. Repeated same-cohort checks
preserve all original deadlines. ROS and monotonic remain separate clocks. The
shared identity lease and all existing original-source freshness constraints are
validated without relaxation. The response describes an instant; a caller must
check original expiry again on receipt, and independently enforce any continuous
coverage or advancing-cohort requirement. No single success implies five seconds
of source health.

## Verification

Pure tests replay production methods without ROS initialization or GPU. The
native test requires **explicit domain177 and localhost1 before initialization**,
uses UUID topics/nodes, creates seven real graph endpoints, publishes only two
small native Infos and lineage (no RGB/depth/cloud payloads), calls the actual
Trigger, and replaces an Info publisher under the same node to prove invalidation.
Its identity/lineage is synthetic test provenance, not actual DeviceInfo or geometry.

Focused command (native wrapper must be run with the explicit isolated environment):

```
python -m pytest -q tests/test_source_binding_health.py tests/test_source_binding_health_ros.py tests/test_admission_proof.py tests/test_aligned_cohort.py tests/test_derived_aligned_hooks.py
```
