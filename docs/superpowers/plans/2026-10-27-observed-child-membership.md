# Observed child admission and exact closed membership

This implements the next missing dense stage in the approved
[native geometry plan](2026-10-23-native-geometry-integration.md), sections 4–6.
The full workshop goal, complete report parity, fwmini4 deployment, physical
cycles and main release remain required. Camera profile evidence at `707351f`
does not establish those requirements.

The retained contract is `analyze_rgbd` in
`ros2/src/so101_station_objects/so101_station_objects/objects.py`, especially
original child validation followed by observation-ROI admission and tight-child
3×3 closing. Existing parent/child preparation and calibrated Geometry remain
separate prerequisites. The current resident ObjectsComponent only reports Core
component counts; this stage is a prerequisite for its future complete geometry
consumer, not a promotion of that component into station observations.

## Contract

- Consume completed ChildLease and GeometryLease for the same exact FramePool
  control block, source slot/cohort, publication session and calibration, on the
  same shared Vulkan Context. Retain both inputs through the consumer's own fence.
- Add an opaque weak Geometry producer token, a validated source-frame accessor
  and a small immutable resident view. The producer control-block identity binds
  its cold rays, board transform and depth scale as a configuration/transform
  epoch. A different producer is rejected even with equal generation integers or
  calibration values. No per-frame calibration array copy or hashing.
- The new observation ROI is distinct from Geometry's support ROI. Count original
  child board-coordinate points with exact inclusive Float64 comparisons, with
  no epsilon. No ROI means all original child points. Eight inside points admit
  observation; seven do not. Preserve the existing native-Float64 and explicit
  software-binary64 precision paths and their admission rules.
- ROI admission is a flag, not clipping or a final object decision. Preserve all
  children in retained color/parent/child order, including ineligible children,
  and their original evidence. Downstream validation must preserve the retained
  rejection order before using the ROI flag. Do not emit accepted reports here.
- Close only observation-eligible children. Use each original child's tight bbox,
  with parent/global offsets correctly applied and exact retained OpenCV 3×3
  dilation/erosion border semantics. Original and closed membership remain
  separate. Keep overlapping closed memberships separate; morphology cannot
  manufacture observed silhouette support. Ineligible closed membership is empty
  and explicitly not computed; its original evidence remains available.
- Use bounded sparse storage: an original ordered-record ID per source pixel,
  immutable root-to-record mapping, and at most nine closed record IDs per pixel.
  Never allocate children×image storage. Keep 4096 record, 4N aggregate membership
  and three-slot bounds explicit. The 4N cap is an admission capacity, not a
  consequence of nine IDs/pixel; exceeding it faults the entire result.
- Ordinary readback is compact counts/status only. No dense host fallback,
  per-frame allocation, device/queue waits or host per-child orchestration. Use
  reusable buffers and fixed/indirect GPU dispatch. Diagnostic dense readback is
  separate and visibly labelled.
- Initial admission follows ChildGraph's existing 640×480/307200-pixel bound.
  Layout/API remain dimension-aware for the required future 720p/1080p full path;
  this initial bound must not redefine the final three-profile goal or resize
  inputs. The standalone ChildConfig minimum remains unchanged; the full analyzer
  constructs the retained max(24, min_parent_pixels), normally 30.

## Implementation and evidence sequence

1. Failing independent CPU/oracle/API tests first. Freeze original memberships,
   ordered metadata and closed membership expectations from retained behavior,
   not the new backend. Cover ROI 7/8, all inclusive boundaries/nextafter cases,
   partial ROI without clipping, tight crop borders/holes/parity, overlapping
   children, empty inputs, all-record order, source/session/config/context mismatch,
   slot retention/reuse and explicit capacity/convergence faults.
2. Add the typed Geometry accessor/token and lease-backed Observed preparation,
   Slang kernels, CMake compilation, native CPU reference and combined fixture
   harness. Geometry support flags and Core Float32 XYZ cannot substitute for the
   new observation predicate. Preserve all existing numerical thresholds.
3. Independent resource worker reviews actual hot paths, allocation/lifetime,
   barriers, readbacks and dispatch. Add static/mutation guards and a measured
   host microbenchmark before any new GPU execution. Report actual slot/payload
   budget including inherited leases; do not claim whole-process zero allocation.
4. Run focused and canonical CPU/native checks with source pins. Then qualify
   actual compiled kernels on Intel and AMD against independently frozen cases,
   including software-binary64 Intel and native-Float64 refusal where unsupported.
   Preserve failures and inspect cause instead of weakening references. Measure
   compact ordinary and diagnostic paths separately.
5. Publish narrow source-bound evidence and integrate the coherent checkpoint.
   Full contours, support-plane fitting, validation/PCA/pickup reports, tracking,
   ROS scene consumers, complete-profile performance, ten-minute soak, physical
   STOP/recovery/cycles and qualified main deployment remain subsequent work.

The curriculum audit found current generated materials and 116 source/artifact
hashes consistent, with 26 focused checks passing. Learner observation, elapsed
classroom pacing, projector/printer inspection and physical rehearsal remain
unverified. Passive hardware audit confirms camera at fwmini4 and EV3/two arm USB
adapters local; an asynchronous connection request is pending while software work
continues. No active owner/service or private calibration is changed by this step.
