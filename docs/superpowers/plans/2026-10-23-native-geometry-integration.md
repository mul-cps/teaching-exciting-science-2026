# Native object geometry: remaining implementation and acceptance

This continues Task 3 of the [approved workshop/station plan](2026-10-22-beginner-vulkan-workshop.md).
It does not replace the full workshop, fwmini4, physical-cycle or release requirements.
Checkpoint `be3219a` supplies raw alignment and compact components; it does not supply
validated complete object geometry. The following are implementation choices and
acceptance dependencies, rather than additional requests for the instructor.

## Authoritative behavior

Use `analyze_rgbd` in
`ros2/src/so101_station_objects/so101_station_objects/objects.py`, its
`colored_object_validation.py` helpers, and
`workshop/digital_twin/grasp_geometry.py` as the retained implementation.
Use `tools/native_geometry_oracle.py` for independent reference evidence.

Preserve complete report schema, types, color/component/contour order, first-five
rejection diagnostics, reasons and qualification flags. Full report tolerance is
absolute `1e-9`; calibrated rays and preparation coordinates use `1e-12`.
Memberships, flags, sample indices/order, booleans and integers are exact.
Do not replace median-centered PCA/SVD or linear percentiles with box moments.

The required evidence has distinct meanings:

1. Original color parents include invalid/out-of-range depth pixels. Parent area
   defaults to 30 pixels, and its OpenCV CCL_GRANA label order matters.
2. Depth children remove both sides of a four-neighbor jump over 12 mm, then use
   eight-connected membership and the retained minimum-size configuration.
3. Validation support includes the original parent within the child's 5x5 halo
   and externally filled contour. Invalid holes remain evidence of missing depth.
4. Final observed child pixels remain distinct from per-piece 3x3 closing and
   contour fill. Morphology cannot manufacture silhouette support.

Preserve depth support fraction 0.90, minimum 24 validation points, depth-gap,
minority and neighbor-coherence gates, final silhouette fill 0.45, sensor-edge
truncation, full-child ROI semantics, and all current footprint/height/opening
limits. Plane residual, saturation and rectangularity remain diagnostics.

Support sampling uses the global row-major stride-8 lattice, radius-3 known-hue
exclusion, inclusive support ROI and seed-17 selection capped at 2,000 points.
Retain the 120 seed-17 RANSAC proposals, first-winning ties, measured-plane
admission/snap rules, and eight Huber refinements. Cached-plane recovery is
limited to its existing typed failure and age rules.

## 1. Freeze the complete CPU contract before porting it

- Freeze deterministic inputs, full reports, ordered intermediate evidence and
  lossless array identities for holes, depth-separated touching regions,
  nonplanar surfaces, sensor edges, concavity, partial observation/pickup ROIs,
  4/5/8-coefficient distortion and more than 2,000 support samples.
- `tools/native_geometry_matrix.py` and `matrix-v1.json` establish the first
  twelve cases. Dense arrays remain in a verified external snapshot; reference
  reports and array hashes are committed. A native result must never regenerate
  its own expected values.
- Separate exact qualification-artifact identity from cross-runtime payload
  regression. Record actual runtime versions. Runtime differences may not excuse
  changed sources, inputs, array hashes, report values, order or types.
- Add a parent-label-order counterexample before implementing the resident
  parent handoff. OpenCV CCL_GRANA is not minimum-pixel order: in a 4x6 mask with
  foreground at `(y=1,x=0)` and `(y=0,x=2)`, labels 1/2 have minimum flat pixels
  6/2. Preserve parent and local child ordering through the full report.

## 2. Make compute features independent of optional video

- Query supported core features once when constructing a Context.
- Request supported `shaderFloat64` in the legacy compute-only device and
  optional-video-create fallback. Keep the existing video feature chain intact.
- Report actually enabled features, and never request unsupported features.
  Features2 and legacy feature pointers must remain mutually exclusive.
- Exercise real create-info host tests, retained-owner allocation/query guards,
  default AMD preparation and combined raw/compact cases, and Intel's explicit
  unsupported-FP64 refusal.
- Exercise the compute-only branch against real AMD hardware using a reviewed,
  process-local capability fault injection. Label injected evidence honestly.
  A generic Mesa version override was ignored by the installed RADV 26.2.3;
  retain that failed test setup and its provenance limitation.

This repairs a feature-negotiation bug. It does not give Intel hardware FP64.

## 3. Resolve Intel precision without weakening the contract

Keep the existing default NativeFloat64 refusal; the software alternative remains
an explicit opt-in after its bounded gates, not automatic camera admission.
Do not use float32 Core XYZ as an unqualified substitute for retained float64 rays.

The selected candidate is a bounded, coordinate-only **Berkeley SoftFloat Release
3e FAST_INT64 subset**, shared between C++ and Slang with uint64 carriers and
uint32 arithmetic limbs. It provides fixed nearest-even add/multiply, exact
binary32 widening, ordered comparisons and finiteness, with gradual underflow and
canonical quiet NaNs. Division, square root, fused arithmetic, configurable rounding
and exception-state equivalence remain outside the subset. Retain the full BSD
notice and source/modification inventory in
`ros2/src/so101_station_vulkan/third_party/softfloat/NOTICE` and `provenance.json`.

[Mesa's SoftFloat-derived GLSL](https://gitlab.freedesktop.org/mesa/mesa/-/blob/mesa-26.2.3/src/compiler/glsl/float64.glsl)
was evaluated as a reference only; its carrier/constant adaptation and non-exact
reciprocal path did not make it the selected implementation. Do not attribute this
subset to Mesa or treat a reciprocal as qualified exact division.

The earlier instance-only audit found physical `shaderInt64` support on both
station GPUs; missing FP64 must not be confused with missing Int64. Context now
requests only supported Int64 alongside supported Float64 in its existing compute
baseline. Actual create-info/enabled-feature host tests cover the independent
feature profiles and optional-video fallback. Physical support and synthetic tests
still do not qualify GPU arithmetic.

`GeometryPrecision::SoftwareBinary64` is an explicit preparation-only opt-in and
requires actually enabled Int64. The existing three-argument constructor retains
`NativeFloat64`, including its explicit unsupported-FP64 refusal. The separate
software shader build option defaults OFF; there is no precision auto-selection
or default-camera wiring. Source ordering, thresholds, arena ABI, resident
ownership and exact discrete comparators remain unchanged. The
[CPU evidence checkpoint](../../performance/software-binary64-20261023.md) records
conformance, independent CPU refinement, compiler/cache-provenance correction and
remaining limits. Reproducible offline corpus/shader/build instructions are in the
[diagnostic README](../../../tools/performance/README.md#software-binary64-primitive-diagnostic).
Root-reviewed GPU evidence now passes primitive bit conformance and all eight
unchanged preparation cases on each tested AMD/Intel device. Both actually enable
Int64; Intel retains physical/enabled Float64 false. Maximum coordinate error is
2.7755575615628914e-17 m under the unchanged 1e-12 limit, with exact flags, counts,
membership/order and ownership contracts. The archive preserves failed protocol
attempts separately from the accepted AMD prefix plus fresh seventeen-step remainder.
This closes only the recorded primitive/preparation diagnostic boundary.

For each changed arithmetic/preparation revision, require independent CPU bit tests,
static SPIR-V capability checks and a separate performance refinement/benchmark
review before GPU execution. Then compare the
same complete preparation outputs on AMD and Intel at unchanged tolerances and
exact membership/order. Measure transfer-inclusive processing and CPU demand;
do not infer frame rate from primitive instruction counts. A prototype that fails
correctness or the station budgets does not become the default backend.

## 4. Specify and implement resident evidence, not diagnostic downloads

The first original-color parent boundary is implemented and checked in the
[resident-parent checkpoint](../../performance/resident-parents-20261023.md).
`ParentPreparation` borrows the completed producer's original known-color mask,
including colored pixels with invalid depth. Exact eight-connected parents use
true minimum 2×2 block keys and color-segmented GRANA IDs for **all** parents
before the area filter. The full label map remains resident; bounded ordered
compact records preserve those IDs. Positive thresholds above image area remain
valid and emit no compact records. No RGB reclassification or depth-cut child
roots stand in for original parents.

`ParentLease` retains the actual source and its metadata through its consumers;
its weak pool token reserves no source slot. Three source-index arenas, immutable
root-walk snapshots, explicit compact overflow/nonconvergence faults, and guarded
map/record accessors preserve the bounded ownership contract. A 4096-record cap
and 32 hook/flatten rounds are budgets, not universal scene/convergence proofs.
The ordinary path reads back only compact results. The cold transfer-source
capability enables a separate explicit diagnostic; it does not add an ordinary
dense host download.

Thirty-four recorded AMD/Intel processes pass exact full-parent map/count/record
comparison or the required unusable gates: thirty normal results and four
invalid-mask/overflow results. CPU/native, independent resource reviews, full
build/CTest/install and source/runtime/SPV seals accompany that evidence. This
closes only the recorded original-parent preparation boundary, not performance,
child/support/report parity, a physical source or the full station release.

Next retain and verify depth children, local support, observed membership and
ordinary resident closed membership. Freeze the retained splitter's actual
**default** CCL labels/order on the exact tight parent crop, including odd sizes
and both crop-origin parities. Parent global GRANA/block ordering cannot be used
as a substitute for crop-local child labels. Preserve the finite inclusive depth
range, strict depth-jump predicate, removal of both bad-edge endpoints, child
area gate and post-child clipped support halo. Separate source/algorithm evidence
from complete crop/halo/membership fixtures and actual runtime qualification.

Current Core normal morphology stores closed counts; dense closed memberships are
still diagnostic-only. Do not enable dense diagnostic downloads to make the
geometry consumer work. Keep calibration/configuration generations and
computational crop origins explicit. Preserve three-slot ownership, latest
pending replacement, bounded capacities, explicit faults and zero frame-time
allocation/device waits. Future consumers must retain their lease through their
own fence, not merely through producer readiness.

Verify every intermediate against the frozen child/support/contour matrix before
implementing accepted object reports. Pool identity is a storage-owner guard;
node-level source health/order, epochs, freshness and timestamped transform/world
admission remain separate. Unsupported configurations must fail visibly instead
of silently using different depth/area thresholds.

### Child preparation implementation

The bounded CPU reference gate is **CLOSED**: the committed
[24-case fixture](../../../tests/fixtures/native_geometry/child-evidence-v1.json),
[validator](../../../tools/native_geometry_child_evidence.py) and
[validator tests](../../../tests/test_native_geometry_child_evidence.py) freeze
crop-local ordering, regions and support against the
[retained splitter](../../../ros2/src/so101_station_objects/so101_station_objects/colored_object_validation.py).
The parent GPU boundary is already recorded above. The following child design is
selected for implementation; it is not evidence of completed child GPU work.

`ChildPreparation` consumes a completed `ParentLease`; `ChildLease` retains it
through consumer fences. Two additive parent accessors return source and
configuration copies. Candidate F32 depth admission, both-endpoint edge cuts and
eight-connected components remain configurable and enforce the same parent
identity. Validate configuration as host doubles before deriving F32 comparison
bits, including derived zero/infinity cases. Keep opaque minimum-pixel roots
separate from the true crop-local block key K. Sort at most 4096 kept children on
the GPU by parent order and K; do not promise exact numeric OpenCV CCL IDs.

Retain compact GPU `ParentChildStatus` for every kept parent: four U32 fields
`{valid_depth_pixels, candidate_pixels, all_children, kept_children}` (16 bytes
per parent), computed in existing passes and retained through aliases. Even when
no child record survives, these counters must reproduce
`outside_observation_depth_range` versus `no_depth_coherent_component` without
dense CPU readback.

Use packed clipped halo storage with default capacity 4N and explicit record,
job and pack overflow faults. Retain region, nearby, cut, original-parent and
support bits. Fill a region through the complement of four-connected exterior
background, then form support from the original parent intersected with the union
of the 5×5 dilated region and filled region. The finite CPU hole-fill pilot passed
109,051 fill comparisons per runtime and 21 accepted support regions across 14
accepted fixture cases; this is not
a universal equivalence proof. Both connectivity graphs use 32 bounded
hook/jump rounds followed by full convergence validation, with no N-step walks or
host convergence loop. Indirect work and explicit barriers avoid a CPU count
round trip.

The earlier reference resource calculation is 24,079,696 payload bytes per slot
and 72,239,088 bytes for three slots. It excludes the newly required parent-status
storage and host-cached decoded records, as well as borrowed parent/Core and
driver storage; it is not an actual or all-inclusive budget. Recompute and report
the actual budget for all named payloads after implementation.
Production admission remains bounded to 640×480. A literal shader adapter may
exercise N ≤ 307200 and axes ≤ 2048 for the 1025×7 fixture: source admission and
kernel parity are separate gates. It must use the same actual command graph and
SPVs, not a copied test loop. There is no production admission bypass or dense
CPU fallback.

Ordinary observed-ROI memberships and 3×3 closed memberships remain a **later,
distinct GPU stage**. Closing raw children before the observation ROI would
change the retained semantics. Real child GPU parity, universal fill equivalence,
range coverage and performance targets remain **UNPROVEN**. Every new data path
requires a separate performance worker, static guards and a microbenchmark before
GPU execution, in the qualification sequence of this
[formal integration plan](2026-10-23-native-geometry-integration.md#6-qualification-and-return-to-the-complete-workshop-release).

## 5. Implement the complete estimator and publication contract

Implement support fitting, validation/rejection order, ordered contours/splitting,
observed-silhouette checks, robust visible geometry and supported pickup estimates.
GPU work owns dense image/point processing and reductions. Bounded native C++
scalar algebra, tracking and serialization may consume compact results; this must
not conceal a dense CPU path or full-frame host readback.

Compare the entire output with the retained oracle, including rejected samples
and false execution/IK/collision flags. Preserve the pinhole-only grasp-ray rule,
which differs from distortion-corrected point-cloud rays.

Integrate through existing source/calibration/transform epoch and freshness
admission, timestamped TF, tracker acquisition and attachment ownership. Preserve
the actual observation stamp during unknown/stale recovery. A raw FrameLease or
component count is not a world-qualified observation. Existing Detection3DArray,
tracking JSON and scene consumers must receive equivalent semantics.

## 6. Qualification and return to the complete workshop release

For every new data path: failing regression first, separate hot-path refinement
with benchmark/static guards, CPU/native checks, then independently checked GPU
fixtures. Preserve failures and freeze actual loaded artifacts and drivers.

Require full native report parity, cancellation/stale/restart/unknown behavior,
and the existing fwmini4 acquisition/processing/latency/CPU targets with actual
devices. Revalidate video interoperability, scene attachment/self-filtering,
RViz/Blockly/notebook concurrency and the ten-minute full-stack soak.

Finish service/source-identity deployment together, the five workshop review
passes, taught segments and EV3 references, STOP/recovery, and three consecutive
physical dispense/convey/grasp/lift/place/release/return cycles. Merge/push main
and prove fwmini4 runs that revision only after the original release gates hold.
