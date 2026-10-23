# Compute-feature prerequisite and CPU geometry contract — 2026-10-23

The supported FP64 baseline is now independent of optional video negotiation.
Context `ddc535619de1edc6879908d50674e2f6c7325b2f5d6bb47eac4cd3c0a5006384`
queries physical core features once and requests only supported shaderFloat64 in
compute-only creation/fallback. Unsupported Intel FP64 remains false. This is a
prerequisite correction, not full geometry implementation or a performance result.
[Machine archive](compute-feature-baseline-20261023.json) preserves original reports,
source/runtime/artifact hashes and separate outcomes below.

## CPU contract and host cost

Final canonical CPU verification: 3,868 passed, 136 skipped, 48 subtests; 14 native
CTests and 347 native ROS tests passed. Focused geometry checks passed 42 tests in
each Python environment without changing numeric, array or discrete tolerances.
The 12-case [retained CPU matrix](../../tools/native_geometry_matrix.py) and
[procedural golden](../../tests/fixtures/native_geometry/matrix-v1.json) preserve
full-report 1e-9, ray 1e-12 and exact array/type gates. Strict identity still rejects
changed runtime provenance. The separate payload contract excludes only four runtime
version strings, retains full-payload hashes and does not relabel a different runtime
as the original qualification artifact. Golden updates were metadata-only.

Across seven million retained-owner Context cache acquisitions, queries, allocations,
Vulkan calls and waits were zero. Each synthetic cold Context queried the baseline
once. Host nanosecond helper timings exclude real driver/GPU cost; no speedup follows.

## Physical support and the next precision path

Separate instance-only probes found `shaderInt64` and `shaderInt16` support on both
AMD 1002:1586 and Intel 8086:46d1. Physical FP64 was true on AMD and false on Intel.
Those probes created no logical device or kernels. Enabled Int64 was not measured;
the current compute-only source baseline requests only supported FP64.

Intel's missing FP64 therefore does not require avoiding Int64 carriers. The next
coordinate prototype can evaluate integer-based binary64 arithmetic after deliberate
supported-feature enablement, independent bit tests and GPU performance/correctness
qualification. Physical capability alone does not qualify that arithmetic.

## GPU chronology and provenance limits

1. Default AMD passed eight preparation cases (maximum coordinate error
   2.7755575615628914e-17 m) and eight raw SDK/Core/Alignment cases. The actual driver
   prehash was missing; mappings/posthash survive, but full pre/post provenance does not.
2. The generic Mesa API override failed its first semantic gate, retaining API 1.4.
   RADV 26.2.3 does not call the generic override helper. This was not a numeric failure.
3. A reviewed capability mask changed only returned videoMaintenance1 to false.
   The actual API stayed 1.4, physical/enabled FP64 stayed true, with one forwarded
   query and one mask. The six-app protocol stopped on an implicit-layer prehash gap;
   its valid capability result does not make that whole protocol pass.
4. A newly registered remaining five-app protocol passed with critical runtime/driver
   prehashes and implicit-layer closure. Masked AMD all_valid 640×480 preparation had
   zero coordinate error, exact flags/support, 307,200 valid pixels and 4,800 support
   samples. Masked AMD and unmasked Intel each passed all eight raw cases with zero
   depth/Core mismatches and XYZ error. Intel physical/enabled FP64 remained false;
   explicit preparation refusal passed. This is not Intel geometry success.

No newly discovered shared-library dependency lacked a prehash in the final five apps.
Two mapped loader-cache observations lacked prehashes; these are noncritical data-cache
provenance limits, distinct from critical driver/library checks. Earlier missing
prehashes remain unrepaired historical limits. All owned resources were released.

The archived [mask shim](../../tools/performance/video_feature_mask.c) is explicit
offline feature-failure injection, not natural video absence or physical API 1.2 evidence.
No shader arithmetic, tolerance or unsupported hardware feature was changed. Native
plane fitting, contours, PCA and complete reports remain unimplemented; see the
[integration plan](../superpowers/plans/2026-10-23-native-geometry-integration.md).
No FPS, GPU speedup, full geometry or physical-station acceptance claim follows.
