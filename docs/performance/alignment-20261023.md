# Exact native alignment qualification, 2026-10-23

**PASS for the bounded offline native alignment path on AMD and Intel.** Both devices passed 65,536 division bit cases with zero mismatches and all eight CPU/actual SDK/GPU alignment cases with exact Z16 and maximum XYZ error 0 m. The unchanged outer `tools.qualify_native_preparation.alignment_result` gate passed at 1e-7 m. The executable's unchanged inner cap is 1e-6 m; passing that cap alone is insufficient. See the [archived machine evidence](alignment-20261023.json).

| Device | Division cases / mismatches | Exact CPU/SDK and GPU/SDK cases | Maximum XYZ error | Outer gate |
|---|---:|---:|---:|---|
| AMD Radeon 8060S / RADV | 65,536 / 0 | 8 / 8 | 0 m | PASS, 1e-7 m |
| Intel ADL-N / ANV | 65,536 / 0 | 8 / 8 | 0 m | PASS, 1e-7 m |

Both loaded SDKs report API version 25803. All 90 frozen manifest entries matched before and after execution. This qualifies the current 640×480 zero-coefficient profile and loaded SDK convention only; another SDK binary/build or profile must repeat the oracle. The bit corpus is not exhaustive over all operand pairs. NaN results use the reviewed NaN-classification comparator.

## Exact arithmetic and unchanged acceptance

The production helper uses native float division only as an approximate seed. For normalized finite inputs, two uint32 limbs represent N=A×2^23 and qB below 2^48. Every accepted quotient verifies qB≤N and N−qB<B. At most four integer corrections are followed, if needed, by exactly 23 restoring-division steps. Remainder comparison and even-quotient tie selection implement round-to-nearest-even; discarded bits and sticky remainder handle subnormals. Bounded normalization, zero signs, infinities, NaNs and overflow are handled explicitly. The helper needs neither FP64 nor Int64.

Both projection and XYZ deprojection division sites use the helper. Transform association, depth reduction, fixtures, CPU oracle hardware division and numeric thresholds remain unchanged. The oracle additionally rejects nonfinite XYZ, strengthening its NaN rejection.

## Independent CPU and offline evidence

The separate arithmetic/performance review passed 11 CPU CTests and 37 Python tests, including 3,088,821 hardware bit comparisons and 1,100,000 independent uint64 quotient/remainder checks. All seven preparation SPIR-V modules passed validation and NoContraction checks; the division probe is Shader-only. Source/compiler, SPIR-V, executable, library and case hashes and measured CPU microbenchmarks are preserved under separate `performance` and `gpu` objects in the JSON archive.

Scalar CPU helper medians were 12.055 ns per projection division and 8.155 ns per deprojection division, versus volatile hardware division 1.708/1.236 ns. Adverse quotient-only fallback medians were 45.948/51.025 ns. These are CPU microbenchmarks, not GPU predictions or measured alignment frame latency. Larger SPIR-V and integer control flow leave GPU performance an open measurement. Normal alignment GPU timing was not executed.

## Pinned identities

- Worktree branch: `feat/beginner-vulkan-20261022`, base HEAD `7b08a18a56796a0c381242561aa55690ae1ac6b7`.
- `alignment.slang`: `00437cde16a72965cb7bb01f338b1b84968655337e58e657b9b74ce2e03a6061`.
- Splat SPIR-V: `69f12068ac1d900a38b263b3101538f9df0d2b9ce43dafe8026035743c14ff96`.
- Finalize SPIR-V: `1a814310811e4772b9f320e6109b9364a20140db3349937de12bb1f56bedb8f6`.
- Divide-probe SPIR-V: `54a0bb4320ea043bfa46f46dc7fdf6986b9155bde28ffbaf91f983ef974c6bff`.

The archive records original aggregate SHA256 values and its UUID-redaction policy. No private numeric calibration, camera serial, fixture or image is imported.

## Remaining boundary

Existing camera acquisition still uses vendor CPU alignment on every vendor/backend path. No camera adapter integration follows this result, and strict raw-camera Vulkan refusal remains unchanged. Live GPU alignment, normal alignment timing, full geometry, live rate and physical-station acceptance remain unqualified. No CC, video or geometry GPU repeat is implied. The pending highest-resolution capture experiment is a separate scope; no unmeasured capture/performance claim is folded into this report.

Original evidence: `/tmp/workshop-alignment-exact-gpu-20261023.md`, its `summary.json`, and independent `/tmp/workshop-alignment-exact-perf-20261023.{md,json}`. The JSON archive preserves their separate evidence boundaries, including the independent review's pre-GPU status.
