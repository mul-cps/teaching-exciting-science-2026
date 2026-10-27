# Exact software binary64 division and square root

> **For agentic workers:** Use the existing subagent-driven implementation and independent review workflow. Keep the full workshop goal active.

**Goal:** Supply the missing exact arithmetic primitives needed by resident geometry on Intel without changing current camera admission or geometry decisions.

**Architecture:** Extend the shared integer-only C++/Slang SoftFloat subset with value-returning division and square root. Use the retained rounding/NaN policy and independently generated expectations. Add a distinct offline diagnostic corpus/shader while preserving the existing primitive diagnostic ABI.

**Tech stack:** C++17, Slang 2026.19, SPIR-V, Berkeley SoftFloat 3e/TestFloat, pytest.

**Spec:** [Native geometry integration](2026-10-23-native-geometry-integration.md), especially sections 3, 5 and 6; [full workshop plan](2026-10-22-beginner-vulkan-workshop.md).

## Constraints and ruling

- This extends the earlier coordinate-only subset; it does not admit the rejected QR estimator, provide complete reports, or enable a new default backend.
- `uint64_t sf64_div_bits(uint64_t a, uint64_t b)` and `uint64_t sf64_sqrt_bits(uint64_t a)` return fixed round-to-nearest/ties-to-even encodings, gradual underflow, signed zero and infinity, and canonical quiet NaN `0xfff8000000000000`.
- No NaN payload, exception-state or configurable rounding equivalence is claimed. Existing add/multiply/conversion/comparison behavior stays unchanged.
- Derive the implementation from the exact existing SoftFloat 3e archive, retain the full BSD notice, and extend source hashes/ranges and modification inventory.
- No floating types, native 128-bit types, mutable global state, allocation, output references, host readback or device synchronization in the arithmetic helper. Loops/corrections must have explicit, justified finite bounds; guard shifts and normalization inputs.
- Preserve `SF64P001`, its seven-word rows and existing shader/CLI behavior. The new corpus uses `SF64D001` and the same header/row widths: inputs `a,b,f32_bits`, then expected division, square root of `a`, binary32 widening and existing four comparison/finiteness flag bits. The corpus kind must be explicit in validation and machine readouts.
- Production geometry tolerances, membership/order, resource ownership and false execution/IK/collision qualification flags are unchanged.

## Review focus

Signed zeros and all infinity/NaN combinations; subnormal inputs/results and underflow/overflow ties; near-perfect squares and quotient rounding boundaries; defined shifts/exponent arithmetic; old/new corpus or shader confusion and unchanged historical evidence.

## Task 1: Exact shared primitives and CPU conformance

**Files:**

- Modify `ros2/src/so101_station_vulkan/shaders/soft_binary64.slang`.
- Modify `ros2/src/so101_station_vulkan/third_party/softfloat/provenance.json`.
- Add `ros2/src/so101_station_vulkan/test/soft_binary64_divsqrt_host_test.cpp`.
- Add `tests/test_soft_binary64_divsqrt.py`.

**Interface:** The two value APIs above, callable unchanged by C++ and Slang.

- [x] Add and observe failing CPU tests for the missing APIs. Cover all special encodings, exponent boundaries, subnormal/normal transitions, perfect-square neighbors and quotient ties. Native volatile IEEE arithmetic with RNE and FTZ/DAZ disabled is one independent oracle; official TestFloat division/sqrt outputs are a second. Canonicalize only NaNs, never finite bits.
- [x] Implement the minimal upstream-derived integer paths, reusing the current rounding/normalization/product helpers. Record every new upstream source hash and adaptation. Preserve old tests and source restrictions.
- [x] Run at least one million deterministic randomized division/sqrt inputs plus the independent TestFloat cases. Add behavioral negative mutants for division rounding/remainder and square-root correction; each mutant must compile and fail.
- [x] Compile and validate a shader that actually calls both APIs. Check actual SPIR-V for enabled Int64 and absence of Float64 capability/types/native floating square root or division. Run focused old/new CPU tests and UBSan without executing a GPU.

## Task 2: Distinct offline Vulkan diagnostic

**Files:**

- Add `tools/performance/soft_binary64_divsqrt_probe.slang`.
- Extend `tools/performance/qualify_soft_binary64.cpp` only for the explicit corpus kind.
- Add `tests/test_soft_binary64_divsqrt_probe.py`.
- Update the software-binary64 section of `tools/performance/README.md`.

**Interface:** `--generate-divsqrt NEW_CORPUS [--count N --seed N --testfloat-dir DIR]`; existing `--validate`, `--validate-spirv` and `--vulkan` accept the explicitly discriminated new corpus and report its kind. Keep existing commands/old corpus semantics intact.

- [x] Observe failing tests for the new command/magic and shader. Test malformed, truncated, oversized and corrupted expectations, wrong arithmetic kind and exclusive fresh outputs. Verify old corpora and commands still behave identically.
- [x] Generate expectations with the independent native/TestFloat oracles, never by invoking the helper. The new shader writes division/sqrt/widen/flags in the existing four-word output shape. Preserve old diagnostic files and immutable receipts.
- [x] Exercise CPU-only generator/validator and shader compilation/validation. Freeze helper, harness, inputs, expected bytes, compiler, SPIR-V and library identities before hardware execution.

## Task 3: Independent resource gate, hardware qualification and checkpoint

**Files:** New bounded host benchmark/static guard tests chosen by the independent reviewer; dated evidence under `docs/performance/` only after actual qualification.

- [x] A separate reviewer checks finite correction bounds, shifts/overflow and absence of new allocation/sync/host work. Add an actual CPU per-operation benchmark and static negative guards before GPU execution. These numbers are host arithmetic costs, not station FPS.
- [x] After semantic/resource approval, run the frozen diagnostic on actual Intel and AMD with explicit ICDs. Require byte-exact expected results; retain failures and all source/compiler/device/driver pins. Do not restart station services or acquire hardware cameras/arms.
- [x] Run the canonical pinned CPU suite and relevant native builds after source freeze. Recheck old geometry SPIR-V/capabilities and existing arithmetic without weakening gates.
- [x] Independently review and commit the coherent source/evidence checkpoint, push the feature branch, and verify the remote SHA.

The full workshop goal remains active after this arithmetic checkpoint: complete geometry, deployment, physical cycles and the workshop release. Main remains gated on that complete objective.
