# Native Slang CPU execution implementation plan

> **For agentic workers:** Use superpowers:subagent-driven-development. Preserve the existing workshop plan and its acceptance gates.

**Goal:** Make the supported Slang CPU compiler usable in the resident RGB-D pipeline and measure it against the existing CPU implementation, without assuming a speedup.

**Architecture:** Add an explicit, optional `cpu-slang` backend. Compile CPU-specific conversion and candidate entry points ahead of time to native objects using Slang's LLVM backend. Reuse the existing C++ connected-component, reduction and morphology algorithms. GPU shaders and their qualification identities remain unchanged. Automatic selection continues to attempt hardware Vulkan first and fall back to the existing native CPU implementation.

**Tech stack:** Slang 2026.19 direct LLVM object-code target, C++17, CMake, existing native/CPU qualification tools.

**Spec:** User request of 2026-10-27: make the CPU compiler usable and verify whether it improves performance. This is an incremental backend within the approved beginner-workshop/Vulkan-first station design.

## Global constraints

- Preserve all numerical, calibration, component, morphology and fault checks.
- No emulated/no-op atomics or barriers; unsupported GPU entries stay in C++.
- CPU entry points reuse existing HSV rules and input layout; native precise floating-point division may replace the GPU-specific corrected divide on CPU, subject to numerical comparison.
- Use direct views of preallocated slot arrays; avoid GPU scratch layouts, redundant reduction initialization and per-frame copies/allocations.
- No runtime compiler, Vulkan instance/device creation, or GPU allocation in `cpu-slang` mode.
- A build without this optional backend must reject explicit selection clearly. Never silently label C++ or Lavapipe execution as native Slang CPU execution.
- Report backend and stage ownership. A faster subset is not a whole-pipeline speedup.
- Independent resource/performance review and a real microbenchmark precede GPU validation of data-path changes.

## Review focus

- Odd frame sizes and trailing RGB/Z16 bytes: no out-of-bounds access.
- Calibration extremes and depth discontinuities: unchanged masks/depth/candidates and XYZ tolerances.
- Empty/full images and component overflow: unchanged compact results and faults.
- Unsupported/unbuilt CPU compiler mode: explicit error with no hidden Vulkan attempt.
- Three retained slots/repeated frames: reusable storage, correct ownership and no per-frame allocation.

## Task 1: Native CPU stages and reusable algorithm boundary

Files: new CPU shader wrapper and C++ adapter/header in `ros2/src/so101_station_vulkan`; common CPU completion boundary in `src/cpu_pipeline.cpp`; native equivalence tests and standalone build/verification helper under `tools/performance`.

- [x] Add failing native tests for direct CPU-stage execution and full C++-reference equivalence, including the five review-focus classes above.
- [x] Implement CPU-only wrapper entry points that write existing slot arrays directly and reuse the original shader's pure HSV helpers. Bind reflected native CPU layouts explicitly, with compile-time ABI checks.
- [x] Split CPU preparation from the existing connected-component/reduction/morphology completion without changing the latter's algorithm. Both CPU implementations call the same completion routine.
- [x] Compile real native objects and run numerical tests, preserving original GPU source hashes. Save compiler commands, identities and logs.

## Task 2: Explicit pipeline selection and reproducible comparison

Files: pipeline selection/submission, CMake optional AOT build, benchmark CLI/report, relevant CPU-result consumers, docs and tests.

- [x] Add failing tests for `cpu-slang`, absent-build rejection, CPU consumer branches, and retained Vulkan-first `auto` behavior.
- [x] Wire the optional compiled backend into the existing pipeline and build; no compiler required on deployed hosts. Report Slang preparation/C++ completion stage ownership.
- [x] Obtain an independent resource review, meaningful allocation/static guards and a native microbenchmark.
- [x] Run matching C++ and Slang-CPU equivalence/benchmarks on the local workstation and fwmini4; separate warm steady-state wall time and CPU time from compilation/startup. Run software-Vulkan comparison only for the same verified scope and configuration.
- [x] Record measured advantages/regressions and keep current CPU fallback selection unless whole-pipeline results justify a separate adoption decision.
- [x] Independently review the final diff, run relevant CPU/native checks, commit and push the coherent checkpoint. Full station/physical acceptance remains open.

## CPU batching ruling (2026-10-27)

The optional Slang CPU backend uses one native invocation per preparation stage, with an explicit whole-frame loop and numthreads(1,1,1). Paired Intel fwmini4 measurements (three rotations, 80 frames, 10 warmup, same CPU affinity) showed lower p50 than the original 256-thread CPU lowering: dense 28.104945 to 26.526479 ms; sparse 7.180134 to 6.241573 ms. Existing C++ remains faster (dense 24.690361 ms, sparse 5.136671 ms). This approves the host-loop adaptation only within explicit optional cpu-slang. Auto/default CPU selection remains unchanged. Full field equivalence, overflow/fault handling, and retained storage contracts are required for both routes. Root evidence: /tmp/workshop-slang-cpu-paired-20261027/remote-measurements/result.json.

## Result boundary

The implementation and comparison steps are complete for the explicit experimental prealigned CPU core. See [results](../../performance/native-slang-cpu-20261027.md) for matched hardware/CPU-only Vulkan timings and the eight separately open canonical CPU failures. These steps do not establish the original workshop/station release or physical acceptance.
