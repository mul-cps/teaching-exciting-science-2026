# Practical Intel perception offload

Base: origin/main 407abe7a16d25cddff8cca36901a37e9ea924b15.
Isolated worktree: teaching-exciting-science-2026-intel-offload-20261021.

1. Integrate explicit `cpu` / `vulkan` (Slang first) / optional `opencl` color-mask backend in the real RGB-D object analyzer and ROS launch. CPU remains default. Lazy worker-local device initialization, one RGB upload and shared integer HSV for all four masks; same-frame CPU fallback on capability or execution failure, reported in output. Preserve all geometry, morphology and admission gates.
2. Add focused parity, fallback, changed-frame/shape and integration tests. Run on CPU and remote Intel hardware. Root coordinates the separate mandatory performance/static review before acceptance.
3. Benchmark complete analyzer p50/p95 and process CPU time at 1920x1080, 960x540 and 640x480 with deterministic multi-frame variants, frozen input hashes, and own DRM engine counters. Compare full outputs excluding backend telemetry. Saved frames do not establish live throughput or physical qualification.
4. Evaluate depth projection/cloud and shared tag/board RGB preprocessing. Reuse existing code where compatible; retain changes only with measured whole-stage gains and unchanged validity/numerical gates. Do not add a neural stack, Float64 dependency, motion changes, or a research framework. If no practical compatible gain, record evidence and exclusion reasons.

Auto selection requires >10% whole-analyzer gain across scenes with no p95 regression and identical outputs. Otherwise GPU selection stays opt-in. No activation, commit or push before supervisor review.

User steering: Vulkan/Slang is the primary implementation; OpenCL remains only a small optional OpenCV path.
