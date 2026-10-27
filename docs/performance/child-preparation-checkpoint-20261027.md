# Resident child preparation checkpoint — 2026-10-27

**UNQUALIFIED experimental implementation.** This checkpoint preserves the
resident depth-child operator, its tests and a reproducible diagnostic. It is
not connected to the camera pipeline and does not qualify the workshop station.
Do not select it as the physical camera path or infer GPU performance from its
CPU tests.

## Fresh software checks

The existing child regression group passed **202 tests in 41.56 seconds**:

```sh
PYTHONDONTWRITEBYTECODE=1 python -m pytest -q -s -p no:cacheprovider \
  tests/test_native_child_contract.py \
  tests/test_native_child_graph_model.py \
  tests/test_native_child_perf.py \
  tests/test_native_child_qualifier.py \
  tests/test_native_child_qualifier_resources.py \
  tests/test_native_child_header_observation_resources.py \
  tests/test_native_child_candidate_observation_resources.py
```

The new hook diagnostic and its independent resource tests passed **46 tests**:

```sh
python -m pytest -q tests/test_native_child_hook_probe.py \
  tests/test_native_child_hook_probe_resources.py
```

A fresh Release build of `resident_children` and
`child_preparation_contract` succeeded with the station Conda wrapper and
Slang 2026.19. `resident_child_host_contract` passed, 1/1. That executable
substitutes Vulkan functions to test ownership and command recording. It does
not execute shaders. Its allocation and timing counters are host-test evidence.

The production child shader remains SHA-256
`8412aec1442aed563f99581a11bd3105db60b60947b5909ccf4e2e57ec0fe0ec`;
`src/children.cpp` remains
`77bb5aafe2caa5a9c106235f6baf936f691745d593ecda24ccaf0fe50e3c93c6`.
No production failure check or geometry threshold was relaxed.

## Hook fault observation

[The preparation utility](../../tools/performance/child_hook_probe.py) creates
an exclusive, separately hashed shader copy. It changes only the failure call
in `child_hook` and inserts a helper that attempts one compare/exchange before
the original fault OR. It records the already computed lane, root and parent
values without adding source loads. The normal production shader is unchanged.

The manifest restricts use to the 7×5, one-workgroup fixture with SHA-256
`e85eb99b89f9173a5ffd50f85638d8b5542dd482f90100d116012da9c368c3dd`.
Root and parent overflow are explicit; the lane field is only eight bits.
An earlier nonzero fault can prevent capture. A captured sample is the first
successful claim, not necessarily the first fault in time; it does not record
the iteration or the result of the root-parent lookup.

```sh
python tools/performance/child_hook_probe.py prepare \
  ros2/src/so101_station_vulkan/shaders/children.slang /absolute/fresh/probe
python tools/performance/child_hook_probe.py decode 0x80000004
```

The decode command above illustrates the format; its word is not an observed
GPU result. Marked faults remain rejected. The qualifier's existing 560-byte
raw fallback can run only for the unmarked, exact fault value 4 after graph
execution; it cannot turn the diagnostic into usable output.

Independent offline compilation and SPIR-V validation passed for all 21
derived kernels. Twenty were byte-identical to the production build. Only
`child_hook` changed: one additional atomic compare/exchange, with unchanged
load, variable, atomic-OR and atomic-minimum instruction counts. This does not
establish zero GPU overhead. Inspection of the production hook's emitted
control flow found no escape around its index and zero-root guards; Intel
runtime behavior remains unverified by that inspection.

## Open correctness findings

- The historical Intel `ChildNonconvergence` failure in the exact fixture still
  needs a fresh hardware reproduction and payload observation. Saved notes
  locate the hook's current-root check; they do not establish a causal fix.
  The old temporary probe archives are no longer available in this session.
- `child_reduce` contains a lane-dependent return before its next workgroup
  barrier. Prior validation normally faults and skips the reducer uniformly,
  but independently reaching this defensive path could hang instead of
  returning a clean fault. Resolve this before qualification. It is not an
  explanation of the earlier hook failure. Cold teardown currently waits for
  submitted fences, so an external process-group deadline remains necessary
  for diagnostic runs.

## CPU Vulkan and execution access

Mesa Lavapipe successfully enumerated here as a Vulkan 1.4 CPU device
(`llvmpipe`, LLVM 22.1.8, Mesa 26.2.3). It is a real Vulkan software driver,
distinct from the substituted Vulkan functions in the host contract.
[Mesa describes the Lavapipe frontend](https://docs.mesa3d.org/sourcetree).

The installed Slang 2026.19 compiler rejected both `child_hook` and
`child_reduce` with `-target cpp`, error E36107 at `InterlockedOr`.
[Slang's CPU-target documentation](https://github.com/shader-slang/slang/blob/master/docs/cpu-target.md)
also describes restrictions on atomics and barriers. Direct Slang CPU output
therefore does not currently replace Vulkan execution for these kernels.

The stock station Context deliberately refuses software Vulkan adapters. A
stock `benchmark_rgbd --backend compare` attempt stopped at that gate before
running shaders. CPU-Vulkan experiments must use a separately identified test
build and must not weaken the production hardware gate or be counted as
Intel/AMD hardware qualification.

Earlier on 27 October, the restricted sandbox session had no `/dev/dri`, a
forced Radeon probe could not enumerate a GPU, and SSH to
`cps-wkstn-fwmini4.local` failed at name resolution after retry. Hardware and SSH
access were subsequently restored in the new execution context on 27 October:
`/dev/dri/renderD128` is visible, local `vulkaninfo` succeeds with Radeon 8060S
and Mesa 26.2.3, and `ssh -F /dev/null bjoern@cps-wkstn-fwmini4.local` succeeds.
The remote USB inventory contains only keyboard and mouse devices. Hardware
reproduction is being prepared; these access checks provide no new GPU-kernel
qualification.

Full geometry integration, fresh hardware qualification, physical calibration,
STOP/recovery, three successful dispense/conveyor/pick/place cycles, the full
station soak and the release on `main` remain outstanding.

## Actual CPU-Vulkan execution

After an independent review, an isolated copy of the Context and qualifier
admitted only a CPU software adapter. Device properties were not altered;
the production hardware gate and all 38 Core/parent/child shader binaries
remained unchanged. The launcher forced Lavapipe after the Conda wrapper,
with an external 90-second process-group deadline for each run.

Both registered runs exited successfully without timeout:

- Core comparison at 64×48, one warmup and three measured iterations. The
  existing comparisons against the native CPU reference all passed.
- The exact 7×5 child fixture above. All six output arrays matched the frozen
  CPU expectation byte for byte, including empty records/packed arrays,
  counts `[1, 0, 0, 0]` and parent status `[[24, 24, 1, 0]]`. The fault was zero.

[The sanitized result](child-preparation-cpu-vulkan-20261027.json) records the
actual software-device identity and array hashes. This is real Vulkan shader
execution on a CPU, not the substituted API used by host tests. The historical
Intel fault did not reproduce on this software driver. Neither run establishes
Intel/AMD GPU correctness, performance or physical-station qualification.
