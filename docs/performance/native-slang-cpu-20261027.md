# Native Slang CPU experiment — 27 October 2026

The optional experimental `cpu-slang` backend is usable on both tested x86-64
hosts, but the existing C++ CPU implementation remains faster. Automatic backend
selection still attempts hardware Vulkan first and uses the existing C++ CPU
fallback. Explicit `cpu-slang` camera mode remains rejected. The
[evidence archive](native-slang-cpu-20261027.json) records inputs, raw results,
source identities and validation boundaries; `qualified` is false.

Slang 2026.19 emits generic LLVM AOT object code for two CPU-specific wrappers:
`cpu_convert` and `cpu_candidates`. They write preallocated Slot arrays directly;
connected components, reduction and morphology use the unchanged shared C++
completion algorithm. This is native CPU execution, with no runtime compiler,
Vulkan instance or GPU resources. Compilation alone does not turn the whole GPU
pipeline into a CPU implementation: 35 of 38 original GPU entries remain
unsupported by the tested LLVM route because reachable atomics or group barriers
are rejected. Only resident convert/candidates and parent labels compiled.
No atomics were replaced with no-ops and no barriers were erased.

## Build and use

From the repository root, choose an output directory that does not exist:

```bash
python tools/performance/build_native_slang_cpu.py \
  --slangc /home/bjoern/.local/opt/slang-2026.19/bin/slangc \
  --output-dir /tmp/native-slang-cpu-fresh --build-tests
/tmp/native-slang-cpu-fresh/native-test
/tmp/native-slang-cpu-fresh/benchmark_rgbd \
  --backend compare-cpu-slang --width 640 --height 480 --frames 2 --warmup 1
/tmp/native-slang-cpu-fresh/benchmark_rgbd \
  --backend cpu-slang --width 640 --height 480 --frames 80 --warmup 10
```

The [build helper](../../tools/performance/build_native_slang_cpu.py) validates
reflected ABI types, offsets and strides before publishing the object. Native
CMake integration uses `-DRESIDENT_SLANG_CPU=ON` and a build-time
`-DSLANGC_EXECUTABLE=…`; the option defaults off. Slang runs during the build,
not during frame processing or deployment. An unbuilt explicit backend refuses
selection. See the [implementation plan](../superpowers/plans/2026-10-27-native-slang-cpu.md)
and [native performance tools](../../tools/performance/README.md).

## Measured results

Wall p50 in milliseconds per output, rounded to two decimals:

| Host / frozen 640×480 input | C++ CPU | Final cpu-slang | Lavapipe, default threading |
|---|---:|---:|---:|
| Intel fwmini4 / mixed dense | 24.69 | 26.53 | 325.09 |
| Intel fwmini4 / sparse | 5.14 | 6.24 | 80.87 |
| AMD workstation / mixed dense | 12.94 | 13.98 | 38.75 |
| AMD workstation / sparse | 1.88 | 2.79 | 10.70 |

Default Lavapipe uses multiple CPU workers and all allowed cores; each native
run uses one pinned core. These are practical configurations with different CPU
budgets. The one-CPU, one-Lavapipe-worker control took **1033.29 / 236.15 ms**
for Intel dense/sparse and **408.20 / 89.21 ms** for AMD dense/sparse.
Process CPU time sums all process threads: Intel dense C++ / cpu-slang /
default Lavapipe consumed **24.655 / 26.498 / 1181.055 CPU ms per output**.

On Intel, final cpu-slang is 7.4% slower than C++ for dense input and 21.5%
slower for sparse input. It is approximately 12.3× / 13.0× faster than default
Lavapipe on those inputs. These ratios do not isolate the compiler: the hybrid
CPU and software-Vulkan routes use different connected-component and morphology
algorithms and scheduling.

The final CPU wrappers use one whole-frame native call per stage with
`numthreads(1,1,1)`, avoiding per-pixel function crossings. Against the initial
256-thread CPU wrapper lowering, paired Intel p50 improved by 5.6% dense and
13.1% sparse. This adaptation is retained only in explicit cpu-slang; it does
not justify changing the existing default CPU fallback.

The final generic object SHA-256 is
`5b6bec194cd0999799ddd2bc74d1b178bf28c63585ae636ad5caa21fda87ce72`.
It is byte-identical to the Intel-tested loop object and works on both hosts.
Original GPU shader sources remain unchanged; their hashes and the private
CPU-only Lavapipe admission delta are recorded in the archive.

## Method and limits

Native measurements use three rotated variant orders, 80 measured frames and
10 warmup frames per run, pinned to the last allowed core: local CPU31 and
remote CPU3. Reported native summaries are medians of three run medians/p95
values, not pooled percentiles. Both Slang variants pass full-output comparison
before timing. Default Lavapipe uses 20 frames plus three warmup frames;
the one-worker control uses 30 plus five. Lavapipe has separate full-output
comparison and explicitly forced software-device admission.

The two frozen 640×480 RGB8/Z16 fixtures are deterministic mixed colors with
holes/depth discontinuities and a sparse red rectangle on gray. Exact byte
sizes, SHA-256 identities, commands and raw outputs are in the JSON archive.
Timing covers submit through finish: Lavapipe upload/readback is included;
fixture memcpy, compilation, construction and comparison are excluded.
Process CPU measurement includes all worker threads. Clocks/governors were not
locked, and these runs do not qualify power savings, control jitter or a loaded
station workload.

Validation recorded 44 focused Python guards, including ABI/resource mutants;
native and absent-build contracts passed locally, and both loop variants passed
native contracts on fwmini4. Three CMake CPU CTests passed. The broader root CPU
run before final batching refinement exited 1: **4299 passed, 9 failed,
136 skipped, 48 subtests passed in 211.03 s**. One extraction-sensitive capacity
source guard was subsequently corrected, with covering tests passing. Eight
separate failures remain open: three immutable Python geometry axis/sign/hash
mismatches and five ROCm CPU test-child failures caused by resolving a symlinked
venv interpreter to base Python without NumPy. No thresholds or goldens changed;
the broader suite is not reported as passing.

This evidence covers only the prealigned RGB8/Z16 core. Capture, depth alignment,
full geometry, tracking, ROS deployment, physical calibration, station operation
and acceptance remain outside the measured and validated scope.
