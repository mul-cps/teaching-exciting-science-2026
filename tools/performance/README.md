# Frozen resident CPU-headroom diagnostic

`resident_headroom.cpp` is the byte-identical diagnostic used for the
[2026-10-23 measurements](../../docs/performance/resident-20261023.md).
SHA256: `5ee654202683a86a411bf9888c19c5caaaea29cde607a2a7e2c47f2113cc95ad`.
It is an offline benchmark, with no ROS, camera or actuator access. The native
pipeline, Vulkan loader/driver, JSON headers and compiled shaders are external
dependencies. It is deliberately separate from the production camera queue:
the archived diagnostic retains its original dispatch order for comparability.

Build against an ABI-consistent native snapshot containing `source/include`,
`build/libresident_rgbd.so`, `build/vulkan-sdk` and `shaders/*.spv`. Run this
compiler command inside the station's reviewed ROS/Vulkan environment. Use the
appropriate system Vulkan loader path for the host:

```sh
task_snapshot=/absolute/reviewed-snapshot
task_source=/absolute/checkout/tools/performance/resident_headroom.cpp
c++ -O2 -std=c++17 \
  -I"$task_snapshot/build/vulkan-sdk" \
  -I"$task_snapshot/source/include" -I"$CONDA_PREFIX/include" \
  "$task_source" -o /tmp/resident-headroom \
  -L"$task_snapshot/build" -Wl,-rpath,"$task_snapshot/build" \
  -lresident_rgbd /usr/lib/x86_64-linux-gnu/libvulkan.so.1 -pthread
```

AMD's measured loader path was `/usr/lib64/libvulkan.so.1`, with the process-scoped
RADV/system-libdrm environment documented in [resident-vulkan.md](../../docs/resident-vulkan.md).
Verify `ldd` resolves the intended snapshot library before each experiment.

The executable takes one absolute JSON configuration path. Required fields:

| Field | Value |
| --- | --- |
| `kind` | `perception` or `control` |
| `start_ns` | Fresh Linux monotonic nanoseconds, at least four seconds in the future |
| `seconds` | Measurement-window duration; the reported trials used 20 |
| `output` | Absolute result JSON path |
| `mode` | Perception only: `cpu` or strict `vulkan` |
| `workload` | Perception only: `dense` or `recorded` |
| `hz` | Perception only: offered frame rate; the comparison used 30 |
| `shaders` | Perception only: absolute path to eight compiled compute shaders |
| `fixture` | Perception only: directory containing `rgb.bin`, `depth.bin`, `case.json` |

Dense input is generated internally; `fixture` is only read for recorded input.
Keep recorded calibration and input files external; compare their SHA256 values
with the machine evidence rather than embedding private station calibration.
Regenerate `start_ns` for every run: saved configurations cannot be replayed
with their old absolute start time. Launch perception and the separate 125 Hz
synthetic control process with the same start and duration. Use an exclusive
timing window; the benchmark does not manage other workloads or CPU governors.

The published comparison ran three repeats of dense and recorded input, each in
old GPU, new GPU, CPU order. Perception performs ten warmup frames and prepares
its CPU oracle before the measurement start. The steady window includes plane
copies, submit/poll/finish and compact verification, with two-millisecond polling.
Initialization, warmup, result serialization and final draining are excluded.
Timing-vector capacity and hot Vulkan-allocation guards must pass. Results are
valid only when `measurement_valid` is true, completed outputs match the CPU
oracle, and the offered/completed/replaced/in-flight counts are reported together.

`CLOCK_PROCESS_CPUTIME_ID` measures perception-process CPU demand. The separate
control task is a scheduler proxy; it does not establish physical control timing,
whole-host CPU savings, live-camera throughput or full-station qualification.

## Raw resident synthetic SDK reference driver

[raw_resident_reference.cpp](raw_resident_reference.cpp) is the unchanged eight-case
combined Alignment/Core driver used for the [offline two-host qualification](../../docs/performance/raw-resident-20261023.md).
Its SHA256 is `31c0165049c236731a1950fe45c78304a1f429217828046597bd2cf8e3a396c5`.
It uses SDK software frames and synthetic calibration/identity; it does not acquire
an actual camera or change camera options. This is correctness evidence, not timing.

Build inside the native station wrapper environment, using ABI-consistent installed
headers/libraries plus Vulkan, librealsense SDK and nlohmann JSON headers. Substitute
reviewed absolute locations and the appropriate host Vulkan loader:

```sh
task_install=/absolute/reviewed-install
task_build=/absolute/reviewed-build
task_source=/absolute/checkout/tools/performance/raw_resident_reference.cpp
"${CXX:-c++}" -O2 -std=c++17 -ffp-contract=off \
  -I"$task_install/include" -I"$task_build/vulkan-sdk" -I"$CONDA_PREFIX/include" \
  "$task_source" -o /tmp/raw-resident-reference \
  -L"$task_install/lib" -Wl,-rpath,"$task_install/lib" \
  -lresident_alignment -lresident_rgbd -lrealsense2 /usr/lib64/libvulkan.so.1 -pthread
```

The argument contract is `BINARY --cpu|--vulkan CORE_SHADERS ALIGNMENT_SHADERS OUTPUT_DIRECTORY`.
Create a fresh private output directory first: the driver requires it to exist and
writes eight synthetic input/calibration/SDK-output sets plus `summary.json`.
Use the Core and Alignment shaders belonging to the same reviewed source snapshot.
For example, inside the same native environment:

```sh
task_output=$(mktemp -d /tmp/raw-resident-reference.XXXXXX)
chmod 700 "$task_output"
/tmp/raw-resident-reference --cpu \
  /absolute/reviewed/core-shaders /absolute/reviewed/alignment-shaders "$task_output"
```

CPU mode exercises all eight SDK/reference cases without GPU dispatch. Vulkan mode
also checks the typed bridge, fences and Core outputs; run it with an external
process watchdog that bounds the entire process lifecycle. The driver's ten-second
poll deadline does not bound a blocked GPU driver/destructor drain. GPU selection
and process-local driver environment must match the reviewed host configuration.
Do not interpret qualification duration or these diagnostic array writes as normal-path latency.

## Offline optional-video feature mask

[video_feature_mask.c](video_feature_mask.c) preserves the reviewed diagnostic
source byte-for-byte (SHA256 `61dafad9997d0d556bcb27359aff74e4cdfdb82115745ddae7f023115f776275`).
Build the C11 source inside the native SDK environment:

```sh
task_source=/absolute/checkout/tools/performance/video_feature_mask.c
task_vulkan_headers=/absolute/reviewed-build/vulkan-sdk
cc -x c -std=c11 -shared -fPIC -O2 -I"$task_vulkan_headers" \
  "$task_source" -o /tmp/video-feature-mask.so -ldl
```

The shim resolves the real Vulkan Features2 query once, forwards it unchanged, then
masks only a returned maintenance1 node's videoMaintenance1 flag. Other chain bytes,
physical API, FP64 and extension inventory remain real. Counters distinguish query
calls from actual true-to-false masks. It is not a station startup configuration.

Use process-local LD_PRELOAD only for explicit offline qualification applications;
never apply it to station startup, a shell session or a service. Run a real-driver
capability probe first: the tested AMD contract was actual API 1.4, physical/enabled
FP64 true, video false, exactly one query and one mask. This injects feature failure;
it does not qualify natural video absence or a physical API 1.2 GPU. Keep Intel
unmasked when verifying unsupported-FP64 refusal.

Freeze source/binary, ICD-resolved driver, snapshot/runtime libraries and implicit-layer
closure hashes before running; verify actual child mappings and posthashes. Use an
owned external process watchdog covering driver/destructor drain. The
[baseline archive](../../docs/performance/compute-feature-baseline-20261023.md)
retains earlier prehash failures and the later five-app pass separately. This helper
provides no performance, codec, full-geometry or station qualification.

## Physical precision metadata

[physical_precision_features.cpp](physical_precision_features.cpp) is the unchanged
instance-only audit source, SHA256
`939e141b23c19f6d441677e781a65a6b1ea3d59a0e5bec6536d7383181c86a58`.
It creates an instance, queries one matching hardware vendor and destroys the instance.
It creates no logical device, shader or command buffer. Physical support, enabled
logical-device features and qualified arithmetic remain separate evidence.

Compile inside the native environment; use the reviewed Vulkan loader path for the host
(`/usr/lib64` on the measured AMD workstation, `/usr/lib/x86_64-linux-gnu` on fwmini4):

```sh
task_source=/absolute/checkout/tools/performance/physical_precision_features.cpp
task_vulkan_headers=/absolute/reviewed-build/vulkan-sdk
c++ -std=c++17 -O2 -I"$task_vulkan_headers" "$task_source" \
  /usr/lib64/libvulkan.so.1 -o /tmp/physical-precision-features
```

Run the binary with `0x1002` for AMD or `0x8086` for Intel in the process-scoped
reviewed ICD environment. Standard output contains JSON physical feature scalars;
standard error contains process maps, which belong in the private evidence directory.
The [metadata findings](../../docs/performance/compute-feature-baseline-20261023.md#physical-support-and-the-next-precision-path)
show Int64 support on both tested GPUs and FP64 support only on AMD.

## Software binary64 primitive diagnostic

[qualify_soft_binary64.cpp](qualify_soft_binary64.cpp) and
[soft_binary64_probe.slang](soft_binary64_probe.slang) provide a separate arithmetic
diagnostic. The [evidence checkpoint](../../docs/performance/software-binary64-20261023.md)
records CPU checks, source/artifact identities and remaining gates. Separate
[isolated preparation timing](../../docs/performance/geometry-preparation-timing-20261023.md)
is available; transfer-inclusive/full-pipeline performance, complete geometry,
default camera admission and physical operation remain unqualified.
Run the examples from the repository root in Bash; choose a fresh task directory.

CPU corpus generation/validation needs a C++17 compiler and no Vulkan headers,
loader, resident library or device. The native volatile oracle does not call the
software helper. Keep its exact floating-point flags:

```bash
task_prefix=$(mktemp -d "${TMPDIR:-/tmp}/software-binary64.XXXXXX")
task_cxx=${CXX:-c++}
task_cxx_flags=(-std=c++17 -O2 -ffp-contract=off -fno-fast-math -frounding-math
                -Wall -Wextra -Wno-missing-field-initializers)
"$task_cxx" "${task_cxx_flags[@]}" -DSOFT_BINARY64_CPU_ONLY \
  tools/performance/qualify_soft_binary64.cpp -o "$task_prefix/qualify-sf64-cpu"
"$task_prefix/qualify-sf64-cpu" --generate "$task_prefix/corpus.bin" --count 65536 --seed 17
"$task_prefix/qualify-sf64-cpu" --validate "$task_prefix/corpus.bin"
```

Generation requires a new file, count 16,384–4,000,000 and a nonzero seed.
Optional `--testfloat-dir` consumes pre-generated `f64_add.txt`, `f64_mul.txt` and
`f32_to_f64.txt`; it does not launch TestFloat. To prepare those files with an
independently built, unmodified TestFloat Release 3e generator:

```bash
task_testfloat_gen=${TESTFLOAT_GEN:?set the reviewed testfloat_gen executable}
task_testfloat_dir="$task_prefix/testfloat"
mkdir "$task_testfloat_dir"
for task_operation in f64_add f64_mul f32_to_f64; do
  "$task_testfloat_gen" -level 1 -rnear_even -tininessafter "$task_operation" \
    > "$task_testfloat_dir/$task_operation.txt"
done
"$task_prefix/qualify-sf64-cpu" --generate "$task_prefix/corpus-testfloat.bin" \
  --count 65536 --seed 17 --testfloat-dir "$task_testfloat_dir"
"$task_prefix/qualify-sf64-cpu" --validate "$task_prefix/corpus-testfloat.bin"
```

Compile the actual probe plus shared helper offline using reviewed Slang 2026.19
and SPIR-V Tools. These commands create no Vulkan instance or device:

```bash
task_slang=${SLANGC_EXECUTABLE:-slangc}
"$task_slang" tools/performance/soft_binary64_probe.slang \
  -entry main -stage compute -target spirv -profile sm_6_0 -fp-mode precise -O2 \
  -I ros2/src/so101_station_vulkan/shaders -o "$task_prefix/probe.spv"
spirv-val --target-env vulkan1.1 "$task_prefix/probe.spv"
"$task_prefix/qualify-sf64-cpu" --validate-spirv "$task_prefix/probe.spv"
```

The probe's offline admission requires Int64 and refuses Float64 capability/types;
`spirv-val` supplies full module validation. Preserve corpus/SPV/source hashes.

For a separately authorized Vulkan run, build against ABI-consistent installed
resident headers/library and the reviewed Vulkan SDK/loader in the same native
environment. Set `SO101_REVIEWED_INSTALL` and `SO101_VULKAN_HEADERS` to the chosen
prefixes; the latter contains `vulkan/vulkan.h`. Record compiler identity and a
meaningful `SOFT_BINARY64_BUILD_PROVENANCE` string, rather than the refused default
`unrecorded`:

```bash
task_install=${SO101_REVIEWED_INSTALL:?set the reviewed installed resident prefix}
task_vulkan_headers=${SO101_VULKAN_HEADERS:?set the reviewed Vulkan include prefix}
task_sha() { sha256sum "$1" | cut -d ' ' -f 1; }
"$task_cxx" --version > "$task_prefix/compiler-version.txt"
task_build_provenance="source_sha256=$(task_sha tools/performance/qualify_soft_binary64.cpp);helper_sha256=$(task_sha ros2/src/so101_station_vulkan/shaders/soft_binary64.slang);shader_sha256=$(task_sha tools/performance/soft_binary64_probe.slang);spirv_sha256=$(task_sha "$task_prefix/probe.spv");compiler_sha256=$(task_sha "$(command -v "$task_cxx")");compiler_version_sha256=$(task_sha "$task_prefix/compiler-version.txt");cxxflags=${task_cxx_flags[*]};slangflags=-entry main -stage compute -target spirv -profile sm_6_0 -fp-mode precise -O2"
"$task_cxx" "${task_cxx_flags[@]}" \
  "-DSOFT_BINARY64_BUILD_PROVENANCE=\"$task_build_provenance\"" \
  -I"$task_install/include" -I"$task_vulkan_headers" \
  tools/performance/qualify_soft_binary64.cpp -o "$task_prefix/qualify-sf64-vulkan" \
  -L"$task_install/lib" -Wl,-rpath,"$task_install/lib" -lresident_rgbd -lvulkan -pthread
ldd "$task_prefix/qualify-sf64-vulkan"
```

Check that dependency resolution selects the intended installed resident library
and loader. Archive the source/library/compiler/SPV identities and actual runtime
mapping/pre/post hashes with the independently reviewed run protocol. The example
build string identifies arithmetic inputs and compiler flags; it does not replace
that runtime provenance closure or authenticate a library chosen by the loader.

Select **one** reviewed ICD per process. A real external watchdog must cover the
whole lifecycle, including blocked driver/destructor calls; the acknowledgment
environment variable alone provides no timeout. This GNU `timeout` example sends
TERM at 60 seconds and KILL after five more seconds:

```bash
task_icd=${SO101_REVIEWED_ICD:?set one reviewed hardware ICD manifest}
timeout --signal=TERM --kill-after=5s 60s env -u VK_ICD_FILENAMES \
  VK_DRIVER_FILES="$task_icd" SOFT_BINARY64_EXTERNAL_WATCHDOG=1 \
  "$task_prefix/qualify-sf64-vulkan" --vulkan \
  "$task_prefix/corpus.bin" "$task_prefix/probe.spv" "$task_prefix/gpu-result"
```

The output directory must not already exist. Use the optional upstream corpus in
place of `corpus.bin` when desired. Set the ICD to the selected AMD or Intel driver
only for that invocation; do not export either selection globally. The tool requires
both physical and actually enabled Int64, processes at most 16,384 rows per batch,
and compares four output words per row exactly. Preserve nonzero exits and partial
artifacts. Even a successful primitive run does not qualify preparation coordinates,
transfer-inclusive performance, full object reports or station operation.

## Geometry preparation timing

[benchmark_geometry.cpp](../../ros2/src/so101_station_vulkan/test/benchmark_geometry.cpp)
measures one retained resident source through the real submit/ready/finish API.
The [timing archive](../../docs/performance/geometry-preparation-timing-20261023.md)
records two synthetic 640×480 cases, native Float64 on AMD and explicit software
binary64 on AMD/Intel. It is not a unique-output, camera, frame-age, full-pipeline
or performance-acceptance test. No default camera/backend selection changes.

Run the following **offline** preparation/build steps from the repository root
inside the reviewed native station environment. The factory computes independent
CPU geometry/counts; it never uses software-helper arithmetic or GPU output as
an oracle. Both frozen cases expect 307,200 valid pixels and 4,800 support points.
Use a fresh prefix; fixture/output creation refuses existing destinations.

```bash
task_geometry_prefix=$(mktemp -d "${TMPDIR:-/tmp}/geometry-preparation.XXXXXX")
task_geometry_build="$task_geometry_prefix/build"
task_geometry_fixtures="$task_geometry_prefix/fixtures"
task_slang=${SLANGC_EXECUTABLE:?set the reviewed native Slang compiler}
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2/conda/so101-conda.sh run python \
  tools/performance/prepare_geometry_benchmark.py --output "$task_geometry_fixtures"
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2/conda/so101-conda.sh run cmake \
  -S ros2/src/so101_station_vulkan -B "$task_geometry_build" \
  -DRMW_IMPLEMENTATION=rmw_cyclonedds_cpp -DCMAKE_BUILD_TYPE=Release \
  -DSLANGC_EXECUTABLE="$task_slang" -DENABLE_SOFTWARE_GEOMETRY=ON
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2/conda/so101-conda.sh run cmake \
  --build "$task_geometry_build" \
  --target benchmark_geometry qualify_geometry software_geometry_spirv
```

The CMake target generates a meaningful nondefault
`GEOMETRY_BENCHMARK_PROVENANCE` stamp with source/compiler hashes and flags,
including `-ffp-contract=off -fno-fast-math -frounding-math`. Real run mode refuses
the fallback `unrecorded` stamp. Manual builds need equivalent reviewed provenance
and ABI-consistent libraries; do not replace the stamp with a dummy value.
Runtime JSON records the actual fixture and five SPV hashes. Preserve compiler
identity, exact flags, parent/helper dependencies and module validation results.

This command validates the fixture, expected-count bounds and software SPV profile
without creating a Vulkan Context/device or collecting timing samples:

```bash
"$task_geometry_build/benchmark_geometry" --offline --software-binary64 \
  "$task_geometry_fixtures/full_xyz_rotation.bin" \
  "$task_geometry_build/software-geometry-shaders" \
  "$task_geometry_prefix/full-xyz-offline.json" \
  --expected-valid 307200 --expected-support 4800
```

Use `--native-float64` with `geometry-shaders` for native-profile offline validation.
A future real run requires both physical and actually enabled Int64 for software,
or Float64 for native; unsupported native Float64 remains refused. The measured
native run was AMD only.

Before timing a new source/runtime/calibration version, freeze inputs and run the
existing `qualify_geometry` plus unchanged `qualify_native_preparation.py compare`
on the independent NPZ. Require the unchanged absolute 1e-12/exact discrete and
lease/fence/generation contracts. Never obtain expected counts from a first GPU run.
The historical full-XYZ case passed these gates in all three measured mode/device
combinations; a new version is a new qualification scope.

For a separately reviewed GPU run, construct the explicit argument vector below.
This block **does not launch it**. Have the reviewed external process-group owner
launch that vector with a 90-second deadline, TERM/KILL/drain ownership, immutable
inputs and stop-first-failure behavior. The owner must also pin the actual loader
per executable, hardware ICD/driver, all critical DRM/core libraries, source,
helper, SPVs and fixture; archive actual maps and post-critical identities before
mapping validation. Unrelated nonexecuting/nonlibrary/nonpinned data is diagnostic,
not a home/cache-directory gate. The acknowledgment alone is not a watchdog.

```bash
task_geometry_precision=--software-binary64
task_geometry_shaders="$task_geometry_build/software-geometry-shaders"
task_geometry_case=full_xyz_rotation
task_geometry_gpu_args=(
  env GEOMETRY_BENCHMARK_EXTERNAL_WATCHDOG=1
  "$task_geometry_build/benchmark_geometry" --run "$task_geometry_precision"
  "$task_geometry_fixtures/$task_geometry_case.bin" "$task_geometry_shaders"
  "$task_geometry_prefix/$task_geometry_case-timing.json"
  --expected-valid 307200 --expected-support 4800
  --warmup 8 --trials 3 --iterations 32 --poll-us 50 --cpu-budget-ms 2000
)
```

Select one reviewed hardware ICD only for that child process; do not export driver
selection or feature masks globally. Native AMD uses `--native-float64` and
`geometry-shaders`. Input uploads, setup, eight warmups and drain are excluded.
A finite ten-second operation/source-fence deadline does not replace the external
owner for blocked driver/destructor calls. Preserve nonzero exits/partial evidence.

Report nearest-rank p95 within each 32-sample trial (rank 31), then median of three
trial p95s, never pooled tails. Keep raw samples, whole-trial process CPU/32,
ordinary-C++new/station Buffer counter scope and requested/observed polling sleep
separate. Do not infer FPS, add these tails to Core/alignment tails or claim a CPU
baseline/speedup; native/software AMD comparison was descriptive without DVFS
isolation. The [measured header snapshot](../../docs/performance/geometry-preparation-timing-20261023.md#measured-source-and-current-tree)
predates the intentional `FramePoolToken` API addition; current transitive sources
must not be presented as the historical measured version.

## Resident original-color parent diagnostic

The [parent checkpoint](../../docs/performance/resident-parents-20261023.md)
qualifies only the recorded original-color parent maps, counts and ordered
records. It does not qualify children, objects, camera sources, world coordinates
or performance. The [factory/comparator](../qualify_native_parents.py) needs only
Python, NumPy and OpenCV; preparing CPU expectations does not need ROS or a
Vulkan installation. It uses the committed eight-witness fixture and explicit
full-mask `CCL_GRANA` per color. It refuses an existing fixture directory and
never generates expectations from a native/GPU output.

Run from the repository root with the reviewed Python environment:

```bash
task_parent_prefix=$(mktemp -d "${TMPDIR:-/tmp}/parent-diagnostic.XXXXXX")
task_parent_fixtures="$task_parent_prefix/fixtures"
python tools/qualify_native_parents.py prepare --output "$task_parent_fixtures"
python tools/qualify_native_parents.py verify "$task_parent_fixtures"
```

The seventeen cases include the eight literal witnesses, exact block-edge and
four-color cases, a 640×480 parent, 4096 isolated records, positive area thresholds
above image area, and invalid-mask/overflow negatives. PARENTQv1 stores a 48-byte
little-endian header followed by N U32 mask words. Calibration/configuration
values 17/23 and all source timestamps/tags are procedural test metadata.

Manually compile [qualify_parents.cpp](qualify_parents.cpp) against the exact
reviewed `resident_parents` and `resident_rgbd` libraries and their current
headers. There is deliberately no GPU CTest or new qualifier CMake target.
Choose the native compiler, SDK headers and actual Vulkan loader consistently
with that runtime. Pin and inspect the resolved libraries; a source tag does not
substitute for an ABI-consistent build or loaded-library evidence.

```bash
task_parent_cxx=${PARENT_CXX:-c++}
task_parent_libdir=${PARENT_LIBRARY_DIR:?reviewed native parent and Core libraries}
task_parent_sdk=${PARENT_VULKAN_INCLUDE:?reviewed directory containing Vulkan headers}
task_parent_loader=${PARENT_VULKAN_LIBRARY:?reviewed actual Vulkan loader file}
task_parent_loader_dir=$(dirname -- "$task_parent_loader")
task_parent_shaders=${PARENT_SHADER_DIR:?validated nine-entry parent shader directory}
task_parent_qualifier_sha=$(python -c 'import hashlib; from pathlib import Path; print(hashlib.sha256(Path("tools/performance/qualify_parents.cpp").read_bytes()).hexdigest())')
task_parent_module_sha=$(python -c 'import hashlib; from pathlib import Path; print(hashlib.sha256(Path("ros2/src/so101_station_vulkan/src/parents.cpp").read_bytes()).hexdigest())')
"$task_parent_cxx" -std=c++17 -O2 -pthread \
  -Iros2/src/so101_station_vulkan/include -I"$task_parent_sdk" \
  tools/performance/qualify_parents.cpp \
  "-DPARENT_QUALIFIER_SOURCE_SHA256=\"$task_parent_qualifier_sha\"" \
  "-DPARENT_MODULE_SOURCE_SHA256=\"$task_parent_module_sha\"" \
  -L"$task_parent_libdir" "-Wl,-rpath,$task_parent_libdir" \
  "-Wl,-rpath,$task_parent_loader_dir" \
  -lresident_parents -lresident_rgbd "$task_parent_loader" -ldl \
  -o "$task_parent_prefix/qualify_parents"
ldd "$task_parent_prefix/qualify_parents"
```

Keep the nine SPV hashes, `source.sha256`, compiler hash/version/flags and
`provenance.txt` with the runtime. Parent shaders need only Shader capability;
neither Float64 nor Int64 is an admission requirement. The native package can
build `resident_parents parent_spirv` in the reviewed station environment using
`SLANGC_EXECUTABLE` or a matching `PRECOMPILED_PARENT_SHADER_DIR`; compiler/cache
identity checks must remain enabled. Do not stamp an older library with the
current source hash without rebuilding/verifying its actual source identity.

This invocation parses and hashes the bounded fixture, without creating a
Vulkan Context. Its optional fresh prefix writes only `.offline.json`:

```bash
task_parent_case=four-colors-2x2
"$task_parent_prefix/qualify_parents" --offline \
  "$task_parent_fixtures/$task_parent_case.parentq" \
  "$task_parent_prefix/parser-only"
```

For a newly reviewed GPU run, construct this argument vector. This block does
not launch it:

```bash
task_parent_gpu_args=(
  "$task_parent_prefix/qualify_parents" --vulkan
  "$task_parent_fixtures/$task_parent_case.parentq" "$task_parent_shaders"
  "$task_parent_prefix/$task_parent_case-result" --external-watchdog-ack
)
```

Have the released, reviewed **external process-group watchdog** execute the
vector with a finite whole-process deadline, TERM/KILL/drain ownership, immutable
inputs, actual loader/ICD/critical-library pre/post seals and no-children checks.
The accepted protocol used 90 seconds per process and stopped on first failure.
The acknowledgment argument alone is not a watchdog. The qualifier's ten-second
source/transfer fences and parent polls cannot bound blocked cold driver calls or
teardown. Select the hardware ICD and loader for that child only; do not install
global overrides. Use fresh output prefixes. No camera, actuator, service or ROS
node is involved.

After the external owner has accepted a successful child, compare its entire
label map, counts and every ordered record against the independent CPU arrays:

```bash
python tools/qualify_native_parents.py compare "$task_parent_fixtures" \
  "$task_parent_case" "$task_parent_prefix/$task_parent_case-result"
```

Run all seventeen cases separately on each reviewed device/runtime. For either
negative case, `finish` and all computed-output accessors must reject, and no
label/count/record files may be published. Retain raw JSON, binary hashes,
comparison payloads and runtime/source identities. CPU fixture regeneration is
reproducibility support; it cannot replace the recorded GPU outputs with new
"GPU goldens". Diagnostic dense readback is outside the ordinary resident path,
and neither these runs nor modeled root-read reductions establish FPS or speedup.
