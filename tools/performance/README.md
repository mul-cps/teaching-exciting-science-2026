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
records CPU checks, source/artifact identities and remaining gates. Performance,
complete geometry, default camera admission and physical operation remain unqualified.
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
