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
