# Intel GPU color masks

The RGB-D object analyzer supports `color_mask_backend:=cpu` (default),
`vulkan` (SlangPy), and optional `opencl` (OpenCV). Select it through either
`objects.launch.py` or `station_live.launch.py`. This changes only RGB color
classification. The existing depth validity, support-plane, morphology,
connectivity, visibility, ROI, bounding-box and grasp gates still run unchanged.

Vulkan uses integer OpenCV HSV arithmetic, one RGB upload, one dispatch for
all four colors, and one packed readback of one byte per pixel. Device,
kernel, division tables and shape-matched input/output buffers persist in the
serial analysis worker. Shape changes replace the retained frame buffers;
images are bounded to 4096*4096 pixels. SlangPy 0.43.1 was tested on Intel ADL-N
with Mesa 25.2.8. No Float64 feature is needed. CPU/software adapters are refused.

There is no GPU import or context creation for the CPU default. Device setup
happens on the first requested GPU frame. Initialization or execution failures
permanently select CPU for that backend instance, recomputing the **same input
frame**. No previous image or detection is substituted. Restart the object node
to retry a failed device. Backend status appears in offline analyzer results
and the node's existing JSON report as `color_mask_backend`, with `requested`,
`selected`, `gpu_available`, device details and `fallback_reason`. `gpu_available`
records successful initialization; selection alone does not prove every OpenCV
operation ran on GPU. The offline benchmark additionally requires own DRM engine
counter activity.

A normal install needs SlangPy in the Python environment that runs the object
node and a hardware Vulkan driver. For fwmini4's existing isolated installation,
the tested SlangPy package is in
`~/.local/share/exciting-science/fwmini4/intel-venv/lib/python3.12/site-packages`.
The following explicit environment exposes it to the existing Conda wrapper;
use after installing this reviewed package into a separate ROS overlay:

```bash
export PYTHONPATH="$HOME/.local/share/exciting-science/fwmini4/intel-venv/lib/python3.12/site-packages${PYTHONPATH:+:$PYTHONPATH}"
export VK_DRIVER_FILES=/usr/share/vulkan/icd.d/intel_icd.json
station-fwmini4 ros ros2 launch so101_station_objects objects.launch.py color_mask_backend:=vulkan
```

The package directory also contains NumPy/OpenCV; keep this environment local
to the selected process. The implementation never edits driver discovery or
process environment at import. Missing dependencies truthfully produce CPU
fallback. No accepted fwmini4 install or running service was changed during
qualification.

Optional OpenCL uses one shared HSV conversion and downloads the four masks
only after classification. OpenCV owns its temporary buffer pool. For the
existing fwmini4 private ICD, discovery requires these explicit process values:

```bash
export OCL_ICD_VENDORS="$HOME/.local/share/exciting-science/fwmini4/intel-icd"
export LD_LIBRARY_PATH="$HOME/.local/share/exciting-science/fwmini4/intel-rootfs/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
station-fwmini4 ros ros2 launch so101_station_objects objects.launch.py color_mask_backend:=opencl
```

CPU remains the sensible default for fwmini4. In the recorded frame plus two
deterministic variants, packed Vulkan improved whole analyzer median time by
4.9–5.5% at 1920x1080, 1.6–1.8% at 960x540, and regressed 0.1–0.8% at 640x480.
This does not satisfy the >10% whole-analyzer gate for automatic GPU selection.
These measurements cover one recorded scene and synthetic brightness/translation
variants; they do not establish independent scene coverage, live rates or
physical station qualification.

Reproduce the paired, alternating-order benchmark (includes transfers, all
analyzer stages and output ownership) with the private Python environment:

```bash
VK_DRIVER_FILES=/usr/share/vulkan/icd.d/intel_icd.json \
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 taskset -c 0-2 \
~/.local/share/exciting-science/fwmini4/intel-venv/bin/python \
  tools/benchmark_intel_perception.py \
  --inputs ~/.local/share/exciting-science/fwmini4/current-20261021 \
  --output /tmp/intel-perception.json
```

The benchmark verifies frozen input hashes, compares complete analyzer outputs
exactly after removing only backend telemetry, refuses silent CPU fallback,
and records p50/p95 wall time, process CPU time, source hashes and DRM counters.
Use `--backend opencl` to compare the optional path in its documented ICD
environment. GPU initialization and first shader compilation are outside warm
latency statistics; each case has two warmups and nine timed repetitions.

## Candidates excluded after measurement

A bounded Float32 Slang depth projection/splat experiment reused the existing
`rocm_projection.py` calibration and compiled CPU occlusion/RGB completion
contract, and compared against the installed RealSense SDK oracle. XYZ matched
exactly, and maximum UV error was 3.58e-7, but half-pixel rounding changed
18/2/2 aligned depth pixels and 10/4/2 RGB samples at color resolutions
1920x1080/640x480/960x540. Explicit Slang `precise` compilation did not remove
the differences. One compensated Float32 division check also failed the exact
depth/RGB gate. The candidate was excluded; no projection code or relaxed
numerical thresholds are installed.

Nine alternating SDK/GPU diagnostic pairs also rejected projection on p95:
Vulkan 56.93/24.41/26.72 ms versus SDK 47.16/15.56/15.66 ms at those same
resolutions. The GPU measurements included upload, initialization/splat
kernels, packed readback, CPU compatibility and owned outputs; each paired SDK
sample was the warm second computation of the same frozen software-frame
cohort. These are stage diagnostics, independent of the accepted mask path.

Shared grayscale and support morphology were screened next. At 1080p CPU
RGB-to-gray cost 1.80 ms and the full-image 7x7 support dilation cost 0.63 ms;
at 640x480 they cost 0.28/0.12 ms. A Vulkan mask round trip cost 11.75/2.44 ms
respectively. Adding independent transfers for these small CPU stages is not
justified. Tags and board pose run in separate ROS consumers, so sharing a GPU
image between them would require additional transport/state ownership beyond
this small backend. Board-plane fitting already samples at most 2000 points;
object morphology runs on small component crops. Those stages stay on CPU.

Detailed artifacts (commands, hashes, both accepted and rejected measurements)
are kept outside Git under
`~/.local/share/exciting-science/station/evidence/2026-10-21-sorting/fwmini4-offload-20261021/`.
