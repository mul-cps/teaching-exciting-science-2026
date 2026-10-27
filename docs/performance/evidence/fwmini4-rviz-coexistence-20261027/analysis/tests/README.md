# Offline RViz coexistence analysis

Only `/tmp/rviz-coexistence-analysis-20261027.py` and this private test directory
are owned by this task. No repository edits, GPU/display/SSH/USB or camera runtime.

CLI:

```sh
python3 /tmp/rviz-coexistence-analysis-20261027.py \
  --base /path/to/extracted \
  --output /path/to/analysis.json
```

Expected inputs for each vga/720p/1080p profile:

- `vulkan/live/PROFILE/vulkan.{json,runner.json}`
- `system-cpu/live/PROFILE/cpu.{json,runner.json}`
- Shared `logs/rviz-PROFILE.log` (ordinary logs mixed with probe JSONL)
- Shared `logs/processes-PROFILE.jsonl` (10Hz `/proc` sampler)

RViz persists through sequential Vulkan and optimized CPU runs. Each mode's
first `experimental` status with the expected backend/alignment establishes its
own approximate collector begin, plus that collector's reported steady duration.
Begin actually follows status receipt immediately; the exact microsecond offset
was not recorded. Same-host shared monotonic time is an explicit assumption.
Only GLX intervals fully contained within those inferred bounds are included;
no fractional overlap or mixing modes. Perception counts, source gaps, age
percentile, CPU and allocation stability are recomputed from retained raw data.
The collector's p95 uses sorted index `floor(.95*(n-1))`, not interpolation.

GLX rows are grouped by display+drawable and never summed. Primary is the largest
actual steady-interval area; every drawable retains dimensions, renderer metadata,
weighted rate, min/max interval rate, elapsed coverage and count. Zero observations,
zero intervals, missing/incomplete metadata, indirect/unverified rendering and
recognized software renderers are errors, not zero FPS or successful GPU evidence.
The swap event is not physical scanout and does not establish fresh camera pixels.
RViz displays actual `/station/resident/frame` camera RGB. Robot poses, world cloud
and brick markers are synthetic; sequential live modes do not have matched pixels.

Observer CPU uses nearest sampled endpoints, independently reporting actual
begin/end, offsets from collector bounds, and duration. PID and `/proc` starttime
must remain unchanged in every sampled row selected between those endpoints;
missing process samples or regressing counters invalidate the process sum. Named
PID aliases and the collector's perception PID are excluded from duplicate external
sums. Per-target CPU is `tick_delta / HZ / actual_duration / 4 *100`; CPUms and RSS
pages are retained. The sampler does not capture page size: derived MiB is explicitly
labelled an assumption of4096-byte pages, not silently read from this local host.
Per-client DRM engine counter deltas are retained independently; client disappearance
or reset is a diagnostic warning, never assumed zero engine use. Engine time is
not GPU wall time, control headroom, or power.

Host busy uses the first eight `/proc/stat` counters, subtracting idle+iowait.
It includes every host task, including the observer, unlike the target-process sum.
Sampler CPU itself has no recorded PID counter and is excluded from target sums.
Sampling-cost max/mean cover the selected accounting window and exclude JSON writes.
Row timestamps denote sampler iteration start; counters are read during that row's
sampling cost. Approximate combined CPU adds collector perception CPU to external
process CPU over different windows and is explicitly labelled approximate; each
component retains its own original window. No station, control, geometry, power,
physical or performance qualification is inferred. Output qualification stays false.

Exit0 means all six analyses are structurally valid; exit1 still writes errors and
available data for every run. Slower-than-target rates are measurements, not parser
errors. Missing files or bad safety/probe/accounting evidence are errors.

Verification:

```sh
PYTHONPYCACHEPREFIX=/tmp/rviz-analysis-tests-20261027/pycache \
  python3 -m unittest discover -s /tmp/rviz-analysis-tests-20261027
PYTHONPYCACHEPREFIX=/tmp/rviz-analysis-tests-20261027/pycache \
  python3 -m py_compile /tmp/rviz-coexistence-analysis-20261027.py
```

17 tests exercise actual CLI success/failure, independent mode windows in shared
logs, partial-interval exclusion, per-viewport FPS, correct CPU denominator and
host idle+iowait accounting, endpoint offsets, DRM deltas, PID replacement, missing
samples, duplicate PID aliases, software/incomplete metadata, zero observations,
invalid/overlapping intervals, changing allocations and missing input files.
`red.log` captures the test-first interface stub failure; `green.log` the final pass.
`existing-perception-check.json` independently reproduces the prior720p scalar GPU
raw collector arithmetic, including19.0914Hz,9.7453%totalCPU,20.4181CPUms/output and
82.543386ms receipt-age p95. It does not fabricate RViz/observer data for that run.

Independent review adds rejection of keyed mode/profile/extent mismatches, nonempty
collector errors and measurement receipt times outside the inferred interval.
No timestamp epsilon is used; all six actual runs satisfy the exact bounds.
