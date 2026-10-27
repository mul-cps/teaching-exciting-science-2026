# Three quality profiles and 30 Hz intake — 27 October 2026

**All three strict Vulkan profiles sustained approximately 30 unique RGB-D input
pairs/s while the same Intel GPU rendered RViz at about 31.25 swaps/s.** One
compute worker takes the newest waiting native cohort; slower processing replaces
that cohort before image copies, alignment or compute. The application retains
at most one active job and one waiting cohort. Frame skipping preserves bounded
work and full input resolution; it does not manufacture fresh perception results.

| Profile | Native RGB / depth | Unique intake Hz* | Fresh output Hz | RViz swaps/s | Perception CPU, % of four-core capacity | Receipt-age p95 ms |
|---|---|---:|---:|---:|---:|---:|
| `realtime` | 640×480 / 640×480 | 30.042 | 29.987 | 31.251 | 4.531 | 28.532 |
| `medium` | 1280×720 / 1280×720 | 30.070 | 13.632 | 31.251 | 9.925 | 106.456 |
| `high_quality` | 1920×1080 / 1280×720 | 30.099 | 6.197 | 31.249 | 17.275 | 196.249 |

All profiles request native RGB8/Z16 at 30 Hz, without image resizing. The default
is `realtime`; `vga`, `720p` and `1080p` remain aliases. These are six sequential
15-second collector windows on fwmini4's Intel N100 with a D435i at USB 5000 Mb/s,
not a ten-minute full-stack or control qualification. *Intake uses differences
between fresh producer counter snapshots divided by their status receipt interval
(about 14 seconds). That window differs from the collector's fresh-output window;
status delivery delay is not bounded. Approximately 30 Hz is the observed nominal
base rate, not an exact every-33.333-ms sensor guarantee.

| Strict Vulkan profile | Replaced waiting inputs* | SDK color/depth ID gaps* | Max capture service gap ms | Max capture call duration ms | Capture gap/duration deadlines >33.333 ms |
|---|---:|---:|---:|---:|---|
| `realtime` | 1 | 1 / 1 | 2.949 | 1.560 | 0 / 0 |
| `medium` | 231 | 6 / 6 | 4.827 | 0.968 | 0 / 0 |
| `high_quality` | 336 | 11 / 11 | 6.499 | 2.905 | 0 / 0 |

*Counts use the producer snapshot window. All Vulkan SDK ID gaps were already
present at the first nonzero periodic snapshot and then stayed constant. The
source watermark spans cold admission into normal capture; these observations
do not locate the missing hardware frames. SDK ID gaps, application replacements
and gaps between measured output IDs are separate quantities. Every recorded
snapshot in the three Vulkan windows conserved its numeric counters and had
active/pending occupancy at most one; contention, no-slot, malformed and
duplicate/rejected counts were zero in those three windows. Sensor input-spacing deadline counters are also retained: 2 ms
polling and sensor timing can produce intervals above 33.333 ms even while intake
averages 30 Hz. They are distinct from the capture **service** deadlines above.
Vulkan device allocation count remained 35; CPU mode remained zero.

RViz used direct `Mesa Intel(R) Graphics (ADL-N)`, with the actual resident camera
RGB through the FrameLease→Image DDS boundary. The 3D robot poses, 20k-point world
surface and brick markers are synthetic load. The largest actual drawable was
574×810; the RGB dock was 640×314. Each drawable's rate is measured independently,
never summed. Swap return is not physical scanout or proof of a fresh camera image
on every redraw. The unchanged scene/configuration/probe and the earlier separate
VGA UI inspection are documented in the [preceding RViz checkpoint](fwmini4-rviz-coexistence-20261027.md).

The explicit optimized CPU comparison remains useful, but **its higher-resolution
intake did not reach 30 Hz**:

| CPU profile | Unique intake Hz* | Fresh output Hz | RViz swaps/s | Perception CPU % | CPU ms/output | Receipt-age p95 ms |
|---|---:|---:|---:|---:|---:|---:|
| `realtime` | 30.000 | 30.017 | 31.246 | 10.150 | 13.525 | 14.755 |
| `medium` | 29.205 | 27.041 | 31.245 | 27.141 | 40.148 | 69.219 |
| `high_quality` | 24.293 | 14.995 | 31.286 | 28.158 | 75.111 | 120.483 |

The high-quality CPU run rejected three duplicate/regressed input cohorts; its
SDK color/depth ID-gap totals were 83/84, versus 13/23 in the medium CPU run.
These remain distinct from the intentional latest-input replacements.

Strict GPU used the qualified scalar SDK 2.58.3; CPU used optimized system SDK
2.58.4. Loaded library paths and hashes were captured from each live container.
These were sequential camera inputs, not equal-pixel replays or controlled-power
trials. Vulkan CPU ms/output was 6.044 / 29.122 / 111.505; at 1080p it uses less
CPU per second while completing fewer outputs and spending more CPU per output
than the CPU path. CPU occupancy alone does not prove uniform offload efficiency,
control-loop headroom, power savings or a speedup. The retained optimized-SDK
alignment incompatibility still prevents silently selecting its strict Vulkan
path; CPU fallback is visible and is not qualified for 30 Hz high-resolution
intake by this test.

Receipt age starts at native cohort receipt, including its wait before selection;
it is not exposure age or pure processing latency. The previous single-thread
baseline received cohorts at a different scheduling point, so its age percentiles
are not a controlled latency comparison. The JSON retains host CPU, per-process
RSS and DRM counters. Perception-plus-graphics CPU sums use nearby but different
accounting windows and remain explicitly approximate. No fresh-frame FPS is
inferred from unrelated latency percentiles.

The implementation is source checkpoint `032fd17e4dcc4ac7f6fcb7ed12f53b944370e276`.
All 105 native snapshot files match it byte-for-byte. The full canonical CPU lane
passed **4341 tests, with 134 skips and 48 passed subtests** in 216.47 s. One test
was strengthened during that run to extract the exact production timer callback;
production and the other 495 pinned files stayed unchanged, and the final exact
handler test passed separately. The extra skip was the optional offline shader
toolchain check; it passed separately with explicit pinned Slang/SPIR-V tools.
The other 133 skips retain their CPU-lane scope. All 14 native host tests passed
on fwmini4 after the 39.9-second isolated build.

Before any GPU validation, independent review passed helper/resource guards,
allocation and lock benchmarks, rejected mutations, token/restart tests and
ThreadSanitizer lifecycle/first-error/concurrent intake-worker-observer checks.
The helper intake benchmark was about 57 ns with zero helper heap allocations;
this excludes SDK getters/polling, ROS/DDS, image copies, driver and GPU costs.
Failure closes the publication session first; UNKNOWN delivery is best effort
under DDS/OOM failure and is locally ordered on the shared measurement writer.
It does not retract committed messages or guarantee ordering across topics.

The isolated camera/graphics processes were stopped after the runs. The existing
station owner remained active with the same PID. Production launch, full geometry,
marker registration acceleration, video, control soak and physical sorting remain
unqualified; the full workshop release is still open. `qualified=false` and
`execution_eligible=false` remain explicit.

[Machine results](fwmini4-quality-profiles-20261027.json) ·
[Raw evidence, independent reviews and reproduction](evidence/fwmini4-quality-profiles-20261027/README.md)
