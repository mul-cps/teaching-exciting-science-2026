# Three-profile capture and frame-skipping evidence

Source checkpoint: `032fd17e4dcc4ac7f6fcb7ed12f53b944370e276`.
The [readout](../../fwmini4-quality-profiles-20261027.md) and
[machine results](../../fwmini4-quality-profiles-20261027.json) distinguish nominal
30 Hz intake, unique perception outputs, capture service deadlines, SDK ID gaps
and application skips. This is an experimental camera/graphics checkpoint;
`qualified=false` and `execution_eligible=false` remain explicit.

- `raw/`: exact 58-file native runtime archive contents: collector/runner JSON,
  launch/GLX/process-sampler logs and graphics process identities. Original archive
  SHA-256 is `a9304d6692553944433a9b46b49938b85c672a5110ebd9cbadc08428e48b572f`.
  Images, private calibrations and credentials are not included.
- `context.json`: commands, source revision, SDK/runtime and test boundaries.
- `checks/`: canonical CPU log/JUnit, source pins before/after, final handler and
  offline shader supplements, isolated native build and 14-test native CTest log.
  The one test-only change during the canonical run and its separate final check
  are explicit; the source held stable.
- `author-checks/`: focused checks and source pins.
- `resource-review/`: independent pre-GPU helper/gate benchmarks, UBSan and
  rejected mutations. Zero allocation claims cover the measured helper/gate only.
- `integration-review/`: independent source, lifecycle, token and ThreadSanitizer
  proofs; TSan does not model the acquire-fence coherence argument.
- `analysis-review/`: analyzer author tests and raw-input inventory.
- `evidence-review/`: independently recomputed raw metrics and provenance.
- `scripts/`: original hardware build/launch/measurement scripts plus the portable
  offline analyzer and its 30 tests. Hardware scripts bind historical isolated
  prefixes; use new output prefixes for another experiment. The unchanged RViz
  scene, assets, probe and reconstruction are in the
  [preceding checkpoint](../fwmini4-rviz-coexistence-20261027/README.md).

From the repository root, verify the published files/source pins and independently
recompute the complete analysis:

```bash
python docs/performance/evidence/fwmini4-quality-profiles-20261027/selfcheck.py
python docs/performance/evidence/fwmini4-quality-profiles-20261027/scripts/test_quality_profiles_rviz_analysis_20261027.py
```

The selfcheck reads artifact files, reads the pinned source commit through Git,
and runs the offline analyzer. It does not import ROS, open devices, execute GPU
commands or move hardware. Git must contain the measured commit. Later working-tree
changes are not checked or qualified by this historical report. File hashes are bound
in `inventory.json`; the inventory excludes itself. Native source pins also bind
all 105 files captured for the remote build to the source checkpoint, rather
than to whatever code is currently under development. The mutable general
resident guide is outside this dated evidence inventory. Absolute
`/tmp` paths in original reviewer reports identify their original scratch roots;
the matching reports and source/log files are preserved in the directories above.
Original reviewer SHA256SUMS may include intentionally omitted compiled
executables; the package inventory is authoritative. Private harness sources and
logs are included, including failed diagnostic-tool attempts alongside final
passing runs.

The actual GPU-backed private Weston/RViz run used native ROS Jazzy, domain 179
and Fast DDS, D435i serial 243322071783 over USB 5000 Mb/s. GPU loaded isolated
scalar SDK 2.58.3; explicit CPU loaded optimized system SDK 2.58.4. Library hashes
come from live process maps. All graphics/camera children were stopped afterward;
the existing station owner remained active. No physical actuation was performed.

Fresh-counter status timestamps approximate sampling time; their DDS receipt
latency is not bounded. The collector's output window is separate. Ordinary
input-spacing deadline counts are nonzero even when the dedicated capture
service has zero gaps/durations above 33.333 ms. CPU fallback medium/high-quality
intake is below 30 Hz. GPU 1080p uses less CPU per second but more CPU per completed
output than CPU mode. These limits are retained, not converted into pass claims.
