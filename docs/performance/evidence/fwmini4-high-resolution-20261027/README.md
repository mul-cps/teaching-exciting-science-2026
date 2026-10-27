# Retained higher-resolution checkpoint evidence

Source checkpoint: `5344d665692ab2dddb0de32ef58c8adf6aedadb8`.

The unchanged `fwmini4-highres-live-20261027.tar.gz` has SHA-256
`bcb4b7b4f2b557ca619935673b29f77b31bf7fdad043634b6a792fd2d5023a08`.
It retains six raw live runs, native build stdout and Intel alignment logs.
`raw-archive-inventory.json` pins all 28 regular archive members. The publication
checked each against the supplied extracted bytes; no archive was regenerated.

Supplemental evidence includes AMD alignment logs, final operational state,
103-file native source-bundle manifest and publication-time commit verification,
native build/live runner scripts, the two compiled alignment SPVs with precise
compiler/provenance/hash records, three independent reviews, and CPU logs/JUnit
with all 464 before/after source pins. No ELFs, images, private calibration files
or credentials are included. Recorded library hashes identify binaries that are
not bundled. The native content bundle preceded the commit; its 103 source hashes
were checked against that exact Git commit after publication.

`cpu/workshop-highres-cpu-20261027.log` is the earlier direct-pytest attempt with
19 collection errors caused by environment mismatch. It is not a passing run.
`cpu/workshop-highres-cpu-canonical-20261027.log` records the two stale source
identity guards after the intended extent change. Both were fixed in tests,
retaining the exact helper hash/arithmetic/numerical checks.
`cpu/workshop-highres-cpu-final-20261027.log` and `.xml` are the final green
canonical run: 4333 passed, 133 skipped, 48 subtests passed. Skips remain pending.

`selfcheck-result.json` records the initial six-run check; `final-selfcheck.json`
records the complete eight-run/Core check. Both record artifact-only validation
and recomputed live
CPU/throughput/source-ID-window metrics. The live runner's `passed:true` means
collector/profile/allocation safety only. Every published performance and whole
station qualification flag remains false; no high-resolution run sustained 30 Hz.
The frozen alignment fixture logs are synthetic even though their execution used
native Intel/AMD Vulkan. The source-camera runs are physical capture but do not
qualify full geometry, markers, control or physical station acceptance.

To repeat the lightweight selfcheck without runtime work:

```bash
python docs/performance/evidence/fwmini4-high-resolution-20261027/selfcheck.py
```

It verifies published hashes, raw archive members, paired CPU pin maps, JUnit and
oracle summaries, SPV identities, and live-run arithmetic/gap counts. It launches
no ROS, camera, native qualifier, compiler or GPU process. The retained scripts
are evidence, not automatically executed. `inventory.json` lists every published
evidence file except itself and the generated `final-selfcheck.json` result.

See the [main note](../../fwmini4-high-resolution-20261027.md) for scope, metric
semantics and the unmet performance targets. Values in raw per-run JSON are the
authority; last producer status counters and host-busy/HWM snapshots use different
intervals from the unique-output steady window.

The unchanged second archive `fwmini4-highres-system-cpu-20261027.tar.gz`
(SHA-256 `604b489246680602737d030d57bef9b3413e19a3101ff26f514cac1ab4397126`)
retains both optimized system SDK2.58.4 CPU runs. Its internal `live/` prefix
belongs to this separate archive, not the scalar archive. The publication verified
all eight regular members against `live-system-sdk/` supplied extracted bytes;
`system-raw-archive-inventory.json` pins them. The SDK difference and sequential
inputs prevent a matched end-to-end causal comparison.

`recorded-core/summary.json` and its profiling script retain the frozen-input
Core probe only. Images, calibration and input metadata remain private on fwmini4.
Successful dense compare invocations are recorded before CPU/GPU timings;
20 measured frames/five warmups per timing mode, two/one for comparison. All live
measurements exclude RViz, so the requested actual GPU RViz30fps at all three
profiles remains pending.
