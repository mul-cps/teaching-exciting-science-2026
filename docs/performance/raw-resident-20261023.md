# Raw resident bridge evidence — 2026-10-23

Recorded offline correctness is PASS; the subsequent live CPU baseline is RED.
This report imports existing evidence and records the outcome as of the 2026-10-23 snapshot.
See the [resource-constrained Vulkan strategy](resource-constrained-vulkan-strategy.md) for future work.
The [machine report](raw-resident-20261023.json) retains per-case counts, hashes,
compiler/module identities, CPU reviews, runtime device/driver identity and chronology.

Scope: branch `feat/beginner-vulkan-20261022`, recorded HEAD
`e515a5cc3f5659cd1375717d4a96cb4f2afd3d9a`; reviewed work includes uncommitted source.
HEAD alone therefore does not identify the tested binaries; use the source/SPIR-V hashes.

## Ownership and transfer contract

Typed source ownership lasts through the consuming Core fence.
Alignment and Core use two ordered submissions with distinct command pools.
Three leased slots bound latest/pending work; producer and consumer fences gate reuse.
There is no CPU aligned-depth roundtrip on this path.
Per cohort host uploads are RGB 921,600 B plus raw Z16 614,400 B.
Aligned depth copies 614,400 B device to device; producer readback is 4 B,
and compact Core readback is 163,856 B.
The CPU helper benchmark reports zero hot heap/buffer allocation and hot waits.
It uses a synthetic Vulkan API: SDK, RMW, real driver, GPU and host image copies
are excluded. These measurements establish helper behavior, not live CPU headroom.

## First RED, cause, and exact correction

The original Core XYZ counter reported 490 failing scalar coordinates on each GPU
under the unchanged `1e-7 m` gate, over all 307,200 float4 outputs, including points
beyond filtering. The causal analysis independently counted 490 distinct pixels;
each had one failing coordinate. The original counter iterates all four lanes per pixel.
There were 297 X and 193 Y errors above the gate; maximum error was
`2.384185791015625e-7 m`. All failing depths lay outside the component filter.
The native divide model matches every CPU bit; the reciprocal × numerator × depth
model matches every actual GPU bit. Extra rounding explains the failures.
AMD emitted reciprocal/multiply ISA was inspected; Intel has complete output-model
agreement, without an instruction-level inspection claim.
The shared exact helper fixes the arithmetic without changing inputs, thresholds,
CPU oracle, far-point output semantics or exception allowances.

Helper SHA256: `43281d1db2cc83f6d47cedcd56a838301030ce9449d4f99d860d66fd6c228cb3`.
Final Core convert SHA256: `86ff1501948b13ae8db84734d75edf8a33e5a5a32654ba939ec022095aae8319`.
Only this module changed among 17 SPIR-V modules; the other 16 remain identical.
Parent/helper composite identities and helper-only cache mutation guards are retained.
Slang 2026.19 used precise FP mode and O2; new shader GPU cost remains unmeasured.

## Final offline PASS

The shared-helper CPU review passed 13 CTests and 107 focused Python tests.
CPU eight-case fixtures and all 32 fixture/calibration/input hashes match the frozen set.
AMD Radeon 8060S / RADV Mesa 26.2.3 and Intel ADL-N / ANV Mesa 25.2.8 each
passed all eight SDK software-frame cases: mask, depth, XYZ, canonical membership,
morphology and compact output had zero mismatches; aligned Z16 and XYZ error were zero.
Each device also passed 65,536 exact divide bit cases and ownership/fence guards.
These correctness-run durations are not alignment/Core latency or FPS measurements.

## Later live Stage2 RED

Reviewed camera source SHA256 is
`b25cbbf69e4be2f110881192ecae7ec9457d4c08c6cdf0290b88fe69b36644db`.
It compiled and was reviewed; its provenance and install RPATH equivalence evidence
are retained separately from the offline math PASS.
The first CPU mode emitted one valid output, then rejected a present original depth
frame with ID 0 after prior depth ID 4. The source guard triggered unknown/restart-required.
The partial steady window was 0.496 s: it is failure evidence, not valid steady timing.
Strict Vulkan and auto were not run. The prescribed order stopped at the first failure.
The guard trigger is established; the SDK/startup cause is unproven. Warming or time-domain
transition remains a hypothesis. No source policy fix or successful retry is claimed.
All owned processes and camera handles closed; no actuator, service, canonical deployment
or main-branch change followed in this recorded run. No later diagnosis is represented here.

No physical geometry, motion or full-station qualification follows from these records.
Original report SHA256 values are retained in JSON. Private scene files, serials,
calibration values, samples, exposure timestamps, environment, PIDs, command lines and
GPU UUID-bearing keys are omitted; array/input hashes remain. Paths are normalized.
