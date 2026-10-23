# Software binary64: bounded GPU conformance and preparation — 2026-10-23

**The explicit software binary64 prototype passes primitive conformance and bounded preparation on both tested GPUs.** Each GPU matched 636,256 primitive output words exactly, and all sixteen preparation cases passed the unchanged 1e-12 coordinate limit and exact discrete contracts. GPU performance, default camera integration, full geometry and physical-station acceptance remain unqualified. The [aggregate evidence archive](software-binary64-20261023.json) records final source/artifact hashes and SHA256 of every original report used here; private runtime inputs and launch details are excluded.

## Arithmetic and integration boundary

The shared C++/Slang helper derives from Berkeley SoftFloat Release 3e FAST_INT64 logic. It implements binary64 add/multiply, binary32 widening, ordered less/less-equal and finiteness using integer carriers. Results retain fixed round-to-nearest/ties-to-even, gradual underflow, exact non-NaN bits including signed zeros/infinities, and canonical quiet NaN `0xFFF8000000000000`. Divide, square root, fused arithmetic, other rounding modes, exception-state equivalence and NaN payload propagation are outside this subset. The [official source documentation and license](https://www.jhauser.us/arithmetic/SoftFloat-3/doc/SoftFloat-source.html) describe the upstream integer requirement and specialization rules. Full BSD attribution and source provenance are [retained in the package](../../ros2/src/so101_station_vulkan/third_party/softfloat/NOTICE) and verified in installed binary materials.

Context now requests physically supported `shaderInt64` alongside supported `shaderFloat64` through the existing feature baseline. Synthetic host tests inspect actual device-create inputs and reported enabled features, including optional-video fallback. Those synthetic checks alone do not establish physical GPU execution; the separately accepted hardware results are below.

The existing three-argument preparation constructor keeps native Float64 as its default. Only explicit `GeometryPrecision::SoftwareBinary64` uses the new path and requires actually enabled Int64; Float64 is not required for that explicit mode. The separate shader build option defaults OFF. No camera wiring or precision auto-selection was added.

Software coordinates preserve the original eleven multiplications and nine separately rounded additions, bit-preserving calibration/arena ABI, ordered ROI predicates, validity thresholds, vivid exclusion, stride-eight sampling, scan order and reductions. Ordinary completion still reads 16 bytes of counts; three fixed resident arenas and existing lease/fence/fault rules remain. Source and host guards found no added ordinary-path allocation, wait or dense readback.

## CPU conformance and full verification

| Evidence | Result | Limit |
|---|---:|---|
| Independent native arithmetic harness | 5,294,430 assertions per ordinary run | Broad random/adversarial coverage, not exhaustive proof |
| Unmodified upstream TestFloat vectors | 93,528: 46,464 add, 46,464 multiply, 600 widening | Canonical NaNs; exception columns not compared |
| Actual shader coordinate/flag expressions compiled as C++ | 20,000 cases | CPU operation ordering and flags; no GPU dispatch |
| Final canonical CPU lane | 3,898 passed, 136 skipped, 48 subtests | Explicit optional-runtime/opt-in exclusions; upstream vectors executed |
| Final canonical native lane | 347 passed, zero skipped | Mandatory native coverage enforced |
| Fresh CPU CTests | 14/14 passed | Synthetic/native CPU boundary |

Tie-even and additive-packing correction mutants are rejected. An attempted UBSan run was blocked by an unavailable runtime library; sanitizer execution is unverified. CPU checks alone do not resolve compiler/backend execution on a GPU; the bounded execution evidence below addresses that separate gate.

The first full CPU attempt stopped at an existing static performance guard: it excluded only the first constructor body after a delegating overload was added. The authorized correction changed only that test to name the exact four-argument cold initializer, preserving every forbidden regex and hot-method check. Six in-memory allocation/wait mutations in submit, a helper and the delegating constructor remain rejected; targeted verification became nine passing tests. The final full lanes then passed. Both the initial interrupted RED and final GREEN reports retain separate hashes in the archive.

A fresh full package configure/build/install succeeded. All 17 old build SPVs and all 16 old installed SPVs are unchanged against their baseline; the seventeenth is a build-only divide diagnostic. A 526-file source seal including the test-only correction had zero before/after drift. Native production artifacts did not change during that correction, so the earlier successful build/install and fourteen CTests were retained. Later README/plan/evidence edits are documentary; all fifty GPU protocol code/test pins still match, and only the README documentary member differs in the broad build manifest.

## Bounded GPU conformance and preparation

The accepted result comprises one carried-forward V3 AMD primitive step and seventeen newly executed V4 steps: a fresh Intel primitive plus eight preparation cases per GPU. AMD primitives were not rerun in V4. The earlier Intel failure was not retrospectively accepted.

| Accepted hardware scope | Physical/enabled Int64 | Physical/enabled Float64 | Primitive result |
|---|---|---|---|
| AMD Radeon 8060S / RADV, 1002:1586 | true/true | true/true | 159,064 rows, ten batches, 636,256 words exact |
| Intel Graphics ADL-N, 8086:46d1 | true/true | false/false | 159,064 rows, ten batches, 636,256 words exact |

Both outputs have SHA256 `994850d825dbf49d19dbb5a5506e8321c033003c4267d2ece69f32b59aa72e2d`, matching the independent immutable native/TestFloat corpus with zero mismatches. The corpus SHA256 is `26c821b1363667135e56c2af24814e91ece715fe51a98348454f993d1a77b93c`; primitive SPV SHA256 is `a68c380a5558b6e3af5c7e6e541a0ddf654ae6e50b1e8998d6f5125918581d8a`. The archive records exact executable hashes and device/driver/API scope.

All eight unchanged CPU-comparator preparation cases passed on each GPU, including Intel without physical or enabled Float64. Absolute tolerance remains **1e-12 m, rtol zero**; maximum dense/selected error was **2.7755575615628914e-17 m** in distortion4/5/8, with zero error in the other five cases.

| Case, both GPUs | Valid pixels | Support samples | Maximum coordinate error (m) |
|---|---:|---:|---:|
| procedural | 12,288 | 63 | 0.0 |
| partial_grid | 13,017 | 69 | 0.0 |
| all_valid | 307,200 | 4,800 | 0.0 |
| all_vivid | 14,111 | 0 | 0.0 |
| hue_gaps | 3,420 | 9 | 0.0 |
| distortion4 | 14,111 | 80 | 2.7755575615628914e-17 |
| distortion5 | 14,111 | 80 | 2.7755575615628914e-17 |
| distortion8 | 14,111 | 80 | 2.7755575615628914e-17 |

Validity/vivid/ROI flags, selected-pixel membership/order, counts and arena byte sizes matched exactly. Every case also passed metadata preservation, source-fault/generation rejection, pending-source fence rejection, copied-lease/source and three-arena saturation, and dropped-source reclamation. No GPU-generated goldens were used.

All eighteen accepted steps preserved registered critical pre/post identities and accepted actual critical mappings. All children exited normally under an external 90-second watchdog with TERM/KILL/drain ownership; no timeout or kill was needed, and no staged GPU children remained. CPU-verified code, shader, corpus and executable identities stayed unchanged during the GPU protocol; elapsed diagnostic duration is not GPU performance evidence.

### Preserved protocol chronology

| Attempt | Accepted state | Stop or completion |
|---|---|---|
| V1 | Zero accepted; AMD raw arithmetic exact | Mesa cache data outside path policy; observed map archive lost before validation failure |
| V2 | Zero accepted; AMD raw arithmetic exact | Separate RADV builtin cache rejected; raw maps were saved |
| V3 | AMD primitive accepted; Intel raw arithmetic exact but unaccepted | Intel's actual DRM library lacked a critical prehash; no preparation ran |
| V4 | One accepted AMD prefix plus seventeen new accepted steps | Fresh strict Intel DRM pins, fresh Intel primitive and all sixteen preparations passed |

Earlier launch policies omitted benign mutable cache data and a critical DRM dependency. Record unrelated nonexecuting data separately, archive maps before validation, and keep executable/source identity rules strict. Correct a missing critical prehash before a fresh run; posthoc identity checks cannot retroactively accept an earlier failed attempt.

## Independent CPU refinement

The only arithmetic refinement returns exact multiplication by ±1 after existing NaN/infinity handling. Final paired measurements used actual ordered coordinate expressions, runtime inputs, seven trials per scenario, and three alternating paired runs. These are **CPU microbenchmarks, not GPU timing**.

| Scenario | Baseline ns/coordinate | Refined ns/coordinate | Change |
|---|---:|---:|---:|
| Arbitrary coefficients | 170.173 | 169.879 | −0.17% |
| Axis-aligned calibration | 77.710 | 67.840 | −12.70% |
| Dense rotated calibration | 142.654 | 143.136 | +0.34% |

The axis-aligned case improves about 12.7%; dense rotation and arbitrary coefficients are neutral at this resolution. Timed heap allocation count is zero. Coordinate SPIR-V grows from 192,728 to 195,608 bytes; other stages are unchanged. No universal speedup, GPU latency, register/divergence behavior or transfer-inclusive benefit follows.

## Artifact provenance and independent review

Independent correctness review found a P2 compiler/provenance invalidation gap: reconfiguration could relabel retained SPVs with a new compiler identity. The resolved implementation uses a content-stable compiler/offline-mode stamp, executable SHA/version/path and dependency tracking, plus the parent/helper composite hash. Published provenance/source digests wait for all stage binaries and sidecars to finish. Real compiler-counter tests cover no-op configuration, same-path compiler replacement, automatic regeneration, offline-to-compiler transition and helper-only stale-cache rejection. Independent follow-up accepted the fix. Multi-entry publication ordering was reviewed from dependencies; forced compiler-failure publication was not injected.

Final Slang 2026.19 software SPVs pass Vulkan 1.1 validation. Coordinates and scatter declare Int64; all five lack Float64 capability and 64-bit floating types.

| Stage | Bytes | SHA256 |
|---|---:|---|
| `geometry_coordinates.spv` | 195,608 | `1c2798cbfb9f4af876f64a483639ec440a51be7e7a1defef8e2b121cca1b98ac` |
| `geometry_support_offsets.spv` | 1,912 | `b208b13263698a1d8b1819ac91a69b47c0d34774a62c2c498d90f00d1762e3dd` |
| `geometry_support_scan.spv` | 3,808 | `9584291624a1231651fa56ac1829c1ea326f988f86eae59f004ea1d6b4510e2f` |
| `geometry_support_scatter.spv` | 3,644 | `79c69a96a5ad2213012b2290c3148539cb3e4ae71e2d76b7bc428443d6a8039b` |
| `geometry_valid_count.spv` | 2,224 | `c41ddcfdd3462963951db7b71730ffbc23bd96cad7aca1191b1b3f30b6120e99` |

The final helper SHA256 is `2cb3eae2dd92ca8a69234af8787e2c3a9ac28726fa440b11e6f6a4041cb96901`; geometry parent SHA256 is `5fd59968529786399cdddebfcf35f1a91c9348992ea084256b482376d06e5b20`. Composite source SHA256 is `b4d6c0ee7925b5b07a4ed1f21292805e151b81519faeb045d82645c597c74a11`. The archive records compiler, source, test and attribution hashes without private locations.

The separate primitive diagnostic and independent-oracle corpora were CPU-validated and resource-reviewed before execution. Its bounded reusable buffers, one active submission, own-fence deadlines and external watchdog requirements remained in force for the accepted two-device runs above.

## Remaining gates

GPU primitive bits and bounded preparation now pass for the two recorded device/driver scopes. Transfer-inclusive GPU processing, CPU demand and latency still require measurement before any default-backend decision. Full reports retain the unchanged 1e-9 numeric boundary plus exact arrays/schema and missing plane/contour/PCA/object stages; default-camera wiring, full objects and physical grasping remain unqualified.

Continue from the [maintained integration plan](../superpowers/plans/2026-10-23-native-geometry-integration.md), [native geometry gap](../native-geometry-qualification.md), and [resource-constrained offload strategy](resource-constrained-vulkan-strategy.md). The archive preserves the original CPU-only snapshot and independent review, every V1–V4 protocol/report digest, and the accepted V4 coordinator identity. Future changes must retain these source/artifact identities and qualification boundaries.

*Packaging note:* one extra trailing newline was removed from the BSD NOTICE and the installed copy was refreshed; the complete license text is unchanged. The archive records old/new NOTICE digests; accepted numerical code, GPU runtime pins and shader/binary identities are unchanged. The broad manifest now differs only in README documentation and this NOTICE formatting metadata.
