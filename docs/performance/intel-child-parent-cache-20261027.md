# Intel child parent membership — 27 October 2026

The final parent-map and reducer repair passes all **58 frozen child-preparation
cases on both native Intel ADL-N and AMD Radeon 8060S**: 116 stock CPU exact
comparisons matched. This validates the frozen child-preparation matrix. Full
geometry, camera operation, GPU performance and physical station qualification
remain separate; no broader adoption decision is made here.

## Observation and change

The original shader (`8412aec1…`) reproduced fault 4. A reviewed, disposable
failure sampler captured pixel 0, root 1 and selected parent 0, with no capture
overflow and no other low fault bit. The unchanged 7×5 fixture selects parent 1
at that pixel. This locates an inconsistent value at the parent-resolution
boundary; it does not identify a driver or compiler defect.

The candidate resolves one-based parent membership during initialization and
stores it in the existing ninth N-word workspace region. Early graph consumers
read this immutable map through gather. Initialization's neighbor checks and
late packed processing retain source resolution: initialization cannot read
unpublished neighbor cache entries, and later scratch phases overwrite the map.
Gather checks zero and upper bounds before converting to zero-based record IDs.
No buffer, pass, descriptor, barrier, copy or capacity was added.

Separately, invalid reducer lanes now fault and skip their contribution while
joining the final workgroup barrier. The existing uniform fault exit remains.
This repairs a latent divergent exit; it does not explain the earlier hook
failure. Actual compiled control flow and an extracted 256-lane CPU phase
witness cover invalid membership, invalid root and parent bounds.

## Earlier frozen-case evidence

The cache-only shader `8dcc34ae…` and final reducer-repaired shader `dc0b1aa1…`
each ran once on Intel ADL-N with the stock native qualifier and original GCC
13 libraries. Each returned status `valid`, fault 0.
Roots, flags, records, statuses, packed data and counts all match the original
expected NPZ byte for byte. All 62 input/runtime post-checks matched.

The final shader also passed the same six-array comparison on Radeon 8060S
using the existing process-local system DRM compatibility profile. All 57 local
input/runtime checks matched. This is the same single-case boundary, with no
new AMD performance or full-backend qualification claim.

The fixture contains 24 connected pixels and minimum child size 32. Its expected
records and packed arrays are empty; this case exercises graph preparation and
rejection counts, not successful packing or final object geometry. The watchdog
did not time out. Process startup duration is not a processing benchmark.

| Artifact | SHA-256 |
|---|---|
| Frozen fixture | `e85eb99b89f9173a5ffd50f85638d8b5542dd482f90100d116012da9c368c3dd` |
| Frozen expected NPZ | `7188a6b5c8fcc2cb23ac0851607480f29fe970520eb8c75af8527fb813ab329d` |
| Failure-sample archive | `07d25197ae1b0a8e0261290bb98e10b4acc4327956d62e3d7a88e698c6222753` |
| Cache-only executed archive | `a24a6cffa0e5eb2b106859da93a4bd5ae72566ad9275f2e39e100c6b6ee5e858` |
| Final reducer-repaired archive | `a476ac83be9ad07752753c7a7153a4f7576312de56beddd33210572bca5d8943` |

The [evidence JSON](intel-child-parent-cache-20261027.json) retains exact input,
expected and output bytes plus comparison and review identities. Local executed
archives are under `/tmp/workshop-child-cache-executed-20261027` and
`/tmp/workshop-child-reducer-executed-20261027`, including
`exact-output-comparison.json`. Independent source/resource reports are under
`/tmp/workshop-child-cache-resource-20261027` and
`/tmp/workshop-child-reducer-resource-20261027`. The final reducer shader is
`dc0b1aa16967479cd85fcfeaad86e00ed193ef03017d009769732416ddb607c5`.
Its later native run has the same narrow frozen-case boundary.

## Full frozen child-preparation matrix

The final shader `dc0b1aa1…` ran the same source- and SPIR-V-pinned matrix on
both devices. Each host passed 58 exact stock comparisons: 29 literal-kernel and
29 production typed-parent cases, comprising **55 valid and 3 deliberately
unusable cases**. Valid outputs match roots, flags, records, statuses, packed
data and counts byte for byte. All 27 valid typed cases also passed the
empty/restore comparisons and lifecycle/retention checks; both unusable typed
cases rejected computed-view access and published no partial result.

Intel's 198 post-run input/runtime pins and AMD's 192 pins all matched. These
are the native GCC 13 Intel lane and the existing GCC 15.3 AMD/system-DRM
compatibility lane, respectively. The fixture and expected bytes were retained
unchanged. Matrix manifest SHA-256 is
`cbc585b0565f905e9a9f5993e43cb7b339b03e7120d416e7ac3a26f821e3a083`.
The source checkpoint is `0542ababa017cce482156c906a52d595a1a9cf50`; the
stock qualifier source remains `01e45df3…`.

The [retained evidence](evidence/child-matrix-dc0b-20261027/README.md) includes
both raw executed-output archives, all per-case comparison hashes, the frozen
matrix and expected NPZ files, both shader caches, exact native commands and the
corrected v2 runner. A CPU-only selfcheck verifies retained archives and reruns
all 116 stock comparisons without a GPU. The earlier v1 input archive is kept
as history: its runner precreated the watchdog output directory and failed
before GPU execution. It is superseded by the retained v2 runner.

## Resource checks and remaining gates

All 21 final SPIR-V kernels compiled and passed validation. The reducer repair
leaves the other 20 binaries byte-identical to the cache-only variant. Static
load counts in hook/jump/validate fell from 92/28/59 to 22/8/19 when introducing
the cache; these are compiler counts, not measured GPU speedups. The original
geometry predicates, fault policy and numerical thresholds are preserved.

The final isolated CPU run passed **4323 tests, 136 skipped, 48 subtests passed
in 180.08 s**, exit 0. All eight shader/test/fixture hashes matched before and
after the run. It used copy-based Python 3.13.12, NumPy 2.5.4,
opencv-python-headless 4.14.0.94 and SciPy 1.17.1 with the canonical
`tools/test_cpu.py` launcher. Skipped checks remain unqualified. Exact command,
pins and log/JUnit digests are in the evidence JSON.

The historical failure sampler still accepts only the original `8412aec1…`
source. Its tests now use that exact frozen fixture and explicitly reject the
current production shader; its provenance pin was not broadened.

The frozen child-preparation Intel/AMD matrix is validated. Transfer-inclusive
GPU timing, full geometry and camera/physical acceptance remain open. The
[native Slang CPU experiment](native-slang-cpu-20261027.md) is a separate
prealigned-core benchmark and does not measure this GPU change.
