# Intel child parent membership — 27 October 2026

The immutable parent-map candidate passes the original failing Intel case with
all six output arrays byte-identical to its frozen CPU reference. This is a
bounded child-preparation result; full geometry, camera operation, performance
and physical station qualification remain separate.

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

## Frozen-case evidence

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

Full frozen-matrix Intel/AMD validation and transfer-inclusive GPU timing remain
required before broader adoption. The
[native Slang CPU experiment](native-slang-cpu-20261027.md) is a separate
prealigned-core benchmark and does not measure this GPU change.
