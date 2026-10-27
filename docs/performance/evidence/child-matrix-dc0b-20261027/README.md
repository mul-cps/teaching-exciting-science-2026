# Retained child-preparation matrix evidence — 27 October 2026

Both native hosts passed the same 58-case source/SPV-pinned matrix:116 stock
CPU exact comparisons,55 valid / 3 unusable each;27 typed valid empty/restore and
lifecycle checks plus2 typed unusable computed-view refusals per host. This is
child preparation only. `qualified:false` remains for the whole goal: no full
geometry, performance, camera/physical or adoption claim.

Source checkpoint: `0542ababa017cce482156c906a52d595a1a9cf50`.
Child shader: `dc0b1aa16967479cd85fcfeaad86e00ed193ef03017d009769732416ddb607c5`.
Stock qualifier: `01e45df35f5e32f2310884de61a0e566cdd2b4a85fcc31f82be65fd57bb5999e`.
Frozen manifest: `cbc585b0565f905e9a9f5993e43cb7b339b03e7120d416e7ac3a26f821e3a083`.

| Archive | Bytes | SHA-256 |
|---|---:|---|
| [intel-executed.tar.gz](intel-executed.tar.gz) | 180338 | `5bc6607b3f5465f9b32912910e14e2e19679e1f70f5f657d6caab2cc88439473` |
| [amd-executed-v2.tar.gz](amd-executed-v2.tar.gz) | 73534 | `ee1615cfd3a64c47ba677fbcc1b026ca152cc1c7f4df44853eb16cba618a3c19` |
| [common-reproduction-v2.tar.gz](common-reproduction-v2.tar.gz) | 226154 | `c4d368023e3c2191bb6be2d02bf898b8edbf270ef24ec6fe49a221ab5d8d13c2` |
| [historical-inputs-v1.tar.gz](historical-inputs-v1.tar.gz) | 77723 | `7240b13bb6c5b16afc27b0e3100ee91ee451c1074e590e2bad695331afe739b7` |

`intel-executed.tar.gz` is the exact root-retained archive; it contains all raw
outputs/watchdogs/native argv and 1122 output-file digests. `amd-executed-v2.tar.gz`
retains all AMD.v2 outputs plus detailed CPU compare data. Intel 198 / AMD 192
post-run pins are all true. Original native terminal records correctly say
`exact_comparison_pending:true`: those records predate the subsequent successful
CPU compares, retained in intel-comparison.json and amd-comparison.json. Their
status must be read together. Per-case output JSON and comparison digests are
also indexed in the main evidence JSON.

`common-reproduction-v2.tar.gz` contains frozen matrix/expected NPZ, parent and
child shader caches/provenance/digests, pinned synthetic source, host configs,
exact 58 native argv per host, native compile commands and corrected runner.
No ELFs, credentials or private calibration files are included. Host absolute
paths describe the recorded environments; rebuilding/rerunning requires the
corresponding native libraries, ICD and compiler profiles. The AMD command
retains the recorded process-local system DRM preload. These are separate
hardware/software profiles, not a timing comparison.

`historical-inputs-v1.tar.gz` is the original `7240b1…` archive with the old runner.
That runner precreated each watchdog output directory, causing the first case
to fail before GPU execution. Preserve it for provenance only. The final runner
is `runner/native-runner-v2.py` inside the common reproduction archive: it
requires a new case directory and lets watchdog90 create it. Existing `.running`
files are retained freshness markers, not evidence of a live process.

To verify the retained evidence with the pinned Python/NumPy environment:

```bash
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  /tmp/workshop-test-pinned313-20261027/bin/python \
  docs/performance/evidence/child-matrix-dc0b-20261027/selfcheck.py
```

The selfcheck hashes archives, rejects unsafe/ELF members, checks synthetic
source/shader/fixture pins and reruns the existing factory verifier plus all 116
stock comparisons against extracted raw arrays. It does not execute native
qualifiers or create a Vulkan context. `selfcheck-result.json` records its run.
No full CPU suite was rerun for this documentation/data publication.

The existing factory commands, from the source checkout, are:

```bash
python tools/qualify_native_children.py prepare --output "$FRESH_MATRIX" \
  --child-shaders "$CHILD_SHADERS" --parent-shaders "$PARENT_SHADERS"
python tools/qualify_native_children.py verify "$MATRIX"
python tools/qualify_native_children.py compare "$MATRIX" "$CASE" "$OUTPUT_PREFIX"
```

Use the retained `matrix/` unchanged to reproduce these exact comparisons;
creating another matrix is a separate preparation. For a native run, compile
using `compile/{intel,amd}-qualifier.json`, review the recorded host config and
invoke the corrected runner with a fresh output path. Its per-case argv uses
watchdog90 and stock qualifier `--external-watchdog-ack`; acknowledgment alone
is not a watchdog. The prior one-case and sampler histories remain in the main
note/JSON and are not rewritten as full-matrix evidence.
