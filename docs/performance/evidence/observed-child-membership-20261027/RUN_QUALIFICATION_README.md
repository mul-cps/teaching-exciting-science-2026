# Explicit Observed GPU qualification runner

`run_qualification.py` is a private, stdlib-only **on-target** supervisor. Source
and production tests remain frozen. It does not SSH or launch camera/ROS/control.
No GPU run is authorized by this README; root launches after both build/gate proofs.

Use `--smoke` first for only scoped-640x480 normal input. A full run executes 28 ordinary corpus cases plus mutations 1–15 on
`scoped-640x480` (43 fresh child processes). Each child offers ONE unique
procedural source cohort. This qualifies Observed preparation from a completed
Frame fixture through actual Parent/Child/Geometry/Observed classes; it does not
qualify Core image conversion, a camera, full object geometry, world publication,
or throughput benefit. Original source IDs are preserved, never restamped as a
stream. JSON reports compact submit→finish wall/process CPU separately from
explicit dense diagnostic transfers; dense readback is intentionally outside the
ordinary production path. All normal and malformed cases must match the frozen
CPU oracle before acceptance.

## Execution location and watchdog

Run the script **on the machine owning the Vulkan process**. On fwmini4, copy
script + corpus there and start the runner on fwmini4. A timeout of local SSH is
not a GPU watchdog. Use an on-host launch with all launcher streams redirected
to on-host files (for example `nohup python3 ... </dev/null >runner.log 2>&1 &`).
The script starts every native child in a new session/process group, owns the
120-second monotonic deadline, sends TERM then KILL (default grace 2 seconds),
and bounds reap at 5 seconds. `waitid(WNOWAIT)` keeps the leader unreaped until
all group signals finish, preventing PID/PGID recycling from selecting another
process group. A normal child exit also cleans remaining owned descendants
before reaping. SIGTERM/SIGHUP/SIGINT handlers only set a deferred abort flag; they never raise
asynchronously during spawn or cleanup. The loop stops and cleanup completes;
stdout/stderr are regular files, never inherited SSH pipes. SIGKILL of the
watchdog itself cannot be contained; do not kill the supervisor externally.
No unbounded wait on an unkillable driver task is promised: an unreaped KILL
failure is recorded and the matrix stops.

On the GPU host, set one reviewed hardware ICD and the NEW isolated native
library prefix; do not inherit a different build's library path. Paths below are
arguments, so substitute exact build-worker receipt paths:

```bash
python3 /tmp/observed-children-native-20261027/run_qualification.py run \
  --executable "$EXE" --build-manifest "$BINARY_PINS_JSON" \
  --parents "$PARENT_CACHE" --children "$CHILD_CACHE" \
  --geometry "$GEOMETRY_CACHE" --observed "$OBSERVED_CACHE" \
  --precision software --corpus "$CORPUS" --results "$NEW_RESULTS" \
  --timeout 120 --grace 2
```

Intel: software Geometry + software Observed, enabled Int64. AMD: native Geometry
+ native Observed, enabled Float64 AND Int64. Parent/Child caches are unchanged.
Do not pass `--repo` on remote; remote execution performs exact identity/status
checks, and remains numerically unqualified until local comparison. On the local
pinned acceptance Python, `--repo "$WT"` may be passed to compare each child
immediately and stop on the first numerical mismatch.

`--dry-run` preflights and writes the full 43-command plan without spawning ANY
native child. It still requires actual accessible executable/library/cache/corpus
files and a fresh results directory. It is suitable before root authorizes GPU.

## Mandatory identities and artifacts

`--build-manifest` uses the resource worker's actual per-host receipt:
`{executable:{path,sha256}, libraries:[{name,path,sha256} x5], source_archive_sha256}`.
The runner verifies actual disk executable and all FIVE native library hashes
before execution and after each child. Each native report must contain exactly
those canonical library paths **and SHA values**, and the expected executable
SHA. It verifies qualifier/helper source hashes against the frozen corpus, and
all four cache source hashes, exact per-SPV hashes, and provenance against the
actual native report. A different library with a correctly shaped digest fails.
The build receipt binds the native source archive; corpus manifests bind oracle
sources and every fixture/expected artifact. Native C++ seals separately reject
wrong precision SPVs and source/build mismatches before Context creation.

A new results directory contains `run.json`, per-child `stdout.log`, `stderr.log`,
`process.json`, `watchdog.json`, and native `result.running`, JSON and diagnostic
arrays. A `.running` file is retained provenance, NOT proof the process remains
alive. Each completed trial records hashes of every artifact. Full remote matrices deliberately use POST-HOC numerical comparison on the
pinned local Python: numerical failure may be found after later GPU cases have
executed, and no numerical success is claimed before that comparison. First native
failure, deadline, identity/status error (or local numerical mismatch when using
`--repo`) creates `failure.json`, preserves all artifacts and stops. No retries,
expected-failure swallowing, or removal of failed runs. Expected malformed input
is a successful native exit reporting `unusable`, with no dense arrays.

## Local numerical acceptance

Copy completed remote results to a NEW local artifact directory unchanged. Then
run using the pinned local Python from the reviewed source worktree:

```bash
python /tmp/observed-children-native-20261027/run_qualification.py compare \
  --build-manifest "$LOCAL_COPY_OF_THIS_HOST_BINARY_PINS" \
  --repo "$WT" --corpus /tmp/observed-real-qualifier-corpus-20261027 \
  --results "$COPIED_RESULTS"
```

Comparison requires exact full43 or smoke1 coverage with no missing/duplicate
trials, the original corpus digest, and revalidates native executable/library/SPV
identity VALUES against the stored initial receipts after transfer. It checks
trial artifact hashes, then frozen retained exact
original membership, ordered records, inclusive ROI counts/flags, independent
closed memberships including overlaps, and compact counts. Geometry diagnostic
coordinates use the existing explicit 1e-12 tolerance; ROI/memberships never use
an epsilon. A failure stops immediately. Only `comparison.json` with 43 matches
is numerical acceptance; `execution_complete` alone is not. Corpus regeneration:
`python tools/qualify_native_observed_children.py prepare --output NEWDIR`
(optionally `--roi XLO XHI YLO YHI ZLO ZHI`; those same values travel to native).
The existing frozen corpus has 28 cases and no observation ROI. Separate
source-pinned ROI corpora can be run under the same bounded matrix.

## CPU-only validation

`python run_qualification.py self-test` launches only Python CPU children. It
checks normal exit, a TERM-ignoring timeout followed by KILL, a SIGHUP-interrupted
supervisor cleaning its child, signals at the spawn-return boundary and during cleanup (including repeated HUP),
and rejection of substituted library hash values.
`runner-self-test.json` records the last passing run. `runner-offline-plan/`
contains a dry-run plan using explicitly synthetic `/usr/bin/true` build receipts
and actual reviewed caches/corpus. It is not GPU or binary-build evidence.
