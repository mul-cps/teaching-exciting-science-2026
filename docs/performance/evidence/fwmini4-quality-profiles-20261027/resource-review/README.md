# Latest-native final candidate phase 2 resource review

Scoped result: PASS for the helper/gate/capture resource boundary. Full lifecycle,
TSan/race review, canonical CPU run and future native execution remain separate.
`review-proof.json` pins the reviewed production source and owned regression test.
No production source was edited; only the root-authorized owned perf test changed.

The actual capture loop only polls its SDK frameset, records receipt/provenance,
validates original metadata and hands off an SDK handle using scheduler try_lock.
Its sole explicit periodic wait is sleep_until(started+2ms), independent of worker
progress. No frame-plane copy, alignment, bridge, DDS, publication-gate mutex or
snapshot occurs on that path. All normal plane copies and compute/publication
remain on the worker; active ownership clears after the final frames publisher
returns. Failure claims use atomic CAS and fast token revocation; the previous
worker-held error/gate mutex is absent from capture failure recording.

This is an own-code resource statement, not a full capture zero-allocation claim.
Actual SDK poll/getters/profile/calibration queries and ROS clock calls can have
external allocation/locking costs; they were not executed or measured here. Fixed
metadata strings are reserved cold. SDK handles released by the outer capture
scope, including displaced handles, are outside the handoff critical section.
The move-only proxy detects zero owned releases inside scheduler operations.

`try_snapshot(result,8)` is diagnostics only. While a real worker holds its writer
epoch, 100k attempts return false, preserve the caller's prior fields and allocate
no heap. Final report_status uses exactly 8 retries, retains last good counters,
and emits explicit freshness/age. The unbounded snapshot convenience method still
exists for host tests; static capture guards forbid both snapshot methods there.

Focused original+independent helper coverage: 6 tests PASS in 1.90s. New durable
static guards cover whole capture helpers, gate revoke and the explicit polling
cadence; 9 capture/gate source mutants plus 6 intake mutants are rejected. A further
static max-store guard rejects relaxed payload stores: RED on the original source,
GREEN after the approved release-store fix. TSan alone does not model this fence
ordering contract.
Existing 300-input 30Hz overload simulations (80/180ms), worker-blocked publication,
100k contended handoffs and 100k zero-heap/RAII cycles pass on helper5c09437e.
A destructive-handoff binary mutant is rejected after 715 owned releases inside
scheduler calls. Actual helper/bounded-observer and gate pass trapped UBSan.

Final host sample, 100k operations each (local uncontrolled CPU, no threshold):

- intake replacement 56.57ns; duplicate 28.87ns; stopped skip 38.59ns;
- dispatch 54.89ns raw; no-slot skip 38.23ns raw; clock baseline 18.02ns;
- full helper cycle 157.94ns; contended intake 39.88ns;
- bounded 8-attempt stale observer 2.82ns (separate sample).

Batch helper costs include fixed native-proxy caller bookkeeping. Small
per-operation raw windows include one clock-sample overhead. They do not measure
SDK, alignment/Core, polling, ROS serialization, actual DDS or target-host timing.
Ordinary new/new[] are intercepted; aligned/C allocation APIs are not intercepted.
Static guards separately deny explicit heap/copy/wait/serialization additions.

## Actual production publication gate

`gate_bench.cpp` includes the actual raw_camera_admission header and links the
actual raw_camera_admission.cpp with function/data sections and linker gc-sections.
Only gate methods survive. `gate-runtime-libraries.txt` shows libc/libstdc++/libm/
libgcc only: no RealSense or Vulkan runtime. Headers are permitted, but no SDK,
GPU, USB or DDS operation runs. Build flags treat SDK includes as system headers
and suppress only existing aggregate missing-field-initializer warnings from the
shared Vulkan/SDK declarations; other enabled warnings remain errors.

A real worker enters gate.publish and blocks in a host CV callback while retaining
the gate's writer mutex. Capture-side revoke still succeeds; 100k subsequent revoked
token calls complete with no heap. A final UNKNOWN waits behind the entered callback,
then runs in order, and no new stale callback enters after revocation. Old tokens
cannot revoke a new session. This is a blocked callback/DDS stand-in, not real DDS.

Successful CAS raw mean 21.87ns versus 18.15ns clock baseline; repeated revoked-token
calls while the publisher retains the mutex average 4.07ns. One first revoke while
blocked took 170ns (one-shot, not a robust latency statistic). Literal-zero token
calls are compiler folded and their reported near-zero loop cost is not a measured
function-call latency. The blocking-revoke mutant adds the mutex to revoke; it
compiles but hits the 2s watchdog behind the deliberately blocked publisher. Actual
source finishes before that callback is released. This proves the mutex-avoidance
witness can catch the dangerous regression.

Reproduction from the named worktree:

```sh
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
ROS_DOMAIN_ID=177 ROS_LOCALHOST_ONLY=1 \
LATEST_NATIVE_REVIEW_ARTIFACT_DIR=/tmp/latest-native-final-resource-review-20261027 \
/tmp/workshop-test-pinned313-20261027/bin/python -m pytest -q \
 tests/test_latest_native_scheduler.py tests/test_latest_native_scheduler_perf_review.py

c++ -std=c++17 -O2 -pthread -Wall -Wextra -Werror \
 -Wno-missing-field-initializers -ffunction-sections -fdata-sections \
 -I ros2/src/so101_station_vulkan/include \
 -isystem /home/bjoern/miniforge3/envs/so101-jazzy/include \
 /tmp/latest-native-final-resource-review-20261027/gate_bench.cpp \
 ros2/src/so101_station_vulkan/src/raw_camera_admission.cpp \
 -Wl,--gc-sections -o /tmp/latest-native-final-resource-review-20261027/gate-bench
/tmp/latest-native-final-resource-review-20261027/gate-bench
```

Proof files include focused-helper.log, microbenchmark.json, bounded-observer.json,
gate-microbenchmark.json, gate-runtime-libraries.txt, both UBSan outputs and two
compiled mutant records. No GPU/SSH/USB, full canonical suite, stage or commit was
performed by this reviewer. This gate does not qualify 30Hz physical capture,
full perception/control/geometry or deployment.

Final frozen source identities: camera25de28160ae268d1959e405d668a699d6ede401df038bfb074baa7c26257dc87,
helper5c09437ee2fc2b3b14b46023106ed92692c296876fc66c70addafdd24b932a21.
All eight capture/worker/status resource bodies are byte-identical to the preceding
reviewed camera0ddd candidate; its additional handler hardening is independently
reviewed by the lifecycle/race reviewer. The only helper change after the prior
resource sample is the approved maxima release-store. Final6focused tests and
rebench bind the new hashes. The owned test is frozen34b941b9e92eb8d0222454185be0f3866b6d34a46831e9e626153eb457e55b5f.
