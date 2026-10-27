# Native RViz GLX swap probe (2026-10-27)

Private diagnostic implementation only. No GPU, display, camera, USB or SSH was
used for the retained tests. No repository source was edited. Worktree identity
at preparation: `feat/beginner-vulkan-20261022`, source checkpoint
`5344d665692ab2dddb0de32ef58c8adf6aedadb8`.

Build/test on the host with native gcc and installed GL/X11 headers:

```sh
cd /tmp/rviz-coexistence-probe-20261027
./build.sh
python3 test_probe.py
```

`libswap_probe.so` links only libc/libdl/pthreads; it neither links a GL driver
nor creates a GL context. To build only the actual probe:

```sh
gcc -std=c11 -O2 -Wall -Wextra -Werror -fPIC -shared \
  swap_probe.c -o libswap_probe.so -ldl -pthread
```

For a separately authorized native RViz run, prepend the actual probe to
`LD_PRELOAD` on the **RViz process alone**, retaining its normal ROS/display/Qt
launch environment. Do not preload `libfake_gl.so`, `libfake_minimal.so`, or
`libaudit_heap.so` into RViz. For example, with the normal native environment
already sourced, a reviewable launch shape is:

```sh
LD_PRELOAD=/absolute/path/libswap_probe.so rviz2 2>rviz-with-swap-jsonl.log
```

This launch was **not executed** here. stderr also contains ordinary RViz logs;
parse only complete JSON lines with `event` equal to `renderer`, `interval`,
`slot_limit`, `clock_error`, or `symbol_error`. Keep the ordinary logs too.

The wrapper resolves `RTLD_NEXT` once with `pthread_once`. Every intercepted
`glXSwapBuffers(display,drawable)` calls the resolved real function exactly once,
with the original arguments, before telemetry work. It does not change swap
interval, issue render commands, synchronize/read back GPU memory, or claim
physical display scanout. The measured event is return from the real swap call
followed by CPU bookkeeping: depending on the driver/swap policy this is render
submission or swap completion, **not physical display refresh or presentation**.
It also does not prove new camera pixels on every swap. A native renderer record
and camera-source unique-output telemetry must be evaluated independently.

The first observed swap for each `(Display*, GLXDrawable)` emits a `renderer`
record with actual current-context `GL_VENDOR`, `GL_RENDERER`, `GL_VERSION`,
`glXIsDirect`, and queried `GLX_WIDTH/GLX_HEIGHT`. JSON strings are escaped into
fixed buffers; strings beyond128 bytes, missing context/functions/strings, or
zero dimensions set `metadata_complete=false`. Such metadata cannot qualify a
hardware-rendered run. Renderer metadata describes that first context; the probe
does not track context replacement or drawable destruction/reuse. GLX only:
EGL/Vulkan swaps are outside its coverage; no records means no measurement.

A fixed16-entry table retains independent per-drawable counters. It never sums
rates across drawables. On a seventeenth distinct pair, a single `slot_limit`
error declares additional drawables untracked; all real swaps continue and the
first16 counters remain valid. There is no slot eviction or implicit aggregation.
The initial swap establishes the interval baseline; subsequent completions count
toward the interval. `total_swaps` includes the initial swap. An `interval` record
is emitted at the first swap at least one monotonic second after that drawable's
previous baseline. `interval_swaps / elapsed_ns` is the authority; `swap_hz` is
truncated to three decimals. Slow/idle drawables do not generate synthetic zero
rows; their next row covers the full elapsed interval. Dimensions are refreshed
only on that drawable's first record and interval records.

There is no hot-path heap allocation or per-frame metadata query. A bounded
CPU table scan, process-local mutex, and monotonic timestamp follow each swap.
The mutex serializes bookkeeping; the real GL call stays outside it. First-record
and once-per-interval metadata queries/JSON writes add diagnostic overhead and
are **not** included in the no-log microbenchmark. stderr logging is synchronous;
a blocked/full output sink can perturb the run. Use a normal local file, retain
failures and compare instrumented/uninstrumented runs if instrument effects matter.
Short/failed writes lose telemetry and must not be interpreted as success.

If the real swap symbol is unavailable, a single `symbol_error` with
`telemetry_valid=false` is emitted and calls return safely. There is no legitimate
operation to forward in that case; that process is not a valid RViz measurement.
Missing optional GL functions produce incomplete metadata without dereferencing
null pointers. A failed monotonic clock produces one invalid-telemetry error;
real swaps have already been forwarded.

`red.log` retains the failing behavioral test against a forwarding-only stub.
`green.log` / `host-test-results.json` retain the successful tests and five paired
500,000-call samples (10,000 warmups each). They exercise the **real compiled
LD_PRELOAD interposition**, fake native GL functions and real libc monotonic time
in the timed path; no graphics device is used. The heap interceptor counts all
malloc/calloc/realloc calls during measured loops. Assertions also cover logging
interval heap use, exact argument forwarding/call counts, independent drawable
and display keys, escaped metadata and dimensions/resizing, full table rejection
without corrupting a retained counter, missing metadata and missing required or
optional dlsym symbols. Static checks forbid GPU wait/readback and heap allocation
calls throughout the wrapper. The tests assert structural correctness, never a
flaky time threshold. Local timings are not fwmini4/RViz/driver performance.

See `host-test-results.json` for measured overhead; `sanitizer.log` for the
UBSan bounds-path run and `mutants.json` for rejection witnesses. `SHA256SUMS`
pins sources, scripts, host results and the built probe used for review. No
station/rendering/performance qualification is implied.

To repeat the full private host verification, including trapped-UBSan table
bounds and five deliberately broken variants, run `./run-tests.sh`. UBSan uses
trap mode to avoid a dependency on the unavailable system `libubsan.so` runtime.
The five mutants remove forwarding, merge drawable counters, substitute wrong
dimensions, add a heap call, or overrun the table scan; each is compiled and
rejected by the same behavioral/static tests. Core dumps are disabled for these
intentionally broken child processes. Rerunning writes fresh cost/log results;
regenerate SHA256SUMS if recording a new snapshot.

No local Ogre library/source was present in the inspected standard and native
ROS-Conda library directories. The probe deliberately does not interpose
`glXGetProcAddress` or its ARB variant: an Ogre path resolving a direct driver swap
pointer could bypass it. The actual RViz validation must observe renderer and
interval records for the intended drawable; absent records invalidate the probe
rather than establishing zero FPS. Do not sum an RViz auxiliary window with its
main viewport, and do not infer source-camera output FPS from redraw swaps.
