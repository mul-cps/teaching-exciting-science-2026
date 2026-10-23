# Intel prealigned Core after shared exact division — 2026-10-23

The corrected Core retains lower measured process CPU cost; recorded input sustains
30 Hz, while dense GPU input reaches 23.07 fps. This is the native **prealigned**
RGB8/Z16 Core subset. Capture, SDK/raw alignment, ROS transport, full geometry and
physical control are excluded. CPU means the same native Core reference path.

See the [archive JSON](intel-core-shared-divide-20261023.json),
[historical resident results](resident-20261023.md), and
[raw resident correctness and live-run chronology](raw-resident-20261023.md).

| Current measurement | Recorded CPU | Recorded GPU | Dense CPU | Dense GPU |
|---|---:|---:|---:|---:|
| Unpaced median trial p95, ms | 5.149 | 13.926 | 25.135 | 42.148 |
| Paced completed / 1,800 offered | 1,800 | 1,800 | 1,800 | 1,384 |
| Paced completed fps | 30.00 | 30.00 | 30.00 | 23.07 |
| Process CPU ms / completed output | 6.112 | 1.620 | 24.725 | 2.221 |
| Median trial p95 synthetic frame age, ms | 7.421 | 24.953 | 27.611 | 76.901 |

GPU process CPU cost per completed output is 73.50% lower for recorded input and
91.02% lower for dense input. Dense GPU trials completed 462/460/462 outputs,
replaced 413 pending frames in aggregate and ended with 3 frames in flight.
There were no slot drops or skipped source arrivals; repeated output was not counted
as delivery. Lower CPU demand per offered frame also reflects reduced dense throughput.

All 24 perception trials passed. All 12 paced compact-output checks and paired
control measurement/capacity guards passed; hot Vulkan allocations remained zero.
The separate 125 Hz synthetic control missed no deadlines or ticks (worst trial p95
lateness 0.310 ms, worst observed 0.389 ms). This is a scheduler proxy, not a safety proof.
Unpaced timing does not independently compare dense arrays or compact output.

Unpaced timing starts after host plane memcpy and uses 50 µs polling. Paced timing
includes plane copies, submission, 2 ms polling, finish and compact verification.
Age starts at synthetic arrival, not exposure. Percentiles are medians of trial p95s.
Process CPU uses CLOCK_PROCESS_CPUTIME_ID over aggregate 60 s steady windows per
mode/workload; it excludes IRQ/kworker and other processes and is not whole-host CPU.

Historical GPU unpaced p95s were 13.812/41.978 ms versus current 13.926/42.148 ms.
This is a descriptive historical comparison, not interleaved causal isolation of helper
cost. Host ABI changed; frequency and temperature were not controlled. No whole-camera
performance, dense GPU 30 Hz guarantee, physical safety or station qualification follows.

Source/runtime/input hashes and both original report hashes are retained in JSON.
Independent privacy review removed residual command arrays and PIDs from the supplied
aggregate, including its private command-manifest path; the manifest hash remains.
No calibration values, serials, UUIDs, environment or per-frame sample payloads are included.
