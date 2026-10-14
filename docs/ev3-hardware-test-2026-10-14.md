# EV3 hardware test — 2026-10-14

Bounded EV3 component tests and PC API checks passed on EV3 serial
`0016538184D5`, running ev3dev-stretch and Pybricks 2.0.0. These results do not
commission either full SO-ARM101/EV3 workflow. Actual cube exit was not verified:
the outlet was partly outside the camera view, and the loaded cube order and
queue were unknown. Disposal and full sorting therefore remain unverified.

## Components and observed results

| Component | Configuration | Observed result |
| --- | --- | --- |
| Ejector | Medium motor on A | Bounded home and ejection stroke checks below |
| Belt | Large motor on D | Settled traversal and repeat-home checks below |
| Touch sensor | S1 | Press and release passed |
| Color sensor | S3 | Red in 10/10 readings; reflection 31–38% |

The final diagnostic traversal settled at these target/actual angles:
`10 → 11`, `132 → 132`, `360 → 360`, `530 → 531` degrees. Final home repeats
were −1 and +1 degrees. The named range 10–530 degrees is the working span;
it was not established by forcing the far hard stop.

Ejector home used a positive bounded stall, reporting relative encoder values
185 and 197 degrees in separate programs, followed by a −200 degree backoff.
The +180/−180 degree ejection stroke returned within 2 degrees. These motor
observations do not establish that a cube exited.

Standalone physical diagnostics used native commands of at most 100 ms,
a 30% duty cap, bounded travel and deadlines, and quiet settling of at least
200 ms with a 1000 ms maximum. Slow switch repeats used 25 degrees/second and
a 25 degree backoff. An initial 5 degree repeat fault was retained as a failed
check; the final repeat error was 1 degree. Settling and slower repeat checks
were necessary before accepting the final repeat result.

## PC API and lease checks

- Four measured sensor/encoder RPCs returned in 31.08–63.10 ms.
- Heartbeats remained healthy for 2 seconds.
- A valid 100 ms D command moved +5 degrees and acknowledged `stopped=True`.
- An invalid 101 ms command was rejected before motion.
- Removing heartbeats confirmed agent lease/framing expiry with a configured
  500 ms lease. The SSH process finished after approximately 1015 ms; neither
  that duration nor the configured lease is a measured motor-stop latency.

Final root sysfs readback confirmed both A and D motors idle with empty
`.state` values.

## Software fixes and remaining gates

The tested API includes these committed corrections:

| Commit | Correction |
| --- | --- |
| `806133c` | Brickrun probe |
| `e102e91` | Motor role types |
| `fa38ea0` | stdout environment |
| `11970e6` | Pipe flush |
| `249c740` | Native chunk I/O |
| `7a69e57` | Batch parsing |

A fresh parent-main pytest run passed all 307 tests in 5.64 seconds.
This documentation update did not rerun those tests.

Full station cycles, loaded cube handling, disposal and complete sorting remain
pending. The saved arm home value **949** is below the calibrated minimum
**1063**. The existing preflight gate remains in force; these EV3 results do
not relax arm limits or commission that home.

Local videos, logs and evidence JSON remain under
`~/.local/share/exciting-science/station/evidence/2026-10-14` and are not included
in the public repository.
