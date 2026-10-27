# Canonical quality-profile RViz offline analyzer

Frozen scripts:

- `/tmp/quality-profiles-rviz-analysis-20261027.py`, source SHA
  `47c121dc75ac299340305af87cd34712ee5f3aeabd0b7fbfaff72b0a32bc9713`;
- `/tmp/test_quality_profiles_rviz_analysis_20261027.py`, tests SHA
  `d486095bc70fcf56e88d7c602e959ce268257d7a1757627c21df4b572ba7146e`.

Both scripts are portable siblings: tests derive the analyzer path from their own
filename and use normal temporary directories. No repository source/tests, SDK,
ROS runtime, GPU, SSH or USB were touched. Previous scheduler/resource proof stays
frozen. The exact earlier reviewed GLX, observer, perception, number/file/JSONL and
DRM helper function ASTs are unchanged (`preserved-reviewed-functions.json`);
only canonical profile maps and new producer analysis/integration were added.
Canonical names are realtime 640x480, medium 1280x720, and high_quality 1920x1080 RGB
with 1280x720 native depth for high_quality. Extents are not resized or inferred.

The historical `analyzer/original-test.py` contained 13 pre-review tests; the current
reviewed fixture contained 17. All 17 are preserved/adapted, plus 13 new producer
cases: 30 tests GREEN. `red.log` records missing producer-analysis behavior before
implementation. `green.log` and py_compile verify the final scripts. A packaging-
only final test delta replaces absolute analyzer/CLI/temp paths; assertions and
fixture arithmetic are unchanged. Negative fixtures cover numeric conservation,
occupancy, inter-snapshot regressions, stale-only/single-fresh/missing/malformed
fields, zero interval, stale corrupted snapshots, policy/profile/freshness,
skip decomposition, stage order and fresh snapshot age. CLI fixtures deliberately
plant an absurd runner producer rate; analysis still computes 30Hz intake from raw
payloads versus 2Hz collector output, proving those quantities are independent.

Reproduce on supplied immutable raw input:

```sh
PYTHONDONTWRITEBYTECODE=1 python /tmp/test_quality_profiles_rviz_analysis_20261027.py
PYTHONDONTWRITEBYTECODE=1 python /tmp/quality-profiles-rviz-analysis-20261027.py  --base /tmp/fwmini4-quality-profiles-rviz-20261027/raw  --output /tmp/quality-profiles-analysis-review-20261027/analysis.json
```

Exit 0 indicates structurally valid offline analysis, not qualification. Every
`experimental` payload must match canonical profile/requested-profile/backend/
alignment, latest_native policy, nonnegative integer counters and finite scalar
metrics. Numeric conservation is recomputed, not trusted from its text flag:

- received = received_unique + duplicate_or_regressed + invalid_received;
- received_unique = completed + skipped + active + pending;
- skipped = dropped + replaced + contention_discard + stopped_discard;
- active/pending each≤1;
- selected≥submitted≥computed≥published, and completed=published.

All cumulative counters and maxima must not regress, including intermediate/stale
samples. Every sample is checked; stale ones are counted but cannot establish rate
endpoints. Fresh requires literal `true` and snapshot age 0. At least two valid fresh
samples with strictly positive actual status receipt interval are required.

Unique intake is last-minus-first received_unique divided by that fresh-status
receipt interval. Producer published/computed completions use their own deltas in
the same independent producer-status window. These are distinct from actual unique
collector outputs over the collector window. Producer snapshot generation times
are not published, so status receipt timing is approximate and DDS delay is not
bounded here. Lifetime reported acquisition/computed/published rates and counters
are preserved separately; their original event-time denominators cannot be rebuilt
from one scalar field and are not silently substituted for snapshot delta rates.
No exact 30Hz assertion or inference from latency percentiles is made.

All six actual runs have valid numeric snapshots. Results:

| Profile/backend | Intake Hz, fresh snapshot delta | Unique collector output Hz | Status window s | Lifetime SDK ID gaps color/depth | Poll gap/duration misses |
|---|---:|---:|---:|---:|---:|
| realtime/vulkan | 30.0416 | 29.9869 | 14.1804 | 1/1 | 0/0 |
| realtime/cpu | 29.9996 | 30.0166 | 14.2002 | 0/0 | 0/0 |
| medium/vulkan | 30.0704 | 13.6323 | 14.2000 | 6/6 | 0/0 |
| medium/cpu | 29.2047 | 27.0412 | 14.1415 | 13/23 | 0/0 |
| high_quality/vulkan | 30.0986 | 6.1971 | 14.1203 | 11/11 | 0/0 |
| high_quality/cpu | 24.2931 | 14.9953 | 14.1604 | 83/84 | 0/0 |

Configured 30fps is nominal configuration, not an exact measured intake assertion.
In particular CPU high_quality intake 24.2931Hz and SDK gaps 83/84 must be retained;
all offered hardware inputs were not shown retained. SDK source-ID gaps are
separate from latest-native replacement/skips and from collector output-ID gaps.
Zero GPU poll-service deadline misses does not erase admitted-unique interarrival
misses or SDK gaps. Raw acquisition gap maxima/deadline counts, capture service
max-gap/max-duration/deadline counts, fresh/stale counts and lifetime counters are
retained in analysis JSON, not rounded away.

Received age remains host receipt to worker/publication (producer lifetime stats)
or host receipt to collector (collector p95), never exposure or processing latency.
GLX frequency remains per drawable, fully contained in each collector window,
not summed and not camera-output FPS. Observer CPU retains its separate nearest-
sample window and offsets; combined CPU is approximate. Qualification, execution
eligibility, station/geometry/control/physical/power acceptance remain false.

`review-proof.json` binds source/tests, 30-test result, raw archive SHA
`a9304d6692553944433a9b46b49938b85c672a5110ebd9cbadc08428e48b572f`
and all 58 raw file hashes unchanged before/after analysis. `analysis.json` contains
all 6 detailed results; `raw-input-sha256.json` binds inputs. Added coherence guards
left every actual analysis result unchanged, captured by comparison with
`analysis-before-coherence-guards.json`. No valid raw artifact was modified.
