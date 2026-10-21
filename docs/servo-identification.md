# Offline servo identification

`tools/servo_identification/analysis.py` reads saved trajectories and encoder logs.
It does not import ROS, serial drivers or a hardware probe. The separate pure
`workshop.station.dynamics_compensation.velocity_lag_candidate` function exports
an **unvalidated offline candidate**; no controller calls it. Defaults are disabled
and zero. The current evidence does not identify a reusable lag coefficient.

## Reproduce the bounded analysis

From the repository root, using Python with NumPy:

```bash
E=/home/bjoern/.local/share/exciting-science/station/evidence/2026-10-21-sorting/speed-qualification
L=/dev/shm/so101-sorting-20261016/rviz-arm-logs
python tools/servo_identification/analysis.py \
  --run "$E/control50-elbow-lift-5deg" \
  --run "$E/native320-v187-out" \
  --run "$E/native320-v187-return" \
  --owner-log "$L/arm-session-6b45c02f4aa341058590c30b90744eb4.jsonl" \
  --owner-log "$L/arm-session-b6f33ade0bf2454b9f8aff3d8d9335d1.jsonl" \
  --calibration /home/bjoern/git/soarm101-highschool-demos/.calibration/hs_rig01_follower.json \
  --historical-probe /home/bjoern/git/teaching-exciting-science-2026/artifacts/choreography/base-system-probe-2026-10-13-01 \
  --output /tmp/servo-identification-NEW-RUN
python -m unittest discover -s tools/servo_identification -p 'test_*.py' -v
```

The output directory must be new. `analysis.json` contains metrics, exact file
hashes, frozen complete-line owner-log prefix sizes/hashes, source/calibration/
owner/goal/path bindings, and an explicitly disabled compensation candidate.
`samples.csv` contains per-joint encoder/held-command/residual/ripple observations.
Archive the corresponding owner-log prefixes before RAM logs disappear. A growing
log produces a different prefix hash on a later run; the recorded byte count
identifies the exact analyzed prefix. Missing or mismatched evidence is listed in
`unavailable` and exits with code 2, while other valid runs are still analyzed.

`evidence-20261021.json` is the compact first result, retaining provenance and
summary metrics. Full JSON/CSV for that result were written to
`/tmp/servo-identification-verified-20261021`. The rejected `pan2-v187-out`
was also requested: it has no terminal controller artifact, so the tool refuses
to substitute feedback from another run. Its original report and the root's
tracking stop remain the evidence for rejection. Do not re-enable that profile.

## What the measurements mean

The bound reader requires one matching preflight and complete feedback event,
exact owner session and source hashes, calibration file hash, effective-path
hash, commanded sample grid, and raw targets within one encoder count of the
preflight. It includes trajectory ticks only, excluding post-path settling.
It reports both directions separately. Physical acceptance remains pending.

Residuals use encoder position minus the most recent quantized target written to
the bus. At 50 Hz, actual write-start and encoder-read-completion timestamps are
available; cadence comes from these observations. Old 20 Hz owner logs lack
actual write timestamps: their residuals pair each encoder observation with its
associated target, and command cadence and lag are **unavailable**. The nominal
20 Hz grid is not reported as measured write timing.

Positions use the station's calibrated midpoint and `360/4095` conversion
(`workshop/station/arm.py`, `pose`); the nominal sensor quantum is slightly
different. One count is about 0.08791 degrees. The stationary-interval and
stick-then-jump counts describe quantized observations; they do not prove friction.
Full-path cubic detrending is a descriptive residual, including nonsteady path
shape; it is **not** a steady-state 2–20 Hz band RMS or a physical vibration model.
Lag is a bounded fit of a delayed held target without offset compensation. It can
absorb friction, static error and profile dynamics and is not a fitted servo law.

A frequency candidate is produced only for an explicitly recorded cruise window,
or the historical probe's planned cruise after ramps plus 0.2 s guards. The fit
uses actual irregular observation timestamps and includes cubic drift. It does
not interpolate encoders onto a fictitious regular high-rate grid. It reports
window duration and `1/duration` frequency resolution, requires at least 30
observations and 1.5 s, and limits the search to `min(20, 0.4/max_gap)` Hz. Thus
20 Hz feedback does not produce an alleged definitive 8.7 Hz identification.
There is no anti-alias filter proof; even valid in-band candidates can alias
higher-frequency motion. Values below a uniform quantization reference withhold
a peak; that reference is not a measurement of noise.

### Verified first observations

| Saved condition | Observation | Limit |
| --- | --- | --- |
| Current 50 Hz, elbow 5 degree lift | Written-goal peak error 1.934 degrees, RMS 1.374; writes 19.743–20.228 ms | One short rest-to-rest move; no steady cruise or independent validation trajectory |
| Earlier 20 Hz, base outward/return | Base written-goal RMS 1.313/1.244 degrees; descriptive detrended RMS 0.294/0.286 | Source/profile/pose differ from elbow run; operator saw jerk despite endpoint success |
| Historical 125 Hz, base 3 degrees/s | Irregular-timestamp sinusoid candidates 8.706/8.669 Hz, positive/negative | Single old pose/profile, no owner/source identity, bus midpoint timestamps |

The current reported safety gate used the ideal spline target and reached
1.957 degrees peak and 1.484 degrees endpoint error. The offline written-goal
peak differs because raw goal quantization and analysis scope differ. Keep the
original safety gate. The current descriptive lag fit is about 0.402 s on only
1.64 s of motion; **do not turn it into compensation**. No matched independent
whole-trajectory train/validation data exists. All exported coefficients are zero.

The old 125 Hz result independently supports the historical frequency claim in
[arm-smoothness-analysis.md](arm-smoothness-analysis.md). It does not transfer the
old gain/startup-force trial effects to today's 50 Hz owner. Those trial tables
remain historical reports; this analysis did not recover independent raw traces
for their individual settings trials. No new torque-saturation, natural-frequency,
gravity, backlash or universal speed-floor claim follows from these data.

## Small optional root-executed experiment

`tools/servo_identification/experiment-plan.json` is a sealed, non-executable
protocol. It records the archived 50 Hz baseline, a small optional A-B-A comparison
for **one** chosen factor, both directions, restoration and stop requirements.
It is blocked until the root records a fresh admitted pose/path/load and coverage;
it is not authorization to actuate. There is no automatic tuner or matrix runner.
The archived baseline is native body/jaw velocity 320/220, acceleration 8,
Torque_Limit 700/250, with the separately owned elbow P24 journal. Base P16/D32/I0
is preserved unless the root explicitly executes a separately journaled trial.

Record a `load_pose_id` shared with the external observation tool, run/goal/owner,
encoder clock domain and timestamp uncertainty, camera/mount calibration and
external angle/uncertainty. In `--conditions`, run labels can additionally specify
`pose_id`, `load_id`, `path_id`, `profile_id`, and `cruise_window_s`. Comparison
matching also checks actual source/calibration/native settings/period and endpoint
pairs. Unknown conditions stay unmatched; no automatic treatment-effect aggregation
is produced. Whole-path holdouts must use different executions, not randomly
split samples from the same trajectory.

For any later velocity-lag candidate, supply explicit per-joint velocities,
nonnegative lag coefficients and correction bounds to the pure helper. It keeps
stationary endpoints and clips each correction. Re-run **all existing dynamics,
collision and effective-path hash validation on corrected samples before any
execution**. Bounds alone do not establish safety; never insert compensation after
validation. No gravity/backlash/notch coefficients or runtime gain changes exist.

### Register documentation boundary

The [Waveshare ST memory table](https://docs.waveshare.com/Memory_Map_Explanation/ST_Servo_Memory_Map_Explanation)
checked on 2026-10-21 places P/D/I at 21/22/23 in EPROM, acceleration at 41 and
Torque Limit at 48 in SRAM. It describes load feedback as drive duty and current
feedback separately. Neither is a direct calibrated shaft-torque measurement.

That primary table describes startup torque at 24 as **one byte**. The installed
LeRobot Feetech table (`soarm101-highschool-demos/.venv/lib/python*/site-packages/lerobot/motors/feetech/tables.py`)
uses `Minimum_Startup_Force: (24, 2)`. The adjacent documented field at 25 makes
this a material model/firmware ambiguity. Startup-force trials remain blocked
until the exact installed model/firmware control table is reconciled. Do not infer
units, byte width, lock behavior or safe writes from a different servo family.
Readback/restoration journals must retain the existing owner's settings and
ownership; a power interruption requires explicit journal recovery before motion.
