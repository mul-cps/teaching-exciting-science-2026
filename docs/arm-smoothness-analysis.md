# Arm smoothness measurements, 2026-10-13

The controlled base-joint test reproduced visible oscillation during steady motion. Stable host timing and smooth commanded profiles did not eliminate it. Increasing trajectory interpolation alone therefore does not resolve the measured behavior.

## Controlled test

The follower base swept 24 degrees in both directions at 1, 3, 8 and 12 degrees/second. Other joint targets stayed fixed. Quintic velocity ramps bounded velocity, acceleration and jerk; the controller ran at 125 Hz. The 128.04-second run recorded 16,006 samples with a maximum sampling gap of 8.563 ms. All other joint encoder positions stayed unchanged. Torque was disabled and temporary acceleration settings restored after completion.

| Cruise speed | Native acceleration | Positive-direction residual RMS | Negative-direction residual RMS | Observation |
|---|---|---|---|---|
| 1 deg/s | 254 | 0.168 deg | 0.117 deg | Irregular stick-slip and backward excursions |
| 3 deg/s | 254 | 0.233 deg | 0.234 deg | Strong approximately 8.7 Hz oscillation in both directions |
| 8 deg/s | 254 | 0.277 deg | 0.302 deg | Strong approximately 8.6–9.2 Hz oscillation |
| 12 deg/s | 254 | 0.023 deg | 0.255 deg | Direction-dependent; only 0.568 seconds of steady cruise per direction |
| 3 deg/s | 50 | 0.236 deg | 0.228 deg | Similar oscillation energy despite reduced native acceleration |

Residuals remove cubic drift and summarize the 2–20 Hz band. Steady windows exclude ramps and an additional 0.2 seconds at either end. These are measurements at one pose, rather than universal speed limits. After the first 0.5 seconds of each stationary hold, the base encoder remained unchanged.

At 3 deg/s, short-window sinusoidal fits gave approximately 0.32 degrees peak amplitude and 8.7 Hz, explaining over 90% of the residual variation. Optical tracking independently found a median 8.725 Hz component across 22 moving robot features: median peak amplitude 0.590 pixels versus 0.0067 pixels for background features. The video was recorded at nominal 25 fps without individual frame timestamps, so it corroborates visible motion but cannot establish precise phase alignment or rule out video aliasing by itself.

## Why faster teleoperation looks better

For position ripple `q = q_mean + A sin(2*pi*f*t)`, the velocity ripple has amplitude `2*pi*f*A`. With `A = 0.32 deg` and `f = 8.7 Hz`, this is about 17.5 deg/s. A mean speed of 3 deg/s can therefore repeatedly reverse; sufficiently fast motion can continue forward while retaining ripple. The 8 deg/s test disproves an assumed safe 8 deg/s speed floor. The brief, direction-dependent 12 deg/s result also does not establish a safe floor.

Recorded teleoperation and its exact-command replay looked smoother to the operator. The replay delivered all 7,500 recorded forward commands without changes, and follower feedback closely reproduced the original run. Both still contained base ripple. Slow choreography also showed prolonged elbow and wrist position plateaus, so every visible jerk should not be attributed to the same oscillation mechanism.

## Interpretation and next steps

The evidence supports motion-induced closed-loop oscillation consistent with servo control, friction, backlash or structural compliance. It does not identify a mechanical natural frequency or prove which component causes the mode. The base's read-only settings were firmware 3.10, position P=16, D=32, I=0, minimum startup force=16 and clockwise/counterclockwise dead zones=1. No PID or EEPROM settings were changed.

Keep the accepted recorded teleoperation as a replay baseline. Avoid extended slow cruises where practical, using smooth acceleration transitions and real stationary holds. Do not impose an abrupt minimum-velocity clamp: it introduces acceleration discontinuities. Continue controlled tests at longer, faster cruises in both directions before selecting speed bands. Any damping or servo-setting experiment should vary one setting at a time, retain a backup, and restore it after comparison. Current/load telemetry in this run was only about 6.25 Hz, insufficient to resolve the 8.7 Hz mode; faster synchronized telemetry would help distinguish control and mechanical effects.

The connected Sonix USB webcam advertises MJPEG modes up to 1080p at 30 fps, with no 60/120 fps modes. A 120-frame live test at 640x480 delivered 29.18 fps. Raising capture from 25 to 30 fps modestly improves visual evidence, while the 125 Hz encoder remains the main frequency measurement.

## Local evidence and reproduction

Raw footage and trajectories remain in ignored `artifacts/choreography/` directories:

- `teleop-paired-2026-10-13-02`: accepted paired capture.
- `teleop-replay-2026-10-13-01`: reverse return followed by exact forward replay.
- `speed-analysis-2026-10-13`: comparison plots and diagnostics.
- `base-system-probe-2026-10-13-01`: probe plan, profiles, feedback, video, analyses and optical tracks.

`python -m workshop.motion_diagnostics --help` describes offline comparison. `python -m workshop.system_probe --help` describes the bounded physical probe; its default is planning only, with `--run` required for hardware motion. `python -m workshop.teleop_replay --help` describes saved-command replay. Hardware operation requires the correct calibration, clear envelope and reachable power cutoff.

## Follow-up controlled settings trials

At the same base pose and 3 deg/s path, single-setting comparisons gave:

| Base setting changed | Positive RMS | Negative RMS | Backward events, positive / negative |
|---|---:|---:|---:|
| Original P=16, D=32, startup force=16 | 0.233 deg | 0.234 deg | 56 / 56 |
| D=48 | 0.222 deg | 0.227 deg | 58 / 58 |
| D=64 | 0.272 deg | 0.285 deg | 61 / 53 |
| Startup force=8 | 0.219 deg | 0.192 deg | 23 / 4 |
| P=8 | 0.185 deg | 0.194 deg | 11 / 0 |

D=64 made the ripple worse. Startup force=8 and P=8 reduced backward excursions,
but did not remove residual motion. P=8 introduced about one degree of average
tracking lag in each direction. These single-pose, single-trial results do not
justify a new general servo default. All changed settings were backed up before
motion, restored afterward, and verified by readback. The arm was left torque-off.

The probe now supports `--quick`, `--focus-speed 3`, `--base-d`, `--base-p` and
`--base-startup-force` for explicit experimental trials. Gain/startup-force fields
are EEPROM parameters: software restoration is verified on normal/error exit,
but a power interruption can prevent it. Keep the saved `probe-plan.json` backup.
The CLI never tunes settings without the corresponding explicit flags.
`python -m workshop.probe_analysis <run-directory>` reproduces cruise/hold metrics,
including settled-hold ranges after the first 0.5 seconds.

New continuous-rhythm commands use a quintic phase-speed ramp and harmonic joint
motion. They carry acceleration through intermediate poses instead of forcing
zero acceleration and jerk at every spline knot. The two hardware trials remain
unaccepted because wrist tracking stopped near -74 degrees; the original tracking
limit aborted them. Physical inspection and full webcam coverage are needed before
continuing. Saved ramp profiles prove command continuity, not physical smoothness.
