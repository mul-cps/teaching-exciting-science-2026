# SO-ARM101 instructor choreography

The follower demo is a joint-space dance, separate from the simulated factory and
from autonomous sorting. It was bench-tested on Rig01 on 2026-10-12 with LeRobot
0.6.1 and the saved `hs_rig01_follower` calibration. The leader is not actuated.

## Preview and run

The standard-library preview never opens a serial port:

```bash
python -m workshop.choreography
```

For hardware, use an environment with `lerobot[hardware,feetech]` and OpenCV.
Keep the arm supported at rest, the workspace clear, the cutoff reachable, and
the entire motion visible to the supervising operator. The webcam is for
monitoring and recording; it does **not** perform collision detection.

This host's validated follower port is
`/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AB9068906-if00`; the leader is
`...5AB9068765-if00`. Do not assume these mappings on another rig.

```bash
python -m workshop.choreography --run --hz 125 \
  --port /dev/serial/by-id/usb-1a86_USB_Single_Serial_5AB9068906-if00 \
  --calibration-dir /home/bjoern/git/soarm101-highschool-demos/.calibration \
  --servo-acceleration 50 \
  --output artifacts/choreography/my-run
```

Use a new output directory each time. Ctrl-C and SIGTERM invoke cleanup, which
disables torque, checks its readback, restores the previous acceleration register,
and closes serial/video resources. Disabling torque can let a gravity-loaded arm
settle. Power loss or forced process termination cannot guarantee software cleanup.

## Curve and timing

The routine rises, turns, waves twice, turns the other way, bows, salutes, and
returns to its measured starting target. Monotone knot velocities allow compatible
poses to flow through. Velocity goes to zero at direction reversals, intentional
holds, and the start/finish. Seventh-order Hermite segments have continuous
position, velocity, acceleration, and jerk at joins (C3). Acceleration and jerk
are zero at each knot; they vary continuously inside the segment, providing
acceleration and deceleration rather than instantaneous speed changes.

The full curve is sampled before use to check overshoot, speed, acceleration, and
jerk, and its timing is scaled with a small margin. The choreography stays within
the adjacent pose bounds; these are **joint-space** checks, not Cartesian clearance
or self-collision checks. The planned sequence is about 40 seconds.

| Joint | Speed limit °/s | Acceleration limit °/s² | Jerk limit °/s³ |
|---|---:|---:|---:|
| Shoulder pan | 14 | 24 | 100 |
| Shoulder lift | 12 | 24 | 80 |
| Elbow | 12 | 24 | 80 |
| Wrist flex | 14 | 28 | 100 |
| Wrist roll | 25 | 50 | 180 |
| Gripper (percent units) | 35 | 70 | 300 |

The controller uses scheduled deadlines at 100–200 Hz. Camera capture, video
encoding, and feedback-file IO run in a separate thread at camera rate. Cyclic GC
is suspended during control and restored afterward; reference counting stays
active. Gaps/transactions exceeding 50 ms and joint tracking errors exceeding
10 degrees (10 percentage points for the gripper) abort the routine. Missed ticks
are skipped rather than replayed as a burst of old commands. This is a measured
Python/Linux control loop, not a hard-real-time guarantee.

## Replayable artifacts

Each run saves these files under its output directory (ignored by Git):

- `trajectory.json`: complete timed waypoints, knot velocities, polynomial type,
  units, limits, calibration hash, control rate, and servo acceleration setting.
  Written **before torque is enabled**.
- `planned-profiles.jsonl`: scheduled position, velocity, acceleration, and jerk
  samples, also written before motion.
- `feedback.jsonl`: actual timestamps, commanded/measured joints, phase, bus time,
  and deadline lateness; flushed roughly every second during the run.
- `trace.json`: the complete or interrupted feedback log, saved on cleanup.
- `hardware-before.json`: the acceleration register values to restore.
- `dance.avi`, `latest.jpg`, and phase snapshots: independent webcam recording.

Replay the plan, **not the noisy measured trajectory**:

```bash
python -m workshop.choreography --run --hz 125 \
  --replay artifacts/choreography/my-run/trajectory.json \
  --port /dev/serial/by-id/usb-1a86_USB_Single_Serial_5AB9068906-if00 \
  --calibration-dir /home/bjoern/git/soarm101-highschool-demos/.calibration \
  --output artifacts/choreography/my-replay
```

Replay refuses a different arm ID/calibration or a starting pose more than
3 degrees/percentage points away; it never automatically drives from an arbitrary
pose to the saved start. It rechecks curve limits and continuity before enabling
torque. The original plan is preserved and its hash recorded in the replay.
Omitting `--servo-acceleration` reuses the plan's setting. `--hz` explicitly selects
the command sampling rate without changing the continuous curve.

The retained, physically tested 125 Hz curve is also checked into
[`trajectories/rig01-flow-2026-10-12.json`](../trajectories/rig01-flow-2026-10-12.json).
It is specific to this arm and calibration; use the same replay checks above.

## API findings

LeRobot 0.6.1's `ActionInterpolator` (`lerobot.utils.action_interpolator`) creates
linear intermediate actions. It is useful for upsampling policy outputs, but does
not enforce acceleration/jerk limits. Its SO follower `send_action` writes target
positions; it is not a multi-waypoint spline follower. This demo therefore plans
the continuous curve and follows it through LeRobot's Feetech bus API, using the
existing `precise_sleep` timing helper.

References: [LeRobot rollout interpolation](https://huggingface.co/docs/lerobot/main/inference)
and [manufacturer servo acceleration examples](https://www.waveshare.com/wiki/ST3215_Servo).
The native `Acceleration` register is temporarily set to 50, below the observed
LeRobot default of 254; these register values are not degrees/s². PID, calibration,
IDs, and EEPROM maximum-acceleration settings are not retuned by the dance.

The initial camera-coupled loop measured 25 Hz with a 67 ms maximum gap. Separating
camera IO raised the rate to about 99.95 Hz; removing unnecessary stops and adding
C3 curves shortened the routine from about 65 to 40 seconds. A controlled replay
with native acceleration 20 increased wrist/gripper tracking lag compared with 50,
so 50 was retained. Run artifacts contain the exact measurements; encoder
quantization, load-dependent error, and mechanical backlash can still affect
physical smoothness.

The physically completed 125 Hz pass measured 124.95 Hz, a 14.6 ms maximum
update interval, and approximately 2.9° maximum angular tracking error. It finished
at its starting target, verified torque-off, and restored acceleration to 254.
The camera remained at 25 fps while motor commands/feedback ran at 125 Hz.

The next candidate, `--body-wave`, adds more pronounced arm rise/fall during the
wrist waves while retaining the same joint displacement bounds. It passes offline
curve checks but its physical trial is pending: the webcam subsequently stopped
streaming with `VIDIOC_STREAMON: Protocol error`, and the trial aborted before
motor connection. The default remains the completed flow routine. Reconnect the
webcam and verify the full view before testing this candidate.

## Continuous rhythm with smooth ramps

`--rhythm` is the new candidate for the requested smooth dance:

```bash
python -m workshop.choreography --rhythm --hz 125
```

It uses coordinated harmonic joint curves and a quintic phase-velocity ramp.
Position, velocity, acceleration and jerk start and finish continuously at rest.
During the dance, acceleration flows through intermediate poses instead of being
reset to zero at every waypoint. Reversals remain smooth; no minimum-speed clamp
or intermediate holds are introduced. The two-cycle plan lasts about 14.5 seconds.

The native acceleration setting is 254 for this candidate because the controlled
probe found no oscillation-energy improvement from 50. This is a servo register
value, not the planned acceleration in degrees/s². The original register is still
restored on exit. Existing saved spline trajectories retain their own setting.

The rhythm keeps the previous joint displacement envelope and checks the entire
position envelope against the actual calibration before torque is enabled.
Schema 3 saves every harmonic, the phase ramp, time scale, limits and start pose;
`--replay` follows the same continuous curve without fitting measured feedback.
New recordings request 30 fps, while motor commands remain at 125 Hz. Each run
also saves `result.json`, including completion/failure and cleanup verification.

**Physical acceptance is pending.** Two trials on 2026-10-13 aborted at the existing
10-degree tracking limit on wrist rotation. The second trial tracked initially,
then stopped near -74 degrees while the target continued. A cable/contact
constraint is possible and needs inspection; it is not a confirmed diagnosis.
Neither interrupted recording is an accepted smooth dance. Torque-off and original
setting restoration were verified. Do not widen the tracking threshold to make
these trials pass.

See [measured arm response](arm-smoothness-analysis.md) for the distinction between
continuous command ramps and motion-induced servo/arm oscillation.

## Measured home and restart

Rig01's current measured rest pose is saved in
[`trajectories/rig01-home.json`](../trajectories/rig01-home.json). Every choreography
hardware start first loads this home, checks its arm ID and calibration hash, and
plans a bounded septic rest-to-rest return from the measured current pose. It seeds
current goals before enabling torque, follows the return with limited speed,
acceleration and jerk, and verifies arrival within one degree/percentage point
before starting the dance. Excessive tracking error aborts the return. A current
pose outside the bounded return envelope is refused rather than driven home blindly.
These are joint-space checks; they do not prove Cartesian collision clearance.

The initial home capture is currently **not actuation-ready**: its shoulder encoder
position is 949, below the existing hardware minimum 1063. The file preserves the
requested physical pose and raw encoder readings. The return refuses it until the
lower-limit decision is resolved. No calibration or EEPROM limit was changed by
home capture. The proposed change and original backup are retained locally in
`artifacts/choreography/home-limit-proposal-2026-10-13/` for review.

`--home` selects another explicit home file. `--home-only` performs and verifies
just the home move. `home-trajectory.json` and `planned-home-profiles.jsonl` are
saved before torque is enabled; the run's `trajectory.json` also references the
home file/hash and return plan. A replay whose saved start differs from home is
refused; regenerate or recapture from home rather than shifting old commands.
Paired-command replay through `workshop.teleop_replay` also goes through home first.
Interactive teleoperation and diagnostic probes retain their separate start rules.

To explicitly redefine home from a new supported, torque-off pose inside the
calibrated range:

```bash
python -m workshop.safe_home --capture \
  --port /dev/serial/by-id/usb-1a86_USB_Single_Serial_5AB9068906-if00 \
  --calibration-dir /home/bjoern/git/soarm101-highschool-demos/.calibration
```

Capture reads the arm and saves the home atomically; it never enables torque or
writes calibration registers.

`--g28` is an alias for `--home-only`: a speed-limited, ramped return and arrival
check without playback. Choreography/replay use this same return stage automatically.

A monitored raw-position return on 2026-10-13 confirmed the home mismatch: the
shoulder followed the slow ramp down to encoder 1065, then failed to follow goals
below its configured minimum 1063. The error check stopped the trial at a goal of
about 1031, with torque-off verified. Thus the saved physical home at 949 is valid
as a manually reachable pose, but is outside the current powered travel range.
The calibration and EEPROM limits remain unchanged. The recording and feedback
are in `artifacts/choreography/home-recovery-2026-10-13-02/`.

A single-register goal probe also observed torque becoming enabled after writing
`Goal_Position`. Restoring an old goal then caused unintended shoulder movement;
torque was disabled immediately afterward. Startup now writes torque-on,
acceleration, the **current** raw goal and a finite speed cap together in one
contiguous register packet per servo. Goal writes are treated as actuation,
including when torque was previously off. Cleanup never restores old position
goals, and torque-off is checked again after restoring other temporary settings.
The speed-cap conversion reads the Phase register to select the documented units.
See [FEETECH's register table](https://ftservo.github.io/ftservo-wiki/reference/parameter/memory-sts/).
