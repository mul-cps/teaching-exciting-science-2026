# Paired teleoperation capture and motion analysis

`workshop.teleop_capture` records calibrated physical-degree readings AND raw encoder counts from both arms, the mapped leader target, the actually sent command, and follower feedback. Read/write midpoint timestamps are separate, so bus ordering is not mistaken for servo lag. Staggered follower speed/current/load/voltage register samples are retained without assuming a torque conversion. The recorder runs independently of the 125 Hz control loop.

Physically match the two arm poses first. Saved calibration files on this rig have different zero references. The mapping is `follower_start + leader - leader_start`: 1:1 angular changes for body joints, calibrated percent changes for the gripper. This is a temporary matched-pose offset, not recalibration or proof of absolute kinematic alignment. Leader torque stays disabled. Follower motion spans its calibrated range, without the dance's arbitrary ±20–35 degree envelope. Rate limits are 60 deg/s for pan/lift/elbow/wrist flex, 90 deg/s roll, 100 percent/s gripper; limiting is logged. These limits are not collision detection.

Example local invocation using the LeRobot environment already installed on the workstation:

```bash
/tmp/lerobot-so101-test/bin/python -m workshop.teleop_capture \
  --run --matched-pose --duration 60 --hz 125 \
  --leader-port /dev/serial/by-id/usb-1a86_USB_Single_Serial_5AB9068765-if00 \
  --follower-port /dev/serial/by-id/usb-1a86_USB_Single_Serial_5AB9068906-if00 \
  --calibration-dir /home/bjoern/git/soarm101-highschool-demos/.calibration \
  --output artifacts/choreography/teleop-new-take
```

Use `--preflight` instead of `--run --matched-pose` to identify devices, check calibration and camera, and read registers without enabling torque. Output must be a new directory. Recording saves metadata before engagement and buffered JSONL during capture, followed by complete `trace.json` on exit. Ctrl-C/SIGTERM stops recording; cleanup disables and reads back torque on both buses. Camera errors, gaps/transactions over 50 ms, and tracking error over 10 degrees/percent stop actuation. Native acceleration/velocity registers are observed, not changed.

The recorded commands, timestamps, raw samples, pose offsets and calibration hashes are retained for later replay reconstruction. This capture format is not accepted directly by the choreography CLI's spline replay schema. Never replay a raw trace without its own starting-pose and workspace checks.

## Offline analysis

```bash
python -m workshop.teleop_analysis artifacts/choreography/teleop-paired-2026-10-13-02
```

Requires NumPy; Matplotlib optionally creates `paired-tracking.png`. Analysis resamples using individual measurement timestamps and uses degree-5 local polynomial derivatives. Jerk RMS is reported across 0.16, 0.24 and 0.40 second windows, because quantized raw third differences are misleading. Derivative edges and windows touching limited commands are excluded. Velocity lag estimates are approximate correlations, not identified transfer functions. `analysis.json` and `paired-resampled.npz` preserve the results.

## 2026-10-13 physical result

Corrected matched-pose take: 7,500 samples, 59.992 s; mean 124.9999 Hz, maximum gap 8.852 ms. Both torque-off readbacks passed. User reported smooth teleoperation. Captured spans: pan 72.5°, elbow 86.1°, wrist flex 93.9°, roll 199.4°. Shoulder lift and gripper each varied only about 0.2 units, so those joints have no useful movement qualification. Approximately 12.2% of samples were command-limited; the analysis excludes these and adjacent derivative windows.

Estimated moving-joint velocity lag: pan 64 ms, elbow 88 ms, wrist flex/roll 72 ms. The follower trajectory is close to the sent target during large sweeps. Faster motion does not necessarily reduce tracking error: faster bins generally have larger positional lag. Visual smoothness and tracking accuracy are separate criteria.

Teleop retained native acceleration register 254; dance used 50. Speed, motion size, servo profiling, and trajectory shape all differ, so this recording does NOT causally prove that faster speed alone fixes jerk. The useful next controlled test is to replay the captured commands at the original timestamps and register setting, then compare the exact same path with acceleration 50. Preserve the successful measured path as the baseline before adding spline resampling, retiming, or feedforward. For subsequent choreographies, investigate avoiding extended tiny creeping movements, while retaining smooth acceleration and reversals.

First take was interrupted after 3,760 samples because the initial recorder incorrectly reused dance displacement bounds; it is not a valid natural-teleop comparison. All local video/raw artifacts stay under ignored `artifacts/choreography/` and are not uploaded by this code.

## Original-timing physical replay

`workshop.teleop_replay` replays the captured **sent commands**, not raw leader samples or interpolated splines. It checks the source completion status, calibration hash, calibrated joint bounds, increasing timestamps/gaps, and per-tick rate limits before opening hardware. Recorded command timing includes serial latency jitter; step bounds use the larger of the actual command interval and the original capture period. No trajectory retiming is applied.

```bash
/tmp/lerobot-so101-test/bin/python -m workshop.teleop_replay \
  artifacts/choreography/teleop-paired-2026-10-13-02 \
  --run \
  --port /dev/serial/by-id/usb-1a86_USB_Single_Serial_5AB9068906-if00 \
  --calibration-dir /home/bjoern/git/soarm101-highschool-demos/.calibration \
  --output artifacts/choreography/teleop-replay-new-take
```

Omit `--run` for offline validation. Hardware requires a working camera and current pose within 3 degrees/percent of the recorded start or finish. If at the finish, the arm returns along the captured commands in reverse, omitting the original initial idle period, then begins the forward replay. Intermediate poses are refused. The complete return/forward plan and source SHA256 are saved before torque engagement. Native acceleration and goal velocity are temporarily restored to the captured values and verified, then their previous values are restored on exit. The leader is not opened or actuated.

Commands follow the original relative command-write timestamps, with overdue commands skipped instead of burst-sent. Bus/control gaps over 50 ms, tracking error over 10 degrees/percent, and camera failures stop the replay. Output includes camera video, feedback JSONL, final trace, replay plan and cleanup readbacks. Like the capture controller, this is a monitored joint-space replay, not collision detection.

Recorded-command hardware replay now begins with the bounded safe-home move from
`trajectories/rig01-home.json` (or `--home`). Its recorded first command must match
that saved home. A recording from a different home is rejected; recapture from the
new home to preserve the original command mapping and timing. The home path and
home-file hash are saved in `replay-plan.json` before torque is enabled.
