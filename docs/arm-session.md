# Resident SO-ARM101 session

`tools/arm_session.py` runs one serialized arm-bus owner so a camera or digital
twin can request a correction while the follower continues holding the last
pose. Its arm configuration needs the exact USB identity, calibration hash,
and measured-home record. The session checks those references and starts with
torque disabled; it does not move or replay a plan at startup. The global
station `enabled`, `commissioned`, and `automatic_start` flags do not admit this
arm-only session; each `move_plan` or `home` JSON line on stdin is an explicit
command.

Start it from the workshop checkout with paths appropriate to the commissioned
station:

```sh
python tools/arm_session.py \
  --config /path/to/station.json \
  --state-file /path/to/run/arm-state.json \
  --logdir /path/to/run/logs
```

The latest-state file is replaced atomically at 20 Hz and records UTC and
monotonic timestamps, actual raw and station-unit positions, torque state,
calibration identity, and the active profile. JSONL samples and command events
are kept in the log directory. Motion plans use raw encoder counts and are
validated in full before enabling torque:

```json
{"command":"read_state"}
{"command":"move_plan","legs":[{"label":"small-correction","target_raw":{"shoulder_pan":2040,"shoulder_lift":2000,"elbow_flex":2000,"wrist_flex":2000,"wrist_roll":2000,"gripper":1500}}]}
{"command":"home"}
{"command":"stop"}
{"command":"exit"}
```

Body-joint legs are limited to 180 counts and the gripper to 650 counts; home
returns are broken into at most 100-count raw sublegs. The control loop checks
the firmware, calibration, and station-configured raw-limit intersection,
position readback, per-frame travel, 20 Hz feedback cadence, arrival, and a
bounded settling interval. Set `"grasp":true` on a leg to accept a stationary
gripper that remains at least ten counts above its closing target after 0.6 s.
That is logged as inferred contact, never as a confirmed grasp. While motion is
running, `read_state`, `stop`, and `exit` remain responsive between bounded bus
transactions; additional move/home requests are rejected until the current
command finishes. `--recover-sag` admits only a first inward recovery of at
most 16 counts on a body joint; the default rejects any initial position beyond
firmware limits. State snapshots report actual counts and list any range
violations without clamping the telemetry.

After a successful move the requested goal and torque remain active across
subsequent commands. `stop`, `exit`, stdin EOF, a control fault, or 60 seconds
without a command disables and verifies torque-off, then restores only the
saved volatile Acceleration, Goal_Velocity, and Torque_Limit registers. It does
not write calibration or EEPROM. A process-level station lock prevents a
second station controller from taking the bus and remains held until torque-off,
RAM restore, and bus close complete. This software does not replace
the physical emergency stop or collision-clearance checks.
