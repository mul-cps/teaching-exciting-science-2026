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
command finishes. `--recover-sag` (default off) permits `home` to recover a body
joint initially at most 16 counts outside its admitted limits. Its first goal
is 16 counts inside that boundary, so commanded travel is at most 32 counts.
The existing six-second leg deadline includes setup, fresh feedback, settlement,
and dwell. Recovery disables gravity trim, holds other goals fixed, rejects
outward or inactive drift above two counts, and caps observed excursion at
34 counts. After reentry, leaving the admitted range aborts. Four quiet samples
and dwell establish admitted reentry; `settlement_purpose=admitted_reentry` and
`target_reached` distinguish this proof from reaching the inset goal. Ordinary
measured-home sublegs then continue. Ordinary plans and gripper commands remain
blocked until normal admission. Firmware limits, calibration and profiles are
unchanged. State snapshots always report actual counts and range violations
without clamping telemetry.

After a successful move the requested goal and torque remain active across
subsequent commands. `stop`, `exit`, stdin EOF, a control fault, or 60 seconds
without a command disables and verifies torque-off, then restores only the
saved volatile Acceleration, Goal_Velocity, and Torque_Limit registers. It does
not write calibration or EEPROM. A process-level station lock prevents a
second station controller from taking the bus and remains held until torque-off,
RAM restore, and bus close complete. This software does not replace
the physical emergency stop or collision-clearance checks.

### Observation after a control fault

After a command fault and successful verified torque-off/RAM cleanup, the resident
owner keeps publishing fresh passive measurements. Snapshots include an immutable
`fault_observation` record and `write_scope: fault_observation`. Only `read_state`,
explicit `stop`, and `exit` are accepted. STOP verifies disarm and preserves the
owner fault latch; queued motion is discarded. Cleanup failure remains fatal.
The fault observer stays resident past the idle deadline and never retries motion.

Follower status reports `owner_faulted: true` and
`recovery_action: explicit_reconnect_then_stop` while these measurements remain
fresh. Use the existing Connect route (`/follower/reconnect`) to explicitly retire
a fresh identity/source/generation-matched fault observer with all six servos OFF.
The bridge sends one exit, waits for termination, and starts a passive owner; an
explicit STOP is then required. Connect refuses a healthy live owner.

`/follower/reset_owner` is the explicit controller reset route for an idle, fresh,
identity/source/generation-matched owner with all six servos OFF, including a
healthy disarmed owner. It uses the same exit/wait/passive-start sequence, never
kills a running owner, and revokes previous measured-home completion. Neither
route homes, arms, modifies joint limits, nor restores old home proof.

A separate `station_reset` operator-maintenance reservation lasts 120 seconds
(ordinary maintenance remains 60 seconds). Its purpose and separately configured
`operator_station_reset_units` are exposed in status. Extended restart units are
usable only under this purpose. Reset acceptance failure retains the reservation;
expiry latches a fault/STOP barrier until an explicit verified STOP.
