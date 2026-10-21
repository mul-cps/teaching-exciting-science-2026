# Blockly station lab — implementation and lesson

## Design

An offline Blockly 10.4.3 editor produces a strict, bounded JSON program. A
loopback-only Python server executes it in simulation by default. Explicit
`--physical` enables an adapter to the existing ROS action/service interfaces;
there is no serial driver, arbitrary code execution or ROS topic publishing.
One program runs at a time. Cancellation, deadlines and any failure request
station STOP; an uncertain service/action delivery is never automatically retried.

ROS Blocky upstream `a34adbdef1c4b92374d2880a755e53436ca24754` is Windows-specific
and does not yet implement service/action clients. This Linux station extension
ports its logging blocks and Python generators with compatible block IDs and
workspace JSON; station automation blocks are additional, explicitly labelled
extensions. It does not claim to run the unmodified Windows application.

## Implementation plan

1. Strict program validation, bounded recorder, simulator and failure tests.
2. ROS adapter using existing guarded services/actions and exact terminal results.
3. Offline browser editor, workspace import/export, simulation and instructor tab.
4. Fake ROS/browser qualification; independent review; physical qualification by
   the instructor on the connected station before any classroom execution.

## Lesson (replace the 35-minute independent challenge)

- 0–5 min: predict the supplied dispense/wait/transfer sequence, run simulation.
- 5–12 min: in pairs teach one fixed-jaw transfer stroke using RViz teleoperation.
  Record measured follower positions, stop recording, then disable teleoperation.
- 12–22 min: connect replay and separate jaw-position blocks. Add a repeat block.
  Treat detected-object grasping as the missing ownership step described below.
- 22–30 min: choose paths using red/blue EV3 color-sensor observations; reject unknowns.
- 30–35 min: demonstrate one simulated cycle; instructor may qualify and run one
  physical cycle. Swap driver and observer roles halfway through.

A recording is not a qualified plan. Replay retains the bridge's collision and
sample-hash checks and may reject a demonstrated path. It starts at the recorded
pose: it never automatically travels there. Record separate strokes with a
constant jaw; changing jaw during recording rejects that recording. Replay speed
is 0.1–1.0, default 0.5; the bridge remains authoritative for all motion limits.

## Start

Simulation needs only standard Python; all browser assets are committed:

```bash
python tools/run_blockly_station.py
# Open http://127.0.0.1:8877
```

Start the existing full station first for a physical session, then in a separate
terminal use the same ROS environment/domain as that station:

```bash
ros2/conda/so101-conda.sh run python tools/run_blockly_station.py --physical --instructor
```

The server binds only loopback. Port 8878 is a separate STOP-only listener with
its own bounded request pool; keep it free together with 8877. The editor's STOP
button uses that listener even when ordinary API requests are occupied. A random
in-memory token and exact host/origin checks protect requests; no token is saved
in the repository. This is a local instructor workstation application, not a
multi-user remote-control server.

Physical mode requires the station API STOP response with a `request_id`
(introduced in commit `2a6572a`). Older APIs fail STOP verification explicitly.
The application never starts a second hardware owner, enables torque at startup,
changes firmware limits or connects USB devices directly. Instructor mode
exposes the existing operator services; those services retain their own admission
checks. Closing physical mode requests a verified station STOP.

### Recording and replay

1. Enable the desired teleoperation mode through RViz under supervision.
2. Set the jaw, hold it fixed and choose a new stroke name such as `lift_closed`.
3. Press **Record follower**, demonstrate a short path, then **Finish recording**.
4. Disable teleoperation in RViz. Return the follower to the recorded starting
   pose under supervision. A replay never makes this approach automatically.
5. Enter that name in a replay block, initially at speed 0.5.
6. Separate jaw blocks use the measured gap in millimetres; they are unavailable
   until the existing SI gap calibration is qualified. Missing angular calibration
   may also prevent the existing calibration-status endpoint from succeeding.

Recordings contain five measured arm angles plus the fixed jaw angle and a
pinned arm/calibration/owner-generation/source identity. They are invalid after an
owner restart/reconnect; record fresh strokes in the new owner session. The
JointState header is a bridge publication timestamp, not a motor acquisition
timestamp: the collector also requires a fresh, identity-matched owner snapshot.
Any >300 ms gap, regressing timestamp, changing jaw or cohort change rejects the
recording. No incomplete recording is saved.

Up to 64 recordings, 600 samples and 60 seconds per recording are retained under
`~/.local/share/exciting-science/blockly-recordings/`. Archive older files while
the server is stopped. Files are never overwritten silently.

A complete detected-object dispense/pick/place program additionally needs the
scene ownership transaction. Current Blockly blocks do **not** implement target
locking, bounded finger-contact allowance, contact inference, attach/lift
verification or detach/release ownership. The jaw-gap block requires a reached
position and rejects inferred contact; it is not a grasp-contact block. A detected
world brick can therefore make jaw closing or lifting fail collision preflight.
Do not remove the brick from the scene or publish arbitrary attachment messages
to bypass that check. Physical tracked-object picking needs a guarded high-level
adapter to the existing ownership protocol first. Free-space taught motion and
EV3 sequencing are the currently exposed physical activities. These capabilities do not establish that a
physical autonomous sorting cycle has passed. The instructor must qualify the
exact strokes, object, jaw gap and environment before students run that cycle.

### Build a complete cycle

Prepare and qualify the strokes before the class. Names below are suggestions,
not supplied recordings. Each stroke's fixed jaw opening must match the preceding
jaw instruction, and its first arm pose must match the preceding stroke's end.

```text
Connect EV3
Repeat 2 times:
  Dispense one brick
  Conveyor right end (the commissioned pickup side)
  If sensor is red:
    Open jaw to the measured release gap
    Replay approach_open
    Close jaw to the measured grasp gap
    Replay lift_closed
    Replay place_left_closed
    Open jaw to the measured release gap
    Replay return_left_open
  Else if sensor is blue:
    Open jaw to the measured release gap
    Replay approach_open
    Close jaw to the measured grasp gap
    Replay lift_closed
    Replay place_right_closed
    Open jaw to the measured release gap
    Replay return_right_open
  Else: log "Unknown: ask the instructor" and STOP
```

For a real cycle, check unknown color **before dispensing or grasping** whenever
the sensor can already observe the object. If color becomes available only at
pickup, branch there before closing the jaw. The sketch's final return must be
inside each color branch so that the correct return stroke is selected. Test
each branch separately in simulation; do not assume a left return starts at the
right destination. The beginner's core task remains the 35-minute simulation
challenge; a physical cycle is an instructor extension using qualified strokes.

Keep demonstrations short and move continuously. Recorded pauses and encoder
variation remain in the stroke; the bridge may lengthen it to satisfy its motion
limits. The replay speed is a request, not a guaranteed execution duration.

## Exposed interfaces

| Blockly function | Existing ROS interface | Completion |
| --- | --- | --- |
| Follower recording | `/follower/joint_states`, `/follower/status` | Read only; fresh pinned cohort |
| Replay stroke | `/follower/arm_trajectory_controller/follow_joint_trajectory` (`FollowJointTrajectory`) | Accepted goal **and** terminal successful result, error code 0 |
| Jaw gap | `/follower/gripper_controller/gripper_cmd` (`GripperCommand`, metres) | Qualified calibration; reached goal and not stalled |
| Jaw readiness | `/follower/gripper_calibration_status` (`Trigger`) | Structured calibrated status |
| EV3 connect/home/left/right/eject | `/station/ev3/connect`, `/home`, `/left_end`, `/right_end`, `/eject` under `/station/ev3` | Exact workflow `job_id` reaches idle with result; faults reject |
| EV3 wake/sleep | `/station/wake`, `/station/sleep` (`Trigger`) | Acknowledged guarded service, workflow terminal where supplied |
| Color condition | `/station/ev3/operator_status` | Fresh connected native EV3 S3 color; unknown is explicit |
| Camera/perception readiness | `/station/operator/status` | Read-only status viewer; no fabricated camera image |
| Teleoperation readiness | `/follower/teleop_status` | Read-only status viewer |
| STOP | `/station/stop`, `/station/status` plus active action cancellation | Exact returned STOP request ID, EV3 stopped and arm disarmed |
| Instructor controls | `/station/operator/{command}` (`Trigger`) | Existing station response; no arbitrary service names |

Instructor allowlist: `emergency_reset`, `reset`, `start`, `base_release`,
`base_confirm`, `teleop`, `teleop_relative`, `teleop_absolute`, `moveit`, `damping`,
`zero_torque`, `restart_moveit`, `restart_camera`, `restart_depth`, `restart_tags`,
`restart_all`, `verify_restart`. After an unknown/failed physical operation, motion
and mode changes stay blocked. Inspect/recover through the existing station
controls, then use **Verify recovery / unlock Blockly**. Unresolved mutating
requests and late action delivery prevent that unlock. No mutation is retried.

The color condition currently uses the EV3 color sensor, not a guessed camera
object. Camera-based object identity/selection remains part of the existing
perception/sorting station; it is not silently substituted for a sensor block.

## Program protocol

`GET /api/status`, `GET /api/recordings`; token-authenticated JSON POST endpoints:

- `/api/program/run`: `{"program": [ ... ]}` → program ID; poll status to terminal.
- `/api/program/stop`: `{}` → verified stop or explicit missing acknowledgement.
- `/api/record/start`: `{"name":"transfer"}`; `/api/record/stop`: `{}`.
- `/api/instructor`: `{"command":"moveit"}`; `/api/recover`: `{}`.
- `/api/simulation/color`: `{"color":"unknown"}`; simulation only.

Commands are `ev3`, `wait`, `replay`, `jaw`, `repeat`, `if_color`, `log`, `stop`.
Programs reject unknown/extra fields, nonfinite numbers, duplicate JSON keys,
more than four nesting levels or 100 expanded steps. Repeat is 1–5, wait 0–10 s,
program deadline 180 s. HTTP bodies are limited to 256 KiB, request sockets to 2 s,
with eight worker slots per listener. Saved workspace JSON is editable Blockly
state; generated Python is a learning preview and is never executed by the server.

## Rehearsal commands

```bash
python -m pytest tests/test_blockly_station.py -q
python tests/blockly_benchmark.py
# With the simulation server running; requires Playwright only on the test PC:
python tests/blockly_browser.py
# Isolated fake ROS only; never point this fixture at the physical station:
ROS_DOMAIN_ID=75 ROS_LOCALHOST_ONLY=1 ros2/conda/so101-conda.sh run python tests/blockly_fake_ros.py
```

The fake ROS rehearsal records a fixed-jaw stroke, replays it at half speed,
executes a metric jaw action and an EV3 job, rejects stale feedback and cancels an
active replay through correlated STOP. These are interface tests, not physical
acceptance. Real motion, camera tracking, dispenser reference and payload contact
must be qualified on the connected hardware separately.

![Offline Blockly station editor](assets/blockly/station-editor.png)

### Software qualification

- 19 focused tests cover program limits, strict JSON/auth/origin checks, fixed-jaw
  recording, source/cohort changes, stale/unknown responses, correlated STOP,
  cancellation failures, mandatory EV3 motion job IDs and explicit recovery.
- Headless Chromium: offline editor, drag a block, save/reopen workspace, ported
  ROS Blocky logging/Python preview, repeat sequence and actual STOP event passed
  with zero JavaScript exceptions. STOP's CORS preflight has one origin header.
- Isolated ROS domain 75: six captured samples, EV3 exact workflow completion,
  half-speed arm replay, metric jaw action, stale rejection and action cancellation
  passed. No physical devices were used by the fake station.
- Independent review measured accepted six-joint callbacks at 4.16 µs p95,
  sixteen-joint callbacks at 3.42 µs, a 32 KiB status message at 31.82 µs and
  100-command validation at 88.3 µs on this workstation. Saturating all eight
  ordinary HTTP workers left the separate STOP listener responsive at 0.81 ms.
  These are software timings, not robot tracking or remote-PC measurements.

Physical platform commissioning and a complete autonomous pick/place cycle remain
separate acceptance steps. In particular, a feeder reference, calibrated jaw gap,
valid demonstrated strokes and current collision scene are required.

### Connected-station rehearsal, 21 October

The local station passed measured recording and physical replay through Blockly,
an EV3 connect → left-end → right-end program, correlated STOP on the separate
listener, and explicit recovery after a missing-stroke error. Recording captured
204 samples during two small supervised MoveIt moves; human leader movement was
not used in this rehearsal. The bridge retimed the 20.35-second recording at
speed 0.5 to 117.22 seconds. All six servos were measured torque off after STOP.
The [commissioning record](blockly-station-commissioning-2026-10-21.json) retains
the source identity, measured results and remaining acceptance steps. Dispensing,
metric jaw control and a full payload pick/place cycle were not commissioned.

## Optional replay timing: skip stationary pauses

**Original timing remains the default**, including for saved workspaces and JSON
programs that omit `timing`. Each replay block can explicitly select **Skip
stationary pauses**. JSON uses `"timing": "skip_stationary_pauses"`; the only
other accepted value is `"original"`.

This option removes only redundant samples in long runs whose five arm positions
are **exactly identical**. The first two and last two samples of every such run
are retained. Every nonzero position edge and its order are unchanged. There is
no filtering, relative offset, new approach movement or modification of saved
recordings. Short runs, turns and corners are retained. Timing of pauses changes;
put deliberate delays in Blockly **Wait** blocks instead.

A replay block displays the recording's original duration and the duration
requested by its timing option and speed. These are **before controller
retiming**, not promises of execution time or collision acceptance. Summaries are
computed when a recording is loaded/saved, not on the motor feedback tick.

Before dispatching a transformed replay, an offline guard compares bounds of two
**nominal requested** quintic curves using the same currently measured start:

1. Reuse the existing trajectory helper's C² quintic spline construction.
2. Convert each segment's coefficients to six degree-five Bezier control points.
3. Bound the five-joint Euclidean distance from those controls to the corresponding
   taught straight segment. Convexity makes this an upper bound for that nominal
   segment, including between samples.
4. Require the transformed curve's global bound to be no larger than the original
   curve's bound. No new numeric tolerance or dynamics threshold is introduced.

A transformed candidate that expands the bound is rejected with instructions to
keep original timing or record a shorter stroke. There is no silent fallback to a
modified timing mode. This conservative comparison can reject a transformation
at a turn even when its numerical increase is small; original timing remains
available.

This guard does **not** certify the eventual executed curve. The bridge may obtain
a newer measured start, retry admission or retime the trajectory. The existing
bridge remains the sole authority for the actual curve, joint/dynamics limits,
complete collision checks and exact owner sample hash. All those checks remain
unchanged.

Offline checks include five-joint turns/corners, constant and no-hold traces,
nonuniform source timestamps, a different current start, 600-sample recordings,
strict JSON settings and preservation of source files. On this workstation,
comparing a 600-knot original against a 594-knot candidate took about 46 ms p95
per replay; it adds no work to the control/feedback callback. Run:

```bash
python -m pytest tests/test_blockly_station.py tests/test_blockly_recording_timing.py -q
python tests/blockly_timing_benchmark.py
```
