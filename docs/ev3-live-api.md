# Persistent native EV3 API

`tools/ev3_native/live_api.py` runs as one persistent Pybricks MicroPython 2.0
process. It binds TCP `127.0.0.1:8766` on the brick. The separate installer
owns the brickrun/flock service wrapper; this daemon does not change the old
station agent or its protocol. Use a tunnel or the PC bridge to reach the
brick's localhost listener.

Startup initializes and brakes large sorter D and medium feeder A, preserving
their encoder origins. It does not home, reset an angle, move a motor, or beep.
Both references start unconfirmed. S1 is the sorter touch switch and S3 is the
color sensor. Default startup is suitable for read-only status/display checks
while mechanical work is in progress.

## Wire contract

Each connection starts with this newline-terminated JSON ready frame:

```json
{"protocol":"ev3-live-api","version":1,"ready":true,"session":"ev3-..."}
```

Requests are newline-terminated ASCII JSON objects, at most 4096 bytes before
the newline. Socket receive chunks are at most 4096 bytes. Every request has
exactly these fields:

```json
{"id":1,"operation":"status","args":{}}
```

IDs are positive integers increasing within one connection. An identical
cached ID/payload returns its original reply without repeating motor motion.
A changed cached payload or stale ID is a protocol fault. The newest 64 replies
are cached; an evicted old ID is stale. A fresh connection starts a new ID/cache
namespace. Do not reconnect and replay a motion after a transport failure:
inspect current status and references before separately admitting new work.
The process session string survives connections and changes on daemon restart.

Replies have one of these forms:

```json
{"id":1,"ok":true,"result":{}}
{"id":1,"ok":false,"fault":{"code":"invalid_request","message":"..."}}
```

Invalid framing, JSON, request structure, or ID brakes both motors, aborts an
active job, and closes the connection. A fault reply gets at most 200 ms of
nonblocking drain after braking. No additional request is admitted during
that drain. Invalid operation arguments return a fault without starting
motion. There is no eval or remote code execution operation.

## Operations

| Operation | Args | Result/behavior |
| --- | --- | --- |
| `heartbeat` | `{}` | `{alive:true,session:...}`; refreshes the active-job lease |
| `status` | `{}` | Cached status, described below |
| `stop` | `{}` | `{stopped:true,status:...}` after both brake attempts; brake failure returns a fault |
| `power_save` | `{"idle_timeout_ms":60000}` | Brakes both motors and enters power-save mode immediately; optional automatic-idle timeout is 1000–3600000 ms |
| `wake` | `{}` | Returns to awake mode without changing references or motor positions |
| `display` | `{text:"..."}` | Schedules printable ASCII text, max 80 characters |
| `beep` | `{count:1}` | Schedules 1–3 nonblocking beeps |
| `confirm_reference` | `{name:"feeder",reference:"eject",angle:0}` | Establishes zero from the user's physical EJECT assertion while idle |
| `confirm_reference` | `{name:"sorter",reference:"home",angle:0}` | Establishes zero while idle, requiring S1 pressed |
| `home_touch` | `{}` | Async sorter switch home: command speed 50°/s, native limits 120°/s and 400°/s², actuation 30%, max 550° travel, 10 s deadline |
| `motor_target` | `{name:"sorter",target:132,speed:600,acceleration:1200,actuation:50}` | Async finite native target; confirmed reference required |
| `eject` | `{cycles:1}` | Async 1–5 pairs of feeder LOAD/READY 209° → EJECT 0°, profile 600/2400/50 |
| `shake` | `{frequency_hz:12.5,speed_dps:600,acceleration:30000,duration_ms:1000}` | Async alternating finite sorter targets within ±20° of current center, clipped to 0–550° |

`angle` may be omitted from confirmation and defaults to zero. Confirmation
means the user has established the actual physical pose; feeder confirmation
performs no stall search. Manual moves invalidate that physical prerequisite.
`home_touch` brakes immediately on a pressed switch, requires quiet settling
with the switch still pressed, and only then resets sorter angle zero. An
initially pressed switch requires no approach command.

`motor_target` accepts sorter targets 0–550° or feeder targets 0–209°. Default
speed is 600°/s, acceleration 1200°/s² for sorter or 2400°/s² for feeder, and
actuation 50%. Allowed speed is 1–600, acceleration 1–30000, actuation 1–50.
Each target has a 6 s deadline: profiles too slow to finish within that deadline
fault instead of extending it. Native commands are asynchronous `run_target`
with `Stop.BRAKE`; the daemon does not restart 100 ms motor slices.

`shake` accepts frequency 1–15 Hz, speed 50–600°/s, acceleration 1000–30000°/s²,
and integer duration 100–2000 ms. Native target reversals are scheduled at each
requested half-period, with no catch-up burst after a delayed tick. Actual
encoder reversals, measured frequency, and peak displacement are reported.
Requested setpoint frequency is not a measured mechanical vibration frequency.

## Jobs, status, and responsiveness

Motion admission replies immediately:

```json
{"id":4,"ok":true,"result":{"accepted":true,"job":{"id":4,"operation":"eject","state":"running","phase":"load_1"}}}
```

The actual job includes timing, motor, target, and operation-specific fields.
There are no unsolicited completion frames; poll `status` to observe progress.
While busy, heartbeat/status/stop/display/beep remain available and further
motor/reference operations are rejected. Status contains `power_mode` (`awake`
or `power_save`) and `power_save_idle_ms` in addition to:

```json
{"session":"ev3-...","connected":true,"busy":false,"job":null,"last_job":null,"angles":{"sorter":0,"feeder":0},"references":{"sorter":null,"feeder":null},"touch":false,"color":"unknown","last_error":null,"sampled_ms":0,"power_mode":"awake","power_save_idle_ms":60000}
```

Terminal `last_job.state` is `complete`, `aborted`, or `fault`. A failed job
has `fault:{code,message}`. Cleanup failure is recorded separately as
`cleanup_fault`. A newly admitted motor job clears the prior `last_error`.
An explicit stop is an aborted job, not successful completion.

The event loop normally polls at most 20 ms. After 60 seconds without a
foreground command it enters power-save mode automatically. Heartbeat and status
requests do not reset this timer, so the persistent host connection can remain
available without keeping the motor station awake. A `power_save` request enters
the mode immediately and may change the timeout for this daemon session.

Entering power-save mode brakes both motors, keeps encoder references intact,
turns off the EV3 status light, stops any scheduled tone, and draws `POWER SAVE`
and `TCP API ready` once on the reflective LCD. The light-off operation is
documented for the [Pybricks 2.0 EV3Brick API](https://docs.pybricks.com/en/v2.0/hubs.html#pybricks.hubs.EV3Brick.light.off).
The daemon does not assume the reflective LCD has a controllable backlight.
While asleep it skips encoder, touch, and color sampling and changes the socket
poll timeout to 250 ms. That caps idle loop ticks at 4 Hz rather than 50 Hz
(92% fewer loop wakeups); it is a software scheduling estimate, not a measured
reduction in watts. `poll()` returns as soon as TCP data arrives, so the first
foreground command is handled on the next network event. Any command except
heartbeat or status wakes the API before processing; motor commands, display,
beep, reference, and stop are covered. `wake` is also available explicitly.
Power-save requests are rejected while a motor job is active. Active jobs retain
the 1000 ms heartbeat watchdog and existing 50 Hz encoder guard sampling.

While awake, status/heartbeat bursts use cached telemetry rather than reading
drivers for every request. Encoder samples refresh at most 50 Hz during jobs
and 2 Hz while idle; S1 is sampled once per active sorter guard tick. Display
color/touch refresh at most 2 Hz. Identical screen content is not redrawn.
Redraws are capped at 2 Hz, and shake suppresses automatic mid-burst redraws
while still accepting explicit changed text at the same cap.

A running job requires heartbeat gaps less than 1000 ms. Expiry, EOF, client
disconnection, protocol error, or service exit brakes both initialized motors
and aborts the job. Idle sessions may persist indefinitely without heartbeats.
Target jobs guard encoder jumps, wrong direction, travel budgets, other-motor
drift, stalls, target error, and quiet settling. Quiet settling requires 200 ms
with range ≤1°, total recoil ≤6°, and a 1000 ms cap. Both motors are braked
independently before saved RAM control limits are restored. Software deadlines
and heartbeat expiry are not native wall-time motor expiries.

Wrong-direction terminal diagnostics include the bounded encoder history and a
fault-time snapshot of motor speed, native `done()` and `stalled()` flags, with
timestamps around each read. These additional native reads occur only after a
wrong-direction sample, before braking and limit restoration.

A job schedules one start beep and two beeps only after successful motor
completion. Speaker tones use negative-duration nonblocking playback and timed
volume gating according to the [Pybricks 2.0 speaker API](https://docs.pybricks.com/en/v2.0/hubs.html#pybricks.hubs.EV3Brick.speaker.beep).
Beeps establish motor job completion, not cube ejection or station admission.

## Host validation

```sh
python3 -m pytest -q tests tools/ev3_native/tests
python3 -m compileall -q tools/ev3_native/live_api.py
```

Fake motors verify finite command ordering, reference admission, duplicate
suppression, stop/heartbeat interruption, homing, bounds, stalls, quiet
settling, and cleanup. A real host loopback socket checks framing and fault
reply delivery; separate performance tests measure cache and screen behavior.
These checks do not establish native socket compatibility or physical motion
qualification on the brick. Start with status/display/stop deployment checks.

On this lab PC port 8766 is already used, so the SSH forward uses PC port 18766. Pass `--port 18766` to the client, or `EV3LiveClient(port=18766)` in Python. Native Unix MicroPython socket binding uses the sockaddr returned by `getaddrinfo`; the EV3 listener stays on localhost 8766.

Use `python3 workshop/station/ev3_live_client.py --port 18766 sleep` to enter
power-save mode immediately, or `sleep --idle-timeout-ms 120000` to set a
two-minute automatic timeout for the running daemon. `wake` or any other
foreground operation wakes it. The sleep command closes its client without
sending the normal stop request, which would otherwise wake the station again.
The CLI `status` monitor also closes without sending `stop`, so checking a
sleeping station leaves its power mode and idle timer unchanged. Generic
`command --json` calls for `status`, `heartbeat`, and `power_save` use the same
close behavior. `sampled_ms` continues to report the last actual encoder read
while asleep; entering power save does not rewrite cached sample timestamps.

Live read-only validation: 50 persistent status requests through USB/SSH measured median 36.87 ms and p95 62.64 ms. This is host round-trip latency, not the motor control rate; native motion profiles run on the brick. Display and two beeps were verified. Native motion jobs are not yet physically verified through this new API.
