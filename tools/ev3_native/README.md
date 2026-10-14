# Native EV3 physical replay tools

These standalone Pybricks MicroPython 2.0 tools reproduce the bounded physical
component tests on 2026-10-14. They use medium feeder motor A, large sorter motor
D, touch sensor S1 and color sensor S3. They do not commission the full station
or change the production agent's existing 100 ms slice backend.

`guarded_motion.py` is the verified diagnostic helper. `fast_sorter.py` performs
one native sorter traverse; `fast_feeder.py` performs one native feeder cycle.
Keep all three files together. The scripts reject a runtime other than
`pybricks-micropython` major 2. Each native target has a separate 6 second
software deadline and encoder, drift, stall, and completion guards. Native
angle targets do not provide the native 100 ms wall-time expiry of the
production backend. Both initialized motors are independently braked in
cleanup before their saved RAM control limits are restored.

## Deploy and run one physical test

Replace `HOST` and `/path/to/ssh-key` with the brick address and your key. Copy
this directory from the repository root:

```sh
scp -i /path/to/ssh-key -r tools/ev3_native robot@HOST:/home/robot/cps-station/
```

Have hands clear and the existing emergency stop accessible before a physical
run. Run one component test and review its camera recording and JSON log
before any further physical run.

The sorter homes with the verified slow touch routine, sets its native profile
to speed 600 degrees/s, acceleration 1200 degrees/s squared, and actuation 50%,
then visits 10, 132, 360, 530, and 15 degrees. It holds 200 ms after each settled
position for the camera, restores the slow profile, and homes again. The
feeder remains braked throughout:

```sh
ssh -i /path/to/ssh-key robot@HOST 'cd /home/robot/cps-station/ev3_native && PYBRICKS_NO_REDIRECT_STDOUT=1 brickrun -r -- pybricks-micropython fast_sorter.py'
```

**The feeder must already be at the physically established relaxed negative
EJECT endpoint.** Its new motor session angle 0 is accepted only because that
physical pose is known. The script does not home, verify the mechanical
reference, or reset the encoder. Manual moves or an unknown pose invalidate
that prerequisite; establish the reference before running this tool. Do not
use a loaded-hopper stall search to assume a home pose.

With that reference established, the feeder uses speed 600 degrees/s,
acceleration 2400 degrees/s squared, and actuation 50%. It performs exactly one
LOAD/READY target 209 degrees followed by EJECT target 0 degrees. Motor D
remains braked, with no sorter home or movement:

```sh
ssh -i /path/to/ssh-key robot@HOST 'cd /home/robot/cps-station/ev3_native && PYBRICKS_NO_REDIRECT_STDOUT=1 brickrun -r -- pybricks-micropython fast_feeder.py'
```

One beep signals the start. Two beeps after cleanup signal successful motor
completion; camera review must establish whether a piece was staged and
pushed out. Landing, disposal, complete sorting, and full station admission
remain separate gates. There are no automatic retries or additional cycles.

## Host checks and physical evidence

No hardware modules are imported by these fake checks:

```sh
python3 -m unittest discover -s tools/ev3_native/tests -p 'test_*.py'
python3 -m compileall -q tools/ev3_native
```

The fake checks preserve the verified helper, sorter, and feeder scenarios,
including command order, finite targets and deadlines, wrong direction,
encoder jumps, travel bounds, switch handling, other-motor drift, stalls,
completion error, cleanup, and Python 3.4 syntax. They are software evidence,
not a substitute for physical acceptance.

Measured physical results and local evidence filenames are recorded in
[the hardware test report](../../docs/ev3-hardware-test-2026-10-14.md).
