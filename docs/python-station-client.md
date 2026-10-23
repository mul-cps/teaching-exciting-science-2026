# Python and Blockly use one station program

`workshop.blockly_client.Program` builds the same bounded commands as Blockly.
The server validates and executes the resulting AST with `ProgramRunner`. Neither
the exported file nor its client creates another robot owner.

```python
from workshop.blockly_client import Program, simulate_program
program = Program().ev3('connect').wait(1).log('Ready')
result = simulate_program(program)  # Explicit offline simulation, no ROS imports.
print(result['events'])
```

Methods return the builder: `wait(seconds)`, `log(message)`, `ev3(command)`,
`station(command)`, `arm(command)`, `jaw(gap_mm)`, `record_stroke(name, seconds)`,
`approach_recording(name, speed=.5)`, `replay(name, speed=.5, timing='original')`,
`pick_place_recording(name, speed=.5)`, `sort_cycle()`, `stop()`, `repeat(count, body)`, and
`if_color(color, then, otherwise)`. Branches and bodies accept another `Program`
or a command list. `.program` returns a validated copy. Existing bounds, allowed
controls and instructor gates apply equally to both languages.

For a prepared shared endpoint:

```python
from workshop.blockly_client import run_program
result = run_program(program, physical=False)  # Explicit shared simulation endpoint.
```

Select `physical=True` only for the prepared physical station. The endpoint must
report the requested mode before submission; missing ROS/services never select a
simulator automatically. The client submits the exact status admission session/epoch with its single run
request, then polls the exact
returned job ID. Complete and intentional stopped outcomes require their full
result. Failure, changed job ID, timeout, interruption and unknown transport
outcomes require a terminal verified STOP. Mutations are never retried.

A notebook without ROS starts a client through this checkout's
`ros2/conda/so101-conda.sh run python` wrapper. `ROS_DOMAIN_ID` is inherited. For a
prepared fwmini4 installation, configure this explicit argv prefix before the
notebook starts:

```sh
export STATION_PROGRAM_LAUNCHER='["~/.local/bin/station-fwmini4","ros-vulkan","python"]'
```

The prefix is parsed as a JSON array and executed without a shell. The selected
Python must see this same checkout and generated ROS service interfaces. Direct
CLI execution also uses the wrapper:

```sh
ros2/conda/so101-conda.sh run python tools/run_station_program.py --simulation program.json
```

`export_python(ast, physical=False)` creates executable Python containing a
syntax-safe JSON literal and one `run_program` call. Authenticated
`POST /api/python` with `{"program": [...]}` returns `{"python": "..."}` using
the server's current mode. The normal export is for the prepared endpoint;
`simulate_program` is the explicit offline route.

## Recording restart behavior

New stroke files use `so101-blockly-stroke-v2`. `binding` retains durable robot,
arm calibration, loaded jaw-gap/joint-angle calibration hashes and source identity; `capture_cohort` retains capture generation and
source provenance; the original `identity` remains available for audit. Only the
owner generation is excluded from the durable comparison. A source change stays
an invalidation until the stroke is taught again or independently revalidated;
it is never silently discarded.

Saved files load with `recording_info[name].state = revalidation_required`.
A physical run checks the current live binding before starting its job, marking
`usable` or `invalid` with a reason. Each individual approach/replay additionally
requires fresh feedback and one unchanged live owner cohort throughout the
operation. File structure alone never authorizes motion. Legacy v1 files retain
their existing identity/source evidence and receive the same live checks. Physical
strokes without both qualified loaded calibration hashes require reteaching. The
hashes describe bytes validated by the resident bridge at startup: editing files
on disk does not change loaded identity until the bridge reloads them. Unqualified
jaw calibration still permits verified torque-OFF recovery, but blocks physical
teaching, approach and replay.
`origin=synthetic_simulation` cannot pass physical replay or approach admission.

An explicit `approach_recording` plans from the current complete arm/jaw state to
the saved first arm position using MoveIt, checks the current scene through the
resident action server, retains the fixed jaw, and requires fresh measured
arrival. A separate `replay` still requires the existing two-degree start check.
Planning failure never dispatches a trajectory. All trajectories use the sole
resident FollowJointTrajectory server and its limits, collision checks and STOP.

Fixed-jaw strokes contain arm coordinates, not grasp ownership or a verified
pick/place transaction. Generic jaw success requires a reached, nonstalled goal;
grip contact needs the existing contact/lift/release verification transaction.
The named [taught pick/place transaction](taught-pick-place.md) is implemented and
uses phase manifests, calibrated TCP/object transforms, qualified jaw/model
provenance and independent tracking verification. No commissioned physical skill
is supplied. Missing or stale qualification blocks execution; ordinary fixed-jaw
replay and simulated events do not establish a physical grasp.
