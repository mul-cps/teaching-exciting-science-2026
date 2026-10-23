# Architecture and entry points

The repository has two primary execution paths: an offline teaching simulator,
and an instructor station with optional physical backends and ROS interfaces.
The generated API documents both, including diagnostic and experimental tools.
A source file being present does not mean that its backend is qualified.

## Offline learning path

`python -m workshop` enters `workshop/__main__.py`. The factory orchestrator in
`workshop/factory.py` connects the robot, conveyor, vision and sorting policy.
`SimBelt`, `SimCamera`, `SimRobot` and `Factory` live together in that module;
`Factory.one_cycle` applies red/blue routing rules and rejects unknown colors.
`workshop/vision.py` provides the optional OpenCV camera adapter.
The `exercises/` scripts expose the same concepts in small student programs;
`check_setup.py` checks laptop readiness and `make_student_bundle.py` packages
the student distribution. Read their API/source pages via the
exercises inventory (`inventory.md#exercises`, generated reference).

The standard simulator runs with the Python standard library. Optional camera
capture adds OpenCV/NumPy, while arm and conveyor remain simulated in this path.
The exercise and material builders are developer utilities, not station control
entry points. `tools/build_materials.py` builds handouts and slides;
`tools/build_lecture_slides.mjs` assembles the current lecture from committed assets.

## Station and bus ownership

`workshop/station/__main__.py` provides the station CLI; `config.py` validates
station configuration. `coordinator.py` coordinates arm/EV3 workflows and the
commissioning/readiness modules record their prerequisites. Backends include
simulation and EV3/arm adapters. `locking.py` coordinates ownership; physical
resources are not interchangeable with simulated resources.

`workshop/station/arm_session.py` owns the persistent arm session and serialized
bus access. `ros_trajectory.py`, `ros_arm_prepare.py`, `ros_workflow.py` and
`ros_sorting.py` adapt the resident owner to ROS operations. The ROS trajectory
bridge keeps the Python owner as the sole Feetech bus owner and exchanges
private runtime state/commands with it. The bridge does not independently open
the serial bus. Trajectory checks, effective-path collision validation and
owner-side path verification are defined in `docs/so101-control.md`.

Use the station module inventory (`inventory.md#workshop`, generated reference) to follow individual
classes and their methods. Recovery, gripper registration, placement and
commissioning modules are explicit stateful workflows; inspecting their API
must not be treated as permission to skip their preconditions.

## ROS package boundaries

| Package | Responsibility and principal source |
| --- | --- |
| `so101_station_bringup` | Launch composition and mock/planning overlays (`launch/`) |
| `so101_station_control` | `FollowJointTrajectory` bridge and leader teleoperation (`trajectory_server.py`, `leader_teleop.py`) |
| `so101_station_perception` | RGB-D capture adapters, marker pipeline, board localization, registration and recovery |
| `so101_station_objects` | Color/geometry detections and contact/pregrasp proposals (`node.py`, `objects.py`) |
| `so101_station_scene` | Planning-scene updates and grasp validation/adaptation |
| `so101_station_map` | Surface voxel map and its ROS wrapper |
| `so101_station_gpu` | Optional GPU/perception helpers |
| `so101_station_operator` | RViz Qt panel and operator API wrapper |
| `so101_station_workflow` | Station API executable and custom service schemas |

`ros2/scripts/so101.sh` and `runtime.sh` select/build the ROS environment;
`ros2/conda/` contains isolated runtime preparation scripts. The pinned upstream
robot stack is described by `ros2/upstream.repos` and is excluded from the
project-owned code inventory when fetched under `.cache`. Build instructions,
provisional calibration assumptions and backend limitations are in
`docs/so101-ros2.md`, `docs/so101-live-station.md` and `docs/so101-control.md`.

The data path is RGB/depth/CameraInfo → marker/registration and object analysis
→ scene/grasp proposals → validated planning → resident owner execution.
The measured joint-state path returns from the owner to ROS/RViz. A retained
camera transform does not establish fresh localization; object and planning
consumers need valid timestamps and registration state. See
`docs/so101-objects.md`, `docs/so101-perception.md` and `docs/so101-scene.md`.

## Perception, digital twin and native experiments

`workshop/perception/` contains reusable geometry, registration, camera,
projection and backend policy code. Native C++ and Slang files implement narrow
backend kernels and compatibility operations. The production ROS package builds the required native bridge sources. Unused research prototypes and duplicate native build scripts have been removed. Vulkan/Slang color-mask selection remains explicit in the station profile; a backend being installed does not select it automatically.

`workshop/digital_twin/` builds the MuJoCo/Viser representation and its mirrored
state, camera and geometry interfaces. `tools/launch_digital_twin.sh` is the
launcher; digital-twin visuals do not establish physical calibration or replay
acceptance. `workshop/teleop_capture.py`, `teleop_analysis.py` and
`teleop_replay.py` provide capture/review/replay helpers with separate live
execution gates.

## Tests and tooling

`tests/` contains Python regression and fixture code. ROS packages have
additional `test/` sources, and `tools/ev3_native/tests/` tests the EV3 bridge.
All are indexed, including private fixtures. Generated API signatures show
declared source types; imported external types are not introspected.
Diagnostic, benchmark, capture, commissioning and material generation scripts
are all discoverable in the tools inventory (`inventory.md#tools`, generated reference). Read their
`main` implementation and argument parser before choosing a command: many
instructor tools can request physical access when explicitly enabled.

## Managed station process lifecycle

The root `./station` command and `scripts/{start,stop,status,purge}.sh` are the public lifecycle interface. They reuse the prepared desktop helper and `tools/station_process.py` (`source/tools/station_process.py.md`, generated reference). Existing `tools/start_station.sh` and `tools/stop_station.sh` remain compatibility wrappers.
That helper validates prerequisites and existing owners, launches one integrated
station session, and records Linux process identities. Its stop path checks
saved process identity before ROS STOP calls or signals, attempts station and
follower STOP independently, and bounds cleanup of its managed session.
Uncertain cleanup retains its ownership record. Starting the helper does not
home or enable motion. Prepared-host profiles and explicit passive bootstrap
options are described in `docs/station-startup.md`; those are deployment
procedures, not actions performed by this documentation build.

The trajectory bridge's `_retry_trajectory_start` is a separate, narrow recovery
path: only a correlated `trajectory-start-rejection-v1` receipt explicitly
asserting no actuation can enter its revalidation. It checks goal/request,
owner/calibration/source identity, checked path, freshness and two-count encoder
jitter before rebuilding and checking a candidate path. This is not a general
retry policy for uncertain ROS commands. Read the bridge and owner APIs in the
inventory, including `tests/test_arm_trajectory_start_receipt.py`.

## Blockly editor, program execution and recorded strokes

`web/blockly-station/app.js` (`source/web/blockly-station/app.js.md`, generated reference),
`ros-blocky-port.js` (`source/web/blockly-station/ros-blocky-port.js.md`, generated reference), HTML and
CSS implement the project-owned editor. The port adapts ROS Blocky logging
blocks; its pinned upstream attribution remains in
`web/blockly-station/THIRD_PARTY.md`. These application files are included in
full. The unmodified Blockly 10.4.3 distribution under `vendor/` receives an
explicit third-party exclusion and upstream reference in the coverage report.
Its minified APIs are not presented as project-authored code.

`tools/run_blockly_station.py` (`source/tools/run_blockly_station.py.md`, generated reference) hosts
the loopback editor and bounded API; simulation is the default. The browser
produces a bounded JSON program rather than executable Python.
`workshop/blockly_program.py` (`source/workshop/blockly_program.py.md`, generated reference) validates
its schema/limits and implements the interpreter and simulation backend.
`blockly_recording.py` (`source/workshop/blockly_recording.py.md`, generated reference) validates
recorded measured strokes and replay points.
`blockly_ros.py` (`source/workshop/blockly_ros.py.md`, generated reference) adapts the program to the
existing ROS action/service contracts when physical mode is explicitly selected.
Executable Python export uses the shared client to submit this same validated
AST through the station service. The web server interprets the AST; it does not
execute uploaded Python. Teaching content comes from `curriculum/workshop.json`
and its generated lesson plan, activity cards and browser mission projection.

Recording/replay pins owner, calibration and source identity. Replay requires
the recorded starting pose and a fixed jaw; jaw-gap commands have a separate
qualified-calibration requirement. STOP verifies the returned request ID against
terminal station status. Unknown command delivery remains unresolved until
explicit recovery; the Blockly adapter never treats it as success or retries it.
`docs/blockly-workshop.md` describes the interface and teaching boundaries.
The static reference build does not start this server or contact ROS.

## EV3 operator interfaces

The native `EV3Panel` and `OperatorPanel` are documented by Doxygen, with complete
source pages for their C++ headers and implementations.
`workshop/station/ev3_operator.py` provides the guarded EV3 operation state;
`workshop/station/sorter_endpoint.py` shares the conservative target/endpoint
motion phase. The operator/workflow wrappers expose these through existing ROS
interfaces. Measured job IDs, completion, cancellation and telemetry checks
remain part of the implementation, not evidence of a qualified physical cycle.
