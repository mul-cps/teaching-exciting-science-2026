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
[exercises inventory](inventory.md#exercises).

The standard simulator runs with the Python standard library. Optional camera
capture adds OpenCV/NumPy, while arm and conveyor remain simulated in this path.
The exercise and material builders are developer utilities, not station control
entry points. `tools/build_materials.py` builds handouts and slides;
`tools/make_assets.py` creates teaching illustrations.

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

Use the [station module inventory](inventory.md#workshop) to follow individual
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
backend kernels/compatibility operations. `tools/intel_grasp_backend/`,
`tools/recovery_cgal/` and the projection/ROCm tools are separate native build
and diagnostic entry points. Their presence does not select them for a live
pipeline. The generated native source pages and Doxygen reference retain these
boundaries rather than implying one universal backend.

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
are all discoverable in the [tools inventory](inventory.md#tools). Read their
`main` implementation and argument parser before choosing a command: many
instructor tools can request physical access when explicitly enabled.
