# fwmini4 classroom PC — 22 October 2026

Use the **bjoern** Linux login. The active graphical session found during remote preparation belonged to `cpsadmin`; its desktop and login settings were not changed. A physical display rehearsal in the bjoern session is still required.

The deployed checkout is `/home/bjoern/git/teaching-exciting-science-2026-current-20261021`, tracking `https://github.com/mul-cps/teaching-exciting-science-2026.git`. Update only with a clean working tree and `git pull --ff-only origin main`.

## Prepared launchers

In the bjoern application menu, open **Student workshop** , **Instructor live lab**, or **Instructor robot and RViz**. Preparation also creates the configured bjoern desktop directory and places the same launchers there. A desktop may ask to trust the launcher on first use.

From a terminal on the PC:

```bash
cd ~/git/teaching-exciting-science-2026-current-20261021
~/.local/share/exciting-science/workshop-20261022/venv/bin/python tools/prepare_workshop_pc.py student
~/.local/share/exciting-science/workshop-20261022/venv/bin/python tools/prepare_workshop_pc.py live
```

Student mode opens the student notebook in authenticated JupyterLab on `127.0.0.1:8888`, with the repository as its file root. Repeated clicks reuse a matching server. Jupyter prints its authenticated URL in its launch terminal/log; do not disable its token. Instructor mode delegates to the dedicated read-only live lab launcher. Its live observations require an instructor-started station; with no hardware connected, use its clearly labelled invented demonstration.

Remote viewing uses an SSH tunnel, keeping Jupyter private:

```bash
ssh -L 8888:127.0.0.1:8888 bjoern@cps-wkstn-fwmini4.local
```

Use the authenticated URL from the server on your own browser. Do not copy tokens into published reports.

## Environment and preparation

The classroom Python environment is separate from robotics Conda and the Slang/Vulkan runtime. System Python prepares it:

```bash
python3 tools/prepare_workshop_pc.py prepare
```

Preparation requires internet for Python packages. Installed versions are recorded in `~/.local/share/exciting-science/workshop-20261022/requirements-installed.txt`; `pip check` must pass. Later notebook execution, console exercises, local photographs, and the HTML sorting simulation need no remote assets. Optional Teachable Machine and Maze sites still require internet; use the local notebook and exercises when unavailable.

## Station and RViz

For the complete manual sequence, fresh joint-limit contract, EV3 native API/tunnel and single-owner recovery, use [the full station startup guide](station-startup.md).

The Student and Instructor live lab launchers only open notebooks. **Instructor robot and RViz** explicitly starts the existing resident owner, read-only leader, camera, and RViz after checking the saved follower calibration hash, saved home identity, and both exact USB serial paths. It preserves the existing owner lock and does not enable teleoperation, calibrate hardware, or run sorting. With no arms connected it fails closed and leaves a terminal explanation. Inspect the existing launcher with `~/.local/bin/station-fwmini4 --help`. Its ROS environment remains the separate pinned Conda installation, with default domain 177 and localhost discovery. Domain 76 belongs to the existing station and must not be started by these classroom launchers.

The reviewed station configuration is `~/.local/share/exciting-science/station/station-fwmini4-20261022.json`; the previous template remains unchanged. Saved calibration JSON files retain their exact original contents in `~/git/soarm101-highschool-demos/.calibration`. The transferred measured home is a historical saved pose, not proof of the currently connected arm state. No owner contract, lease, or previous live feedback was transferred.

No-device launch gate check (expected to refuse missing arms):

```bash
python3 tools/prepare_workshop_pc.py station-preflight
```

The camera CPU repair is built using the existing wrapper, without attaching devices:

```bash
ros2/conda/so101-conda.sh realsense-cpu-build
```

Before starting hardware tomorrow, the instructor must identify attached devices and select the station configuration and saved calibration for this PC. Use one existing resident follower owner. RViz should attach to that station rather than launching a second owner. Existing teleoperation instructions are in [so101-ros-teleop.md](so101-ros-teleop.md); their Rig01 default serial and calibration paths require explicit configuration for this PC. The notebook is observation only, and live readings do not prove physical sorting or a 60 Hz control rate.

## Rehearsal evidence

The final report is copied into the instructor evidence directory `workshop-beginner-rebuild/fwmini4-final`. It records checkout SHA, dependency checks, fresh kernel notebook output, authenticated loopback HTTP, console exercises, and the extracted learner ZIP. Hardware, USB cameras, physical display, live control rate, and pick and place remain separate acceptance steps.


## First hardware connection: passive contract bootstrap

The classroom notebook checks are complete without devices. The robot launcher additionally requires a **new contract generated on this PC**, after the identified arms are connected and all previous owners are stopped. A transferred historical home or another machine's runtime contract cannot satisfy this step.

Run this once from a bjoern terminal after connecting and supporting the arms. It starts the existing **observe-only** owner briefly, reads the saved calibration and actual firmware/admitted bounds, exports the contract, then closes that owner. It does not calibrate, home, enable teleoperation, or command motion. Do not run it while a resident owner is already running.

```bash
cd ~/git/teaching-exciting-science-2026-current-20261021
~/.local/share/exciting-science/station/venv/bin/python tools/freeze_ros_joint_limits.py \
  --config ~/.local/share/exciting-science/station/station-fwmini4-20261022.json \
  --state-file "${XDG_RUNTIME_DIR:-/dev/shm}/exciting-science/arm-state.json" \
  --output ~/.local/share/exciting-science/station/joint-limits-current.json \
  --lock "${XDG_STATE_HOME:-$HOME/.local/state}/exciting-science/station.lock" \
  --log-dir "${XDG_RUNTIME_DIR:-/dev/shm}/exciting-science/arm-owner-logs" \
  --bootstrap-owner
```

Only after successful bootstrap, open **Instructor robot and RViz**. Its launch forwards that exact contract, state file, lock and log directory to the existing resident owner. The planner and owner independently validate calibration, source and bounds; source changes can require a fresh bootstrap. The retained historical measured home remains separate from current live pose and homing admission. This physical bootstrap has **not** been tested on fwmini4 because no devices were connected during preparation.

R5 software rehearsal on 21 October passed three fresh notebook kernels, authenticated loopback Student/Instructor apps, all six extracted ZIP entrypoints with system Python without installed packages, both ZIP notebooks with the prepared kernel, nine bundled training PNGs, 18 slide PDF pages, domain75 fake ROS freshness recovery, and offscreen native Qt plugin checks. Actual display/login, webcam permissions, Teachable Machine training, EV3 hardware and physical robot execution remain unverified.

## Blockly launchers

The bjoern desktop and application menu also provide **Student Blockly simulation** and **Instructor Blockly PHYSICAL ROS**. Both open `http://127.0.0.1:8877`; keep its separate STOP port `8878` free. All editor assets are local files. The simulation server uses standard Python and does not import ROS or hardware drivers. On another student laptop, extract the full repository and run:

```bash
python3 tools/run_blockly_station.py
```

The physical launcher is an explicit instructor action. It uses `station-fwmini4 ros-vulkan` with domain **177**, `--physical --instructor`, and the existing station's guarded ROS interfaces. Start and qualify the station separately first. Opening Blockly never starts a serial driver, starts the station, homes, enables teleoperation, or executes a program. Real program execution and instructor controls can command the already running station. Unconfigured restart controls remain unqualified.

Repeated clicks reuse only the same checkout and mode, with a matching live process identity and HTTP mode. A simulation server is never converted into a physical server. If the other mode is running, close it explicitly before selecting the new mode:

```bash
cd ~/git/teaching-exciting-science-2026-current-20261021
python3 tools/prepare_workshop_pc.py blockly-close
python3 tools/prepare_workshop_pc.py blockly-simulation
# Explicit instructor physical adapter, after station qualification:
python3 tools/prepare_workshop_pc.py blockly-physical
```

The launcher records the managed process/session identity under `~/.local/share/exciting-science/workshop-20261022/blockly-server.json` and keeps simulation and physical recordings in separate directories there. Explicit close uses the existing bounded process cleanup; closing a physical adapter requests station STOP, whose acknowledgement must be checked in its log and the station. Stale records are archived without signalling a reused PID. An occupied port without matching ownership is refused; the launcher never replaces an unrelated server. Logs are in `blockly-server.log` alongside the record. See [blockly-workshop.md](blockly-workshop.md) for the lesson and program limits. Physical Blockly acceptance on fwmini4 remains unverified.

## Final Blockly software rehearsal

The final remote software rehearsal used published source **`a4e859464f16144b09f7fc4105b8db752f93f619`** in the existing checkout, under **bjoern**. Its working tree was clean. The earlier R5 evidence remains a separate historical snapshot; notebook source hashes still match those fresh-kernel results.

- The installed simulation launcher reused its exact managed process. An attempted mode change was refused before any ROS adapter launch. Two immediate close/reopen cycles released both ports and created distinct owned process identities.
- Headless Chromium, connected through an isolated SSH proxy to the N100's loopback server, passed program execution, workspace save/load, block dragging, logging/Python preview and the separate STOP listener. Browser-only status fixtures confirmed that missing, stale, busy and unconfigured capabilities disable instructor controls. There were no JavaScript errors. This did not test the PC's physical display or a physical adapter.
- 59 focused software tests and seven documentation tests passed. The ROS workspace rebuilt all 21 packages. The existing fake-owner retry tests passed all 35 parameterized cases on isolated ROS domain 75; their original fixtures were invoked directly because normal pytest collection encounters an unrelated optional `trimesh` import. No geometry packages were added to ROS.
- The actual published learner ZIP ran its Blockly repeat program and console exercises with `python3 -S`. It includes vendored browser assets and nine training PNGs, and excludes the physical ROS adapter. The refreshed static reference passed 715 source pages and 722 HTML pages with local links and anchors checked (`--no-doxygen`).

A fresh, idle **SIMULATION** server was left ready on ports 8877/8878 with an empty action trace; the desktop launcher reopens it. Results are stored under `~/.local/share/exciting-science/workshop-20261022/evidence/` and copied to the instructor's `workshop-beginner-rebuild/fwmini4-final` evidence folder. The generated reference is outside the checkout at `~/.local/share/exciting-science/workshop-20261022/code-reference/site/index.html`.

**Hardware commissioning remains open on fwmini4.** No real joint-limit contract was generated, no physical Blockly server or station units were started, and no actuators were accessed. Updated owner source requires the real passive bootstrap after device attachment. Individual service units are still unconfigured and service restarts disabled: browser capability checks and Qt mocks do not qualify Restart MoveIt, full reset or physical STOP here. Display/login, cameras, Teachable Machine training, EV3 hardware and pick/place remain unverified. Only bjoern's launchers were prepared; cpsadmin's desktop was not changed.

## Explicit native recovery profile (software verified; hardware uncommissioned)

The operator API now has an explicit `recovery_profile:=native_ros` option for
native CPU RGB-D and Vulkan masks. The default remains `derived_rocm`; its
coordinator and derived-source admission checks are unchanged. Native recovery
uses the existing owner/maintenance and correlated unit-job protocol, without
adding a coordinator, source protocol, or additional hardware owner.

Native configuration must supply `native_publisher_nodes_json`: an exact map
from each of `rgb_image`, `depth_image`, `rgb`, `depth`, `tags`, `objects`,
`objects_status`, `scene_status`, `joint_states`, and `movegroup` to its absolute
ROS node name. This must come from the reviewed component manifest, not names
inferred from publisher counts. Explicitly set the native aligned depth image
and CameraInfo topics, and `native_mask_backend:=vulkan`. The existing object
status endpoint is `/so101/grasp_status`. The launch files expose these settings;
the integrated launch prefixes them with `operator_`.

The proof requires unique expected publishers, changed DDS identities for
restarted components (an absent old publisher is allowed), unchanged identities
for other components, and fresh advancing actual payload metadata. It joins tags
and tracking detections to observed camera exposures, checks the actual selected
mask backend, and joins the newest settled scene/application to the newest
admitted detection IDs. Invalid/unavailable/busy status revokes current readiness.
The original scene/world, measured-joint bracket, owner identity, six-torque-OFF,
STOP cancellation, exact manager job and unknown-outcome gates remain. Images
stay subscribed through the final MoveIt read; final source/receipt freshness
and publisher identity are checked again afterward. Metadata windows are bounded
to 128 records per endpoint; image pixels are neither copied nor scanned.
Existing float scene/status stamps receive a 1 microsecond join tolerance;
integer image/tag/detection stamps match exactly.

This is software recovery evidence only: it establishes neither ROCm lineage nor
image quality, geometric accuracy, or physical pick-and-place qualification.
Remote component units are still a deployment/commissioning prerequisite;
unconfigured Restart MoveIt / Restart entire station buttons remain unavailable.

### Review evidence, 2026-10-21

Source base: `fc42d53`. In the isolated native-recovery worktree, 135 focused
operator/native/emergency/reset/teleop tests passed. Three real ROS domain75,
localhost-only fixture tests passed in 5.708 seconds using the existing D6
OperatorMaintenance interface overlay. The fixture uses fake owner/manager
services, actual DDS payload callbacks and publisher replacement. It covers full
reset with absent camera endpoints, MoveIt restart, fresh→stale→fresh evidence,
expiry during the MoveIt read, and an unknown manager job retaining the barrier.
It does **not** exercise real systemd processes, hardware, or a real MoveGroup.
The independent review measured approximately 1.21 ms per metadata check with
128-row windows on the development host; target N100 timing is unmeasured.

Reproduce unit tests:

```sh
python -m pytest -q tests/test_operator_native_recovery.py \
  tests/test_station_emergency_reset.py tests/test_operator_moveit_restart.py \
  tests/test_operator_restart.py tests/test_operator_teleop_modes.py
```

Run `tests/operator_native_recovery_domain75.py` only in a built ROS overlay
with the actual generated OperatorMaintenance interface available, and explicitly
set `ROS_DOMAIN_ID=75 ROS_LOCALHOST_ONLY=1`. It refuses other domains. No physical
units, devices, actuator operations, or domain76 were used for these checks.

### Implemented component units; disabled installation and commissioning

The component installer and guarded lifecycle are implemented in source commit
`9ed044c50d40f7951cbe773c5569c5e335780bb1`. This supersedes the earlier unit-recipe
plan. Installation and physical commissioning are separate steps. No native
component units were started or enabled during these software checks.

The explicit `component` enum selects existing station actions:
`owner`, `camera`, `tags`, `movegroup`, `objects`, `scene`, `leader`, `apis`, or
`rviz`. Default `integrated` behavior is retained. Only `owner` starts the resident
bridge and robot state publisher; only `camera` starts the camera driver. Tags
run without a second camera. APIs, the owner, and RViz are excluded from operator
restart allowlists; camera and depth refer to the same actual camera unit.
There is no new controller, coordinator, or runtime supervision loop.

The native manifest explicitly pins **RGB 640×480×30 and depth 640×480×30**,
matching the prepared `station-fwmini4 camera-lowpower` wrapper recipe (line55,
checked read-only on the PC). These are candidate low-power settings, **not** a
measured live camera/vision throughput result. Integrated defaults remain RGB
1920×1080×30 and depth 1280×720×30. Every component command uses the same manifest
parameters, source hashes, domain177, CPU repair/SDK paths, and Vulkan backend.

#### Render and install disabled files

Use the tested `bjoern` account and the clean published checkout. Build its ROS
overlay before commissioning; unit startup verifies installed launch/API/control
origins against the current source files and refuses a stale overlay.

```sh
cd ~/git/teaching-exciting-science-2026-current-20261021
python3 tools/install_station_user_units.py render \
  --output-dir /tmp/fwmini4-units-review \
  --profile fwmini4 --color-mask-backend vulkan
```

After source review/merge and the matching ROS overlay build:

```sh
python3 tools/install_station_user_units.py install \
  --profile fwmini4 --color-mask-backend vulkan
systemctl --user daemon-reload
systemd-analyze --user verify \
  "${XDG_CONFIG_HOME:-$HOME/.config}/systemd/user"/so101-fwmini4-*.service
```

The installer writes `~/.local/share/exciting-science/station/fwmini4-units.json`
and eight `so101-fwmini4-*.service` files under the current user's configuration
directory. It does not reload, enable, start, open devices, or generate a contract.
Units have `Restart=no` and no `[Install]`/`WantedBy` autostart section. Different
existing files are preserved and installation refuses to overwrite them.

If an already verified EV3 SSH forward will be reused, include `--existing-tunnel`
in **both** generator/install and startup commands. Otherwise the API component
owns the same explicit tunnel recipe as integrated startup.

#### Later connected-device commissioning: explicit start and stop

Run these commands after the devices are attached and during connected-device
commissioning; they were **not run** during disabled preparation. The existing exact USB,
calibration, CPU repair and EV3 prerequisite checks remain mandatory. Contract
bootstrap is passive and uses the identified devices; it is never replaced by a
copied historical contract.

```sh
./tools/start_station.sh --profile fwmini4 --deployment user-units \
  --color-mask-backend vulkan --bootstrap-owner
./tools/stop_station.sh --profile fwmini4 --deployment user-units
```

Unit startup returns only after the existing non-motion EV3 Connect, correlated
station STOP completion and fresh matched six-joint torque OFF. This does not
qualify pick-and-place or prove that Restart MoveIt/full station recovery works
on connected hardware. Those buttons remain a commissioning gate.

To open **RViz only** after the managed native station has been commissioned,
use its exact manifest command; this role cannot start another owner:

```sh
python3 - <<'PY_RVIZ'
from pathlib import Path
import subprocess
from tools.install_station_user_units import load_manifest, component_command
manifest = load_manifest(Path.home() / '.local/share/exciting-science/station/fwmini4-units.json')
subprocess.run(component_command(manifest, 'rviz'), check=True)
PY_RVIZ
```

The existing integrated robot desktop route must not run alongside unit mode.
The tested account remains `bjoern`; physical display login remains unverified.

#### Failure handling and software evidence

Startup/shutdown records are bound to the reviewed manifest, exact user-manager
bus/boot identity, real unit fragment, InvocationID, PID/starttime and cgroup.
Legitimate panel restarts are recognized only through the existing owner's
correlated resolved manager receipts. STOP is attempted for the same matched
owner even when it is faulted or has pending external work. Unit termination
requires the matching workflow STOP `request_id` to report `pending=false`,
`ev3_stopped=true`, `arm_disarmed=true`, plus a fresh measured six-OFF follower.
Trigger admission alone is insufficient.

A failure before the workflow API starts attempts independent follower STOP
and **retains the prefix ownership record for inspection** because workflow/EV3
completion cannot be proved. A new failed owner invocation, unknown manager/STOP
outcome, changed identity, or unknown cleanup also retains the record; there is
no broad kill, automatic retry, or automatic second owner. Resolve the recorded
condition before attempting another startup.

Software verification: **51** lifecycle/renderer/process tests passed, including
negative manifest, missing API, unknown STOP, request-correlation and failed-owner
cases; **186** combined lifecycle and operator regressions passed. Four actual
ROS launch-role test bodies and six original camera launch tests passed without
executing nodes. Ordinary ROS pytest collection was blocked by an unrelated
optional `trimesh` skip, so those ten original bodies ran directly. Independent
review approved the frozen core and measured median 3.74ms/p95 4.34ms to render all
eight unit strings on the development host. No timers, publishers or subscriptions
were added to the lifecycle/installer modules.

Local `systemd-analyze --user verify` accepted all eight syntax fixtures, which
substituted only `/usr/bin/true` for the PC-only wrapper. Production unit checking
on the development host reported the expected missing
`~/.local/bin/station-fwmini4`; exact production verification belongs on the PC
after disabled installation. This component-unit revision has not
been installed remotely; no native units were started, and no physical/domain76
acceptance is claimed by this report.

## N100 synthetic gripper camera replay, 2026-10-22

The existing `GpuPointCamera` EGL/GLES visualization renderer passed a bounded,
offscreen replay on fwmini4's Intel N100. GL vendor was `Intel`, renderer
`Mesa Intel(R) Graphics (ADL-N)`, and version
`OpenGL ES 3.2 Mesa 25.2.8-0ubuntu0.24.04.4`; the existing hardware-renderer
guard passed. Vulkan/Slang masks remain the primary compute offload. This result
covers the existing synthetic camera visualization, with no backend changes.

The remote checkout was clean at `fc42d532d577db546180883d8efb7e4df4fe0f7d`.
Although newer instructor source was at `f6e190f`, the three exercised
renderer/geometry/mesh files matched it byte for byte. Each case used three
warmup frames and 30 measured frames paced at 3 Hz, with 20,000 fixed RGB-D
samples from the approved saved aligned capture. CAD cases included all three
gripper visuals, totaling 76,146 triangles. Source intrinsics and recorded
transforms were preserved; offline URDF FK matched the recorded gripper transform
to a maximum matrix-element difference of `3.33e-16`.

| Resolution | CAD | Render p50 / p95 (ms) | Replay p50 / p95 (ms) |
|---|---|---|---|
| 320×240 | off | 4.77 / 4.98 | 9.48 / 9.71 |
| 320×240 | on | 6.25 / 6.49 | 11.01 / 11.24 |
| 640×480 | off | 10.63 / 11.11 | 15.89 / 16.35 |
| 640×480 | on | 12.62 / 12.97 | 17.88 / 18.32 |

Replay time includes resident RGB-D sample gather/deprojection, rendering,
readback, provenance construction and byte serialization. It excludes camera
acquisition, file/image decoding, ROS/DDS publication, TF lookup, annotation and
GUI. Valid sample indices and cached mesh uploads were prepared before timing.
Readback waits include pending GPU draws; these are wall-clock measurements,
not GPU timestamps. Both measured and CAD pixels were present in output images.
Keep the production 320×240, 20k-point, 3 Hz defaults: its CAD replay p95 was
11.24 ms within a 333 ms period. This isolated result does not establish headroom
with all station workloads running concurrently.

The workshop venv lacked SciPy, so the run used the existing
`station-fwmini4 ros-vulkan python` environment (SciPy 1.16.2), with one BLAS/OMP
thread and no dependency installation. The active seat0/tty2 desktop belonged
to cpsadmin; bjoern's sessions were headless. No GUI access or RViz GUI replay
was attempted. No motion or camera acquisition was performed. The saved capture
remains explicitly unqualified for physical gripper mapping and base calibration;
this replay establishes neither geometric accuracy nor physical acceptance.

Raw captures, benchmark script, per-frame timings, output images and full report
remain outside Git in the instructor evidence directory:
`~/.local/share/exciting-science/station/evidence/2026-10-21-sorting/workshop-beginner-rebuild/fwmini4-render-check/`.
The source capture was
`pick-place-resume/restored-station/deploy-v7-start-retry/jaw-orient/baseline-aligned/`
under the same `2026-10-21-sorting` evidence root. Exact SHA-256 pins:

```text
# Exercised source, identical in remote fc42d53 and instructor f6e190f:
ros2/src/so101_station_perception/so101_station_perception/virtual_camera_gpu.py
98deffedd572ab8577273c9dac9c479f073f5d5c8e8fa289fab03e2a5cb14f16
workshop/digital_twin/virtual_camera.py
4b084b3be754b7f6fb2f58e0cd33bde55cb547182bc6926aaca6fd81bcce0f06
workshop/digital_twin/gripper_mesh.py
6abdbc8ddd00b4e001c94819539ec8c3e01d28a21f37725c960c85cca8e1dcfe
# Approved saved input bundle:
capture.json 065abfcb5055af72afba491d13757a8b3a0f63ff37187ee2ba93dc4aee089d74
depth.npy    cfa7f7e426bd4942e8f14d1c724c129db6024b14292ee09a3d7a691f0009f8fd
color.png    f88abf0ec0c2fc5e6061b3c100333d76c7a7a151aa34b53870336354d7ac98c9
robot.urdf   090c1aa1bc81c8d30d27f6b0cbe7efa5c0b9ad6a483b983ce79fd3ebf2ff03ef
```
