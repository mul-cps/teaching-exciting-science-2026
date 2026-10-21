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

## Explicit native recovery profile (implementation review, not commissioned)

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

### Next deployment task: small portable unit recipe

Prepare a user-relative manifest and real user units, without enabling or
starting them during installation:

| Unit role | Existing composition to reuse | Restart allowlist |
| --- | --- | --- |
| Resident owner + robot state publisher | Split `moveit_resident.launch.py` with explicit bridge/RSP/MoveGroup switches; retain singleton lock and real contract bootstrap | Excluded |
| CPU camera | `perception.launch.py` with marker/tag/diagnostic children disabled, exact serial/profile and validated CPU repair/SDK paths | Camera and depth share this one unit |
| Tags | Native marker pipeline with explicit camera topics and existing calibration inputs | Tags |
| MoveGroup | Same resident launch with bridge/RSP/leader/RViz disabled | MoveGroup |
| Vulkan objects | `objects.launch.py`, native aligned depth, `color_mask_backend:=vulkan` | Objects |
| SceneUpdater | Existing scene launch with exact objects/MoveIt service endpoints | Scene |
| Read-only leader, when required | Existing calibrated reader launch | Read-only leader |
| Operator/workflow APIs and optional EV3 tunnel | Existing API/SSH commands and reviewed configuration | Excluded |
| RViz | Existing guarded RViz command in the logged-in desktop session | Excluded |

Every command uses the existing `station-fwmini4 ros-vulkan` environment after
clean Conda activation. Generate API and bridge allowlists from the same manifest;
forward all reset-unit slots to the owner. Extend the existing start/stop lifecycle
for that explicit unit profile, preserving STOP acknowledgement, fresh real
contract generation, and PID/owner/lock checks. Keep current integrated startup
and local ROCm defaults. This unit recipe is a plan, not an installed or qualified
remote configuration.
