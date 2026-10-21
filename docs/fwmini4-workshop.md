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
