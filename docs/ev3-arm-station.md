# EV3 and SO-ARM101 station

The station supports `arm_loads_sorter` (arm loads a stopped sorter) and
`arm_unloads_sorter` (EV3 sorts, stops, then arm picks and places). Original EV3
firmware over direct USB is the default. The [official Color Sorter](https://pybricks.com/ev3-micropython/examples/color_sorter.html)
uses feeder port A and sorter belt port D. The observed model has a medium
feeder on A (`lego-ev3-m-motor`, original firmware type 7) and a large sorter
on D (`lego-ev3-l-motor`, type 8); inverted motor types are rejected. The optional legacy EV3 Pybricks
MicroPython **major version 2** backend uses a pinned SSH connection and a
bundled brick agent. Current Pybricks hub/browser tooling is not this backend.

Software simulation, protocol fakes and installer verification do not establish
physical acceptance. Neither firmware/workflow combination has completed a
physical station cycle. On the current host the EV3 is absent. The existing arm
home contains a raw value of **949**, below its calibrated minimum **1063**;
physical preflight must reject it. Do not relax those limits or rewrite EEPROM
to admit that home. Commission a valid measured home first.

## Linux installation

Offline simulation requires Python 3.11+. Physical arm packages require Python
3.12 or 3.13, internet access, system `libusb-1.0`, and (for Pybricks) OpenSSH.
From a checkout:

```bash
STATION_PYTHON=python3.13 tools/install_station.sh
```

The installer creates `$HOME/.local/share/exciting-science/station/venv`, installs
the package (not an editable checkout), preserves `station.json` on rerun, and
creates a `station` launcher in that directory. Override `STATION_INSTALL_DIR`
for an alternative location. It never changes system Python, USB permissions,
network settings or firmware. It verifies both simulations and local read-only
inventory, with no hardware connections. It uses `uv` if available, otherwise
Python's `venv` and that environment's `python -m pip`.

CPU torch 2.11.0+cpu and torchvision 0.26.0+cpu come first from the official
PyTorch CPU index. LeRobot[feetech] 0.6.1 and PyUSB 1.3.1 come from PyPI, with
CPU pins retained during the second installation. Feetech SDK 1.0.0, pyserial 3.5
and deepdiff 8.6.2 are pinned to the verified arm stack. The dependency check and
imports verify CPU rather than CUDA/ROCm builds. Transitive dependencies are
resolved at installation time; this is not a hash-locked or disconnected bundle.
Installing the `station` extra directly needs those CPU wheels staged first.

The account needs read/write access to the exact EV3 USB node and arm serial
adapter. Inspect `ls -l /dev/serial/by-id/` and the matching `/dev/bus/usb` node.
Ask the host administrator for narrowly scoped VID/PID/serial udev permissions
or the site's serial group/ACL; reconnect after changes and renew login for
group membership. The installer does not assume sudo or install rules. Verify
system libusb through the installer's backend check; PyUSB alone cannot supply
it. Never use the first available tty device as the arm identity.

## Read-only inventory and simulation

```bash
station="$HOME/.local/share/exciting-science/station/station"
"$station" --detect
"$station" --simulate --workflow arm_loads_sorter
"$station" --simulate --workflow arm_unloads_sorter
```

`--detect` only reads local USB sysfs and serial symlinks. It neither opens a
device nor connects via SSH; Pybricks remote identity remains unverified until
explicit activation preflight. Original EV3 identity must match 0694:0005 plus
the commissioned serial. Arm identity requires its exact VID/PID/serial and one
persistent by-id path. Adapter identity does not itself prove follower role.

For Pybricks, configure firmware `pybricks`, the reachable SSH host/user/port,
optional private-key path, and the brick's expected `/etc/machine-id`. Provision
key authentication and verify the host key independently before activation;
BatchMode and StrictHostKeyChecking remain enabled. The remote brick needs
legacy `pybricks-micropython` 2, `brickrun` and Python 3 for independent platform
and port-driver identity checks. The backend deploys the packaged agent under
its configured remote directory. It never configures Wi-Fi, USB networking or
password login, and never applies the original firmware USB filter to SSH.

## First commissioning

First-time calibration, pose teaching, clearance geometry and grasp validation
are unavoidable physical setup. Repeat startup can be automated after these
are completed; the template intentionally invents no arm poses or routes.

Set the exact identities, actual motor/sensor ports and directions, measured
finite feeder homing travel, calibrated limits and file SHA256, and arm ID.
The calibration filename must match the arm ID. Body poses use degrees and the
gripper uses percent. With the independent stop accessible and the setup
supervised, capture supported torque-off poses:

```bash
python="$HOME/.local/share/exciting-science/station/venv/bin/python"
config="$HOME/.local/share/exciting-science/station/station.json"
"$python" -m workshop.station.commission --config "$config" \
  --capture-pose home --output home.json --update-config "$config"
"$python" -m workshop.station.commission --config "$config" \
  --capture-pose clearance --output clearance.json --update-config "$config"
```

Also teach `source`, `loading`, `pickup` and all four color destinations, plus
any intermediate waypoints needed for collision clearance. Pose capture opens
the selected arm and verifies torque off; it does not command movement or
perform automatic calibration. Route export is non-actuating and can be done
without devices:

```bash
"$python" -m workshop.station.commission --config "$config" \
  --set-route home home --update-config "$config"
"$python" -m workshop.station.commission --config "$config" \
  --set-route clearance clearance --update-config "$config"
"$python" -m workshop.station.commission --config "$config" \
  --set-route load_piece source loading --update-config "$config"
"$python" -m workshop.station.commission --config "$config" \
  --set-route pick_and_place pickup blue --color blue --update-config "$config"
```

Repeat color routes for green/yellow/red and add measured intermediate poses as
needed. Each adjacent move must satisfy the displacement envelope. Commands
back up changed files and reset `commissioned` and `automatic_start` to false.
Capturing poses or exporting routes does not validate reach, collision geometry,
feed clearance or grasp. Complete supervised bounded acceptance for the chosen
backend and each workflow before setting `enabled`, `commissioned` and
`automatic_start` true in the configuration. Keep finite `cycle_count`.

## Repeat activation and faults

```bash
"$station" --activate
```

Activation acquires the process lock, admits configuration, connects and
preflights both devices, confirms EV3 stop before arm home, confirms arm
clearance before EV3 home, then runs the configured finite cycles. A Pybricks
lease fault prevents the next operation and shares cancellation with the arm.
There is no automatic reconnect or feed replay. Keep all processes under the
same account and default lock path for activation and commissioning. Both
entry points share the same lock; capture holds it through shutdown and config
export. Route-only export also takes it to serialize configuration teaching
against activation. If overriding `--lock`, pass the identical path to both
commands; different paths defeat mutual exclusion.

Logs append JSON lines to
`$XDG_STATE_HOME/exciting-science/station.jsonl` (default
`~/.local/state/exciting-science/station.jsonl`); arm plans/traces are under
`exciting-science/station-arm`. Use `--log PATH` to choose a run log. Ctrl-C or
SIGTERM records cancellation and attempts shutdown. All exits independently
attempt both stops before both closes, including startup and logging faults.
Cleanup errors preserve the originating fault. The software is not a hard
real-time safety controller; OS/serial calls can block and the independent
physical stop remains necessary.

After a fault use the independent stop, inspect the piece, motor state, logs,
identity, calibration and geometry, then explicitly restart a new session only
when the cause is resolved. Never retry a feed whose physical completion is
uncertain. Physical acceleration, USB timing, real color readings, reliable
homing/grasp and both full workflows remain pending bench acceptance.
