# EV3 Color Sorter and SO-ARM101 station

## Goal and user decisions

Provide a self-contained, easily installed PC-controlled station using the LEGO
Education EV3 Core Set 45544 Color Sorter and the existing calibrated LeRobot
SO-ARM101. Support both configurable workflows: `arm_loads_sorter` and
`arm_unloads_sorter`. Support original EV3 firmware and an EV3 Pybricks runtime.
After initial commissioning, the operator powers on the devices and runs one PC
command; discovery, connection, startup checks, and bounded homing are automatic.
The user approved the shared-coordinator design on 2026-10-13 and subsequently
requested self-contained installation and automatic activation.

## Installation and firmware

Provide one PC installer and one station entry point. Pin optional hardware
dependencies, document supported OS/Python versions, and ship configuration,
runtime scripts, and simulation fixtures in the repository/package. The offline
simulator remains usable without attached hardware. Do not change host Ethernet,
DNS, routing, or unrelated network configuration.

The original-firmware backend uses the EV3 direct-command protocol over USB;
it requires neither a brick-side Python interpreter nor a firmware replacement.
The Pybricks backend uses a supplied EV3-side agent and a PC transport verified
against the selected EV3 MicroPython environment. Provision that agent from the
PC once the supported runtime is installed. Installing an EV3 MicroPython SD
image is a first-time provisioning step, not something silently performed on
every boot. The two backends implement the same station operations. Transport
details and dependency choices must be verified from primary documentation
before implementation; do not substitute the existing ev3dev2/RPyC probe for
either backend.

## Hardware configuration

Official example defaults: feeder motor A, sorter motor D, touch sensor S1,
color sensor S3. Default color destinations in sorter motor degrees are blue
10, green 132, yellow 360, and red 530. These are model defaults requiring
commissioning checks, not measured properties of the attached station.

Configuration includes firmware backend, expected EV3 identity, arm USB identity,
arm calibration identity/hash, workflow, port mappings, motor directions,
homing speed/travel/time limits, color destinations, operation timeouts, taught
arm poses and bounded trajectories, tolerances, and enabled/commissioned state.
Reject duplicate ports, unknown workflows/colors, invalid limits, nonfinite
values, missing taught poses, and mismatched calibration before actuation.
Prefer persistent USB identities over changing device-number paths.

## Automatic startup and calibration

1. Discover actual USB devices and identify the EV3 and SO-ARM; fail clearly on
   absence or ambiguity. Do not select an arbitrary serial adapter.
2. Connect and check firmware/backend, motor and sensor ports, calibration
   identity, stored pose limits, and configured station geometry.
3. Load existing arm calibration. Missing or inconsistent calibration enters a
   commissioning-needed state with arm torque disabled; never infer mechanical
   limits by driving joints to stops.
4. Arm through the existing atomic current-position startup helper and move
   along the validated bounded path to the taught clearance/home pose. Confirm
   encoder arrival before allowing EV3 homing or station movement.
5. Home the EV3 sorter against S1 with bounded travel/time and the commissioned
   direction. Initialize the feeder with a bounded model-specific procedure.
   A failed sensor or homing operation stops the motors and prevents cycling.
6. Announce readiness and execute the selected workflow when the commissioned
   automatic-start setting is enabled. Stop after the configured cycle count
   or on cancellation/fault.

First-time arm calibration and teaching loading/pickup/drop/clearance poses
require physical commissioning. Repeat startup automatically verifies and loads
that calibration and re-establishes the home reference; it does not claim to
perform unobservable geometry calibration. The current saved arm home has
shoulder raw 949 below the configured minimum 1063; startup must reject it until
that commissioning mismatch is explicitly resolved. Do not silently alter
servo EEPROM limits or saved calibration.

## Coordination and workflows

One coordinator owns the cycle; neither backend independently begins a sorting
batch. Each command has a completion acknowledgement, operation identifier,
deadline, and fault result. Serialize actuation and verify arrival/stoppage;
elapsed sleep alone is not a movement-completion signal.

`arm_loads_sorter`: with EV3 stopped, the arm picks one piece, presents it for
color sensing, places it in the commissioned loading location, and retreats to
clearance. Only after confirmed clearance does EV3 position the destination and
eject that piece. Verify completion before loading another piece.

`arm_unloads_sorter`: with the arm at clearance, EV3 senses and presents one
piece at the commissioned pickup location, then stops. The arm picks it,
places it in its configured color destination, and returns to clearance before
EV3 advances another piece. The pickup location is taught for this workflow;
do not assume the stock model automatically provides an accessible gripper
pickup point without checking its geometry.

Handle one piece at a time. Unknown/unstable colors, missing objects, timeout,
disconnect, tracking error, or stop request prevent further feeding and enter
FAULT. Stop both subsystems, attempt each cleanup independently, and record any
cleanup failure. Never blindly replay a feed/eject command after reconnect;
require operator reconciliation of object state before resuming.

## Repository integration and observability

Add a dedicated station module, EV3 backends, the brick-side runtime where
needed, sample configuration, installer/launcher, focused tests, and setup and
commissioning documentation. Reuse existing arm trajectory, home, and atomic
startup helpers after tracing their interfaces; preserve the student simulator.
Save taught/replayable trajectories and timestamped startup/cycle/fault logs.
Keep site-specific serial identities and taught calibration separate from the
portable example. Do not ship invented physical poses as validated defaults.

## Acceptance gates

- Fresh-environment installer and CLI work; packaged runtime/configuration files
  are available outside the source checkout.
- Simulation runs both workflows and asserts the arm/EV3 clearance interlocks.
- Protocol/backend tests cover malformed replies, command completion, timeouts,
  disconnect, sensor failures, cancellation, and independent shutdown attempts.
- Configuration tests reject unsafe or incomplete commissioning data.
- Missing hardware and the current home mismatch fail before unsafe actuation.
- On the real station, verify identity, ports, bounded homing, taught clearance,
  pickup/loading geometry, gripping, color sensing, and stop behavior before
  reporting either workflow physically validated.
- Verify repeat power-on startup and a completed cycle for each supported
  backend/workflow combination; explicitly report combinations not tested.

## Primary references

- LEGO building guide listing:
  https://education.lego.com/en-us/product-resources/mindstorms-ev3/downloads/building-instructions/
- Pybricks model example and port/angle defaults:
  https://pybricks.com/ev3-micropython/examples/color_sorter.html

## Review status

The conversational architecture is approved. This written specification includes
the subsequent automatic-start/installability requirement and awaits review
before the implementation plan is prepared.
