# EV3 and arm station Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox syntax for tracking.

**Goal:** Install and activate a PC-controlled Color Sorter/SO-ARM101 station supporting both user-selected workflows and two firmware backends.

**Architecture:** A single coordinator serializes acknowledged device operations. Backends expose the same homing, sensing, positioning, feeding, stopping, and status interface. Simulation uses the same coordinator and configuration validation as hardware.

**Tech Stack:** Python 3.11+, JSON configuration, pytest, LeRobot's existing calibrated follower API, EV3 USB direct commands for original firmware, and a bundled Pybricks EV3 MicroPython 2 agent launched through SSH/brickrun.

**Spec:** `stale/docs/superpowers/specs/2026-10-13-ev3-arm-station-design.md`

## Global Constraints

- Support `arm_loads_sorter` and `arm_unloads_sorter`.
- Do not change host Ethernet, DNS, routing, or unrelated network configuration.
- Prefer persistent USB identities over changing device-number paths.
- Reject missing or inconsistent calibration before actuation; never infer arm limits by driving joints to stops.
- Current shoulder home raw 949 is below minimum 1063; reject it without altering EEPROM or calibration.
- Handle one piece at a time. Never blindly replay feed/eject after reconnect.
- Original firmware requires no brick-side Python. Pybricks requires initial SD-card provisioning.
- Preserve the existing student simulator and unrelated dirty ignore files.

## Review Focus

- Multiple attached bricks or serial adapters: reject ambiguity rather than choose the first.
- Half-completed eject followed by disconnect: fault without retrying an irreversible operation.
- Installed wheel outside the checkout: bundled configuration and brick agent remain accessible.
- Touch sensor stuck pressed/unpressed: homing detects failure with bounded travel/time.
- One device stop raises: still attempt stopping the other and preserve both errors.

## Task 1: Configuration, coordinator, and simulation

**Files:** create `workshop/station/config.py`, `coordinator.py`, `simulation.py`, `__init__.py`, `resources/station.example.json`; create `tests/test_station.py`.

**Interfaces:** `load_config(path, physical=False) -> dict`; `run_cycle(config, ev3, arm, log)`; EV3 `home()`, `read_color() -> str`, `position(color)`, `feed()`, `stop()`, `wait_stopped()`; arm `home()`, `load_piece()`, `pick_and_place(color)`, `clear()`, `wait_clear()`, `stop()`.

- [ ] Write failing sequencing tests using recording fake backends:

```python
def test_feed_requires_confirmed_clearance(station):
    station.arm.wait_clear_error = TimeoutError("clearance")
    with pytest.raises(TimeoutError):
        station.cycle("arm_loads_sorter")
    assert "feed" not in station.events

def test_unknown_color_stops_both(station):
    station.ev3.color = "unknown"
    with pytest.raises(ValueError):
        station.cycle("arm_unloads_sorter")
    assert station.ev3.stopped and station.arm.stopped
```

- [ ] Run `pytest tests/test_station.py -q`; confirm missing implementation failure.
- [ ] Implement validated schema, simulation backends, and serialized coordinator. Loading sequence: arm load/present, EV3 read color, arm clear/confirm, EV3 position/feed/confirm. Unloading sequence: arm clear/confirm, EV3 read/position/feed/stop/confirm, arm pick/place/clear/confirm. Validate complete color/pose coverage before physical startup.
- [ ] Add tests for both event sequences, invalid workflow, duplicate ports, missing poses, nonfinite limits, unknown color, failed acknowledgement, and independent cleanup when either stop raises. Never retry feed automatically.
- [ ] Run focused tests and commit this independently runnable simulator.

## Task 2: Original-firmware USB backend

**Files:** create `workshop/station/ev3_usb.py`, `discovery.py`; create `tests/test_ev3_usb.py`.

**Interfaces:** `discover_ev3() -> list[identity]`; `OriginalEV3(config, transport=None)` implements Task 1 EV3 methods. Inject transport for deterministic tests.

- [ ] Verify LEGO communication/firmware developer kits and the maintained direct-command library API; record exact selected dependency and supported USB identity in documentation.
- [ ] Write failing tests around transport packets/replies, counter matching, firmware errors, incomplete replies, timeout, device ambiguity, port-type mismatch, and post-disconnect non-retry behavior.

```python
def test_bad_reply_faults_without_feed_retry(backend, transport):
    transport.reply = b"\x00"
    with pytest.raises(Exception):
        backend.feed()
    assert transport.feed_attempts == 1
```

- [ ] Implement device identification, finite USB timeouts, reply validation, sensor reads, encoder reads, and bounded movement. Completion polls encoder/stopped status with a deadline instead of assuming a sleep equals arrival.
- [ ] Implement touch homing: verify switch transitions, bound time and encoder travel, stop in `finally`, then zero sorter reference only after successful arrival. Feeder initialization likewise bounds time/travel; use commissioned mechanical direction and limits.
- [ ] Test stuck switches, excessive travel, tracking stall, cancellation, and stop on every fault. Run focused tests and commit.

## Task 3: Pybricks backend and automatic deployment

**Files:** create `workshop/station/ev3_pybricks.py`, `resources/ev3_agent.py`; create `tests/test_ev3_agent.py`.

**Interfaces:** `PybricksEV3(config, transport=None)` implements Task 1 EV3 methods. Agent receives newline-delimited JSON requests with `id`, `operation`, and `args`; replies carry matching `id`, success/result or fault.

- [ ] Verify EV3 MicroPython 2 and `brickrun` launch support; document initial SD provisioning and existing SSH connection requirements. Do not modify host networking or embed passwords.
- [ ] Write failing tests for id mismatch, malformed response, missing acknowledgement, duplicate feed id, connection loss, and deployment failure.

```python
def test_duplicate_feed_is_not_executed_twice(agent):
    request = {"id": "cycle-1-feed", "operation": "feed", "args": {}}
    agent.dispatch(request)
    agent.dispatch(request)
    assert agent.feeder.calls == 1
```

- [ ] Implement packaged-agent upload and PC launch; use a supported dedicated command channel, isolate console diagnostics from protocol output, and verify a ready/version/identity handshake before motion.
- [ ] Agent implements bounded nonblocking motor operations, switch/color/encoder reads, idempotent request handling within a connection, local cancellation and command deadlines. Communication loss must not leave indefinite motor motion. Never resume a previous feed after agent restart.
- [ ] Test bounded homing and EOF cleanup with fake Pybricks modules. Run focused tests and commit.

## Task 4: Calibrated arm adapter and commissioning

**Files:** create `workshop/station/arm.py`, `commission.py`; create `tests/test_station_arm.py`. Existing `safe_home.py`, `servo_start.py`, and trajectory helpers are reused, not rewritten.

**Interfaces:** `LeRobotArm(config)` implements Task 1 arm methods; commissioned paths identify calibration hash and named pickup/load/drop/clearance poses.

- [ ] Trace `graft callers` for each reused helper before any existing-symbol edits. Verify current LeRobot API in the installed runtime.
- [ ] Write failing tests for missing calibration, changed hash, current invalid home, outside-limit pose, missing color drop, excessive displacement, startup failure, and tracking error.

```python
def test_invalid_home_never_arms(adapter, bus):
    adapter.home_raw["shoulder_lift"] = 949
    adapter.minimum_raw["shoulder_lift"] = 1063
    with pytest.raises(ValueError):
        adapter.home()
    assert not bus.armed
```

- [ ] Implement persistent-identity connection and complete preflight before atomic arming. Reuse bounded home planner and current-position atomic startup, track actual encoder arrival, and stop on deadline/tracking violations. Never restore old goal-position registers.
- [ ] Add a commissioning command that records torque-disabled measured poses and saves calibrated replayable paths/configuration; do not mark geometry/grasp success commissioned solely from encoder capture.
- [ ] Test all failure paths and final torque-off verification; run existing home/startup/trajectory tests alongside focused adapter tests and commit.

## Task 5: Installer, activation CLI, packaging, and docs

**Files:** create `workshop/station/__main__.py`, `tools/install_station.sh`, `docs/ev3-arm-station.md`; modify `pyproject.toml`, `README.md`; create `tests/test_station_cli.py`.

**Interfaces:** `python -m workshop.station --config PATH --simulate`; `--detect` is read-only; `--activate` discovers, validates, homes, and runs configured finite cycles on commissioned hardware. Installer creates a dedicated venv and a launcher without changing system Python.

- [ ] Write failing tests for simulation of both workflows, detect without motion, incomplete commissioning rejection, package resource availability, invalid configuration diagnostics, and cancellation logs.

```python
def test_resources_in_installed_package():
    from importlib.resources import files
    root = files("workshop.station").joinpath("resources")
    assert root.joinpath("station.example.json").is_file()
    assert root.joinpath("ev3_agent.py").is_file()
```

- [ ] Pin verified hardware extras; package JSON and agent resources explicitly. Installer checks Python/version, installs extras into its own venv, copies a user-editable example, verifies CLI/simulation, and prints exact commissioning/activation commands. Preserve existing user configuration on rerun.
- [ ] Document supported Linux setup, USB access configuration without sudo assumptions, original versus Pybricks setup, device identities, initial pose teaching, one-command repeat startup, logs, fault recovery, and current physical blockers. Update status table without claiming physical acceptance.
- [ ] Build/install wheel in a fresh venv, run both simulations outside checkout, run full pytest, compileall, shell syntax check, and diff check. Review complete diff against spec; commit/push verified work.
- [ ] If devices are available, run read-only identification first; execute only commissioned bounded motion. Record the backend/workflow combinations actually tested and keep untested combinations explicitly pending.

## Execution and completion

Use isolated worktree ownership for implementation workers in accordance with
`/home/bjoern/AGENTS.md`. The supervisor reviews integration and evidence. Task 1
defines shared interfaces; backend tasks may proceed independently afterward;
CLI integration follows those deliveries. Never equate fake-backend tests with
physical validation. The user has approved the design and requested execution;
this written plan is the remaining review artifact before implementation.
