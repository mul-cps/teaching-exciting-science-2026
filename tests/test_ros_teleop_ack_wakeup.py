"""ACK wakeups run through the ROS executor on domain 75; no serial access."""
import ast
import inspect
import os
import textwrap
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace

import pytest

rclpy = pytest.importorskip("rclpy")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros2/src/so101_station_control"))
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from so101_station_control.trajectory_server import TrajectoryBridge, TELEOP_JOINTS


@pytest.fixture
def bridge(monkeypatch, tmp_path):
    assert os.environ.get("ROS_DOMAIN_ID") == "75"
    rclpy.init()
    monkeypatch.setattr(TrajectoryBridge, "_start_owner", lambda node: None)
    node = TrajectoryBridge(parameter_overrides=[
        Parameter("station_root", value=str(Path(__file__).resolve().parents[1])),
        Parameter("state_file", value=str(tmp_path / "state.json"))])
    node._teleop_state = "active"
    node._teleop_session = "current-generation"
    node._active_goal = "teleop"
    node._teleop_origin = dict.fromkeys(TELEOP_JOINTS, 0.)
    node._teleop_follower_start = dict.fromkeys(TELEOP_JOINTS, 0.)
    latest(node, 0)
    node._teleop_last_source_sequence = -1
    commands = []
    node._send = lambda command: commands.append((command, threading.get_ident()))
    yield node, commands
    node.destroy_node()
    rclpy.shutdown()


def latest(node, sequence):
    with node._teleop_latest_lock:
        node._teleop_latest = dict(stamp_s=time.monotonic(), sequence=sequence,
                                   pose=dict.fromkeys(TELEOP_JOINTS, 0.))


def ack(node, owner_sequence, **changes):
    reply = dict(command="teleop_step", session_id="current-generation",
                 sequence=owner_sequence, ok=True)
    reply.update(changes)
    return node._route_teleop_response(reply)


def spin_for(executor, duration):
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=.005)


def test_ack_signals_executor_and_preserves_one_inflight(bridge):
    node, commands = bridge
    node._teleop_tick()
    assert len(commands) == 1
    latest(node, 1)
    node._teleop_timer_tick()
    reader = threading.Thread(target=ack, args=(node, 0))
    reader.start()
    reader.join()
    assert len(commands) == 1  # owner reader must never execute ROS control
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        spin_for(executor, .04)
        assert [command["sequence"] for command, _ in commands] == [0, 1]
        # Timer and guard both run, but may not submit behind the outstanding ACK.
        assert node._teleop_inflight_sequence == 1
    finally:
        executor.shutdown()


@pytest.mark.parametrize("change", [
    {"session_id": "old-generation"}, {"sequence": 42},
    {"ok": False}, {"sequence": None}, {"sequence": True},
    {"ok": "true"}, {"ok": 1},
])
def test_rejected_or_unmatched_ack_never_wakes(bridge, monkeypatch, change):
    node, commands = bridge
    node._teleop_tick()
    latest(node, 1)
    wakeups = []
    monkeypatch.setattr(node, "_wake_teleop_executor", lambda: wakeups.append(True))
    ack(node, 0, **change)
    assert wakeups == []
    assert len(commands) == 1
    node._teleop_tick()
    assert len(commands) == 1
    if change == {"ok": False}:
        assert node._teleop_state == "stopping"
    else:
        assert node._teleop_inflight_sequence == 0


@pytest.mark.parametrize("gate", ["stopping", "disabled", "stop_latched", "closed"])
def test_queued_wakeup_rechecks_stop_cancel_and_destroy_gates(bridge, gate):
    node, commands = bridge
    node._teleop_tick()
    latest(node, 1)
    node._teleop_timer_tick()
    ack(node, 0)
    if gate == "stop_latched":
        node._stop_latched = True
    elif gate == "closed":
        node._closed = True
    else:
        node._teleop_state = gate
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        spin_for(executor, .03)
        assert len(commands) == 1
    finally:
        executor.shutdown()


def test_stopping_ack_does_not_trigger_guard(bridge, monkeypatch):
    node, commands = bridge
    node._teleop_tick()
    node._teleop_state = "stopping"
    wakeups = []
    monkeypatch.setattr(node, "_wake_teleop_executor", lambda: wakeups.append(True))
    ack(node, 0)
    assert not node._teleop_inflight
    assert node._teleop_step_done.is_set()
    assert not wakeups


def test_rapid_duplicate_ack_cannot_release_new_command(bridge):
    node, commands = bridge
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        for sequence in range(30):
            latest(node, sequence)
            spin_for(executor, .009)
            assert len(commands) == sequence + 1
            if sequence:
                ack(node, sequence - 1)
                assert node._teleop_inflight_sequence == sequence
            ack(node, sequence)
            spin_for(executor, .002)
            assert len(commands) == sequence + 1
        assert [command["sequence"] for command, _ in commands] == list(range(30))
    finally:
        executor.shutdown()


def test_8_5ms_ack_avoids_timer_quantization(bridge):
    node, commands = bridge
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    running = threading.Event()
    running.set()
    readers = []
    original_send = node._send

    def send(command):
        assert command["command"] == "teleop_step"
        original_send(command)
        def respond():
            time.sleep(.0085)
            ack(node, command["sequence"])
        reader = threading.Thread(target=respond)
        readers.append(reader)
        reader.start()
    node._send = send

    def source():
        sequence = 0
        deadline = time.monotonic()
        while running.is_set():
            latest(node, sequence)
            sequence += 1
            deadline += .008
            time.sleep(max(0., deadline - time.monotonic()))
    publisher = threading.Thread(target=source)
    publisher.start()
    begin = time.monotonic()
    try:
        spin_for(executor, 1.2)
        duration = time.monotonic() - begin
        assert len(commands) / duration >= 100
        ids = [command["sequence"] for command, _ in commands]
        assert ids == list(range(len(ids)))
    finally:
        running.clear()
        publisher.join()
        node._teleop_state = "stopping"
        executor.shutdown()
        for reader in readers:
            reader.join()


def test_shutdown_invalid_guard_is_fail_closed(bridge, monkeypatch):
    node, commands = bridge
    monkeypatch.setattr(node._teleop_ack_guard, "trigger", lambda: (_ for _ in ()).throw(RuntimeError("shutdown")))
    node._wake_teleop_executor()
    assert commands == []


def test_wakeup_never_executes_or_waits_for_control_on_reader_thread():
    # Pin the hot-path contract: no inline control, polling, or blocking waits
    # may replace the coalescing ROS signal when a reply is received.
    forbidden = {"_teleop_tick", "_send", "sleep", "wait", "join", "spin", "spin_once"}
    for method in (TrajectoryBridge._route_teleop_response,
                   TrajectoryBridge._wake_teleop_executor):
        tree = ast.parse(textwrap.dedent(inspect.getsource(method)))
        calls = {item.func.attr for item in ast.walk(tree)
                 if isinstance(item, ast.Call) and isinstance(item.func, ast.Attribute)}
        assert not calls.intersection(forbidden)


def test_queued_ack_still_stops_for_stale_source(bridge):
    node, commands = bridge
    node._teleop_tick()
    latest(node, 1)
    node._teleop_latest["stamp_s"] -= 1.
    node._teleop_timer_tick()
    ack(node, 0)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        spin_for(executor, .025)
        assert [command["command"] for command, _ in commands] == ["teleop_step", "teleop_stop"]
        assert node._teleop_state == "stopping"
    finally:
        executor.shutdown()


def test_failed_reply_without_sequence_stops_instead_of_releasing_active_stream(bridge):
    node, commands = bridge
    node._teleop_tick()
    latest(node, 1)
    node._route_teleop_response(dict(command="teleop_step", session_id="current-generation",
                                     ok=False, error="preflight rejected"))
    node._teleop_tick()
    assert node._teleop_state == "stopping"
    assert node._teleop_step_done.is_set()
    assert len(commands) == 1


def test_destruction_rejects_step_before_owner_write(bridge, monkeypatch):
    node, commands = bridge
    node._closed = True
    node._process = SimpleNamespace(poll=lambda: None)
    writes = []
    monkeypatch.setattr(node, "_write_owner_command", lambda *args: writes.append(args))
    try:
        with pytest.raises(RuntimeError, match="destruction"):
            TrajectoryBridge._send(node, {"command": "teleop_step"})
        assert writes == []
    finally:
        node._process = None


def test_fast_ack_and_queued_guard_never_request_extra_wakeup(bridge, monkeypatch):
    node, commands = bridge
    wakeups = []
    monkeypatch.setattr(node, "_wake_teleop_executor", lambda: wakeups.append(True))
    node._teleop_tick()
    # A queued guard seeing the next step in flight must not masquerade as
    # a timer blocked at the end of its 8 ms period.
    node._teleop_tick()
    ack(node, 0)
    assert wakeups == []
    latest(node, 1)
    node._teleop_tick()
    ack(node, 1)
    assert wakeups == []


def test_blocked_timer_requests_one_exact_token_wakeup(bridge, monkeypatch):
    node, commands = bridge
    wakeups = []
    monkeypatch.setattr(node, "_wake_teleop_executor", lambda: wakeups.append(True))
    node._teleop_tick()
    node._teleop_timer_tick()
    node._teleop_timer_tick()
    assert node._teleop_waiting_ack == ("current-generation", 0)
    ack(node, 0)
    ack(node, 0)
    assert wakeups == [True]
    assert node._teleop_waiting_ack is None
    latest(node, 1)
    node._teleop_tick()
    ack(node, 1)
    assert wakeups == [True]


def test_waiting_old_generation_never_wakes_new_token(bridge, monkeypatch):
    node, commands = bridge
    wakeups = []
    monkeypatch.setattr(node, "_wake_teleop_executor", lambda: wakeups.append(True))
    node._teleop_tick()
    node._teleop_waiting_ack = ("old-generation", 0)
    ack(node, 0)
    assert wakeups == []
    assert node._teleop_waiting_ack is None


def test_clear_session_discards_pending_timer_wait(bridge):
    node, commands = bridge
    node._teleop_tick()
    node._teleop_timer_tick()
    node._clear_teleop()
    assert node._teleop_waiting_ack is None
