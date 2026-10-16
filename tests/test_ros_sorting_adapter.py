"""ROS sorting adapter contract tests; no ROS graph or devices are started."""
import json
from pathlib import Path
import signal
import sys
import threading
import time
from types import ModuleType, SimpleNamespace

import pytest


from workshop.station import ros_sorting as sorting
from tools import ros_sort_station as adapter


DESTINATIONS = {"red": [0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0]}


class SortingFake:
    def __init__(self, *, pick_result=None):
        self.pick_result = pick_result or {
            "state": "pick_place_complete", "track_id": "red-0009",
            "pickup_verified": True, "lift_verified": True,
            "release_verified": True,
        }
        self.calls = []

    def require_ready(self, *args):
        self.calls.append(("require_ready", args))
        return True

    def wait_clear(self, deadline, cancel):
        self.calls.append(("wait_clear", deadline, cancel))
        return True

    def dispense(self, deadline, cancel):
        self.calls.append(("dispense", deadline, cancel))
        return {"job_id": 17}

    def detect(self, deadline, cancel):
        self.calls.append(("detect", deadline, cancel))
        return {"track_id": "red-0009", "color": "red"}

    def pick_place(self, target, destination, deadline, cancel):
        self.calls.append(("pick_place", target, destination, deadline, cancel))
        return self.pick_result

    def stop(self):
        self.calls.append(("stop",))


class _Trigger:
    class Request:
        pass


class _CancelGoal:
    class Request:
        pass

    class Response:
        ERROR_NONE = 0


class _FakeFuture:
    def __init__(self, result=None):
        self._result = result

    def done(self):
        return True

    def result(self):
        return self._result


class _ServiceClient:
    def __init__(self, name, *, response=None, wait_hook=None):
        self.name = name
        self.response = response
        self.wait_hook = wait_hook
        self.requests = 0

    def wait_for_service(self, timeout_sec=None):
        if self.wait_hook:
            return self.wait_hook(timeout_sec)
        return True

    def call_async(self, _request):
        self.requests += 1
        return _FakeFuture(self.response)


class _FakeNode:
    def __init__(self, *, clock=100.0, on_spin=None, clients=None):
        self.clock = clock
        self.on_spin = on_spin
        self.clients = clients or {}
        self.created_clients = {}

    def create_subscription(self, *_args, **_kwargs):
        return object()

    def create_client(self, _service_type, name):
        response = (SimpleNamespace(return_code=_CancelGoal.Response.ERROR_NONE, goals_canceling=[])
                    if name.endswith("/_action/cancel_goal") else None)
        client = self.clients.get(name, _ServiceClient(name, response=response))
        self.created_clients[name] = client
        return client

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=int(self.clock * 1e9)
        ))

    def spin_once(self):
        if self.on_spin:
            self.on_spin()


class _FakeEV3:
    def __init__(self, responses=None, *, session="ev3-session", fail_stop=False):
        self.session = session
        self.responses = list(responses or [])
        self.fail_stop = fail_stop
        self.calls = []

    def call(self, operation, payload=None):
        self.calls.append((operation, payload))
        if operation == "stop" and self.fail_stop:
            raise RuntimeError("EV3 stop link failed")
        if self.responses:
            response = self.responses.pop(0)
            return response
        if operation == "status":
            return {"session": self.session, "busy": False,
                    "references": {"sorter": True, "feeder": True}}
        return {"accepted": True, "job": {"id": 17}}


def _module(monkeypatch, name, **attrs):
    module = ModuleType(name)
    module.__path__ = []
    for key, value in attrs.items():
        setattr(module, key, value)
    monkeypatch.setitem(sys.modules, name, module)
    return module


def _install_ros_fakes(monkeypatch, *, spin_once=None):
    _module(monkeypatch, "action_msgs")
    _module(monkeypatch, "action_msgs.srv", CancelGoal=_CancelGoal)
    _module(monkeypatch, "action_msgs.msg", GoalStatusArray=type("GoalStatusArray", (), {}))
    _module(monkeypatch, "rclpy")
    _module(monkeypatch, "rclpy.qos", qos_profile_sensor_data=object())
    _module(monkeypatch, "sensor_msgs")
    _module(monkeypatch, "sensor_msgs.msg", JointState=type("JointState", (), {}))
    _module(monkeypatch, "std_msgs")
    _module(monkeypatch, "std_msgs.msg", String=type("String", (), {}))
    _module(monkeypatch, "std_srvs")
    _module(monkeypatch, "std_srvs.srv", Trigger=_Trigger)
    _module(monkeypatch, "tf2_ros", Buffer=lambda: object(),
            TransformListener=lambda *_args: object())
    rclpy = sys.modules["rclpy"]
    rclpy.ok = lambda: True
    rclpy.spin_once = lambda _node, timeout_sec=0.0: spin_once and spin_once()
    return rclpy


def _settings(tmp_path):
    station = tmp_path / "station.json"
    station.write_text(json.dumps({
        "enabled": True, "commissioned": True, "automatic_start": True,
    }))
    execution = tmp_path / "grasp-execution.json"
    execution.write_text(json.dumps({"qualified": True}))
    return {
        "station_config": str(station),
        "grasp_execution_config": str(execution),
        "frame_id": "charuco_board",
        "pickup_bounds_m": [-0.2, 0.2, -0.2, 0.2, 0.0, 0.4],
        "tip_link": "gripper_frame_link",
        "clear_joint_pose_rad": dict.fromkeys(adapter.ARM_JOINTS, 0.),
    }


def _adapter(monkeypatch, tmp_path, *, ev3=None, node=None, spin_once=None):
    rclpy = _install_ros_fakes(monkeypatch, spin_once=spin_once)
    node = node or _FakeNode()
    devices = adapter.ROSSortingDevices(
        _settings(tmp_path), ev3 or _FakeEV3(), tmp_path,
        node=node, rclpy=rclpy,
    )
    return devices, node, rclpy


def test_readiness_is_read_only_and_cancellable_without_home_motion(monkeypatch, tmp_path):
    cancel = threading.Event()
    ready_client = _ServiceClient(
        "/follower/sorting_ready",
        wait_hook=lambda _timeout: (cancel.set() or False),
    )
    ev3 = _FakeEV3()
    node = _FakeNode(clients={"/follower/sorting_ready": ready_client})
    devices, node, _ = _adapter(monkeypatch, tmp_path, ev3=ev3, node=node)

    with pytest.raises(InterruptedError, match="cancel"):
        devices.require_ready(time.monotonic() + 5.0, cancel)

    assert "/follower/home" not in node.created_clients
    assert node.created_clients["/follower/sorting_ready"].requests == 0
    assert [name for name, _ in ev3.calls] == ["status"]


def test_ready_service_success_is_queried_after_configuration_gates(monkeypatch, tmp_path):
    response = SimpleNamespace(success=True, message="measured home is current")
    ready_client = _ServiceClient("/follower/sorting_ready", response=response)
    node = _FakeNode(clients={"/follower/sorting_ready": ready_client})
    devices, node, _ = _adapter(monkeypatch, tmp_path, node=node)

    devices.require_ready(time.monotonic() + 5.0, threading.Event())

    assert node.created_clients["/follower/sorting_ready"].requests == 1
    assert "/follower/home" not in node.created_clients


def test_run_sorting_passes_root_cancel_to_readiness_and_stops_before_dispense():
    cancel = threading.Event()
    events = []

    class CancelDuringReadiness(SortingFake):
        def require_ready(self, deadline, received_cancel):
            assert deadline > time.monotonic()
            assert received_cancel is cancel
            self.calls.append(("require_ready", deadline, received_cancel))
            cancel.set()
            return True

    devices = CancelDuringReadiness()
    with pytest.raises(InterruptedError, match="cancelled"):
        sorting.run_sorting(devices, destinations=DESTINATIONS, cycles=1,
                            log=events.append, cancel=cancel)

    assert not any(call[0] == "dispense" for call in devices.calls)
    assert devices.calls[-1] == ("stop",)
    assert not any(event["event"] == "cycle_start" for event in events)


def test_detect_requires_fresh_in_roi_stable_new_target_and_distinct_stamps(
        monkeypatch, tmp_path):
    now = [100.0]
    stamps = iter([99.4, 100.01, 100.02, 100.02, 100.03, 100.04])
    devices_holder = {}

    def publish_next():
        stamp = next(stamps)
        now[0] = stamp
        if stamp == 99.4:
            position = [0.0, 0.0, 0.1]
        elif stamp == 100.01:
            position = [0.3, 0.0, 0.1]  # outside commissioned ROI
        else:
            position = [0.01, 0.02, 0.1]
        status = {
            "state": "tracking", "frame_id": "charuco_board", "stamp": stamp,
            "grasp_candidates": [{"track_id": 9}],
            "objects": [{"track_id": 9, "color": "red",
                         "position_world": position}],
        }
        devices_holder["devices"]._status(SimpleNamespace(data=json.dumps(status)))

    class ClockNode(_FakeNode):
        def get_clock(self):
            return SimpleNamespace(now=lambda: SimpleNamespace(
                nanoseconds=int(now[0] * 1e9)))

    node = ClockNode(on_spin=publish_next)
    devices, _, _ = _adapter(monkeypatch, tmp_path, node=node, spin_once=publish_next)
    devices_holder["devices"] = devices
    devices.dispensed_after = 100.0
    devices.baseline_ids = {"old-object"}

    target = devices.detect(time.monotonic() + 5.0, threading.Event())

    assert target["track_id"] == "red-0009"
    assert target["position"] == (0.01, 0.02, 0.1)
    assert now[0] == 100.04  # repeated 100.02 did not count as a new image


def test_roi_rejects_outside_and_nonfinite_positions(tmp_path):
    settings = _settings(tmp_path)
    assert adapter.ROSSortingDevices._inside(
        SimpleNamespace(settings=settings), (0.2, -0.2, 0.4))
    assert not adapter.ROSSortingDevices._inside(
        SimpleNamespace(settings=settings), (0.2001, 0.0, 0.1))
    with pytest.raises(ValueError, match="invalid observed object position"):
        adapter.ROSSortingDevices._inside(
            SimpleNamespace(settings=settings), (float("nan"), 0.0, 0.1))


@pytest.mark.parametrize("last_job", [
    {"id": 18, "operation": "eject", "state": "complete"},
    {"id": 17, "operation": "eject", "state": "complete",
     "cleanup_fault": "jam-clear failed"},
])
def test_ev3_job_identity_or_cleanup_fault_aborts_without_second_feed(
        monkeypatch, tmp_path, last_job):
    ev3 = _FakeEV3(responses=[
        {"accepted": True, "job": {"id": 17}},
        {"session": "ev3-session", "busy": False,
         "last_job": last_job, "last_error": None},
    ], fail_stop=True)
    devices, _, _ = _adapter(monkeypatch, tmp_path, ev3=ev3)
    devices.require_ready = lambda *_args: True
    devices.wait_clear = lambda *_args: True
    events = []

    with pytest.raises(RuntimeError) as raised:
        sorting.run_sorting(devices, destinations=DESTINATIONS, cycles=2,
                            log=events.append)

    assert sum(name == "eject" for name, _ in ev3.calls) == 1
    assert sum(name == "stop" for name, _ in ev3.calls) == 1
    assert not any(event["event"] == "cycle_complete" for event in events)
    assert any("sorting stop failed: EV3 stop link failed" in note
               for note in getattr(raised.value, "__notes__", []))


def test_pick_place_cancellation_signals_and_reaps_child_and_stops_ev3(
        monkeypatch, tmp_path):
    cancel = threading.Event()
    signal_calls = []
    wait_calls = []

    class Child:
        returncode = None

        def __init__(self, *_args, **_kwargs):
            pass

        def poll(self):
            return self.returncode

        def send_signal(self, value):
            signal_calls.append(value)
            self.returncode = -int(value)

        def wait(self, timeout=None):
            wait_calls.append(timeout)
            return self.returncode

    node = _FakeNode(on_spin=cancel.set)
    devices, _, _ = _adapter(monkeypatch, tmp_path, node=node,
                              spin_once=cancel.set)
    monkeypatch.setattr(adapter.subprocess, "Popen", Child)
    devices.require_ready = lambda *_args: True
    devices.wait_clear = lambda *_args: True
    devices.dispense = lambda *_args: {"job_id": 17}
    devices.detect = lambda *_args: {"track_id": "red-0009", "color": "red"}

    with pytest.raises(InterruptedError, match="cancel"):
        sorting.run_sorting(devices, destinations=DESTINATIONS, cycles=1,
                            log=lambda _record: None, cancel=cancel)

    assert signal_calls == [signal.SIGINT]
    assert wait_calls == [5.0]
    assert any(name == "stop" for name, _ in devices.ev3.calls)


def test_failed_pick_place_proof_never_admits_a_second_cycle():
    devices = SortingFake(pick_result={
        "state": "pick_place_complete", "track_id": "red-0009",
        "pickup_verified": True, "lift_verified": True,
        "release_verified": False,
    })
    events = []

    with pytest.raises(RuntimeError, match="must all be verified"):
        sorting.run_sorting(devices, destinations=DESTINATIONS, cycles=2,
                            log=events.append)

    assert sum(call[0] == "dispense" for call in devices.calls) == 1
    assert sum(call[0] == "pick_place" for call in devices.calls) == 1
    assert not any(event["event"] == "cycle_complete" for event in events)
    assert devices.calls[-1] == ("stop",)


def test_client_cannot_submit_an_action_after_final_server_cancel(monkeypatch, tmp_path):
    events = []
    devices, _, _ = _adapter(monkeypatch, tmp_path)

    class Child:
        returncode = None

        def poll(self):
            return self.returncode

        def send_signal(self, value):
            events.append("interrupt_client")
            self.returncode = -int(value)

        def wait(self, timeout=None):
            events.append("client_exited")
            return self.returncode

    devices.child = Child()
    for client in devices.cancel_clients:
        original = client.call_async
        def record(request, original=original):
            events.append("cancel_server")
            return original(request)
        client.call_async = record
    devices.stop()
    assert events == ["interrupt_client", "client_exited", "cancel_server", "cancel_server"]


@pytest.mark.parametrize("failure", ["signal_race", "shutdown_timeout"])
def test_child_shutdown_failure_cannot_skip_either_server_cancel(monkeypatch, tmp_path, failure):
    devices, _, _ = _adapter(monkeypatch, tmp_path)

    class Child:
        def poll(self):
            return None

        def send_signal(self, value):
            if failure == "signal_race":
                raise ProcessLookupError("client just exited")

        def wait(self, timeout=None):
            raise adapter.subprocess.TimeoutExpired("grasp", timeout)

        def terminate(self):
            pass

        def kill(self):
            pass

    devices.child = Child()
    with pytest.raises(RuntimeError):
        devices.stop()
    assert all(client.requests == 1 for client in devices.cancel_clients)


def test_stop_waits_for_terminal_action_after_cancel_acceptance(monkeypatch, tmp_path):
    devices, _, rclpy = _adapter(monkeypatch, tmp_path)
    goal_id = [9] * 16
    devices.cancel_clients[0].response = SimpleNamespace(
        return_code=0, goals_canceling=[SimpleNamespace(goal_id=SimpleNamespace(uuid=goal_id))])
    calls = []
    def spin(*args, **kwargs):
        calls.append(True)
        devices._action_status(0, SimpleNamespace(status_list=[SimpleNamespace(
            goal_info=SimpleNamespace(goal_id=SimpleNamespace(uuid=goal_id)), status=5)]))
    rclpy.spin_once = spin
    devices.stop()
    assert len(calls) == 1


def test_cancel_acceptance_alone_cannot_confirm_stop(monkeypatch, tmp_path):
    devices, _, _ = _adapter(monkeypatch, tmp_path)
    devices.cancel_clients[0].response = SimpleNamespace(
        return_code=0, goals_canceling=[SimpleNamespace(goal_id=SimpleNamespace(uuid=[9]*16))])
    clock = iter(range(1000))
    monkeypatch.setattr(adapter.time, "monotonic", lambda: float(next(clock)))
    with pytest.raises(RuntimeError, match="did not confirm terminal"):
        devices.stop()
    assert devices.cancel_clients[1].requests == 1


def test_grasp_client_gets_cycle_deadline_and_commissioned_clear_pose(monkeypatch, tmp_path):
    devices, _, _ = _adapter(monkeypatch, tmp_path)
    commands = []

    class Child:
        returncode = 0

        def __init__(self, command, **kwargs):
            commands.append(command)
            Path(command[command.index("--output")+1]).write_text(json.dumps({
                "state": "pick_place_complete", "track_id": "red-0009",
                "pickup_verified": True, "lift_verified": True, "release_verified": True}))

        def poll(self):
            return 0

    monkeypatch.setattr(adapter.subprocess, "Popen", Child)
    result = devices.pick_place({"track_id": "red-0009", "color": "red"},
                                 DESTINATIONS["red"], time.monotonic()+120., threading.Event())
    command = commands[0]
    assert 119. <= float(command[command.index("--timeout")+1]) <= 120.
    assert json.loads(Path(command[command.index("--return-joint-pose")+1]).read_text()) == dict.fromkeys(adapter.ARM_JOINTS, 0.)
    assert result["release_verified"] is True
