"""Subprocess/native-ROS contract test for the opt-in tracked grasp CLI.

The calibration files and ROS services are deterministic synthetic fixtures.
The resident arm owner and the planning/scene endpoints are fake; no serial
devices or camera SDK are opened.
"""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy", reason="native ROS 2 environment required")
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetMotionPlan
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint

from so101_station_control.trajectory_server import ARM_JOINTS, TELEOP_JOINTS, TrajectoryBridge
from tools.so101_grasp_plan import PoseTarget, staged_targets


ROOT = Path(__file__).resolve().parents[1]
TIP_LINK = "gripper_frame_link"
TOUCH_LINK = "moving_jaw_so101_v1_link"
TRACK_ID = "red-0001"
NUMERIC_TRACK_ID = 1
FRAME_ID = "base_link"
TEST_ARM_ACTION = "/synthetic/follower/follow_joint_trajectory"
BASE_OBJECT = (0.30, 0.0, 0.18)
PLACE_POSE = (0.45, 0.05, 0.20, 0.0, 0.0, 0.0, 1.0)


def _write_calibrations(directory: Path):
    jaw = {
        "schema": "so101-jaw-gap-calibration-v1", "qualified": True,
        "closed_gap_m": 0.004, "open_gap_m": 0.075,
        "closed_percent": 0.0, "open_percent": 100.0,
    }
    # Synthetic regression transform matching the existing grasp-frame test:
    # tip_from_grasp has a +90 degree Y rotation and a 20/0/50 mm offset.
    tip_from_grasp = [
        [0.0, 0.0, 1.0, 0.02],
        [0.0, 1.0, 0.0, 0.0],
        [-1.0, 0.0, 0.0, 0.05],
        [0.0, 0.0, 0.0, 1.0],
    ]
    tcp = {
        "schema": "so101-grasp-tcp-calibration-v1", "qualified": True,
        "tcp_link": TIP_LINK,
        "units": {"translation": "m", "rotation": "quaternion_xyzw"},
        "grasp_frame_convention": "x_closing_z_approach",
        "tip_from_grasp": tip_from_grasp,
        "tcp_from_object": [[1.0, 0.0, 0.0, 0.0],
                             [0.0, 1.0, 0.0, 0.0],
                             [0.0, 0.0, 1.0, 0.0],
                             [0.0, 0.0, 0.0, 1.0]],
    }
    camera = {
        "schema": "camera-arm-calibration-v1", "qualified": True,
        "transform_direction": "base_from_camera",
        "base_from_camera": tcp["tcp_from_object"],
    }
    robot_description = (
        '<robot name="synthetic-sort-test">'
        f'<link name="{TIP_LINK}"/><link name="{TOUCH_LINK}"/>'
        '</robot>')
    files = {}
    for name, value in (("jaw.json", jaw), ("tcp.json", tcp),
                        ("camera-arm.json", camera)):
        path = directory / name
        path.write_text(json.dumps(value), encoding="utf-8")
        files[name] = path
    digest = hashlib.sha256(robot_description.encode("utf-8")).hexdigest()
    execution = {
        "schema": "so101-grasp-execution-config-v1", "qualified": True,
        "jaw_calibration": str(files["jaw.json"]),
        "tcp_calibration": str(files["tcp.json"]),
        "camera_arm_calibration": str(files["camera-arm.json"]),
        "effective_model_sha256": digest,
        "model_parameter_node": "/move_group",
        "touch_links": [TOUCH_LINK],
        "open_gap_m": 0.075, "close_gap_m": 0.004,
    }
    files["execution.json"] = directory / "execution.json"
    files["execution.json"].write_text(json.dumps(execution), encoding="utf-8")
    return files, robot_description


def _registration(stamp, *, tracking=True):
    return {"state": "tracking" if tracking else "lost",
            "observation_valid": tracking, "stamp": stamp,
            "transform_direction": "base_from_camera"}


def _candidate(position=BASE_OBJECT):
    return {
        "candidate_id": f"{TRACK_ID}-g0", "track_id": NUMERIC_TRACK_ID,
        "frame_id": FRAME_ID, "geometric_only": True,
        "ik_validated": False, "collision_checked": False,
        "contact_position_world": list(position),
        "closing_axis_world": [1.0, 0.0, 0.0],
        "approach_direction_world": [0.0, 0.0, -1.0],
    }


def _status(*, phase, stamp, tracking=True):
    object_position = BASE_OBJECT
    if phase in {"lift", "lift_observed", "place", "retreat"}:
        object_position = phase_object_position("lift" if phase == "lift_observed" else phase)
    elif phase in {"released", "released_observed"}:
        object_position = PLACE_POSE[:3]
    object_data = {"track_id": NUMERIC_TRACK_ID, "geometric_only": True,
                   "position_world": list(object_position), "size": [0.04, 0.04, 0.04]}
    status = {
        "state": "tracking" if tracking else "lost", "frame_id": FRAME_ID,
        "stamp": stamp, "objects": [object_data],
        "grasp_candidates": [_candidate()],
    }
    return status


def _tcp_calibration():
    return {
        "tip_from_grasp": [
            [0.0, 0.0, 1.0, 0.02], [0.0, 1.0, 0.0, 0.0],
            [-1.0, 0.0, 0.0, 0.05], [0.0, 0.0, 0.0, 1.0]],
        "tcp_from_object": [[1.0, 0.0, 0.0, 0.0],
                             [0.0, 1.0, 0.0, 0.0],
                             [0.0, 0.0, 1.0, 0.0],
                             [0.0, 0.0, 0.0, 1.0]],
    }


def phase_object_position(phase):
    if phase == "lift":
        # Top-down approach means the retreat/lift target moves upward.
        contact = PoseTarget(FRAME_ID, BASE_OBJECT, (1.0, 0.0, 0.0, 0.0))
        targets = dict(staged_targets(contact, place=PoseTarget(
            FRAME_ID, PLACE_POSE[:3], PLACE_POSE[3:]),
            tip_from_grasp=_tcp_calibration()["tip_from_grasp"]))
        return targets["lift"].position
    return PLACE_POSE[:3]


class _Fixture(Node):
    def __init__(self, *, lose_on_first_plan=False, cancel_on_lift=False):
        super().__init__("ros_grasp_cli_native_fixture")
        self.lose_on_first_plan = lose_on_first_plan
        self.cancel_on_lift = cancel_on_lift
        self.phase = "initial"
        self.phase_changed_at = time.monotonic()
        self.lost = False
        self.plans = []
        self.scene_requests = []
        self.events = []
        self.gripper_commands = []
        self.arm_goal_count = 0
        self.arm_terminal_statuses = []
        self.publish_count = 0
        self.lift_goal_started = threading.Event()
        self.lift_cancelled = threading.Event()
        self.robot_description = ""
        self.model_node = Node("move_group")
        self.model_node.declare_parameter("robot_description", "")
        self.registration_pub = self.create_publisher(
            String, "/so101_perception/charuco_world/status", 10)
        self.status_pub = self.create_publisher(String, "/so101/grasp_status", 10)
        self.joints_pub = self.create_publisher(JointState, "/follower/joint_states", 10)
        self.scene_ack_pub = self.create_publisher(
            String, "/so101_scene/attachment_status", 10)
        self.create_subscription(String, "/so101_scene/attachment_request",
                                 self._scene_request, 10)
        self.create_service(GetMotionPlan, "/follower/plan_kinematic_path",
                            self._plan)
        self.create_timer(0.04, self._publish_live_observations)
        self.arm_server = ActionServer(
            self, FollowJointTrajectory,
            TEST_ARM_ACTION,
            execute_callback=self._execute_arm,
            goal_callback=lambda _goal: GoalResponse.ACCEPT,
            cancel_callback=self._cancel_arm)

    def _now(self):
        return self.get_clock().now()

    def _now_seconds(self):
        return self._now().nanoseconds * 1e-9

    def _publish_live_observations(self):
        self.publish_count += 1
        stamp = self._now_seconds()
        registration = String()
        registration.data = json.dumps(_registration(stamp, tracking=not self.lost))
        self.registration_pub.publish(registration)
        status = String()
        status.data = json.dumps(_status(phase=self.phase, stamp=stamp,
                                         tracking=not self.lost))
        self.status_pub.publish(status)
        joints = JointState()
        joints.header.stamp = self._now().to_msg()
        joints.name = list((*ARM_JOINTS, "gripper"))
        joints.position = [0.0] * len(joints.name)
        self.joints_pub.publish(joints)
        if self.phase in {"lift", "released"} and time.monotonic() - self.phase_changed_at >= 0.30:
            if self.phase == "lift":
                self.events.append("evidence:lift_observation")
                self.phase = "lift_observed"
            elif self.phase == "released":
                self.events.append("evidence:release_observation")
                self.phase = "released_observed"

    def set_robot_description(self, robot_description):
        self.robot_description = robot_description
        self.model_node.set_parameters([Parameter("robot_description", value=robot_description)])
    def _plan(self, request, response):
        index = len(self.plans)
        req = request.motion_plan_request
        constraints = req.goal_constraints[0]
        requested_position = tuple(
            constraints.position_constraints[0].constraint_region.primitive_poses[0].position.__getattribute__(axis)
            for axis in ("x", "y", "z"))
        self.plans.append({"index": index, "position": requested_position,
                           "group": req.group_name, "planner": req.planner_id})
        if self.lose_on_first_plan and index == 0:
            self.lost = True
            self.phase = "lost"
            self.phase_changed_at = time.monotonic()
            # The planner service and timer share a mutually exclusive callback
            # group. Publish the loss here so the client actually receives it
            # while its planning request is still pending.
            stamp = self._now_seconds()
            registration = String()
            registration.data = json.dumps(_registration(stamp, tracking=False))
            self.registration_pub.publish(registration)
            status = String()
            status.data = json.dumps(_status(phase="lost", stamp=stamp, tracking=False))
            self.status_pub.publish(status)
            # Let the client receive lost localization before this plan returns.
            time.sleep(0.16)
        response.motion_plan_response.error_code.val = MoveItErrorCodes.SUCCESS
        response.motion_plan_response.planning_time = 0.01
        trajectory = response.motion_plan_response.trajectory.joint_trajectory
        trajectory.joint_names = list(ARM_JOINTS)
        point = JointTrajectoryPoint()
        point.positions = [0.01 * (index + 1)] * len(ARM_JOINTS)
        point.time_from_start.sec = 1
        trajectory.points = [point]
        return response

    def _scene_request(self, message):
        request = json.loads(message.data)
        self.scene_requests.append(request)
        action = request["action"]
        self.events.append(f"scene:{action}")
        states = {"lock": "locked", "unlock": "unlocked",
                  "allow_contact": "contact_allowed", "disallow_contact": "contact_disallowed",
                  "attach": "attached", "detach": "detached"}
        if action == "detach":
            self.phase = "released"
            self.phase_changed_at = time.monotonic()
        ack = String()
        ack.data = json.dumps({"request_id": request["request_id"],
                               "track_id": request["track_id"], "state": states[action]})
        self.scene_ack_pub.publish(ack)

    def _cancel_arm(self, _goal_handle):
        self.lift_cancelled.set()
        return CancelResponse.ACCEPT

    def _execute_arm(self, goal_handle):
        index = self.arm_goal_count
        self.arm_goal_count += 1
        stage = ("pregrasp", "contact", "lift", "place", "retreat")[index]
        self.events.append(f"arm:{stage}")
        if stage == "lift":
            self.phase = "lift"
            self.phase_changed_at = time.monotonic()
            self.lift_goal_started.set()
            if self.cancel_on_lift:
                deadline = time.monotonic() + 8.0
                while not self.lift_cancelled.is_set() and time.monotonic() < deadline:
                    time.sleep(0.01)
                goal_handle.canceled()
                self.arm_terminal_statuses.append(GoalStatus.STATUS_CANCELED)
                return FollowJointTrajectory.Result()
        goal_handle.succeed()
        self.arm_terminal_statuses.append(GoalStatus.STATUS_SUCCEEDED)
        return FollowJointTrajectory.Result()


class _FakeOwnerProcess:
    def poll(self):
        return None

    def wait(self, timeout=None):
        return 0

    def terminate(self):
        return None


def _create_bridge(monkeypatch, tmp_path):
    state_file = tmp_path / "owner-state.json"
    state_file.write_text(json.dumps({
        "schema": 1, "event": "sample", "arm_id": "synthetic-arm",
        "calibration_sha256": "b" * 64, "monotonic": time.monotonic(),
        "raw": {joint: 1000 for joint in TELEOP_JOINTS},
        "pose": {**dict.fromkeys(ARM_JOINTS, 0.0), "gripper": 50.0},
    }))

    def start_fake_owner(bridge):
        bridge._state_file = state_file
        bridge._process = _FakeOwnerProcess()
        bridge._owner_started = time.monotonic()
        bridge._expected_arm_id = "synthetic-arm"
        bridge._expected_calibration_sha256 = "b" * 64
        bridge._gripper_raw_limits = (500, 1500)
        bridge._gripper_jaw = {"closed_gap_m": 0.004, "open_gap_m": 0.075,
                               "closed_percent": 0.0, "open_percent": 100.0}

    def send_to_owner(bridge, command):
        if command["command"] == "move_gripper":
            bridge_fixture = bridge._test_fixture
            bridge_fixture.gripper_commands.append(command)
            target_raw = command["target_raw"]
            is_close = target_raw == 500
            bridge_fixture.events.append("gripper:close" if is_close else "gripper:open")
            target_percent = (target_raw - 500) * 100.0 / 1000.0
            bridge._response_queue.put({"ok": True, "state": {
                "event": "plan_complete",
                "pose": {"gripper": 50.0 if is_close else target_percent},
                "legs": [{"label": "gripper_command", "contact_inferred": is_close}],
            }})
        elif command["command"] == "cancel_hold":
            bridge._response_queue.put({"ok": True,
                                        "state": {"event": "trajectory_cancel_hold"}})

    monkeypatch.setattr(TrajectoryBridge, "_start_owner", start_fake_owner)
    monkeypatch.setattr(TrajectoryBridge, "_send", send_to_owner)
    return state_file


def _spin_executor(executor, stop):
    while not stop.is_set() and rclpy.ok():
        executor.spin_once(timeout_sec=0.02)


def _start_fixture(monkeypatch, tmp_path, *, lose_on_first_plan=False, cancel_on_lift=False):
    if not rclpy.ok():
        rclpy.init(args=["ros-grasp-cli-native-test"])
    fixture = _Fixture(lose_on_first_plan=lose_on_first_plan,
                       cancel_on_lift=cancel_on_lift)
    state_file = _create_bridge(monkeypatch, tmp_path)
    bridge = TrajectoryBridge(parameter_overrides=[
        Parameter("enable_hardware", Parameter.Type.BOOL, False),
        Parameter("station_root", Parameter.Type.STRING, str(ROOT)),
        Parameter("state_file", Parameter.Type.STRING, str(state_file)),
        Parameter("gripper_jaw_calibration", Parameter.Type.STRING, "synthetic-test-only"),
        Parameter("action_name", Parameter.Type.STRING, "/unused/follower/arm_trajectory"),
    ])
    bridge._test_fixture = fixture
    def refresh_owner_state():
        state = json.loads(state_file.read_text(encoding="utf-8"))
        state["monotonic"] = time.monotonic()
        state_file.write_text(json.dumps(state), encoding="utf-8")
    fixture.create_timer(0.05, refresh_owner_state)
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(fixture)
    executor.add_node(fixture.model_node)
    executor.add_node(bridge)
    stop = threading.Event()
    spin_thread = threading.Thread(target=_spin_executor,
                                   args=(executor, stop), daemon=True)
    spin_thread.start()
    return fixture, bridge, executor, stop, spin_thread


def _launch_cli(tmp_path, files):
    output = tmp_path / "grasp-result.json"
    command = [
        sys.executable, str(ROOT / "tools/ros_plan_grasp.py"),
        "--execute", "--track-id", TRACK_ID,
        "--frame", FRAME_ID, "--tip-link", TIP_LINK,
        "--place-pose", *(str(value) for value in PLACE_POSE),
        "--execution-config", str(files["execution.json"]),
        "--trajectory-action", TEST_ARM_ACTION,
        "--timeout", "45", "--planning-time", "0.5",
        "--output", str(output),
    ]
    process = subprocess.Popen(command, cwd=ROOT, stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT, text=True,
                               env=os.environ.copy())
    return process, output, command


def _communicate_cli(process, fixture, timeout):
    try:
        return process.communicate(timeout=timeout)
    except subprocess.TimeoutExpired:
        process.kill()
        stdout, stderr = process.communicate(timeout=3)
        try:
            graph = {
                "nodes": fixture.get_node_names(),
                "joint_subscribers": [item.node_name for item in
                    fixture.get_subscriptions_info_by_topic("/follower/joint_states")],
                "status_subscribers": [item.node_name for item in
                    fixture.get_subscriptions_info_by_topic("/so101/grasp_status")],
                "registration_subscribers": [item.node_name for item in
                    fixture.get_subscriptions_info_by_topic("/so101_perception/charuco_world/status")],
            }
        except Exception as error:
            graph = {"error": repr(error)}
        raise AssertionError(
            "grasp CLI timed out; "
            f"plans={len(fixture.plans)} scene={ [r['action'] for r in fixture.scene_requests] } "
            f"gripper={len(fixture.gripper_commands)} arm={fixture.arm_goal_count} "
            f"pubs={fixture.publish_count} domain={os.environ.get('ROS_DOMAIN_ID')}; "
            f"graph={graph!r}; output={stdout or stderr!r}")


def _cleanup(fixture, bridge, executor, stop, spin_thread):
    stop.set()
    executor.shutdown(timeout_sec=2.0)
    spin_thread.join(timeout=2.0)
    executor.remove_node(bridge)
    executor.remove_node(fixture.model_node)
    executor.remove_node(fixture)
    bridge.destroy_node()
    fixture.model_node.destroy_node()
    fixture.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def test_execute_cli_runs_complete_synthetic_native_ros_pick_place(monkeypatch, tmp_path):
    files, robot_description = _write_calibrations(tmp_path)
    fixture, bridge, executor, stop, spin_thread = _start_fixture(monkeypatch, tmp_path)
    fixture.set_robot_description(robot_description)
    process, output, command = _launch_cli(tmp_path, files)
    try:
        stdout, _ = _communicate_cli(process, fixture, timeout=18)
        assert process.returncode == 0, f"CLI exited {process.returncode}:\n{stdout}\n{command}"
        artifact = json.loads(output.read_text(encoding="utf-8"))
        assert artifact["state"] == "pick_place_complete"
        assert artifact["execution_enabled"] is True
        assert artifact["pickup_verified"] is True
        assert artifact["lift_verified"] is True
        assert artifact["release_verified"] is True
        assert artifact["object_attachment"]["state"] == "detached"
        assert artifact["gripper"]["state"] == "released"
        assert [request["action"] for request in fixture.scene_requests] == [
            "lock", "allow_contact", "attach", "disallow_contact", "detach"]
        assert [item["command"] for item in fixture.gripper_commands] == [
            "move_gripper", "move_gripper", "move_gripper"]
        assert [event for event in fixture.events if event.startswith(("scene:", "gripper:", "arm:"))] == [
            "scene:lock", "gripper:open", "arm:pregrasp", "scene:allow_contact",
            "arm:contact", "gripper:close", "scene:attach", "scene:disallow_contact",
            "arm:lift", "arm:place", "gripper:open", "scene:detach", "arm:retreat"]
        assert [segment["name"] for segment in artifact["segments"]] == [
            "pregrasp", "contact", "lift", "place", "post_release_tool_retreat"]
        assert fixture.arm_terminal_statuses == [GoalStatus.STATUS_SUCCEEDED] * 5
        assert len(fixture.plans) == 5
        assert fixture.plans[0]["position"][2] > fixture.plans[1]["position"][2]
        assert fixture.plans[2]["position"][2] > fixture.plans[1]["position"][2]
        assert fixture.plans[3]["position"] == pytest.approx(PLACE_POSE[:3])
        assert fixture.plans[4]["position"] != pytest.approx(PLACE_POSE[:3])
    finally:
        if process.poll() is None:
            process.kill()
            process.wait(timeout=3)
        _cleanup(fixture, bridge, executor, stop, spin_thread)


def test_execute_cli_lost_localization_during_first_plan_never_sends_arm_action(
        monkeypatch, tmp_path):
    files, robot_description = _write_calibrations(tmp_path)
    fixture, bridge, executor, stop, spin_thread = _start_fixture(
        monkeypatch, tmp_path, lose_on_first_plan=True)
    fixture.set_robot_description(robot_description)
    process, output, command = _launch_cli(tmp_path, files)
    try:
        stdout, _ = _communicate_cli(process, fixture, timeout=18)
        assert process.returncode == 2, f"expected localized fail-closed CLI result:\n{stdout}\n{command}"
        artifact = json.loads(output.read_text(encoding="utf-8"))
        assert artifact["state"] == "execution_failed"
        assert "target tracking became stale while planning" in artifact["error"]
        assert fixture.arm_goal_count == 0
        assert [request["action"] for request in fixture.scene_requests] == ["lock", "unlock"]
        assert [command["target_raw"] for command in fixture.gripper_commands] == [1500]
    finally:
        if process.poll() is None:
            process.kill()
            process.wait(timeout=3)
        _cleanup(fixture, bridge, executor, stop, spin_thread)


def test_execute_cli_cancellation_during_lift_keeps_attached_scene_and_terminal_goal(
        monkeypatch, tmp_path):
    files, robot_description = _write_calibrations(tmp_path)
    fixture, bridge, executor, stop, spin_thread = _start_fixture(
        monkeypatch, tmp_path, cancel_on_lift=True)
    fixture.set_robot_description(robot_description)
    process, output, command = _launch_cli(tmp_path, files)
    try:
        assert fixture.lift_goal_started.wait(timeout=25), "CLI did not reach attached lift action"
        process.send_signal(signal.SIGINT)
        stdout, _ = _communicate_cli(process, fixture, timeout=18)
        assert process.returncode == 2, f"expected cancellation artifact:\n{stdout}\n{command}"
        artifact = json.loads(output.read_text(encoding="utf-8"))
        assert artifact["state"] == "execution_failed_attached_stop"
        assert artifact["object_attachment"]["state"] == "attached_stop_retain_ownership"
        assert artifact["pickup_verified"] is False
        assert [request["action"] for request in fixture.scene_requests] == [
            "lock", "allow_contact", "attach", "disallow_contact"]
        assert "detach" not in [request["action"] for request in fixture.scene_requests]
        assert fixture.arm_terminal_statuses[-1] == GoalStatus.STATUS_CANCELED
        assert fixture.lift_cancelled.is_set()
    finally:
        if process.poll() is None:
            process.kill()
            process.wait(timeout=3)
        _cleanup(fixture, bridge, executor, stop, spin_thread)
