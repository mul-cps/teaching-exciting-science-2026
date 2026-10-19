"""Run the actual CLI against read-only synthetic ROS endpoints, never hardware."""
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
import uuid

import pytest

pytest.importorskip("rclpy", reason="native ROS 2 environment required")
import rclpy
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetMotionPlan
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from tools.ros_plan_current import ALL_JOINTS

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def synthetic_station():
    if os.environ.get("ROS_DOMAIN_ID") != "89":
        pytest.skip("isolated native ROS domain 89 required")
    rclpy.init()
    namespace = "/calibration_preflight_" + uuid.uuid4().hex
    node = Node("calibration_preflight_fixture")
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    servers = []
    state = {"namespace": namespace, "parameter_requests": [], "plans": 0, "actions": 0}
    publisher = node.create_publisher(JointState, namespace + "/joint_states", 10)

    def publish():
        message = JointState()
        message.header.stamp = node.get_clock().now().to_msg()
        message.name = list(ALL_JOINTS)
        message.position = [0.] * len(ALL_JOINTS)
        publisher.publish(message)

    node.create_timer(.04, publish)

    def plan(_request, response):
        state["plans"] += 1
        # Deliberate stop at planning: an admitted preflight must not move anything.
        response.motion_plan_response.error_code.val = -1
        return response

    node.create_service(GetMotionPlan, namespace + "/plan_kinematic_path", plan)

    def execute(goal):
        state["actions"] += 1
        goal.abort()
        return FollowJointTrajectory.Result()

    servers.append(ActionServer(node, FollowJointTrajectory,
        namespace + "/arm_trajectory_controller/follow_joint_trajectory", execute))

    def expose_mode(mode):
        if mode == "unavailable":
            return

        def get_mode(request, response):
            state["parameter_requests"].append(list(request.names))
            if mode == "timeout":
                time.sleep(4.)
            if mode == "missing":
                response.values = [ParameterValue(type=ParameterType.PARAMETER_NOT_SET)]
            elif mode == "wrong_type":
                response.values = [ParameterValue(type=ParameterType.PARAMETER_STRING,
                                                   string_value="true")]
            else:
                response.values = [ParameterValue(type=ParameterType.PARAMETER_BOOL,
                                                   bool_value=mode == "true")]
            return response

        node.create_service(GetParameters, namespace + "/trajectory_server/get_parameters", get_mode)

    state["expose_mode"] = expose_mode
    try:
        yield state
    finally:
        executor.shutdown(timeout_sec=5.)
        thread.join(timeout=5.)
        for server in servers:
            server.destroy()
        node.destroy_node()
        rclpy.shutdown()


def run_cli(station, tmp_path, *flags):
    command = [sys.executable, str(ROOT / "tools/ros_plan_current.py"),
               "--output-directory", str(tmp_path / "result"),
               "--namespace", station["namespace"], "--timeout", "2.5", *flags]
    return subprocess.run(command, cwd=ROOT, stdout=subprocess.PIPE,
                          stderr=subprocess.STDOUT, text=True, timeout=8.)


@pytest.mark.parametrize("mode", ["false", "missing", "wrong_type", "unavailable", "timeout"])
def test_calibration_execution_rejects_before_planning_or_action(synthetic_station, tmp_path, mode):
    station = synthetic_station
    station["expose_mode"](mode)
    result = run_cli(station, tmp_path, "--execute", "--calibration")
    assert result.returncode == 1, result.stdout
    assert "calibration_endpoint_mode" in result.stdout, result.stdout
    if mode == "timeout":
        assert "timed out waiting for calibration_endpoint_mode" in result.stdout
    elif mode == "unavailable":
        assert "parameter service unavailable within timeout" in result.stdout
    assert station["plans"] == 0
    assert station["actions"] == 0
    assert not (tmp_path / "result").exists()
    expected = [] if mode == "unavailable" else [["calibration_endpoint_mode"]]
    assert station["parameter_requests"] == expected


def test_typed_true_preflight_admits_planning(synthetic_station, tmp_path):
    station = synthetic_station
    station["expose_mode"]("true")
    result = run_cli(station, tmp_path, "--execute", "--calibration")
    assert result.returncode == 1  # Synthetic planner intentionally rejects.
    assert "MoveIt planning failed" in result.stdout, result.stdout
    assert station["parameter_requests"] == [["calibration_endpoint_mode"]]
    assert station["plans"] == 1
    assert station["actions"] == 0


@pytest.mark.parametrize("flags", [[], ["--calibration"], ["--execute"]])
def test_other_modes_do_not_query_calibration_mode(synthetic_station, tmp_path, flags):
    station = synthetic_station
    station["expose_mode"]("false")
    result = run_cli(station, tmp_path, *flags)
    assert result.returncode == 1
    assert "MoveIt planning failed" in result.stdout, result.stdout
    assert station["parameter_requests"] == []
    assert station["plans"] == 1
    assert station["actions"] == 0
