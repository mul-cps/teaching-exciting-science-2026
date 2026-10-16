#!/usr/bin/env python3
"""Run a near-zero 5-DOF FK/IK/planning smoke; never execute the trajectory."""

import os
from pathlib import Path

import rclpy
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy
from rosidl_runtime_py.convert import message_to_yaml


def main() -> int:
    rclpy.init()
    robot = MoveItPy(
        node_name="so101_plan_only_smoke",
        remappings={"joint_states": "/follower/joint_states"},
    )
    group = "manipulator"
    tip = "gripper_frame_link"
    robot_model = robot.get_robot_model()

    # The pinned SRDF's `zero` state is the mock's initial state. Move a small
    # amount in two joints, compute FK, then solve that pose from the start
    # state. This tests real 5-DOF IK while avoiding an arbitrary 6D target.
    start_state = RobotState(robot_model)
    start_state.set_to_default_values(group, "zero")
    start_state.update()
    with robot.get_planning_scene_monitor().read_only() as planning_scene:
        start_valid = bool(planning_scene.is_state_valid(start_state, group))
        start_colliding = bool(planning_scene.is_state_colliding(start_state, group, True))
    print(
        f"START STATE VALID: valid={start_valid} collision={start_colliding} source=SRDF:zero",
        flush=True,
    )
    if not start_valid:
        print("INVALID START STATE: planning aborted", flush=True)
        os._exit(1)

    target_reference = RobotState(robot_model)
    target_reference.set_to_default_values(group, "zero")
    target_joint_positions = list(target_reference.get_joint_group_positions(group))
    target_joint_positions[0] += 0.05
    target_joint_positions[1] += 0.03
    target_reference.set_joint_group_positions(group, target_joint_positions)
    target_reference.update()
    target_pose = target_reference.get_pose(tip)

    ik_state = RobotState(robot_model)
    ik_state.set_to_default_values(group, "zero")
    if not ik_state.set_from_ik(group, target_pose, tip, 0.5):
        print("NO IK: PickIK could not recover the FK target from SRDF zero", flush=True)
        os._exit(1)

    arm = robot.get_planning_component(group)
    arm.set_start_state(robot_state=start_state)
    arm.set_goal_state(robot_state=ik_state)
    planned = arm.plan()
    if not planned:
        print("NO PLAN: PickIK solution for small joint-space offset", flush=True)
        os._exit(1)

    trajectory_msg = planned.trajectory.get_robot_trajectory_msg()
    points = trajectory_msg.joint_trajectory.points
    if len(points) < 2:
        print("NO PLAN: trajectory has fewer than two points", flush=True)
        os._exit(1)
    start = points[0].positions
    finish = points[-1].positions
    if len(start) != len(finish) or max(abs(end - begin) for begin, end in zip(start, finish)) < 1e-3:
        print("NO PLAN: trajectory has no meaningful joint displacement", flush=True)
        os._exit(1)

    trajectory_path = Path(os.environ.get("SO101_PLAN_OUTPUT", "ros2/.cache/plan-only-smoke.yaml")).expanduser().resolve()
    trajectory_path.parent.mkdir(parents=True, exist_ok=True)
    trajectory_path.write_text(message_to_yaml(trajectory_msg), encoding="utf-8")
    print(
        "PLAN ONLY: 5-DOF PickIK plan has "
        f"{len(points)} points and saved to {trajectory_path}; no execution requested",
        flush=True,
    )
    # Upstream's MoveItPy smoke also bypasses Python teardown after shutting
    # down MoveItCpp; Jazzy's pybind teardown otherwise exits with SIGSEGV.
    os._exit(0)


if __name__ == "__main__":
    raise SystemExit(main())
