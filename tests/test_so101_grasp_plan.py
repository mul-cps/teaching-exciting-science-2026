import pytest

from tools.so101_grasp_plan import PoseTarget, candidate_for_track, staged_targets
from ros2.src.so101_station_scene.so101_station_scene.grasp_planner import (
    attach_collision_object, detach_collision_object,
)


def test_stages_follow_tool_positive_z_and_keep_orientation():
    contact = PoseTarget("base_link", (0.1, 0.2, 0.3), (0, 0, 0, 1))
    stages = staged_targets(contact, approach_m=0.06, lift_m=0.08)
    assert [name for name, _ in stages] == ["pregrasp", "contact", "lift"]
    assert stages[0][1].position == pytest.approx((0.1, 0.2, 0.36))
    assert stages[1][1].position == contact.position
    assert stages[2][1].position == pytest.approx((0.1, 0.2, 0.38))
    assert all(target.orientation_xyzw == contact.orientation_xyzw for _, target in stages)


def test_stages_normalize_quaternion_and_accept_explicit_place():
    contact = PoseTarget("board", (0, 0, 1), (0, 0, 0, 2))
    place = PoseTarget("board", (1, 2, 3), (0, 0, 0, 1))
    stages = staged_targets(contact, place=place)
    assert stages[-1] == ("place", place)
    assert stages[0][1].position == pytest.approx((0, 0, 1.06))
    assert stages[0][1].orientation_xyzw == (0, 0, 0, 1)


@pytest.mark.parametrize("value", [0, -0.1, 0.31, float("nan")])
def test_stage_distance_must_be_bounded(value):
    with pytest.raises(ValueError):
        staged_targets(PoseTarget("base", (0, 0, 0), (0, 0, 0, 1)), approach_m=value)


def test_candidate_is_selected_by_stable_track_and_stays_geometric():
    status = {"grasp_candidates": [{
        "track_id": 4, "candidate_id": "red-0004-g0", "frame_id": "charuco_board",
        "contact_position_world": [0.01, 0.02, 0.03],
        "closing_axis_world": [1, 0, 0], "approach_direction_world": [0, 0, 1],
        "ik_validated": False, "collision_checked": False,
    }]}
    target = candidate_for_track(status, "red-0004")
    assert candidate_for_track(status, "4") == target
    assert target.frame_id == "charuco_board"
    assert target.position == pytest.approx((0.01, 0.02, 0.03))
    assert target.orientation_xyzw == pytest.approx((0, 0, 0, 1))
    with pytest.raises(ValueError, match="no geometric"):
        candidate_for_track(status, "blue-0002")


def test_attachment_diffs_require_explicit_gripper_touch_links():
    moveit_msgs = pytest.importorskip("moveit_msgs.msg")
    CollisionObject = moveit_msgs.CollisionObject
    obj = CollisionObject()
    obj.id = "colorbrick_red-0004"
    obj.operation = CollisionObject.ADD
    with pytest.raises(ValueError, match="touch link"):
        attach_collision_object(obj, link_name="gripper_frame_link", touch_links=[])
    attach = attach_collision_object(
        obj, link_name="gripper_frame_link", touch_links=["left_finger", "right_finger"])
    assert attach.is_diff and attach.robot_state.is_diff
    attached = attach.robot_state.attached_collision_objects[0]
    assert attached.object.id == obj.id
    assert attached.touch_links == ["left_finger", "right_finger"]

    detach = detach_collision_object(obj.id)
    removed = detach.robot_state.attached_collision_objects[0]
    assert detach.is_diff and detach.robot_state.is_diff
    assert removed.object.id == obj.id
    assert removed.object.operation == CollisionObject.REMOVE
