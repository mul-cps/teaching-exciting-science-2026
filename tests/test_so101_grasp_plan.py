import pytest

from tools.so101_grasp_plan import PoseTarget, candidate_for_track, staged_targets
from ros2.src.so101_station_scene.so101_station_scene.grasp_planner import (
    attach_collision_object, detach_collision_object,
)


def test_stages_retreat_opposite_approach_and_keep_orientation():
    contact = PoseTarget("base_link", (0.1, 0.2, 0.3), (0, 0, 0, 1))
    stages = staged_targets(contact, approach_m=0.06, lift_m=0.08)
    assert [name for name, _ in stages] == ["pregrasp", "contact", "lift"]
    assert stages[0][1].position == pytest.approx((0.1, 0.2, 0.24))
    assert stages[1][1].position == contact.position
    assert stages[2][1].position == pytest.approx((0.1, 0.2, 0.22))
    assert all(target.orientation_xyzw == contact.orientation_xyzw for _, target in stages)


def test_stages_normalize_quaternion_and_accept_explicit_place():
    contact = PoseTarget("board", (0, 0, 1), (0, 0, 0, 2))
    place = PoseTarget("board", (1, 2, 3), (0, 0, 0, 1))
    stages = staged_targets(contact, place=place)
    assert stages[-1] == ("place", place)
    assert stages[0][1].position == pytest.approx((0, 0, .94))
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


@pytest.mark.parametrize("eligible", [None, False, "true", 1])
def test_geometric_preview_requires_explicit_pickup_admission_for_execution(eligible):
    item = {"track_id": 4, "candidate_id": "green-0004-g0", "frame_id": "board",
            "contact_position_world": [.3, .2, .01], "closing_axis_world": [1, 0, 0],
            "approach_direction_world": [0, 0, -1]}
    if eligible is not None:
        item["pickup_eligible"] = eligible
    status = {"grasp_candidates": [item]}
    assert candidate_for_track(status, "green-0004").position == (.3, .2, .01)
    with pytest.raises(ValueError, match="pickup"):
        candidate_for_track(status, "green-0004", require_pickup_eligible=True)
    item["pickup_eligible"] = True
    assert candidate_for_track(status, "green-0004", require_pickup_eligible=True)


def test_top_down_candidate_retreats_above_contact_and_composes_calibrated_tip():
    import numpy as np
    from scipy.spatial.transform import Rotation
    # Physical grasp +Z points down; its origin is the brick contact center.
    contact = PoseTarget("board", (.1, .2, .03), (1., 0., 0., 0.))
    tip_from_grasp = np.eye(4)
    tip_from_grasp[:3, :3] = Rotation.from_euler("y", 90, degrees=True).as_matrix()
    tip_from_grasp[:3, 3] = [.02, .0, .05]
    place = PoseTarget("board", (.3, .2, .1), (0., 0., 0., 1.))
    targets = staged_targets(contact, tip_from_grasp=tip_from_grasp, place=place)
    for (name, tip), expected_z in zip(targets[:3], (.09, .03, .11)):
        world_from_tip = np.eye(4)
        world_from_tip[:3, :3] = Rotation.from_quat(tip.orientation_xyzw).as_matrix()
        world_from_tip[:3, 3] = tip.position
        world_from_grasp = world_from_tip @ tip_from_grasp
        assert world_from_grasp[:3, 3] == pytest.approx([.1, .2, expected_z])
        assert world_from_grasp[:3, :3] == pytest.approx(np.diag([1., -1., -1.]))
    assert targets[-1] == ("place", place)
    different_offset = tip_from_grasp.copy()
    different_offset[0, 3] += .01
    changed = staged_targets(contact, tip_from_grasp=different_offset)
    assert changed[1][1].position != targets[1][1].position


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
