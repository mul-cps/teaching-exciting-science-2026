"""CPU endpoint checks: planner SUCCESS must not hide a clamped IK result."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from tools.ros_plan_grasp import validate_endpoint_pose, validate_endpoint_fk_response
from tools.so101_grasp_plan import PoseTarget


def pose(x=0., angle=0., *, frame="base_link", scale=1.):
    return PoseTarget(frame, (x, 0., 0.),
                      (0., 0., scale*math.sin(angle/2), scale*math.cos(angle/2)))


def response(actual, *, code=1, links=("tip",)):
    stamped = NS(header=NS(frame_id=actual.frame_id), pose=NS(
        position=NS(**dict(zip(("x", "y", "z"), actual.position))),
        orientation=NS(**dict(zip(("x", "y", "z", "w"), actual.orientation_xyzw)))))
    return NS(error_code=NS(val=code), fk_link_names=list(links), pose_stamped=[stamped])


def test_exact_endpoint_metrics():
    metrics = validate_endpoint_pose(pose(), pose())
    assert metrics == {"position_error_m": 0., "rotation_error_rad": 0.,
                       "position_tolerance_m": .008, "rotation_tolerance_rad": .35,
                       "frame_id": "base_link"}


@pytest.mark.parametrize("scale", [-3., .25, 2.])
def test_quaternion_normalization_and_sign_do_not_change_orientation(scale):
    assert validate_endpoint_pose(pose(angle=.9), pose(angle=.9, scale=scale))[
        "rotation_error_rad"] == pytest.approx(0., abs=5e-8)


def test_spherical_bound_and_translation_boundary():
    assert validate_endpoint_pose(pose(), pose(x=.008, angle=.35))
    for actual in (pose(x=.008001), pose(angle=.350001)):
        with pytest.raises(ValueError, match="misses target"):
            validate_endpoint_pose(pose(), actual)


def test_spherical_bound_rejects_combined_axis_rotation_inside_planner_axis_box():
    # 0.3 rad about each of X and Y, composed: each planner axis can pass but
    # total rotation is >0.35 rad, deliberately rejected by this stricter gate.
    s, c = math.sin(.15), math.cos(.15)
    actual = PoseTarget("base_link", (0., 0., 0.), (s*c, c*s, s*s, c*c))
    with pytest.raises(ValueError, match="rotation"):
        validate_endpoint_pose(pose(), actual)


def test_clamped_wrist_success_response_is_rejected():
    # Observed MoveGroup case: requested wrist 2 rad, actual endpoint at pi/2.
    with pytest.raises(ValueError, match="misses target"):
        validate_endpoint_fk_response(pose(angle=2.), response(pose(angle=math.pi/2)), "tip")


def test_wrong_frame_is_rejected():
    with pytest.raises(ValueError, match="frame"):
        validate_endpoint_pose(pose(), pose(frame="camera"))


@pytest.mark.parametrize("field,value", [
    ("position", (float("nan"), 0., 0.)),
    ("orientation_xyzw", (0., 0., 0., float("inf"))),
    ("orientation_xyzw", (0., 0., 0., 0.)),
])
def test_nonfinite_or_zero_pose_is_rejected_even_if_constructed_unsafely(field, value):
    actual = pose()
    object.__setattr__(actual, field, value)
    with pytest.raises(ValueError, match="finite|nonzero"):
        validate_endpoint_pose(pose(), actual)


@pytest.mark.parametrize("code,links", [(0, ("tip",)), (1, ("wrong",)),
                                          (1, ()), (1, ("tip", "tip"))])
def test_fk_failure_or_incorrect_links_fail_closed(code, links):
    with pytest.raises(ValueError, match="FK"):
        validate_endpoint_fk_response(pose(), response(pose(), code=code, links=links), "tip")


def test_successful_fk_response_records_pose_metrics():
    metrics = validate_endpoint_fk_response(pose(), response(pose(x=.004, angle=.1)), "tip")
    assert metrics["position_error_m"] == pytest.approx(.004)
    assert metrics["rotation_error_rad"] == pytest.approx(.1)


@pytest.mark.parametrize("kwargs", [{"pos_tol": -1.}, {"rot_tol": float("nan")}])
def test_invalid_tolerances_rejected(kwargs):
    with pytest.raises(ValueError, match="tolerances"):
        validate_endpoint_pose(pose(), pose(), **kwargs)


def test_fk_requires_single_pose_and_matching_frame_and_finite_response():
    for actual in (pose(frame="camera"),):
        with pytest.raises(ValueError):
            validate_endpoint_fk_response(pose(), response(actual), "tip")
    malformed = response(pose())
    malformed.pose_stamped *= 2
    with pytest.raises(ValueError, match="exactly"):
        validate_endpoint_fk_response(pose(), malformed, "tip")
    malformed = response(pose())
    malformed.pose_stamped[0].pose.position.x = float("nan")
    with pytest.raises(ValueError, match="finite"):
        validate_endpoint_fk_response(pose(), malformed, "tip")


def test_endpoint_gate_precedes_state_update_and_execution_for_both_modes():
    """Static flow contract without importing native ROS or opening any devices."""
    source = (Path(__file__).parents[1] / "tools/ros_plan_grasp.py").read_text()
    tree = ast.parse(source)
    loop = next(n for n in ast.walk(tree) if isinstance(n, ast.For) and
                ast.unparse(n.target) == "(name, target)")
    gate_index = next(i for i, n in enumerate(loop.body) if isinstance(n, ast.Try) and
                      "validate_endpoint_fk_response" in ast.unparse(n))
    gate = loop.body[gate_index]
    update_index = next(i for i, n in enumerate(loop.body) if isinstance(n, ast.Expr) and
                        ast.unparse(n).startswith("current.update("))
    execution_index = next(i for i, n in enumerate(loop.body) if isinstance(n, ast.If) and
                           ast.unparse(n.test) == "args.execute" and
                           "move_trajectory" in ast.unparse(n))
    assert gate_index < update_index < execution_index
    assert len([n for n in ast.walk(gate) if isinstance(n, ast.Call) and
                ast.unparse(n.func) == "node.fk.call_async"]) == 1
    assert "ALL_JOINTS" in ast.unparse(gate)
    assert "points[-1].positions" in ast.unparse(gate)
    assert "endpoint_validation_failed" in ast.unparse(gate.handlers)
    assert "disallow_contact" in ast.unparse(gate.handlers)
    assert any(isinstance(n, ast.Break) for n in ast.walk(gate.handlers[0]))
    assert not any(isinstance(n, ast.Call) and ast.unparse(n.func) == "move_trajectory"
                   for n in ast.walk(gate))
