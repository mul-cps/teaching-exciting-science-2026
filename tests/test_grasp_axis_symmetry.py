"""Pure CPU checks for opt-in parallel-jaw closing-axis symmetry."""
import ast
from copy import deepcopy
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from tools.ros_plan_grasp import parse_args
from tools.so101_grasp_plan import candidate_for_track


def geometric_status():
    return {"grasp_candidates": [{"track_id": 3, "candidate_id": "green-0003-g0",
        "frame_id": "base_link", "contact_position_world": [.1, .2, .3],
        "closing_axis_world": [1., 0., 0.], "approach_direction_world": [0., 0., -1.],
        "pickup_eligible": True, "ik_validated": False, "collision_checked": False}]}


def test_flip_preserves_approach_and_source_and_default_orientation():
    status = geometric_status()
    original = deepcopy(status)
    default = candidate_for_track(status, 3)
    positive = candidate_for_track(status, 3, closing_axis_sign=1)
    negative = candidate_for_track(status, 3, closing_axis_sign=-1)
    assert default == positive
    first = Rotation.from_quat(positive.orientation_xyzw).as_matrix()
    flipped = Rotation.from_quat(negative.orientation_xyzw).as_matrix()
    np.testing.assert_allclose(flipped[:, 0], -first[:, 0], atol=1e-12)
    np.testing.assert_allclose(flipped[:, 2], first[:, 2], atol=1e-12)
    assert negative.position == positive.position
    assert status == original


@pytest.mark.parametrize("sign", [True, False, 0, 2, "-1", None])
def test_invalid_sign_rejected(sign):
    with pytest.raises(ValueError, match="closing_axis_sign"):
        candidate_for_track(geometric_status(), 3, closing_axis_sign=sign)


@pytest.mark.parametrize("sign", [1, -1])
def test_pickup_and_source_geometry_guards_preserved(sign):
    status = geometric_status()
    status["grasp_candidates"][0]["pickup_eligible"] = False
    with pytest.raises(ValueError, match="pickup"):
        candidate_for_track(status, 3, require_pickup_eligible=True, closing_axis_sign=sign)
    for flag in ("ik_validated", "collision_checked"):
        status = geometric_status()
        status["grasp_candidates"][0][flag] = True
        with pytest.raises(ValueError, match="geometric_only"):
            candidate_for_track(status, 3, closing_axis_sign=sign)


def test_cli_flip_is_opt_in():
    argv = ["--track-id", "green-0003", "--output", "unused.json"]
    assert parse_args(argv).flip_closing_axis is False
    assert parse_args(argv + ["--flip-closing-axis"]).flip_closing_axis is True


def test_every_cli_candidate_lookup_receives_sign():
    source = Path(__file__).resolve().parents[1] / "tools" / "ros_plan_grasp.py"
    calls = [node for node in ast.walk(ast.parse(source.read_text()))
             if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
             and node.func.id == "candidate_for_track"]
    assert calls
    assert all(any(keyword.arg == "closing_axis_sign" for keyword in call.keywords)
               for call in calls)
