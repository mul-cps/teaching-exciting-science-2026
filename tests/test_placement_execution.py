import uuid
import numpy as np
import pytest
from workshop.digital_twin.placement_region import QualifiedFrameRegistration
from workshop.station.placement_execution import resolve_tool_destination, same_destination, DestinationPose
from tools.so101_grasp_plan import PoseTarget


def test_selected_region_composes_commissioned_tool_height_and_rejects_stale():
    selection = str(uuid.uuid4())
    registration = QualifiedFrameRegistration('camera', 'base_link', np.eye(4), 'frozen-sha')
    pose = np.eye(4)
    pose[:3, 3] = [.3, .1, .05]
    observation = dict(state='tracked', valid=True, selection_id=selection, frame_id='camera',
        stamp_ns=1_000_000_000, camera_from_goal_stamp_ns=1_000_000_000,
        camera_from_goal=pose.tolist(), fit_rms_m=.001,
        goal_geometry={"width_m": .2, "height_m": .2}, plane_metrics={"plane_rms_m": .001},
        goal_corners_camera=(np.array([[-.1,-.1,0],[.1,-.1,0],[.1,.1,0],[-.1,.1,0]])+pose[:3,3]).tolist())
    tool = np.eye(4)
    tool[2, 3] = .04
    kwargs = dict(selection_id=selection, registration=registration, received_monotonic=10.,
                  now_monotonic=10.1, now_ros_ns=1_100_000_000, goal_from_tip=tool, footprint_m=[.04,.04],
                  tool_footprint_m=[.05,.05], clearance_m=.005, registration_uncertainty_m=.002)
    resolved, xyz, quaternion, area = resolve_tool_destination(observation, **kwargs)
    assert resolved.registration_sha256 == 'frozen-sha'
    np.testing.assert_allclose(xyz, [.3, .1, .09])
    np.testing.assert_allclose(quaternion, [0, 0, 0, 1])
    with pytest.raises(ValueError):
        resolve_tool_destination(observation, **dict(kwargs, now_monotonic=10.5))
    with pytest.raises(ValueError):
        resolve_tool_destination(observation, **dict(kwargs, goal_from_tip=None))
    with pytest.raises(ValueError):
        resolve_tool_destination(observation, **dict(kwargs, selection_id=str(uuid.uuid4())))


def test_changed_goal_requires_new_plan():
    original = DestinationPose('base_link', (.3, .1, .1), (0, 0, 0, 1), 'selection', 'sha', (.1,.1))
    assert same_destination(original, original)
    assert not same_destination(original, DestinationPose('base_link', (.302, .1, .1), (0, 0, 0, 1), 'selection', 'sha', (.1,.1)))
    assert not same_destination(original, DestinationPose('camera', (.3, .1, .1), (0, 0, 0, 1), 'selection', 'sha', (.1,.1)))

    from dataclasses import replace
    assert not same_destination(original, replace(original, selection_id='new-selection'))
    assert not same_destination(original, replace(original, registration_sha256='changed-sha'))
    assert not same_destination(original, replace(original, area_half_extents_m=(.08,.1)))


def test_tiny_area_and_oversized_footprints_are_rejected():
    from workshop.station.placement_execution import require_area_contains
    from workshop.digital_twin.placement_region import PlacementGoalTarget
    registration = QualifiedFrameRegistration('camera', 'base_link', np.eye(4), 'sha')
    target = PlacementGoalTarget('base_link', (0,0,0), (0,0,0,1), np.eye(4), 'id', 1, 'sha', area_width_m=.06, area_height_m=.06, fit_uncertainty_m=.001)
    corners = np.array([[-.03,-.03,0],[.03,-.03,0],[.03,.03,0],[-.03,.03,0]])
    status = dict(goal_corners_camera=corners.tolist(), fit_rms_m=.001)
    kwargs = dict(footprint_m=[.04,.04], tool_footprint_m=[.03,.03], clearance_m=.005,
                  registration_uncertainty_m=.002, goal_from_tip=np.eye(4))
    assert require_area_contains(status, target, registration, **kwargs) == (.03,.03)
    with pytest.raises(ValueError, match='footprint'):
        require_area_contains(status, target, registration, **dict(kwargs, footprint_m=[.08,.08]))
    with pytest.raises(ValueError):
        require_area_contains(dict(status, goal_corners_camera=(corners*.2).tolist()), target, registration, **kwargs)
    with pytest.raises(ValueError, match='ordering'):
        require_area_contains(dict(status, goal_corners_camera=corners[[0,2,1,3]].tolist()), target, registration, **kwargs)
    moved = corners + [.04, 0, 0]
    with pytest.raises(ValueError, match='inconsistent'):
        require_area_contains(dict(status, goal_corners_camera=moved.tolist()), target, registration, **kwargs)


def test_actual_carried_dimensions_are_admitted_conservatively_for_rotation():
    from workshop.station.placement_execution import require_observed_footprint
    require_observed_footprint([.02,.02,.01], [.04,.04])
    with pytest.raises(ValueError, match='actual carried'):
        require_observed_footprint([.04,.04,.01], [.04,.04])
    with pytest.raises(ValueError):
        require_observed_footprint(None, [.04,.04])
