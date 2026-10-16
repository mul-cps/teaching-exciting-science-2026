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


def tilted_image_selection(scale=1.):
    """Perfect pinhole backprojection of an image rectangle onto a tilted plane."""
    from workshop.digital_twin.placement_region import pose_from_rectangle
    # Camera rays are pixel offsets normalized by focal length. Plane:
    # z=.6+.55*x+.30*y, producing a convex perspective quadrilateral.
    image = np.array([[-.25,-.18],[.25,-.18],[.25,.18],[-.25,.18]])*scale
    rays = np.column_stack((image, np.ones(4)))
    distance = .6/(1.-.55*rays[:,0]-.30*rays[:,1])
    corners = rays*distance[:,None]
    pose, geometry = pose_from_rectangle(corners)
    selection = str(uuid.uuid4())
    status = dict(state='tracked', valid=True, selection_id=selection, frame_id='camera',
        stamp_ns=1_000_000_000, camera_from_goal_stamp_ns=1_000_000_000,
        camera_from_goal=pose.tolist(), fit_rms_m=.0001, goal_geometry=geometry,
        plane_metrics={'plane_rms_m': .0001}, goal_corners_camera=corners.tolist())
    registration = QualifiedFrameRegistration('camera', 'base_link', np.eye(4), 'synthetic-pinned-sha')
    kwargs = dict(selection_id=selection, registration=registration, received_monotonic=10.,
        now_monotonic=10.1, now_ros_ns=1_100_000_000, goal_from_tip=np.eye(4),
        footprint_m=[.04,.04], tool_footprint_m=[.03,.03], clearance_m=.005,
        registration_uncertainty_m=.002)
    return status, kwargs


def test_perfect_tilted_perspective_image_selection_resolves_to_usable_goal(tmp_path):
    from workshop.digital_twin.placement_region import resolve_placement_goal
    status, kwargs = tilted_image_selection()
    import json
    from workshop.digital_twin.placement_region import load_qualified_frame_registration
    base_from_camera = np.eye(4)
    base_from_camera[:3,:3] = [[0,-1,0],[1,0,0],[0,0,1]]
    base_from_camera[:3,3] = [.1,-.2,.3]
    calibration = tmp_path/'synthetic-qualified-registration.json'
    calibration.write_text(json.dumps(dict(schema='camera-arm-calibration-v1', qualified=True,
        transform_direction='base_from_camera', camera_frame='camera',
        base_from_camera=base_from_camera.tolist())))
    kwargs['registration'] = load_qualified_frame_registration(calibration)
    target = resolve_placement_goal(status, expected_selection_id=kwargs['selection_id'],
        registration=kwargs['registration'], received_monotonic=10., now_monotonic=10.1,
        now_ros_ns=1_100_000_000)
    corners = np.asarray(status['goal_corners_camera'])
    # Prove this is not the old axis-aligned/rectangle synthetic special case.
    assert not np.isclose(np.linalg.norm(corners[1]-corners[0]), np.linalg.norm(corners[2]-corners[3]))
    resolved, position, quaternion, area = resolve_tool_destination(status, **kwargs)
    assert resolved.selection_id == target.selection_id
    assert resolved.registration_sha256 == kwargs['registration'].sha256
    np.testing.assert_allclose(position, target.position_m)
    assert min(area) > .05


def test_perspective_small_area_and_offset_tool_protrusion_fail():
    status, kwargs = tilted_image_selection(.10)
    with pytest.raises(ValueError, match='footprint'):
        resolve_tool_destination(status, **kwargs)
    status, kwargs = tilted_image_selection()
    offset = np.eye(4)
    offset[0,3] = .16
    with pytest.raises(ValueError, match='footprint'):
        resolve_tool_destination(status, **dict(kwargs, goal_from_tip=offset))


def test_perspective_quad_still_rejects_nonplanar_and_malformed_corners():
    status, kwargs = tilted_image_selection()
    corners = np.asarray(status['goal_corners_camera'])
    normal = np.asarray(status['camera_from_goal'])[:3,2]
    nonplanar = corners.copy()
    nonplanar[2] += .004*normal
    with pytest.raises(ValueError, match='inconsistent'):
        resolve_tool_destination(dict(status, goal_corners_camera=nonplanar.tolist()), **kwargs)
    with pytest.raises(ValueError, match='ordering'):
        resolve_tool_destination(dict(status, goal_corners_camera=corners[[0,2,1,3]].tolist()), **kwargs)


def test_perspective_tool_inside_bounding_box_but_outside_quad_is_rejected():
    status, kwargs = tilted_image_selection()
    pose = np.asarray(status['camera_from_goal'])
    polygon = (np.asarray(status['goal_corners_camera'])-pose[:3,3])@pose[:3,:3]
    tool = np.eye(4)
    tool[:2,3] = [-.16,.13]
    tool_vertices = np.array([[-1,-1],[1,-1],[1,1],[-1,1]])*.002+tool[:2,3]
    margin = .005+.002+3*np.hypot(.0001,.0001)
    assert np.all(tool_vertices > polygon[:,:2].min(axis=0)+margin)
    assert np.all(tool_vertices < polygon[:,:2].max(axis=0)-margin)
    with pytest.raises(ValueError, match='footprint'):
        resolve_tool_destination(status, **dict(kwargs, goal_from_tip=tool, tool_footprint_m=[.004,.004]))
