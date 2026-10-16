"""Real URDF volumes against synthetic aligned depth; no device or oracle mock."""
from pathlib import Path
import time

import numpy as np
import pytest

from workshop.station.recovery_collision_model import URDFCollisionModel
from workshop.station.recovery_depth_scene import DepthRecoveryScene
from workshop.station.camera_recovery import JOINTS, plan_bound_reentry


def urdf(box_size='.02 .02 .02', *, box_origin='0 0 .3', gripper_type='revolute'):
    links = '<link name="base_link"/>'
    joints = ''
    parent = 'base_link'
    for index, raw_joint in enumerate(JOINTS):
        name = f'link{index}'
        geometry = (f'<collision><origin xyz="{box_origin}"/><geometry><box size="{box_size}"/></geometry></collision>'
                    if raw_joint == 'gripper' else '')
        links += f'<link name="{name}">{geometry}</link>'
        kind = gripper_type if raw_joint == 'gripper' else 'revolute'
        joints += (f'<joint name="{raw_joint}_joint" type="{kind}"><parent link="{parent}"/>'
                   f'<child link="{name}"/><axis xyz="0 1 0"/>'
                   '<limit lower="-3.14" upper="3.14" effort="1" velocity="1"/></joint>')
        parent = name
    return f'<robot name="test">{links}{joints}</robot>'


def calibration(**overrides):
    values = {name: {'urdf_joint': name+'_joint', 'raw_reference': 2000,
        'position_reference': 0., 'units_per_count': .001, 'unit': 'radians'} for name in JOINTS}
    values.update(overrides)
    return values


def model(xml=None, **kwargs):
    return URDFCollisionModel(xml or urdf(), joint_calibration=calibration(),
                              calibration_qualified=True, base_frame='base_link', **kwargs)


def scene(collision_model=None, depth=None, **kwargs):
    collision_model = collision_model or model()
    options = dict(depth_m=np.ones((100, 100), dtype=np.float32) if depth is None else depth,
        intrinsics={'fx': 100., 'fy': 100., 'ppx': 49.5, 'ppy': 49.5}, distortion=np.zeros(5),
        base_from_camera=np.eye(4), base_frame='base_link', camera_frame='camera_optical_frame',
        camera_serial='synthetic-qualified', registration_qualified=True,
        model_sha256=collision_model.model_sha256, calibration_sha256=collision_model.calibration_sha256,
        stamp_monotonic=time.monotonic(), arm_stamp_monotonic=time.monotonic(),
        arm_raw={name: 2000 for name in JOINTS}, model=collision_model,
        clearance_m=.01, voxel_size_m=.005, depth_uncertainty_m=.002,
        geometry_uncertainty_m=.001)
    options.update(kwargs)
    return DepthRecoveryScene(**options)


def test_actual_box_model_proves_complete_inflated_volume_against_clear_depth():
    value = scene()
    assert value.minimum_clearance_m(value.arm_raw) == .01


def test_depth_obstacle_inside_robot_volume_is_not_hidden_by_surface_only_sampling():
    depth = np.ones((100, 100), dtype=np.float32)
    depth[49:51, 58:61] = .298
    value = scene(depth=depth)
    moved = dict(value.arm_raw, shoulder_pan=2100)
    assert value.minimum_clearance_m(moved) is None


@pytest.mark.parametrize('bad_depth', [0., np.nan, np.inf, .1])
def test_invalid_or_occluding_depth_never_proves_free_space(bad_depth):
    depth = np.ones((100, 100), dtype=np.float32)
    depth[45:55, 45:55] = bad_depth
    value = scene(depth=depth)
    assert value.minimum_clearance_m(value.arm_raw) is None


def test_unknown_outside_camera_field_of_view_rejects():
    value = scene(base_from_camera=np.array([[1,0,0,2], [0,1,0,0], [0,0,1,0], [0,0,0,1.]]))
    assert value.minimum_clearance_m(value.arm_raw) is None


def test_full_projected_voxel_footprint_checks_pixels_between_corners():
    # A single interior pixel blocks proof although none of the voxel corners
    # projects onto it. This catches corner-only and center-only ray checks.
    depth = np.ones((100, 100), dtype=np.float32)
    depth[49, 48] = 0
    value = scene(depth=depth)
    assert value.minimum_clearance_m(value.arm_raw) is None


def test_transformed_camera_and_nonzero_distortion_are_used():
    transform = np.eye(4)
    transform[0, 3] = .06
    depth = np.ones((100, 100), dtype=np.float32)
    depth[43:57, 22:38] = .2
    value = scene(depth=depth, base_from_camera=transform,
                  distortion=np.array([.04, -.01, .001, -.001, 0]))
    assert value.minimum_clearance_m(value.arm_raw) is None


@pytest.mark.parametrize('options', [
    {'base_frame': 'wrong'}, {'camera_frame': ''}, {'camera_serial': ''},
    {'model_sha256': '0'*64}, {'calibration_sha256': '0'*64},
    {'registration_qualified': False}, {'voxel_size_m': .006},
    {'base_from_camera': np.diag([2., 1, 1, 1])},
    {'stamp_monotonic': float('nan')},
])
def test_identity_frames_registration_and_grid_are_validated(options):
    with pytest.raises(ValueError):
        scene(**options)


def test_gripper_uses_explicit_calibrated_angle_and_changes_real_geometry():
    value = model()
    raw = {name: 2000 for name in JOINTS}
    initial = value.forward_kinematics(raw)['link5']
    changed = value.forward_kinematics(dict(raw, gripper=2500))['link5']
    assert not np.allclose(initial, changed)
    assert value.joint_positions(dict(raw, gripper=2500))['gripper_joint'] == pytest.approx(.5)
    mapping = calibration()
    mapping['gripper']['unit'] = 'percent'
    with pytest.raises(ValueError, match='unit|gripper'):
        URDFCollisionModel(urdf(), joint_calibration=mapping, calibration_qualified=True, base_frame='base_link')


def test_voxel_boundary_is_conservative_and_self_carve_is_strict_interior():
    value = scene(model(urdf(box_origin='0 0 .3025')))
    occupied, interior = value.model.voxel_volume(value.arm_raw, voxel_size_m=.005,
                                                  padding_m=0., interior_margin_m=.001)
    assert len(occupied['link5']) > len(interior['link5'])
    # Cells touching the boundary cannot be automatically vacated as self.
    assert set(map(tuple, interior['link5'])).issubset(set(map(tuple, occupied['link5'])))
    assert not any(np.allclose((index+.5)*.005, [0, 0, .3125]) for index in interior['link5'])


def test_collision_mesh_uses_scaled_convex_volume_and_unresolved_uri_fails(tmp_path):
    import trimesh
    path = tmp_path/'box.stl'
    trimesh.creation.box(extents=[.01, .01, .01]).export(path)
    xml = urdf().replace('<box size=".02 .02 .02"/>', f'<mesh filename="{path}" scale="2 3 4"/>')
    value = model(xml)
    shape = value.shapes[0]
    assert shape.kind == 'convex'
    assert shape.extents_m == pytest.approx([.02, .03, .04], abs=1e-7)
    with pytest.raises((ValueError, FileNotFoundError)):
        model(xml.replace(str(path), 'package://missing/box.stl'))


def test_self_collision_is_checked_unless_exact_link_pair_explicitly_allowed():
    xml = urdf().replace('<link name="link4"></link>', '<link name="link4"><collision><origin xyz="0 0 .3"/>'
                         '<geometry><sphere radius=".02"/></geometry></collision></link>')
    raw = {name: 2000 for name in JOINTS}
    assert scene(model(xml)).minimum_clearance_m(raw) is None
    assert scene(model(xml, allowed_self_contacts=[('link4', 'link5')])).minimum_clearance_m(raw) == .01


def test_reentry_checks_between_endpoint_geometry_not_only_endpoints():
    mapping = calibration()
    mapping['shoulder_pan']['units_per_count'] = .05
    collision_model = URDFCollisionModel(urdf(box_size='.002 .002 .002'), joint_calibration=mapping,
        calibration_qualified=True, base_frame='base_link')
    raw = {name: 2000 for name in JOINTS}
    raw['shoulder_pan'] = 1990
    depth = np.ones((100, 100), dtype=np.float32)
    depth[48:52, 33:36] = .27
    value = scene(collision_model, depth=depth, arm_raw=raw,
                  intrinsics={'fx': 40., 'fy': 40., 'ppx': 49.5, 'ppy': 49.5})
    target = dict(raw, shoulder_pan=1996)
    assert value.minimum_clearance_m(raw) == .01
    assert value.minimum_clearance_m(target) == .01
    assert value.minimum_clearance_m(dict(raw, shoulder_pan=1993)) is None
    assert value.minimum_segment_clearance_m(raw, target) is None
    bounds = {name: (1994, 3000) for name in JOINTS}
    firmware = {name: (0, 4095) for name in JOINTS}
    with pytest.raises(ValueError, match='clearance'):
        plan_bound_reentry(raw, bounds, firmware, scene=value, now=max(value.stamp_monotonic, value.arm_stamp_monotonic),
            expected_model_sha256=value.model_sha256, expected_calibration_sha256=value.calibration_sha256,
            expected_camera_serial=value.camera_serial)


@pytest.mark.parametrize('geometry', ['<sphere radius=".01"/>', '<cylinder radius=".01" length=".03"/>'])
def test_real_sphere_and_cylinder_volumes_are_checked(geometry):
    value = scene(model(urdf().replace('<box size=".02 .02 .02"/>', geometry)))
    assert value.minimum_clearance_m(value.arm_raw) == .01


def test_acquisition_arrays_and_calibration_cannot_drift_after_identity_binding():
    value = scene()
    with pytest.raises((AttributeError, TypeError, ValueError)):
        value.arm_raw['gripper'] = 2500
    with pytest.raises((AttributeError, TypeError, ValueError)):
        value.depth_m[0, 0] = 5
    with pytest.raises((AttributeError, TypeError, ValueError)):
        value.model.calibration['gripper']['units_per_count'] = .1
    with pytest.raises((AttributeError, TypeError, ValueError)):
        value.camera_serial = 'different'


def test_support_contact_requires_explicit_stationary_link_and_never_clears_unknown_depth():
    plane = {'name': 'support', 'normal': [0, 0, 1], 'offset_m': -.295}
    value = scene(support_planes=[plane])
    assert value.minimum_clearance_m(value.arm_raw) is None
    plane['stationary_contact_links'] = ['link5']
    value = scene(support_planes=[plane])
    assert value.minimum_clearance_m(value.arm_raw) == .01
    assert value.minimum_clearance_m(dict(value.arm_raw, shoulder_pan=2100)) is None
    depth = np.full((100, 100), .295, dtype=np.float32)
    value = scene(depth=depth, support_planes=[plane])
    assert value.minimum_clearance_m(value.arm_raw) is None


def test_continuous_segment_can_be_proved_against_a_real_clear_scene():
    value = scene()
    raw = dict(value.arm_raw)
    assert value.minimum_segment_clearance_m(raw, dict(raw, wrist_flex=2002)) == .01
    assert value.model.maximum_displacement_m(raw, dict(raw, wrist_flex=2002)) > 0


def test_segment_support_exemption_does_not_cover_a_link_that_starts_stationary_then_moves():
    plane = {'name': 'support', 'normal': [0, 0, 1], 'offset_m': -.295,
             'stationary_contact_links': ['link5']}
    value = scene(support_planes=[plane])
    assert value.minimum_clearance_m(value.arm_raw) == .01
    assert value.minimum_segment_clearance_m(value.arm_raw, dict(value.arm_raw, wrist_flex=2001)) is None


def test_checked_in_so101_collision_meshes_are_real_solids_and_contacts_fail_closed():
    import xml.etree.ElementTree as ET
    path = Path(__file__).resolve().parents[1]/'workshop/digital_twin/assets/so101_new_calib.urdf'
    root = ET.parse(path).getroot()
    # Explicit saved-package mapping; production never guesses a mesh path.
    for mesh in root.findall('.//mesh'):
        mesh.set('filename', 'package://station_cad/'+Path(mesh.get('filename')).name)
    mapping = calibration()
    for name in JOINTS:
        mapping[name]['urdf_joint'] = name
    actual = URDFCollisionModel(ET.tostring(root).decode(), joint_calibration=mapping,
        calibration_qualified=True, base_frame='base_link',
        package_paths={'station_cad': path.parent/'cad'})
    assert len(actual.shapes) >= 17
    assert all(shape.kind == 'convex' for shape in actual.shapes)
    raw = {name: 2000 for name in JOINTS}
    occupied, interior = actual.voxel_volume(raw, voxel_size_m=.005, interior_margin_m=.001)
    assert sum(map(len, occupied.values())) > 10_000
    # Raw triangle-soup STL/concave CAD has no certified physical-material
    # interior. Convex collision enclosures alone cannot supply that proof.
    assert sum(map(len, interior.values())) == 0
    # Preload immutable model volumes before acquisition; this test deliberately
    # supplies no qualified self-contact exclusions or measured camera artifact.
    started = time.monotonic()
    value = scene(actual)
    assert value.minimum_clearance_m(raw) is None
    assert 'self collision' in value.last_rejection
    assert time.monotonic()-started < .5


def test_oracle_expires_instead_of_reusing_an_old_success(monkeypatch):
    value = scene()
    assert value.minimum_clearance_m(value.arm_raw) == .01
    monkeypatch.setattr('workshop.station.recovery_depth_scene.time.monotonic', lambda: value.stamp_monotonic+.501)
    assert value.minimum_clearance_m(value.arm_raw) is None


def test_real_so101_single_view_self_surface_leaves_backside_padding_unknown():
    import xml.etree.ElementTree as ET
    path = Path(__file__).resolve().parents[1]/'workshop/digital_twin/assets/so101_new_calib.urdf'
    root = ET.parse(path).getroot()
    for mesh in root.findall('.//mesh'):
        mesh.set('filename', 'package://station_cad/'+Path(mesh.get('filename')).name)
    # Exact upstream so101_arm.srdf declarations, supplied explicitly. These
    # semantics are a benchmark fixture, not qualification of the physical rig.
    declared_pairs = [('base_link', 'shoulder_link'), ('base_link', 'upper_arm_link'),
        ('gripper_link', 'lower_arm_link'), ('gripper_link', 'moving_jaw_so101_v1_link'),
        ('gripper_link', 'wrist_link'), ('lower_arm_link', 'moving_jaw_so101_v1_link'),
        ('lower_arm_link', 'upper_arm_link'), ('lower_arm_link', 'wrist_link'),
        ('moving_jaw_so101_v1_link', 'wrist_link'), ('shoulder_link', 'upper_arm_link')]
    mapping = calibration()
    for name in JOINTS:
        mapping[name]['urdf_joint'] = name
    actual = URDFCollisionModel(ET.tostring(root).decode(), joint_calibration=mapping,
        calibration_qualified=True, base_frame='base_link', allowed_self_contacts=declared_pairs,
        package_paths={'station_cad': path.parent/'cad'})
    raw = {name: 2000 for name in JOINTS}
    volume, _ = actual.voxel_volume(raw, voxel_size_m=.005, interior_margin_m=.001)
    # Prewarm exact candidate geometry outside the acquisition freshness window.
    actual.voxel_volume(raw, voxel_size_m=.005, padding_m=.011, interior_margin_m=.001)
    depth = np.full((480, 640), 2., dtype=float)
    # Render the front of conservative current-body voxels. Every covered pixel
    # receives the nearest voxel-corner depth; no unseen backside is made free.
    corners = np.array([[x, y, z] for x in (-.5, .5) for y in (-.5, .5) for z in (-.5, .5)])*.005
    indices = np.unique(np.concatenate(list(volume.values())), axis=0)
    for index in indices:
        points = (index+.5)*.005+corners
        camera = points*np.array([1, -1, -1])+np.array([0, 0, 1])
        uv = camera[:, :2]/camera[:, 2, None]*380+np.array([319.5, 239.5])
        low, high = np.floor(uv.min(axis=0)).astype(int), np.ceil(uv.max(axis=0)).astype(int)
        if np.any(low < 0) or high[0] >= 640 or high[1] >= 480:
            continue
        view = depth[low[1]:high[1]+1, low[0]:high[0]+1]
        np.minimum(view, camera[:, 2].min(), out=view)
    transform = np.diag([1., -1., -1., 1.])
    transform[2, 3] = 1
    value = scene(actual, depth=depth, base_from_camera=transform,
                  intrinsics={'fx': 380., 'fy': 380., 'ppx': 319.5, 'ppy': 239.5})
    assert value.minimum_clearance_m(raw) is None
    assert 'occluded' in value.last_rejection


def test_convex_collision_enclosure_does_not_carve_empty_mesh_gaps_as_current_robot(tmp_path):
    import trimesh
    left = trimesh.creation.box(extents=[.01, .01, .01])
    right = left.copy()
    left.apply_translation([-.02, 0, 0])
    right.apply_translation([.02, 0, 0])
    path = tmp_path/'separated.stl'
    trimesh.util.concatenate([left, right]).export(path)
    xml = urdf().replace('<box size=".02 .02 .02"/>', f'<mesh filename="{path}"/>')
    actual = model(xml)
    assert not actual.shapes[0].self_interior_qualified
    occupied, interior = actual.voxel_volume({name: 2000 for name in JOINTS}, voxel_size_m=.005,
                                             interior_margin_m=.001)
    assert len(occupied['link5']) > 0
    assert len(interior['link5']) == 0


def test_watertight_convex_mesh_can_supply_strict_physical_self_interior(tmp_path):
    import trimesh
    path = tmp_path/'solid.obj'
    trimesh.creation.box(extents=[.04, .04, .04]).export(path)
    xml = urdf().replace('<box size=".02 .02 .02"/>', f'<mesh filename="{path}"/>')
    actual = model(xml)
    assert actual.shapes[0].self_interior_qualified
    occupied, interior = actual.voxel_volume({name: 2000 for name in JOINTS}, voxel_size_m=.005,
                                             interior_margin_m=.001)
    assert 0 < len(interior['link5']) < len(occupied['link5'])
