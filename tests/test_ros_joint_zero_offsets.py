"""Model-only origin correction leaves commands/encoder coordinates untouched."""
import ast
import math
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros2/src/so101_station_control'))
from so101_station_control.model_offsets import apply_joint_zero_offsets, apply_model_offsets

CONFIG = ROOT / 'ros2/src/so101_station_control/config/model_offsets.yaml'
SOURCE = ROOT / 'workshop/digital_twin/assets/so101_new_calib.urdf'
OFFSETS = {'shoulder_lift': 11.616411167163976,
           'elbow_flex': -10.348454041608484, 'wrist_flex': -9.05585412227618}


def fk_api():
    tree = ast.parse((ROOT / 'tools/fit_ros_robot_registration.py').read_text())
    functions = [node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name in
                 ('_numbers', '_transform', 'parse_urdf', '_axis_rotation', 'forward_kinematics')]
    namespace = dict(np=np, math=math, Path=Path, ET=ET)
    exec(compile(ast.Module(body=functions, type_ignores=[]), 'pure-urdf-fk', 'exec'), namespace)
    return namespace['parse_urdf'], namespace['forward_kinematics']


def test_exact_frozen_configuration_is_provisional_and_identity_bound():
    config = yaml.safe_load(CONFIG.read_text())
    assert config['joint_zero_offsets_deg'] == OFFSETS
    assert config['gripper_mount_roll_deg'] == -90.
    assert config['qualified'] is False
    assert config['status'] == 'PROVISIONAL'
    assert config['provenance']['arm_id'] == 'hs_rig01_follower'
    assert config['provenance']['calibration_sha256'] == 'bc688147848f386899f8f34eb09cdb285fcdb0fc8de576b78a50a8efaf7f424c'


def test_fk_origin_correction_equals_old_q_plus_offset_for_every_link():
    parse, fk = fk_api()
    old = SOURCE.read_text()
    new = apply_joint_zero_offsets(old, OFFSETS)
    original, corrected = ET.fromstring(old), ET.fromstring(new)
    for joint in corrected.findall('joint'):
        name = joint.get('name')
        if name in OFFSETS:
            joint.find('origin').set('rpy', original.find(f"joint[@name='{name}']/origin").get('rpy'))
    assert ET.tostring(corrected) == ET.tostring(original)
    old_links, old_joints = parse(original)
    new_links, new_joints = parse(ET.fromstring(new))
    rng = np.random.default_rng(13)
    for _ in range(12):
        q = dict(zip(('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper'), rng.uniform(-1, 1, 6)))
        before = fk(old_links, old_joints, q | {name: q[name] + math.radians(value) for name, value in OFFSETS.items()})
        after = fk(new_links, new_joints, q)
        for name in before:
            np.testing.assert_allclose(after[name], before[name], rtol=0, atol=1e-12)


@pytest.mark.parametrize('offsets', [None, [], {'shoulder_pan': 1}, {'wrist_roll': 1}, {'gripper': 1},
                                    {'shoulder_lift': True}, {'elbow_flex': '1'}, {'wrist_flex': float('nan')},
                                    {'wrist_flex': float('inf')}, {'wrist_flex': 12.00001}])
def test_invalid_config_rejected(offsets):
    with pytest.raises(ValueError):
        apply_joint_zero_offsets(SOURCE.read_text(), offsets)


def test_empty_and_zero_offsets_preserve_original_bytes_and_limits_at_boundary():
    original = SOURCE.read_text()
    assert apply_joint_zero_offsets(original, {}) == original
    assert apply_joint_zero_offsets(original, {'wrist_flex': 0.}) == original
    for value in (-12., 12.):
        apply_joint_zero_offsets(original, {'wrist_flex': value})


def test_joint_axis_and_origin_are_required_and_finite():
    for xml in ('<robot/>', '<robot><joint name="wrist_flex" type="fixed"/></robot>',
                '<robot><joint name="wrist_flex" type="revolute"><origin rpy="nan 0 0"/><axis xyz="0 0 1"/></joint></robot>',
                '<robot><joint name="wrist_flex" type="revolute"><origin/><axis xyz="0 0 0"/></joint></robot>'):
        with pytest.raises(ValueError):
            apply_joint_zero_offsets(xml, {'wrist_flex': 1.})


def test_shared_pipeline_applies_mount_then_zeros_once():
    from so101_station_control.model_offsets import apply_gripper_mount_roll
    config = yaml.safe_load(CONFIG.read_text())
    assert apply_model_offsets(SOURCE.read_text(), config) == apply_joint_zero_offsets(
        apply_gripper_mount_roll(SOURCE.read_text(), -90.), OFFSETS)
    for filename in ('ros2/src/so101_station_control/launch/moveit_resident.launch.py',
                     'ros2/src/so101_station_bringup/launch/guarded_moveit_rviz.launch.py'):
        text = (ROOT / filename).read_text()
        assert 'apply_model_offsets(' in text
        assert text.count('apply_model_offsets(') == 1
    for filename in ('ros2/src/so101_station_perception/so101_station_perception/virtual_gripper_camera.py',
                     'workshop/digital_twin/ros_state_node.py'):
        text = (ROOT / filename).read_text()
        assert '/follower/robot_description' in text
        assert 'apply_joint_zero_offsets(' not in text


def test_model_offsets_only_run_at_description_load_and_never_touch_robot_io():
    path = ROOT / 'ros2/src/so101_station_control/so101_station_control/model_offsets.py'
    tree = ast.parse(path.read_text())
    calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call)]
    forbidden = ('open', 'read_text', 'read_bytes', 'write_text', 'write_bytes', 'create_client', 'create_timer', 'publish', 'call_async', 'set_parameters')
    assert not any(isinstance(node.func, ast.Name) and node.func.id in forbidden or
                   isinstance(node.func, ast.Attribute) and node.func.attr in forbidden for node in calls)
    assert not any(isinstance(node, (ast.While, ast.AsyncFunctionDef)) for node in ast.walk(tree))
