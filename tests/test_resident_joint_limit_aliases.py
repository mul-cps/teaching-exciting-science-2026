"""Offline regression for MoveIt YAML anchors aliasing per-joint dictionaries."""
import ast
from pathlib import Path

import pytest
import yaml

ROOT=Path(__file__).resolve().parents[1]
SOURCE=ROOT/'ros2/src/so101_station_control/launch/moveit_resident.launch.py'
TREE=ast.parse(SOURCE.read_text())
JOINTS=('shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll')
FUNCTION=next(n for n in TREE.body if isinstance(n,ast.FunctionDef) and n.name=='_independent_joint_limits')
NAMESPACE={}
exec(compile(ast.Module(body=[FUNCTION],type_ignores=[]),str(SOURCE),'exec'),NAMESPACE)


@pytest.mark.parametrize('fixture',['all_shared','station_config'])
def test_calibrated_bounds_are_independent_despite_yaml_anchors(fixture):
    if fixture=='station_config':
        original=yaml.safe_load((ROOT/'ros2/config/rig01_moveit_limits.yaml').read_text())['joint_limits']
    else:
        original=yaml.safe_load('''
shoulder_pan: &station_arm_limits
  has_position_limits: true
  min_position: -1.57
  max_position: 1.57
  has_velocity_limits: true
  max_velocity: 0.125
  has_acceleration_limits: true
  max_acceleration: 0.2
  has_jerk_limits: true
  max_jerk: 2.0
shoulder_lift: *station_arm_limits
elbow_flex: *station_arm_limits
wrist_flex: *station_arm_limits
wrist_roll: *station_arm_limits
''')
    assert original['shoulder_pan'] is original['wrist_roll']
    before={j:dict(values) for j,values in original.items()}
    independent=NAMESPACE['_independent_joint_limits'](original)
    assert independent==original
    assert len({id(values) for values in independent.values()})==len(independent)
    # Distinct calibration bounds must survive the final wrist-roll update.
    expected={j:(-1.1-index*.1,1.2+index*.1) for index,j in enumerate(JOINTS)}
    for joint,(low,high) in expected.items():
        independent[joint].update(has_position_limits=True,min_position=low,max_position=high)
    for joint,(low,high) in expected.items():
        assert (independent[joint]['min_position'],independent[joint]['max_position'])==(low,high)
        assert {k:v for k,v in independent[joint].items() if k not in {'min_position','max_position'}}=={
            k:v for k,v in before[joint].items() if k not in {'min_position','max_position'}}
    assert original==before


def test_launch_detaches_aliases_before_first_per_joint_update():
    launch=next(n for n in TREE.body if isinstance(n,ast.FunctionDef) and n.name=='_launch_nodes')
    copies=[n for n in ast.walk(launch) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name)
        and n.func.id=='_independent_joint_limits']
    updates=[n for n in ast.walk(launch) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)
        and n.func.attr=='update']
    assert len(copies)==1 and len(updates)>=2
    assert copies[0].lineno<min(n.lineno for n in updates)
