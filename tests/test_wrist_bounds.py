"""Wrist envelope matches the existing URDF; no recalibration or hardware."""
import hashlib
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml

from workshop.station.ros_trajectory import (
    ARM_JOINTS, POSITION_LIMITS_RAD, _position_limits,
    prepare_trajectory, prepare_supervised_calibration,
)

ROOT=Path(__file__).resolve().parents[1]
WRIST='wrist_flex'
MID=1995.5
MODEL_LIMIT=1.65806
CAL_RANGES={'shoulder_pan':(745,3470),'shoulder_lift':(1063,3240),
            'elbow_flex':(848,3055),'wrist_flex':(820,3171),
            'wrist_roll':(0,4095),'gripper':(1872,3366)}


def angle(raw):return (raw-MID)*2*math.pi/4095


def prepare(values,limits=None):
    poses=[[0.,0.,0.,value,0.] for value in values]
    return prepare_trajectory(ARM_JOINTS,[{'time':i*10.,'positions':p} for i,p in enumerate(poses)],
        start_radians=dict(zip(ARM_JOINTS,poses[0])),max_velocity=1.,max_acceleration=2.,
        position_limits=limits)


def test_wrist_defaults_match_unchanged_official_model_and_inherited_dynamics():
    path=ROOT/'workshop/digital_twin/assets/so101_new_calib.urdf'
    assert hashlib.sha256(path.read_bytes()).hexdigest()=='3a65d2d35e68a8d2f0c2cc176d19b884506543c93ba72980145b80abe276022c'
    element=ET.parse(path).find("joint[@name='wrist_flex']/limit")
    assert POSITION_LIMITS_RAD[WRIST]==(float(element.get('lower')),float(element.get('upper')))==(-MODEL_LIMIT,MODEL_LIMIT)
    config=yaml.safe_load((ROOT/'ros2/config/rig01_moveit_limits.yaml').read_text())['joint_limits']
    assert (config[WRIST]['min_position'],config[WRIST]['max_position'])==(-MODEL_LIMIT,MODEL_LIMIT)
    assert {k:v for k,v in config[WRIST].items() if k not in ('min_position','max_position')}=={
        k:v for k,v in config['shoulder_pan'].items() if k not in ('min_position','max_position')}
    assert POSITION_LIMITS_RAD['shoulder_lift']==(-1.74533,1.74533)
    assert POSITION_LIMITS_RAD['elbow_flex']==(-1.69,1.69)
    for joint in ('shoulder_pan','wrist_roll'):
        assert POSITION_LIMITS_RAD[joint]==(-math.pi/2,math.pi/2)
        assert (config[joint]['min_position'],config[joint]['max_position'])==(-1.57079632679,1.57079632679)


@pytest.mark.parametrize('raw',[915,3076])
def test_model_admitted_integer_edges_pass_and_old_ninety_policy_rejects(raw):
    q=angle(raw)
    assert prepare([q,q]).samples[0].positions[3]==q
    old=dict(POSITION_LIMITS_RAD);old[WRIST]=(-math.pi/2,math.pi/2)
    with pytest.raises(ValueError,match='position limit exceeded: wrist_flex'):
        prepare([q,q],old)


@pytest.mark.parametrize('raw',[914,3077,820,3171])
def test_one_count_outside_model_and_calibration_extrema_stay_rejected(raw):
    with pytest.raises(ValueError,match='position limit exceeded: wrist_flex'):
        prepare([angle(raw),angle(raw)])


def test_continuous_bounds_and_owner_narrowing_remain_strict():
    for value in (-MODEL_LIMIT,MODEL_LIMIT):prepare([value,value])
    owner=dict(POSITION_LIMITS_RAD);owner[WRIST]=(-math.pi/2,math.pi/2)
    assert _position_limits(owner)[WRIST]==(-math.pi/2,math.pi/2)
    with pytest.raises(ValueError,match='position limit exceeded: wrist_flex'):
        prepare([math.radians(93.)]*2,owner)
    midpoint=(820+3171)/2
    admitted=(math.ceil(-MODEL_LIMIT*4095/(2*math.pi)+midpoint),
              math.floor(MODEL_LIMIT*4095/(2*math.pi)+midpoint))
    assert admitted==(915,3076)


def test_analytic_extrema_cannot_overshoot_expanded_wrist_model():
    with pytest.raises(ValueError,match='interpolated position limit exceeded: wrist_flex'):
        prepare([MODEL_LIMIT-.1,MODEL_LIMIT,MODEL_LIMIT,MODEL_LIMIT-.1])


def test_supervised_calibration_uses_same_model_envelope_with_unchanged_raw_zero():
    start={name:round((lo+hi)/2) for name,(lo,hi) in CAL_RANGES.items()}
    start.update(wrist_flex=3075,gripper=1954);target=start|{'wrist_flex':3076}
    measured={name:(start[name]-sum(CAL_RANGES[name])/2)*2*math.pi/4095 for name in ARM_JOINTS}
    points,prepared=prepare_supervised_calibration(start,target,measured,
        {name:bounds[0] for name,bounds in CAL_RANGES.items()},
        {name:bounds[1] for name,bounds in CAL_RANGES.items()})
    assert prepared.samples[-1].positions[3]==pytest.approx(angle(3076))
    assert target['gripper']==start['gripper']==1954
