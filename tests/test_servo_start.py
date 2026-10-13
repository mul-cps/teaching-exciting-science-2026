import pytest
from workshop.servo_start import arm_payload


def test_arm_packet_sets_torque_acceleration_current_position_and_speed_together():
    assert arm_payload(949,1,68,1000)==[1,1,181,3,232,3,68,0]
    assert arm_payload(4095,50,254,0)==[1,50,255,15,0,0,254,0]


@pytest.mark.parametrize('q,a,v,t',[(4096,1,68,0),(-1,1,68,0),(949,0,68,0),(949,1,0,0),(949,1,68,-1)])
def test_arm_packet_rejects_invalid_or_unlimited_startup(q,a,v,t):
    with pytest.raises(ValueError):arm_payload(q,a,v,t)


def test_startup_caps_use_reported_phase_units_and_gripper_percent():
    from workshop.choreography import JOINTS
    from workshop.servo_start import startup_caps
    class Bus:
        def __init__(self,phase):self.phase=phase
        def read(self,*args,**kwargs):return self.phase
    speeds=dict.fromkeys(JOINTS,6)
    calibration={k:dict(range_min=0,range_max=4095) for k in JOINTS}
    calibration['gripper']=dict(range_min=1872,range_max=3366)
    fine=startup_caps(Bus(12),speeds,calibration)
    assert fine['shoulder_lift']==68
    assert fine['gripper']==89
    assert startup_caps(Bus(0),speeds,calibration)['shoulder_lift']==1
