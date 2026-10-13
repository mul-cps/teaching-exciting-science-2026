"""Arm STS servos with current goals and speed caps in one register packet."""
import math
from workshop.choreography import JOINTS


def arm_payload(position, acceleration, velocity, goal_time=0):
    values=(position,acceleration,velocity,goal_time)
    if any(not isinstance(x,int) for x in values) or not (0<=position<=4095 and 1<=acceleration<=254 and 1<=velocity<=32767 and 0<=goal_time<=65535):
        raise ValueError('Invalid or unlimited atomic arming parameters')
    return [1,acceleration,position&255,position>>8,goal_time&255,goal_time>>8,velocity&255,velocity>>8]


def startup_caps(bus, speeds, calibration):
    caps={}
    for k in JOINTS:
        phase=bus.read('Phase',k,num_retry=2)
        unit_deg_s=6*(.0146 if phase&4 else .732)
        angular_speed=speeds[k]
        if k=='gripper':
            angular_speed*= (calibration[k]['range_max']-calibration[k]['range_min'])*360/4095/100
        caps[k]=max(1,math.floor(angular_speed/unit_deg_s))
    return caps


def arm_current_pose(bus, acceleration, velocity):
    """This call intentionally enables torque; no separate goal seeding is safe."""
    from scservo_sdk import GroupSyncWrite
    if any(bus.read('Torque_Enable',k,num_retry=2) for k in JOINTS):
        raise RuntimeError('Atomic startup requires torque off')
    raw=bus.sync_read('Present_Position',normalize=False,num_retry=2)
    goal_time={k:bus.read('Goal_Time',k,num_retry=2) for k in JOINTS}
    group=GroupSyncWrite(bus.port_handler,bus.packet_handler,40,8)
    for k in JOINTS:
        payload=arm_payload(int(raw[k]),int(acceleration[k]),int(velocity[k]),int(goal_time[k]))
        if not group.addParam(bus.motors[k].id,payload):
            raise RuntimeError('Atomic startup payload rejected')
    if group.txPacket()!=0:
        raise RuntimeError('Atomic startup transmission failed')
    for register,expected in [('Torque_Enable',dict.fromkeys(JOINTS,1)),('Acceleration',acceleration),('Goal_Velocity',velocity)]:
        if any(bus.read(register,k,num_retry=2)!=expected[k] for k in JOINTS):
            raise RuntimeError(f'Atomic startup {register} readback failed')
    return raw
