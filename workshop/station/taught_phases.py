"""Pure admission for fixed-jaw taught phases; resident bridge owns motion."""
import math
from workshop.blockly_recording import JOINTS, replay_points


def admit_phase(recording,identity,measured,speed):
    points=replay_points(recording,identity,speed,physical=True)
    if any(abs(measured[j]-value)>math.radians(2.) for j,value in zip(JOINTS,points[0]['positions'])):
        raise RuntimeError('taught phase start differs by more than two degrees')
    if abs(measured['gripper']-recording['jaw_radians'])>.015:
        raise RuntimeError('taught phase fixed jaw differs from live jaw')
    return points


def require_phase_cohort(identity,expected,measured,jaw=None):
    if identity!=expected: raise RuntimeError('owner cohort changed during taught transaction')
    if jaw is not None and abs(measured['gripper']-jaw)>.015:
        raise RuntimeError('jaw changed during fixed-jaw taught phase')
