"""Conservative commissioning admission for the EV3 sorter axis only."""
import math

SPEED_DPS = 50.
ACCELERATION_DPS2 = 400.
ACTUATION_PERCENT = 30.
PHASE_DEADLINE_S = 6.
SETTLE_RESERVE_S = 1.


def finite(value):
    return type(value) in (float, int) and math.isfinite(value)


def validate_target(target):
    if not finite(target) or not 0 <= target <= 550:
        raise ValueError('sorter absolute target must be finite within 0..550 degrees')


def stationary_snapshot(status, *, session):
    if (not isinstance(status, dict) or status.get('session') != session
            or status.get('connected') is not True or status.get('busy') is not False
            or status.get('power_mode') != 'awake' or status.get('last_error')
            or status.get('references', {}).get('sorter') != 'home'):
        raise ValueError('same-session awake idle sorter with exact home reference and no fault required')
    angle = status.get('angles', {}).get('sorter')
    control = status.get('motor_control', {}).get('sorter', {})
    stamp, sampled = control.get('angle_sample_ms'), status.get('sampled_ms')
    speed = control.get('speed_sample', {})
    if (not finite(angle) or not 0 <= angle <= 550
            or not finite(stamp) or stamp < 0 or not finite(sampled)
            or not 0 <= sampled-stamp <= 100
            or not finite(control.get('angle_read_ms')) or not 0 <= control['angle_read_ms'] <= 250
            or not finite(speed.get('speed_dps')) or abs(speed['speed_dps']) > 3
            or not finite(speed.get('sampled_ms')) or not 0 <= speed['sampled_ms']-stamp <= 750
            or not finite(speed.get('duration_ms')) or not 0 <= speed['duration_ms'] <= 250):
        raise ValueError('fresh finite sorter encoder and stopped native-speed telemetry required')
    return float(angle), float(stamp)


def position_arguments(status, *, session, target):
    validate_target(target)
    angle, stamp = stationary_snapshot(status, session=session)
    distance = abs(target-angle)
    ramp_distance = SPEED_DPS*SPEED_DPS/ACCELERATION_DPS2
    duration = (2*math.sqrt(distance/ACCELERATION_DPS2) if distance <= ramp_distance
                else distance/SPEED_DPS+SPEED_DPS/ACCELERATION_DPS2)
    if duration+SETTLE_RESERVE_S > PHASE_DEADLINE_S:
        raise ValueError('requested distance exceeds conservative six-second phase budget; use smaller absolute steps')
    return dict(name='sorter', target=float(target), speed=SPEED_DPS,
                acceleration=ACCELERATION_DPS2, actuation=ACTUATION_PERCENT)


def motion_progress(status, *, session, previous_stamp, progress_time, now):
    control = status.get('motor_control', {}).get('sorter', {})
    sampled = control.get('angle_sample_ms')
    measured, remote_now = status.get('angles', {}).get('sorter'), status.get('sampled_ms')
    if (status.get('session') != session or status.get('connected') is not True
            or status.get('power_mode') != 'awake' or status.get('references', {}).get('sorter') != 'home'
            or not finite(measured) or not 0 <= measured <= 550
            or not finite(sampled) or sampled < previous_stamp
            or not finite(remote_now) or not 0 <= remote_now-sampled <= 100):
        raise ValueError('sorter active telemetry/session/reference invalid')
    if sampled > previous_stamp:
        return sampled, now
    if now-progress_time > .75:
        raise TimeoutError('sorter encoder telemetry stopped advancing')
    return previous_stamp, progress_time


def terminal_position(status, *, session, job_id, target, after_stamp):
    last = status.get('last_job') or {}
    if (status.get('busy') is not False or last.get('id') != job_id
            or last.get('operation') != 'motor_target' or last.get('motor') != 'sorter'
            or last.get('state') != 'complete' or last.get('cleanup_fault') or status.get('last_error')):
        raise ValueError('sorter position native job did not complete cleanly')
    angle, stamp = stationary_snapshot(status, session=session)
    if stamp <= after_stamp or abs(angle-target) > 3.:
        raise ValueError('sorter terminal encoder does not verify target within three degrees')
    return angle


def control_diagnostics(status):
    control = status.get('motor_control', {}).get('sorter', {})
    tolerances = control.get('target_tolerances')
    position_tolerance = (tolerances[1] if isinstance(tolerances, (list, tuple)) and len(tolerances) == 2 else None)
    return dict(native_control=control, native_position_tolerance_deg=position_tolerance,
                position_tolerance_exceeds_terminal_gate=(not finite(position_tolerance) or position_tolerance > 3.))
