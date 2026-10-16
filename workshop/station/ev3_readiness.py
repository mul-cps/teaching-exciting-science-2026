"""Read-only native station admission; cached sleeping telemetry is insufficient."""
import time
from workshop.station.sorter_position import finite


def stationary_station_snapshot(status, *, session):
    if (not isinstance(status, dict) or status.get('session') != session
            or status.get('connected') is not True or status.get('busy') is not False
            or status.get('power_mode') != 'awake' or 'last_error' not in status or status['last_error'] is not None
            or status.get('references') != {'sorter': 'home', 'feeder': 'eject'}
            or (status.get('last_job') or {}).get('cleanup_fault')):
        raise ValueError('EV3 requires same-session connected awake idle fault-free home/eject references')
    angles, stamps = {}, {}
    for name in ('sorter', 'feeder'):
        angle = status.get('angles', {}).get(name)
        control = status.get('motor_control', {}).get(name, {})
        stamp, sampled = control.get('angle_sample_ms'), status.get('sampled_ms')
        speed = control.get('speed_sample', {})
        position_valid = finite(angle) and (0 <= angle <= 550 if name == 'sorter' else abs(angle) <= 3.)
        if (not position_valid
                or not finite(stamp) or stamp < 0 or not finite(sampled) or not 0 <= sampled-stamp <= 100
                or not finite(control.get('angle_read_ms')) or not 0 <= control['angle_read_ms'] <= 250
                or not finite(speed.get('speed_dps')) or abs(speed['speed_dps']) > 3
                or not finite(speed.get('sampled_ms')) or not 0 <= speed['sampled_ms']-stamp <= 750
                or not finite(speed.get('duration_ms')) or not 0 <= speed['duration_ms'] <= 250):
            raise ValueError('fresh finite stopped native telemetry required for '+name+
                             (' at the verified eject endpoint within three degrees' if name == 'feeder' else ''))
        angles[name], stamps[name] = float(angle), float(stamp)
    return angles, stamps


def require_fresh_stationary(client, *, session, deadline, cancel, spin, clock=time.monotonic):
    deadline = min(deadline, clock()+2.)
    def sample():
        if cancel.is_set():
            raise InterruptedError('EV3 readiness cancelled')
        if clock() >= deadline:
            raise TimeoutError('EV3 encoder telemetry did not advance before readiness deadline')
        before = clock()
        status = client.call('critical_status')
        if clock()-before > .75:
            raise TimeoutError('EV3 readiness telemetry receipt too slow')
        angles, stamps = stationary_station_snapshot(status, session=session)
        if cancel.is_set():
            raise InterruptedError('EV3 readiness cancelled')
        if clock() >= deadline:
            raise TimeoutError('EV3 readiness deadline exceeded')
        return status, angles, stamps
    _, first_angles, first_stamps = sample()
    while True:
        spin(deadline, cancel)
        status, angles, stamps = sample()
        for name in first_stamps:
            if (not 0 <= stamps[name]-first_stamps[name] <= 1500
                    or abs(angles[name]-first_angles[name]) > 1.):
                raise ValueError('EV3 '+name+' telemetry changed unexpectedly during readiness')
        if all(stamps[name] > first_stamps[name] for name in first_stamps):
            return status
