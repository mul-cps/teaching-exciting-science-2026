"""Small, bounded EV3 operator projection; never establishes motor references.

Physical direction mapping is deliberately absent until the station provides
qualified endpoints. Native 550 degrees is a guard boundary, not an endpoint.
"""
import math


def operator_status(cache, workflow, stop, *, fresh, endpoints=None):
    native = cache.get('status') or {}
    references = native.get('references') or {}
    connected = cache.get('connected') is True
    busy = native.get('busy') is True or workflow.get('state') not in ('idle', 'fault')
    fault = workflow.get('fault') or native.get('last_error') or cache.get('error')
    ready = connected and fresh and not busy and not fault and not stop.get('pending')
    power_available = native.get('power_mode') in ('awake', 'power_save')
    def capability(enabled, reason):
        return dict(enabled=bool(enabled), reason=reason)
    endpoints = endpoints or {}
    capabilities = {
        'connect': capability(not busy, 'Explicit connection only; no homing or reference reset'),
        'home': capability(ready and power_available, 'Wakes on explicit click; bounded D/S1 conveyor switch home'),
        'left_end': capability(False, 'Physical left endpoint has no commissioned direction mapping'),
        'right_end': capability(False, 'Physical right endpoint has no commissioned direction mapping; 550 is a guard boundary'),
        'eject': capability(ready and power_available and references == {'sorter': 'home', 'feeder': 'eject'},
                            ('One LOAD/READY 209 to EJECT 0 cycle' if references.get('feeder') == 'eject' else
                             'Feeder EJECT reference missing: manually position pusher, then Record feeder eject reference')),
        'confirm_feeder': capability(ready and power_available,
            'Manually place pusher fully EJECTED; explicit physical confirmation records current feeder zero'),
        'wake': capability(ready, 'Explicit native wake'),
        'sleep': capability(ready, 'Idle native power save'),
        'reset_fault': capability(connected and not busy, 'Inspect mechanism and STOP acknowledgements before resetting'),
    }
    for side in ('left', 'right'):
        target = endpoints.get(side)
        configured = type(target) in (int, float) and math.isfinite(target) and 0 < target < 550
        capabilities[side+'_end'] = capability(ready and power_available and references.get('sorter') == 'home' and configured,
            ('Configured conveyor '+side+' endpoint: '+str(target)+' degrees; conservative position guards apply')
            if configured else 'Conveyor '+side+' endpoint configuration required; no assumed encoder limit')
    angles = {}
    for name in ('sorter', 'feeder'):
        value = native.get('angles', {}).get(name)
        angles[name] = value if type(value) in (int, float) and math.isfinite(value) else None
    # Exclude unbounded workflow events, diagnostic logs and job timing arrays.
    return dict(schema=1, connected=connected, fresh=bool(fresh and connected), busy=busy,
                session=str(native.get('session') or '')[:96], angles=angles,
                references={name: (str(references[name])[:32] if references.get(name) is not None else None)
                            for name in ('sorter', 'feeder')},
                touch=native.get('touch') if type(native.get('touch')) is bool else None, color=str(native.get('color', 'unknown'))[:64],
                power_mode=str(native.get('power_mode', 'unknown'))[:32], fault=str(fault or '')[:512],
                stop_pending=bool(stop.get('pending')), capabilities=capabilities,
                workflow={key: str(workflow.get(key) or '')[:512] for key in ('state', 'job_id', 'fault')})
