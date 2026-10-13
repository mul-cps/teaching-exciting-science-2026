"""Continuous harmonic choreography with a C3 phase acceleration/deceleration ramp."""
import math
from workshop.choreography import JOINTS, DISPLACEMENT

SPEED = dict(zip(JOINTS, (30, 20, 35, 35, 35, 40)))
ACCELERATION = dict(zip(JOINTS, (80, 40, 80, 80, 150, 100)))
JERK = dict(zip(JOINTS, (400, 160, 400, 400, 700, 400)))


def phase_state(plan, elapsed):
    scale = plan['time_scale']
    t = elapsed / scale
    speed = plan['phase_speed']
    ramp = plan['phase_ramp']
    distance = 2 * math.pi * plan['cycles']
    duration = distance / speed + ramp
    if t <= 0:
        return 0., 0., 0., 0.
    if t >= duration:
        return distance, 0., 0., 0.
    def rise(local):
        u = local / ramp
        return (speed*ramp*(2.5*u**4-3*u**5+u**6),
                speed*(10*u**3-15*u**4+6*u**5),
                speed/ramp*(30*u**2-60*u**3+30*u**4),
                speed/ramp**2*(60*u-180*u**2+120*u**3))
    if t < ramp:
        q, v, a, j = rise(t)
    elif t > duration-ramp:
        q, v, a, j = rise(duration-t)
        q, a = distance-q, -a
    else:
        q, v, a, j = speed*ramp/2+speed*(t-ramp), speed, 0., 0.
    return q, v/scale, a/scale**2, j/scale**3


def rhythm_state(plan, elapsed):
    theta, rate, acceleration, jerk = phase_state(plan, elapsed)
    p, v, a, j = {}, {}, {}, {}
    for k in JOINTS:
        q = plan['start'][k]
        d1 = d2 = d3 = 0.
        for term in plan['harmonics'][k]:
            amplitude, n = term['amplitude'], term['order']
            s, c = math.sin(n*theta), math.cos(n*theta)
            if term['kind'] == 'sin':
                q += amplitude*s
                d1 += amplitude*n*c
                d2 -= amplitude*n*n*s
                d3 -= amplitude*n**3*c
            else:
                q += amplitude*(c-1)
                d1 -= amplitude*n*s
                d2 -= amplitude*n*n*c
                d3 += amplitude*n**3*s
        p[k], v[k] = q, d1*rate
        a[k] = d2*rate**2+d1*acceleration
        j[k] = d3*rate**3+3*d2*rate*acceleration+d1*jerk
    if elapsed <= 0 or elapsed >= plan['duration']:
        p = dict(plan['start'])
    return dict(t=elapsed, phase='continuous rhythm', position=p, velocity=v, acceleration=a, jerk=j)


def rhythm_peaks(plan):
    peaks = [dict.fromkeys(JOINTS, 0.) for _ in range(3)]
    for i in range(2049):
        state = rhythm_state(plan, plan['duration']*i/2048)
        for k in JOINTS:
            if abs(state['position'][k]-plan['start'][k]) > DISPLACEMENT[k]+1e-7:
                raise ValueError('Rhythm exceeds the choreography workspace')
            if not 0 <= state['position']['gripper'] <= 100:
                raise ValueError('Rhythm exceeds gripper range')
            for peak, field in zip(peaks, ('velocity', 'acceleration', 'jerk')):
                peak[k] = max(peak[k], abs(state[field][k]))
    return peaks


def make_rhythm_plan(start, hz):
    if set(start) != set(JOINTS) or not all(math.isfinite(v) for v in start.values()):
        raise ValueError('A finite six-joint start pose is required')
    if not 0 <= start['gripper'] <= 100 or not 100 <= hz <= 200:
        raise ValueError('Invalid gripper pose or control rate')
    amplitudes = [(20, 'sin', 1), (-11, 'cos', 1), (12.5, 'cos', 1),
                  (9, 'cos', 1), (25, 'sin', 1), (-min(20, (100-start['gripper'])/2), 'cos', 2)]
    harmonics = {k: [dict(amplitude=amp, kind=kind, order=n)] for k, (amp, kind, n) in zip(JOINTS, amplitudes)}
    duration = 4*math.pi/1.4+1.2
    plan = dict(schema=3, interpolation='harmonic-phase-ramp', choreography='rhythm',
                hz=hz, units='degrees; gripper percent', start=dict(start),
                harmonics=harmonics, cycles=2, phase_speed=1.4, phase_ramp=1.2,
                time_scale=1., duration=duration, speed_limits=dict(SPEED),
                acceleration_limits=dict(ACCELERATION), jerk_limits=dict(JERK),
                stages=[dict(name='continuous rhythm', begin=0., move=duration, hold=0., target=dict(start))])
    peaks = rhythm_peaks(plan)
    scale = max(1., max(peaks[0][k]/SPEED[k] for k in JOINTS),
                max(math.sqrt(peaks[1][k]/ACCELERATION[k]) for k in JOINTS),
                max((peaks[2][k]/JERK[k])**(1/3) for k in JOINTS))*1.02
    plan['time_scale'] = scale
    plan['duration'] *= scale
    plan['stages'][0]['move'] = plan['duration']
    plan['profile_peaks'] = dict(zip(('velocity', 'acceleration', 'jerk'), rhythm_peaks(plan)))
    return plan


def validate_rhythm(plan):
    if plan.get('choreography') != 'rhythm' or plan.get('cycles') != 2:
        raise ValueError('Invalid rhythm profile')
    for field in ('phase_speed', 'phase_ramp', 'time_scale', 'duration'):
        if not isinstance(plan.get(field), (int, float)) or not math.isfinite(plan[field]) or plan[field] <= 0:
            raise ValueError('Invalid rhythm timing')
    if plan['phase_ramp'] > 2*math.pi*plan['cycles']/plan['phase_speed']:
        raise ValueError('Invalid rhythm ramp')
    expected = (2*math.pi*plan['cycles']/plan['phase_speed']+plan['phase_ramp'])*plan['time_scale']
    if abs(expected-plan['duration']) > 1e-7:
        raise ValueError('Invalid rhythm duration')
    if set(plan['harmonics']) != set(JOINTS):
        raise ValueError('Invalid rhythm joints')
    for terms in plan['harmonics'].values():
        if len(terms) != 1 or any(t['kind'] not in ('sin', 'cos') or t['order'] not in (1, 2)
                                 or not math.isfinite(t['amplitude']) for t in terms):
            raise ValueError('Invalid rhythm harmonics')
    for field, limit in [('speed_limits', SPEED), ('acceleration_limits', ACCELERATION), ('jerk_limits', JERK)]:
        if plan.get(field) != limit:
            raise ValueError('Rhythm limits differ from the approved profile')
    for peak, limit in zip(rhythm_peaks(plan), (SPEED, ACCELERATION, JERK)):
        if any(peak[k] > limit[k]+1e-6 for k in JOINTS):
            raise ValueError('Rhythm exceeds velocity, acceleration or jerk limits')
