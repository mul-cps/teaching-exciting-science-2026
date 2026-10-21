"""Pure OFFLINE velocity-lag candidate export; no runtime integration.

Caller must run the existing full dynamics, collision and effective-path hash
validation on the returned samples. No correction may be applied after that.
Coefficients default to zero; this module does not fit or enable coefficients.
"""
import math


def velocity_lag_candidate(samples, *, enabled=False, lag_seconds=None,
                           max_correction_rad=None):
    """Return new {time, positions} samples and a bounded correction receipt.

    Samples supply explicit per-joint velocities in rad/s, calculated by the
    trajectory generator. Bounds are caller-provided offline experiment bounds,
    not a physical safety guarantee. Endpoints remain exact; nonzero endpoint
    velocities are rejected when enabled to avoid silently changing a moving
    boundary condition. The output is an unvalidated candidate, never a command.
    """
    lag_seconds = dict(lag_seconds or {})
    max_correction_rad = dict(max_correction_rad or {})
    if type(enabled) is not bool or len(samples) < 2:
        raise ValueError('enabled must be boolean; at least two samples required')
    joints = set(samples[0]['positions'])
    if set(lag_seconds)-joints or set(max_correction_rad)-joints:
        raise ValueError('coefficient/bound names must belong to the trajectory')
    for values in (lag_seconds, max_correction_rad):
        if any(not math.isfinite(v) or v < 0 for v in values.values()):
            raise ValueError('coefficients/bounds must be finite and nonnegative')
    previous = -math.inf
    corrected = []
    peak = dict.fromkeys(joints, 0.)
    for index, sample in enumerate(samples):
        stamp = sample['time']
        positions = sample['positions']
        if not math.isfinite(stamp) or stamp < 0 or stamp <= previous:
            raise ValueError('sample times must be finite, nonnegative and strictly increasing')
        previous = stamp
        if set(positions) != joints or any(not math.isfinite(v) for v in positions.values()):
            raise ValueError('sample positions must have matching joints and finite values')
        output = dict(positions)
        if enabled:
            velocities = sample.get('velocities', {})
            if set(velocities) != joints or any(not math.isfinite(v) for v in velocities.values()):
                raise ValueError('enabled correction requires explicit finite per-joint velocities')
            if index in (0,len(samples)-1) and any(abs(v) > 1e-12 for v in velocities.values()):
                raise ValueError('enabled candidate requires stationary endpoints')
            for joint in joints:
                lag = lag_seconds.get(joint, 0.)
                bound = max_correction_rad.get(joint, 0.)
                correction = max(-bound,min(bound,lag*velocities[joint]))
                peak[joint] = max(peak[joint],abs(correction))
                output[joint] += correction
        corrected.append({'time': stamp, 'positions': output})
    return {'samples': corrected, 'enabled': enabled, 'lag_seconds': lag_seconds,
            'max_correction_rad': max_correction_rad, 'peak_correction_rad': peak,
            'validation_status': 'UNVALIDATED_OFFLINE_CANDIDATE',
            'required_next_step': 'full dynamics + collision checks + effective-path SHA on these corrected samples'}
