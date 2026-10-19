"""Offline validation and smooth interpolation for ROS arm trajectories.

MoveIt point derivatives are retained in the audit record but are not used to
drive the arm: planners may emit acceleration jumps at adjacent knots. A
clamped cubic spline through positions gives a C2 path with zero endpoint
velocity. The original knot times are retained; unsafe timing is rejected.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import hashlib
import json
from types import MappingProxyType

ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
MAX_POINTS = 1000
START_TOLERANCE_RAD = math.radians(1.0)
# The pinned SO-ARM101 model admits lift ±1.74533, elbow ±1.69 and wrist
# flex ±1.65806 rad. Pan and wrist roll remain ±90 degrees. Owner execution
# additionally intersects these limits with its calibrated/configured/firmware
# raw admission; this model envelope never grants hardware authority.
POSITION_LIMITS_RAD = MappingProxyType({
    joint: ((-1.74533, 1.74533) if joint == "shoulder_lift" else
            (-1.69, 1.69) if joint == "elbow_flex" else
            (-1.65806, 1.65806) if joint == "wrist_flex" else (-math.pi / 2, math.pi / 2))
    for joint in ARM_JOINTS
})
ROS_MAX_VELOCITY_RAD_S = 0.125
ROS_MAX_ACCELERATION_RAD_S2 = 0.2
ROS_MAX_JERK_RAD_S3 = 2.0
MAX_COLLISION_CHECK_STATES = 12000


@dataclass(frozen=True)
class Sample:
    time: float
    positions: tuple[float, ...]


@dataclass(frozen=True)
class PreparedTrajectory:
    input_joint_names: tuple[str, ...]
    duration: float
    samples: tuple[Sample, ...]
    input_points: tuple[dict, ...]
    ignored_derivatives: bool


def _number(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{label} must be finite")
    return float(value)


def _solve(lower, diagonal, upper, rhs):
    """Solve a tridiagonal system in linear time."""
    n = len(rhs)
    c = list(upper)
    d = list(rhs)
    b = list(diagonal)
    for i in range(1, n):
        if abs(b[i - 1]) < 1e-14:
            raise ValueError("trajectory spline is singular")
        factor = lower[i] / b[i - 1]
        b[i] -= factor * c[i - 1]
        d[i] -= factor * d[i - 1]
    if abs(b[-1]) < 1e-14:
        raise ValueError("trajectory spline is singular")
    result = [0.0] * n
    result[-1] = d[-1] / b[-1]
    for i in range(n - 2, -1, -1):
        result[i] = (d[i] - c[i] * result[i + 1]) / b[i]
    return result


def _spline(times, values):
    """Return C2 quintic segments with zero velocity/acceleration at ends."""
    n = len(times)
    h = [times[i + 1] - times[i] for i in range(n - 1)]
    slope = [(values[i + 1] - values[i]) / h[i] for i in range(n - 1)]
    lower, diagonal, upper = [0.0] * n, [0.0] * n, [0.0] * n
    rhs = [0.0] * n
    diagonal[0], upper[0] = 2 * h[0], h[0]
    rhs[0] = 6 * slope[0]  # zero starting velocity
    for i in range(1, n - 1):
        lower[i] = h[i - 1]
        diagonal[i] = 2 * (h[i - 1] + h[i])
        upper[i] = h[i]
        rhs[i] = 6 * (slope[i] - slope[i - 1])
    lower[-1], diagonal[-1] = h[-1], 2 * h[-1]
    rhs[-1] = -6 * slope[-1]  # zero terminal velocity
    second = _solve(lower, diagonal, upper, rhs)
    cubic = []
    for i in range(n - 1):
        hi = h[i]
        cubic.append((values[i], slope[i] - hi * (2 * second[i] + second[i + 1]) / 6,
                      second[i] / 2, (second[i + 1] - second[i]) / (6 * hi)))
    velocity = [0.0] * n
    acceleration = list(second)
    acceleration[0] = acceleration[-1] = 0.0
    for i in range(1, n - 1):
        left = cubic[i - 1]
        right = cubic[i]
        left_v = left[1] + 2 * left[2] * h[i - 1] + 3 * left[3] * h[i - 1] ** 2
        velocity[i] = (left_v + right[1]) / 2
    quintic = []
    for i, hi in enumerate(h):
        p0, p1 = values[i], values[i + 1]
        v0, v1 = velocity[i], velocity[i + 1]
        a0, a1 = acceleration[i], acceleration[i + 1]
        c0, c1, c2 = p0, v0, a0 / 2
        dp = p1 - (c0 + c1 * hi + c2 * hi ** 2)
        dv = v1 - (c1 + 2 * c2 * hi)
        da = a1 - 2 * c2
        c3 = (10 * dp - 4 * dv * hi + 0.5 * da * hi ** 2) / hi ** 3
        c4 = (-15 * dp + 7 * dv * hi - da * hi ** 2) / hi ** 4
        c5 = (6 * dp - 3 * dv * hi + 0.5 * da * hi ** 2) / hi ** 5
        quintic.append((c0, c1, c2, c3, c4, c5))
    return quintic


def _evaluate(coeff, x):
    value = 0.0
    for coefficient in reversed(coeff):
        value = value * x + coefficient
    return value


def _poly_roots(coefficients, low, high):
    """Real roots in a closed interval for low-degree polynomials."""
    coeff = list(coefficients)
    while len(coeff) > 1 and abs(coeff[-1]) < 1e-14:
        coeff.pop()
    degree = len(coeff) - 1
    if degree <= 0:
        return []
    if degree == 1:
        root = -coeff[0] / coeff[1]
        return [root] if low <= root <= high else []
    critical = _poly_roots([i * coeff[i] for i in range(1, len(coeff))], low, high)
    boundaries = [low, *critical, high]
    roots = []
    for point in critical:
        if abs(_evaluate(coeff, point)) < 1e-10:
            roots.append(point)
    for left, right in zip(boundaries, boundaries[1:]):
        fl, fr = _evaluate(coeff, left), _evaluate(coeff, right)
        if fl == 0:
            roots.append(left)
        if fr == 0:
            roots.append(right)
        if fl * fr >= 0:
            continue
        for _ in range(60):
            mid = (left + right) / 2
            fm = _evaluate(coeff, mid)
            if abs(fm) < 1e-14:
                left = right = mid
                break
            if fl * fm <= 0:
                right, fr = mid, fm
            else:
                left, fl = mid, fm
        roots.append((left + right) / 2)
    return sorted(set(roots))


def _check_limits(coeffs, durations, joint, max_velocity, max_acceleration, max_jerk,
                  position_limits):
    position_min, position_max = position_limits
    for coeff, duration in zip(coeffs, durations):
        velocity_coeff = [i * coeff[i] for i in range(1, len(coeff))]
        acceleration_coeff = [i * velocity_coeff[i] for i in range(1, len(velocity_coeff))]
        jerk_coeff = [i * acceleration_coeff[i] for i in range(1, len(acceleration_coeff))]
        snap_coeff = [i * jerk_coeff[i] for i in range(1, len(jerk_coeff))]
        velocity_points = [0.0, duration, *_poly_roots(acceleration_coeff, 0.0, duration)]
        acceleration_points = [0.0, duration, *_poly_roots(jerk_coeff, 0.0, duration)]
        vmax = max(abs(_evaluate(velocity_coeff, x)) for x in velocity_points)
        amax = max(abs(_evaluate(acceleration_coeff, x)) for x in acceleration_points)
        jerk_points = [0.0, duration, *_poly_roots(snap_coeff, 0.0, duration)]
        jmax = max(abs(_evaluate(jerk_coeff, x)) for x in jerk_points)
        position_points = [0.0, duration, *_poly_roots(velocity_coeff, 0.0, duration)]
        positions = [_evaluate(coeff, x) for x in position_points]
        if min(positions) < position_min - 1e-9 or max(positions) > position_max + 1e-9:
            raise ValueError(f"interpolated position limit exceeded: {joint}")
        if vmax > max_velocity + 1e-9:
            raise ValueError(f"velocity limit exceeded: {joint} {vmax:.6g} > {max_velocity:.6g} rad/s")
        if amax > max_acceleration + 1e-9:
            raise ValueError(f"acceleration limit exceeded: {joint} {amax:.6g} > {max_acceleration:.6g} rad/s^2")
        if jmax > max_jerk + 1e-9:
            raise ValueError(f"jerk limit exceeded: {joint} {jmax:.6g} > {max_jerk:.6g} rad/s^3")


def _position_limits(admitted):
    if admitted is None:
        return POSITION_LIMITS_RAD
    if not isinstance(admitted, dict) or set(admitted) != set(ARM_JOINTS):
        raise ValueError("position_limits must contain exactly the five arm joints")
    effective = {}
    for joint in ARM_JOINTS:
        bounds = admitted[joint]
        if not isinstance(bounds, (list, tuple)) or len(bounds) != 2:
            raise ValueError("position_limits require a minimum/maximum pair: " + joint)
        low, high = (_number(value, "position_limits/" + joint) for value in bounds)
        if low >= high:
            raise ValueError("position_limits are inverted or empty: " + joint)
        model_low, model_high = POSITION_LIMITS_RAD[joint]
        low, high = max(low, model_low), min(high, model_high)
        if low >= high:
            raise ValueError("position_limits do not intersect model envelope: " + joint)
        effective[joint] = (low, high)
    return effective


def prepare_trajectory(joint_names, points, *, start_radians, max_velocity,
                       max_acceleration, max_jerk=ROS_MAX_JERK_RAD_S3, period=0.05,
                       position_limits=None):
    """Validate a ROS-like position trajectory and generate 20 Hz spline samples.

    Input derivatives are preserved but intentionally not commanded. `start`
    must agree with fresh measured state; the measured values become the exact
    first knot so the path does not begin with a discontinuity. Optional
    `position_limits` are already admitted joint bounds in radians; they can
    narrow, but never widen, the per-joint model envelope.
    """
    limits = _position_limits(position_limits)
    names = tuple(joint_names)
    if len(names) != len(ARM_JOINTS) or set(names) != set(ARM_JOINTS):
        raise ValueError("joint_names must contain exactly the five arm joints")
    max_velocity = _number(max_velocity, "max_velocity")
    max_acceleration = _number(max_acceleration, "max_acceleration")
    max_jerk = _number(max_jerk, "max_jerk")
    period = _number(period, "period")
    if min(max_velocity, max_acceleration, max_jerk, period) <= 0:
        raise ValueError("velocity, acceleration, jerk, and sample period must be positive")
    if not isinstance(points, (list, tuple)) or not 2 <= len(points) <= MAX_POINTS:
        raise ValueError(f"trajectory must contain 2..{MAX_POINTS} points")
    if not isinstance(start_radians, dict) or set(start_radians) != set(ARM_JOINTS):
        raise ValueError("fresh start state must contain exactly the five arm joints")
    start = {joint: _number(start_radians[joint], "start state") for joint in ARM_JOINTS}
    ordered = []
    times = []
    audit = []
    for index, point in enumerate(points):
        if not isinstance(point, dict) or set(point) - {"time", "positions", "velocities", "accelerations"}:
            raise ValueError("trajectory point has unsupported fields")
        t = _number(point.get("time"), f"point {index} time")
        if (index == 0 and abs(t) > 1e-9) or (times and t <= times[-1]):
            raise ValueError("trajectory times must start at zero and be strictly increasing")
        raw_positions = point.get("positions")
        if not isinstance(raw_positions, (list, tuple)) or len(raw_positions) != len(names):
            raise ValueError("each point must provide five positions")
        by_name = {name: _number(value, f"point {index} position")
                   for name, value in zip(names, raw_positions)}
        values = [by_name[j] for j in ARM_JOINTS]
        for joint, value in zip(ARM_JOINTS, values):
            low, high = limits[joint]
            if not low <= value <= high:
                raise ValueError(f"position limit exceeded: {joint}")
        derivatives = {}
        for field in ("velocities", "accelerations"):
            vals = point.get(field)
            if vals is not None:
                if not isinstance(vals, (list, tuple)) or len(vals) != len(names):
                    raise ValueError(f"each point {field} must have five values")
                derivatives[field] = {name: _number(v, f"point {index} {field}")
                                      for name, v in zip(names, vals)}
        if index == 0:
            for joint, value in zip(ARM_JOINTS, values):
                if abs(value - start[joint]) > START_TOLERANCE_RAD:
                    raise ValueError(f"start position mismatch: {joint}")
                values[ARM_JOINTS.index(joint)] = start[joint]
        ordered.append(values)
        times.append(t)
        audit.append({"time": t, "positions": by_name, **derivatives})
    duration = times[-1]
    if duration > 300:
        raise ValueError("trajectory duration exceeds 300 seconds")
    coeff_by_joint = []
    segment_durations = [times[i + 1] - times[i] for i in range(len(times) - 1)]
    for index, joint in enumerate(ARM_JOINTS):
        coeffs = _spline(times, [row[index] for row in ordered])
        _check_limits(coeffs, segment_durations, joint, max_velocity, max_acceleration,
                      max_jerk, limits[joint])
        coeff_by_joint.append(coeffs)
    count = int(math.ceil(duration / period - 1e-12))
    # Keep writes on the 20 Hz grid. If the trajectory ends between ticks,
    # issue its terminal hold on the next tick rather than burst-writing early.
    sample_times = [i * period for i in range(count + 1)]
    samples = []
    segment = 0
    for t in sample_times:
        trajectory_time = min(duration, t)
        while segment < len(segment_durations) - 1 and trajectory_time > times[segment + 1]:
            segment += 1
        x = min(segment_durations[segment], max(0.0, trajectory_time - times[segment]))
        samples.append(Sample(t, tuple(_evaluate(coeff_by_joint[j][segment], x)
                                       for j in range(len(ARM_JOINTS)))))
    return PreparedTrajectory(names, duration, tuple(samples), tuple(audit),
                              any("velocities" in p or "accelerations" in p for p in audit))


def path_digest(prepared, *, held_gripper_raw=None):
    # Suppress harmless last-bit differences between pose-degrees->radians in
    # the ROS process and raw-count->radians in the station process.
    payload = {"times": [round(sample.time, 12) for sample in prepared.samples],
               "positions": [[round(value, 12) for value in sample.positions]
                             for sample in prepared.samples]}
    if held_gripper_raw is not None:
        payload["held_gripper_raw"] = int(held_gripper_raw)
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    return hashlib.sha256(encoded).hexdigest()


def dense_path_samples(prepared, *, max_joint_step_rad=0.01,
                       max_states=MAX_COLLISION_CHECK_STATES):
    """Interpolate sampled commands so adjacent checks differ by at most 0.01 rad."""
    max_joint_step_rad = _number(max_joint_step_rad, "max_joint_step_rad")
    if max_joint_step_rad <= 0 or max_states < 2:
        raise ValueError("dense validation bounds must be positive")
    dense = [prepared.samples[0].positions]
    for left, right in zip(prepared.samples, prepared.samples[1:]):
        delta = max(abs(b - a) for a, b in zip(left.positions, right.positions))
        count = max(1, math.ceil(delta / max_joint_step_rad))
        if len(dense) + count > max_states:
            raise ValueError(f"collision validation would exceed {max_states} states")
        for index in range(1, count + 1):
            ratio = index / count
            dense.append(tuple(a + (b - a) * ratio
                               for a, b in zip(left.positions, right.positions)))
    return tuple(dense)


SUPERVISED_QUALIFICATION = "SUPERVISED_COMMISSIONING_NOT_COLLISION_CERTIFIED"
SUPERVISED_LIMITS = (.025, .05, .2)


def prepare_supervised_calibration(start_raw, target_raw, start_radians,
                                   admitted_min, admitted_max):
    """One fixed-jaw, slow commissioning spline; grants no collision authority."""
    joints = set(ARM_JOINTS) | {"gripper"}
    for name, values in (("start", start_raw), ("target", target_raw),
                         ("minimum", admitted_min), ("maximum", admitted_max)):
        if (not isinstance(values, dict) or set(values) != joints or
                any(type(v) is not int or not 0 <= v <= 4095 for v in values.values())):
            raise ValueError(name + " requires six hardware-range integer raw values")
    if start_raw['gripper'] != target_raw['gripper']:
        raise ValueError("supervised gripper must remain fixed")
    for joint in joints:
        if not (admitted_min[joint] < admitted_max[joint] and
                admitted_min[joint] <= start_raw[joint] <= admitted_max[joint] and
                admitted_min[joint] <= target_raw[joint] <= admitted_max[joint]):
            raise ValueError("supervised raw bounds violated: " + joint)
    if not isinstance(start_radians, dict) or set(start_radians) != set(ARM_JOINTS):
        raise ValueError("supervised start requires five measured angles")
    start = {j: _number(start_radians[j], j) for j in ARM_JOINTS}
    scale = 2*math.pi/4095
    end = {j: start[j]+(target_raw[j]-start_raw[j])*scale for j in ARM_JOINTS}
    delta = max(abs(end[j]-start[j]) for j in ARM_JOINTS)
    velocity, acceleration, jerk = SUPERVISED_LIMITS
    # Conservative zero-endpoint spline timing, then analytic validation below.
    duration = max(.5, 1.875*delta/velocity,
                   math.sqrt(6*delta/acceleration), (60*delta/jerk)**(1/3))*1.02
    duration = math.ceil(duration/.05)*.05
    points = [{'time': 0., 'positions': [start[j] for j in ARM_JOINTS]},
              {'time': duration, 'positions': [end[j] for j in ARM_JOINTS]}]
    bounds = {j: (start[j]+(admitted_min[j]-start_raw[j])*scale,
                  start[j]+(admitted_max[j]-start_raw[j])*scale) for j in ARM_JOINTS}
    prepared = prepare_trajectory(ARM_JOINTS, points, start_radians=start,
        max_velocity=velocity, max_acceleration=acceleration, max_jerk=jerk,
        position_limits=bounds)
    return points, prepared
