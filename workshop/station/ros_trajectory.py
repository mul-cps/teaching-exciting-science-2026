"""Offline validation and smooth interpolation for ROS arm trajectories.

MoveIt point derivatives are retained in the audit record but are not used to
drive the arm: planners may emit acceleration jumps at adjacent knots. A
clamped cubic spline through positions gives a C2 path with zero endpoint
velocity. The original knot times are retained; unsafe timing is rejected.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
MAX_POINTS = 1000
START_TOLERANCE_RAD = math.radians(1.0)
POSITION_LIMIT_RAD = math.pi / 2


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


def _check_limits(coeffs, durations, joint, max_velocity, max_acceleration):
    for coeff, duration in zip(coeffs, durations):
        velocity_coeff = [i * coeff[i] for i in range(1, len(coeff))]
        acceleration_coeff = [i * velocity_coeff[i] for i in range(1, len(velocity_coeff))]
        jerk_coeff = [i * acceleration_coeff[i] for i in range(1, len(acceleration_coeff))]
        velocity_points = [0.0, duration, *_poly_roots(acceleration_coeff, 0.0, duration)]
        acceleration_points = [0.0, duration, *_poly_roots(jerk_coeff, 0.0, duration)]
        vmax = max(abs(_evaluate(velocity_coeff, x)) for x in velocity_points)
        amax = max(abs(_evaluate(acceleration_coeff, x)) for x in acceleration_points)
        position_points = [0.0, duration, *_poly_roots(velocity_coeff, 0.0, duration)]
        positions = [_evaluate(coeff, x) for x in position_points]
        if min(positions) < -POSITION_LIMIT_RAD - 1e-9 or max(positions) > POSITION_LIMIT_RAD + 1e-9:
            raise ValueError(f"interpolated position limit exceeded: {joint}")
        if vmax > max_velocity + 1e-9:
            raise ValueError(f"velocity limit exceeded: {joint} {vmax:.6g} > {max_velocity:.6g} rad/s")
        if amax > max_acceleration + 1e-9:
            raise ValueError(f"acceleration limit exceeded: {joint} {amax:.6g} > {max_acceleration:.6g} rad/s^2")


def prepare_trajectory(joint_names, points, *, start_radians, max_velocity,
                       max_acceleration, period=0.05):
    """Validate a ROS-like position trajectory and generate 20 Hz spline samples.

    Input derivatives are preserved but intentionally not commanded. `start`
    must agree with fresh measured state; the measured values become the exact
    first knot so the path does not begin with a discontinuity.
    """
    names = tuple(joint_names)
    if len(names) != len(ARM_JOINTS) or set(names) != set(ARM_JOINTS):
        raise ValueError("joint_names must contain exactly the five arm joints")
    max_velocity = _number(max_velocity, "max_velocity")
    max_acceleration = _number(max_acceleration, "max_acceleration")
    period = _number(period, "period")
    if min(max_velocity, max_acceleration, period) <= 0:
        raise ValueError("velocity, acceleration, and sample period must be positive")
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
            if abs(value) > POSITION_LIMIT_RAD:
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
        _check_limits(coeffs, segment_durations, joint, max_velocity, max_acceleration)
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
