# Preserve requested duration during collinear collapse

The offline timing comparison found that collinear collapse could accelerate an
intentionally slow MoveIt request. The helper now chooses the larger of the
original requested total duration and its existing safe rest-to-rest minimum,
then retains the existing nanosecond ceiling and complete spline validation.
The caller still selects this candidate only when it shortens the independently
validated, uniformly retimed trajectory.

Bounded pure fixtures: a 0.1 rad move requested in 0.4 s retains the exact previous
1.750015573 s duration and sample digest; the same move requested in 8 s now
retains 8 s (previously 1.750015573 s). Both retain the 50 Hz grid, endpoints and
current analytic velocity/acceleration/jerk admission. An ordinary safe 8 s
trajectory prepared without collapse retains its previous duration and digest.

Verified with `python -m pytest tests/test_ros_trajectory.py -q -k
collinear_candidate_preserves_requested_duration_floor` (2 passed). The slow case
failed against the original helper before the fix. The original ordinary safe
trajectory was compared directly against `df302ad` using pure Python.

This total-duration floor does not enforce per-phase velocity ceilings or make
motion-mode presets physically qualified. Full exact-sample collision and owner
preflight remain required. No runtime, hardware, limits or protocol changes were
made; the frozen `df302ad` timing-comparison artifacts remain unchanged.
