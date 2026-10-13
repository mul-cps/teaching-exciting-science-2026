"""Guard CPU tick behavior without hardware or timing-dependent pass criteria."""
import ast
import inspect
import textwrap

import pytest

from tools.benchmark_station_arm import benchmark
from workshop.choreography import smooth_state
from workshop.safe_home import home_at
from workshop.station.arm import LeRobotArm


ALLOWED_CALLS = {
    'RuntimeError', 'TimeoutError', 'ValueError', 'abs', 'any', 'dict', 'dict.fromkeys',
    'int', 'isinstance', 'math.isfinite', 'max', 'min', 'set', 'home_at', 'smooth_state',
    'self._check', 'self._raw_goal', 'self._raw_pose', 'self._validate_raw', 'self._pose',
    'self.abort_event.is_set', 'self.abort_event.wait', 'self.bus.sync_read',
    'self.bus.sync_write', 'time.monotonic', 'trace.append',
}


def unexpected_calls(source):
    tree = ast.parse(textwrap.dedent(source))
    return {ast.unparse(node.func) for node in ast.walk(tree) if isinstance(node, ast.Call)} - ALLOWED_CALLS


def test_tick_path_has_no_disk_logging_or_unreviewed_blocking_operations():
    # Explicitly cover the complete local call chain, including spline evaluation.
    # New calls require review; allowing bus IO is intentional and the benchmark
    # counts two telemetry reads and one position packet per six-joint tick.
    functions = [LeRobotArm._execute, LeRobotArm._check, LeRobotArm._raw_goal,
                 LeRobotArm._raw_pose, LeRobotArm._validate_raw, LeRobotArm._pose,
                 home_at, smooth_state]
    for function in functions:
        assert not unexpected_calls(inspect.getsource(function)), function.__qualname__


@pytest.mark.parametrize('operation', ["open('trace.json', 'w')", "atomic_json(path, trace)",
                                        "logger.info('tick')", 'os.fsync(fd)',
                                        'torch.cuda.synchronize()', 'stream.flush()'])
def test_guard_detects_forbidden_tick_operation(operation):
    assert unexpected_calls('def tick():\n    ' + operation)


@pytest.mark.parametrize('hz', [100, 125, 200])
def test_benchmark_exercises_actual_six_joint_follower_without_timing_threshold(hz):
    result = benchmark(repetitions=2, hz=hz)
    assert result['ticks'] > 2 * hz
    assert result['measured_ticks'] == result['ticks'] - 2
    assert result['joints'] == 6
    assert 0 < result['median_us'] <= result['p95_us'] <= result['max_us']
