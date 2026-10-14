"""Bounded CPU/loopback checks; fake drivers never open EV3 hardware."""
import ast
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
NATIVE = ROOT / 'tools' / 'ev3_native' / 'live_api.py'


def test_native_motor_dispatch_is_explicitly_nonblocking():
    # A forgotten wait=False stalls the peer's heartbeat and stop processing.
    tree = ast.parse(NATIVE.read_text(), feature_version=(3, 4))
    commands = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Attribute):
            continue
        if node.func.attr in ('run_target', 'run_time'):
            commands.append(node)
            wait = next((keyword.value for keyword in node.keywords if keyword.arg == 'wait'), None)
            assert isinstance(wait, ast.Constant) and wait.value is False, (
                'native %s at line %s must explicitly use wait=False' % (node.func.attr, node.lineno))
        assert node.func.attr not in ('run_until_stalled', 'run_angle'), (
            'blocking motor primitive at line %s' % node.lineno)
    assert commands, 'the guard must cover a real native motor dispatch'


def test_benchmark_reports_real_native_and_loopback_costs():
    from tools.benchmark_ev3_live_api import benchmark
    result = benchmark(iterations=12, ticks=25)
    assert result['scope'].startswith('CPU fake drivers and TCP loopback')
    for operation in ('status_json', 'heartbeat_json', 'status_loopback', 'heartbeat_loopback', 'tick', 'shake_tick', 'display_json', 'screen_tick'):
        timing = result['timings'][operation]
        assert timing['samples'] >= 12
        assert 0 <= timing['median_us'] <= timing['p95_us'] <= timing['max_us']
    assert result['loopback_requests'] >= 24
    assert result['shake']['state'] == 'complete'
    assert result['shake']['motor_dispatches'] > 1
    assert result['display']['screen_frames'] == result['display']['updates']


def test_status_and_heartbeat_bursts_use_cached_driver_snapshot():
    from tools.benchmark_ev3_live_api import Harness
    harness = Harness()
    harness.api.tick()
    reads = harness.read_counts()
    for _ in range(200):
        assert harness.request('status')['ok']
        assert harness.request('heartbeat')['ok']
    assert harness.read_counts() == reads, 'request frequency must not become driver-read frequency'


def test_shake_tick_keeps_lcd_out_of_motor_dispatch_hotpath():
    from tools.benchmark_ev3_live_api import Harness
    harness = Harness()
    harness.start_shake()
    frames = harness.brick.screen.frames
    for _ in range(90):
        harness.advance()
        # Repeated polling must not refresh the LCD or bypass the sampling cache.
        for _ in range(4):
            assert harness.request('status')['ok']
            assert harness.request('heartbeat')['ok']
    assert harness.api.status()['busy']
    assert harness.motors['sorter'].dispatches > 20
    assert harness.brick.screen.frames - frames <= 1, '50 Hz motion must not trigger LCD redraws'
    assert harness.touch.reads <= 95, 'touch should be sampled at most once per tick'
    assert all(motor.reads <= 100 for motor in harness.motors.values()), (
        'each encoder should be sampled once per tick, independently of polling frequency')


def test_display_deduplicates_unchanged_text():
    from tools.benchmark_ev3_live_api import Harness
    harness = Harness()
    harness.request('display', text='Mechanics paused')
    for _ in range(100):
        harness.advance()
        harness.request('display', text='Mechanics paused')
    assert harness.brick.screen.frames <= 2, 'repeating unchanged text must not churn the framebuffer'


@pytest.mark.parametrize('option,value', [('iterations', 0), ('ticks', 0), ('iterations', True)])
def test_benchmark_rejects_unbounded_or_empty_options(option, value):
    from tools.benchmark_ev3_live_api import benchmark
    with pytest.raises(ValueError):
        benchmark(**{option: value})


def test_display_changes_eventually_reach_lcd_during_shake():
    from tools.benchmark_ev3_live_api import Harness
    harness = Harness()
    harness.start_shake()
    harness.advance()
    harness.request('display', text='Check alignment')
    for _ in range(30): harness.advance()
    assert any('Check alignment' in line[2] for line in harness.brick.screen.lines), (
        'LCD shake optimization must still deliver an explicit display update')
    assert harness.brick.screen.frames <= 3


def test_changing_display_text_respects_lcd_cooldown():
    from tools.benchmark_ev3_live_api import Harness
    harness = Harness()
    for index in range(100):
        harness.request('display', text='Sample ' + str(index))
        harness.advance()
    assert harness.brick.screen.frames <= 5, 'LCD refresh must stay at or below 2 Hz'
    for _ in range(25): harness.advance()
    assert any('Sample 99' in line[2] for line in harness.brick.screen.lines)


def test_socket_readiness_bursts_do_not_resample_active_drivers():
    from tools.benchmark_ev3_live_api import Harness
    harness = Harness()
    harness.start_shake()
    harness.advance()
    reads = harness.read_counts()
    # The daemon can wake repeatedly before the next 20 ms sample deadline.
    for _ in range(100): harness.api.tick()
    assert harness.read_counts() == reads, 'ready sockets must not turn driver sampling into a busy loop'


def test_cached_idle_angles_refresh_within_half_second():
    from tools.benchmark_ev3_live_api import Harness
    harness = Harness()
    harness.api.tick()
    assert harness.request('status')['result']['angles']['feeder'] == 0
    harness.motors['feeder'].value = 17
    for _ in range(25): harness.advance()
    assert harness.request('status')['result']['angles']['feeder'] == 17
