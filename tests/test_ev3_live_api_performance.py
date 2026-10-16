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
    assert result['idle']['sleep_status_heartbeat_speed_reads'] == 0


def test_status_and_heartbeat_bursts_use_cached_driver_snapshot():
    from tools.benchmark_ev3_live_api import Harness
    harness = Harness()
    harness.api.tick()
    reads = harness.read_counts()
    for _ in range(200):
        assert harness.request('status')['ok']
        assert harness.request('heartbeat')['ok']
    after = harness.read_counts()
    assert after['sorter']['angle'] == reads['sorter']['angle']
    assert after['feeder']['angle'] == reads['feeder']['angle']
    assert after['sorter']['speed'] - reads['sorter']['speed'] == 200
    assert after['feeder']['speed'] - reads['feeder']['speed'] == 200
    assert after['touch'] == reads['touch'] and after['color'] == reads['color']


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


def test_display_changes_wait_until_shake_has_braked_and_settled():
    from tools.benchmark_ev3_live_api import Harness
    harness = Harness()
    harness.start_shake()
    harness.advance()
    harness.request('display', text='Check alignment')
    frames = harness.brick.screen.frames
    for _ in range(30): harness.advance()
    assert harness.api.job is not None
    assert harness.brick.screen.frames == frames, 'active motion must defer LCD I/O'
    for _ in range(150):
        harness.advance()
        if harness.api.job is None:
            break
        assert harness.brick.screen.frames == frames, 'settling must also defer LCD I/O'
    assert harness.api.last_job['state'] == 'complete'
    assert harness.api.job is None
    assert all(motor.target is None for motor in harness.motors.values()), 'both axes must be braked'
    assert any('Check alignment' in line[2] for line in harness.brick.screen.lines)
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
    harness.advance()
    after = harness.read_counts()
    assert after['touch'] == reads['touch'] + 1, 'the next 20 ms tick must sample touch'
    assert after['sorter']['angle'] == reads['sorter']['angle'] + 1
    assert after['feeder']['angle'] == reads['feeder']['angle'] + 1
    assert after['color'] == reads['color']


def test_cached_idle_angles_refresh_within_half_second():
    from tools.benchmark_ev3_live_api import Harness
    harness = Harness()
    harness.api.tick()
    assert harness.request('status')['result']['angles']['feeder'] == 0
    harness.motors['feeder'].value = 17
    for _ in range(25): harness.advance()
    assert harness.request('status')['result']['angles']['feeder'] == 17


def test_critical_status_reads_truthful_encoders_and_speeds_without_optional_work():
    from tools.benchmark_ev3_live_api import Harness
    harness = Harness()
    harness.api.connect()
    harness.clock.ms = 400  # Old cached angles represent the observed idle skew.
    harness.motors['sorter'].value = 13
    before = harness.read_counts()
    result = harness.request('critical_status')['result']
    after = harness.read_counts()
    assert result['angles']['sorter'] == 13
    for name in ('sorter', 'feeder'):
        assert after[name]['angle'] == before[name]['angle']+1
        assert after[name]['speed'] == before[name]['speed']+1
        assert result['motor_control'][name]['angle_sample_ms'] == 400
        assert result['motor_control'][name]['speed_sample']['sampled_ms'] == 400
    assert after['touch'] == before['touch'] and after['color'] == before['color']
    assert result['critical_sample'] == {'started_ms': 400, 'finished_ms': 400}
    assert all(motor.dispatches == 0 for motor in harness.motors.values())
    harness.request('power_save')
    before = harness.read_counts()
    asleep = harness.request('critical_status')['result']
    assert asleep['power_mode'] == 'power_save'
    assert harness.read_counts() == before


def test_critical_status_checks_watchdog_after_a_blocking_encoder_read():
    from tools.benchmark_ev3_live_api import Harness
    harness = Harness()
    harness.api.connect()
    harness.api.references['sorter'] = 'home'
    assert harness.request('motor_target', name='sorter', target=12, speed=50, acceleration=400, actuation=30)['ok']
    original = harness.motors['sorter'].angle
    def delayed():
        harness.clock.ms += 1100
        return original()
    harness.motors['sorter'].angle = delayed
    result = harness.request('critical_status')['result']
    assert not result['busy']
    assert result['last_error']['code'] == 'heartbeat_expired'
    assert harness.motors['sorter'].target is None


@pytest.mark.parametrize('operation', ['shake', 'eject'])
@pytest.mark.parametrize('other_drift', [0, 4])
def test_critical_read_never_dispatches_due_phase_and_checks_fresh_other_axis(operation, other_drift):
    from tools.benchmark_ev3_live_api import Harness
    harness = Harness()
    harness.api.connect()
    if operation == 'shake':
        harness.start_shake()
        harness.clock.ms = harness.api.job['_next_half']
        other = 'feeder'
    else:
        harness.api.references['feeder'] = 'eject'
        assert harness.request('eject', cycles=1)['ok']
        harness.motors['feeder'].value = 209
        harness.api.angles['feeder'] = 209
        harness.api.begin_settle()
        harness.clock.ms = 250
        other = 'sorter'
    harness.motors[other].value = other_drift
    dispatches = {name: motor.dispatches for name,motor in harness.motors.items()}
    result = harness.request('critical_status')['result']
    assert {name: motor.dispatches for name,motor in harness.motors.items()} == dispatches
    if other_drift:
        assert not result['busy']
        assert 'Other motor drift' in result['last_error']['message']
        assert all(motor.target is None for motor in harness.motors.values())
    else:
        assert result['busy']


def test_critical_blocking_read_crossing_shake_end_brakes_without_reversal():
    from tools.benchmark_ev3_live_api import Harness
    harness = Harness()
    harness.api.connect()
    harness.start_shake()
    harness.clock.ms = harness.api.job['_end']-5
    harness.api.last_heartbeat = harness.clock.ms
    original = harness.motors['sorter'].angle
    def crossing():
        harness.clock.ms += 10
        return original()
    harness.motors['sorter'].angle = crossing
    dispatches = harness.motors['sorter'].dispatches
    result = harness.request('critical_status')['result']
    assert not result['busy']
    assert 'Shake duration expired' in result['last_error']['message']
    assert harness.motors['sorter'].target is None
    assert harness.motors['sorter'].dispatches == dispatches
