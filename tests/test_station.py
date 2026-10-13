import json
from importlib.resources import files

import pytest

from workshop.station.config import load_config, validate_config
from workshop.station.coordinator import run_cycle
from workshop.station.simulation import SimArm, SimEV3


def example():
    return json.loads(files('workshop.station').joinpath('resources/station.example.json').read_text())


def station(workflow='arm_loads_sorter'):
    config = example()
    config['workflow'] = workflow
    events = []
    ev3 = SimEV3(events)
    arm = SimArm(events, ev3)
    return config, ev3, arm, events


@pytest.mark.parametrize('workflow,expected', [
    ('arm_loads_sorter', ['ev3.stop', 'ev3.wait_stopped', 'arm.load_piece', 'ev3.read_color', 'arm.clear', 'arm.wait_clear', 'ev3.position:blue', 'ev3.feed', 'ev3.stop', 'ev3.wait_stopped']),
    ('arm_unloads_sorter', ['arm.clear', 'arm.wait_clear', 'ev3.read_color', 'ev3.position:blue', 'ev3.feed', 'ev3.stop', 'ev3.wait_stopped', 'arm.pick_and_place:blue', 'arm.clear', 'arm.wait_clear']),
])
def test_workflow_sequence(workflow, expected):
    config, ev3, arm, events = station(workflow)
    run_cycle(config, ev3, arm, [])
    assert events == expected


def test_feed_requires_confirmed_clearance():
    config, ev3, arm, events = station()
    arm.wait_clear_error = TimeoutError('clearance')
    with pytest.raises(TimeoutError):
        run_cycle(config, ev3, arm, [])
    assert 'ev3.feed' not in events
    assert ev3.stopped and arm.stopped


def test_unknown_color_stops_both():
    config, ev3, arm, events = station('arm_unloads_sorter')
    ev3.color = 'unknown'
    with pytest.raises(ValueError):
        run_cycle(config, ev3, arm, [])
    assert ev3.stopped and arm.stopped
    assert 'ev3.feed' not in events


@pytest.mark.parametrize('subsystem', ['ev3', 'arm'])
def test_cleanup_independent(subsystem):
    config, ev3, arm, events = station('arm_unloads_sorter')
    ev3.color = 'unknown'
    setattr(ev3 if subsystem == 'ev3' else arm, 'stop_error', RuntimeError('stop failed'))
    log = []
    with pytest.raises(ValueError, match='color'):
        run_cycle(config, ev3, arm, log)
    assert 'ev3.stop' in events and 'arm.stop' in events
    assert any(e['event'] == 'cleanup_failure' for e in log)


def test_failed_stoppage_prevents_arm_pick():
    config, ev3, arm, events = station('arm_unloads_sorter')
    ev3.wait_stopped_error = TimeoutError('stoppage')
    with pytest.raises(TimeoutError):
        run_cycle(config, ev3, arm, [])
    assert not any(e.startswith('arm.pick') for e in events)
    assert events.count('ev3.feed') == 1


@pytest.mark.parametrize('mutate', [
    lambda c: c.update(workflow='other'),
    lambda c: c['ev3']['ports'].update(sorter='A'),
    lambda c: c['ev3']['destinations'].pop('red'),
    lambda c: c['ev3']['homing'].update(speed_dps=float('nan')),
    lambda c: c['timeouts'].update(operation_s=True),
    lambda c: c.update(cycle_count=True),
    lambda c: c['ev3']['directions'].update(feeder=0),
    lambda c: c['arm']['limits']['gripper'].update(min=101),
])
def test_reject_invalid_config(mutate):
    config = example()
    mutate(config)
    with pytest.raises(ValueError):
        validate_config(config)


def test_template_simulates_but_cannot_actuate(tmp_path):
    path = tmp_path / 'station.json'
    path.write_text(json.dumps(example()))
    assert load_config(path)['commissioned'] is False
    with pytest.raises(ValueError, match='commission'):
        load_config(path, physical=True)


def test_physical_missing_poses_rejected():
    config = example()
    config.update(enabled=True, commissioned=True)
    with pytest.raises(ValueError):
        validate_config(config, physical=True)


def test_simulation_enforces_interlocks():
    _, ev3, arm, _ = station()
    arm.clear_confirmed = False
    with pytest.raises(RuntimeError):
        ev3.feed()
    ev3.stopped = False
    with pytest.raises(RuntimeError):
        arm.load_piece()


def commissioned(tmp_path):
    config = example()
    config.update(enabled=True, commissioned=True)
    config['ev3']['usb']['serial'] = 'EV3-test'
    config['arm']['usb']['serial'] = 'arm-test'
    config['ev3']['feeder']['home_travel_deg'] = 720
    calibration = tmp_path / 'calibration.json'
    calibration.write_text('{}')
    import hashlib
    config['arm']['calibration'] = {'path': str(calibration), 'sha256': hashlib.sha256(calibration.read_bytes()).hexdigest()}
    home = tmp_path / 'home.json'
    home.write_text('{}')
    config['arm']['home_file'] = str(home)
    config['arm']['poses'] = {p: {j: 0 for j in config['arm']['limits']} for p in ('home','clearance','loading','pickup','source','blue','green','yellow','red')}
    config['arm']['trajectories'] = {'home':['home'], 'clearance':['clearance'], 'load_piece':['source','loading'], 'pick_and_place':{color:['pickup',color,'clearance'] for color in ('blue','green','yellow','red')}}
    return config


def test_commissioned_schema_and_hash(tmp_path):
    config = commissioned(tmp_path)
    assert validate_config(config, physical=True) is config
    config['arm']['calibration']['sha256'] = '0' * 64
    with pytest.raises(ValueError, match='hash'):
        validate_config(config, physical=True)


@pytest.mark.parametrize('mutate', [
    lambda c: c['arm']['poses'].pop('pickup'),
    lambda c: c['arm']['poses']['home'].update(gripper=101),
    lambda c: c['arm']['poses']['home'].update(shoulder_pan=float('inf')),
    lambda c: c['arm']['trajectories']['pick_and_place'].pop('blue'),
    lambda c: c['arm']['trajectories'].update(clearance=['missing']),
    lambda c: c['arm']['limits']['elbow_flex'].update(speed=1000),
    lambda c: c['ev3'].update(firmware='ev3dev'),
    lambda c: c['ev3']['usb'].update(vendor_id=True),
    lambda c: c['arm']['usb'].update(serial=''),
    lambda c: c['timeouts'].update(stop_s=0),
    lambda c: c['ev3']['feeder'].update(home_travel_deg=0),
    lambda c: c['ev3'].update(firmware='pybricks'),
])
def test_reject_bad_physical_config(tmp_path, mutate):
    config = commissioned(tmp_path)
    mutate(config)
    with pytest.raises(ValueError):
        validate_config(config, physical=True)


def test_false_acknowledgement_blocks_feed():
    config, ev3, arm, events = station()
    arm.wait_clear = lambda: False
    with pytest.raises(RuntimeError, match='acknowledgement'):
        run_cycle(config, ev3, arm, [])
    assert 'ev3.feed' not in events


def test_cancellation_stops_both():
    config, ev3, arm, events = station()
    def cancel():
        raise KeyboardInterrupt()
    arm.load_piece = cancel
    with pytest.raises(KeyboardInterrupt):
        run_cycle(config, ev3, arm, [])
    assert ev3.stopped and arm.stopped
    assert 'ev3.feed' not in events


def test_feed_failure_never_retried():
    config, ev3, arm, events = station()
    def uncertain_feed():
        events.append('ev3.feed')
        raise ConnectionError('reply lost after eject')
    ev3.feed = uncertain_feed
    with pytest.raises(ConnectionError):
        run_cycle(config, ev3, arm, [])
    assert events.count('ev3.feed') == 1


def test_late_acknowledgement_blocks_next_operation(monkeypatch):
    config, ev3, arm, events = station()
    clock = iter([0, 100])
    monkeypatch.setattr('workshop.station.coordinator.time.monotonic', lambda: next(clock))
    with pytest.raises(TimeoutError):
        run_cycle(config, ev3, arm, [])
    assert 'arm.load_piece' not in events


def test_cycles_are_serialized():
    import threading
    config, ev3, arm, events = station()
    entered, release = threading.Event(), threading.Event()
    original = arm.load_piece
    def hold_piece():
        entered.set()
        assert release.wait(2)
        original()
    arm.load_piece = hold_piece
    failures = []
    def cycle():
        try:
            run_cycle(config, ev3, arm, [])
        except BaseException as error:
            failures.append(error)
    first = threading.Thread(target=cycle)
    second = threading.Thread(target=cycle)
    first.start()
    assert entered.wait(2)
    second.start()
    # A second call must not stop or position the EV3 during the first load.
    assert events == ['ev3.stop', 'ev3.wait_stopped']
    release.set()
    first.join(2)
    second.join(2)
    assert not first.is_alive() and not second.is_alive()
    assert not failures
    assert events[:10] == events[10:]


@pytest.mark.parametrize('operation,path', [('home',['loading']), ('clearance',['pickup']), ('load_piece',['loading','source'])])
def test_trajectory_semantics_require_safe_destination(tmp_path, operation, path):
    config = commissioned(tmp_path)
    config['arm']['trajectories'][operation] = path
    with pytest.raises(ValueError):
        validate_config(config, physical=True)


def test_cleanup_logging_failure_preserves_original_fault_and_second_stop():
    config, ev3, arm, events = station('arm_unloads_sorter')
    ev3.color = 'unknown'
    ev3.stop_error = RuntimeError('first stop failed')
    attempted_logs = []
    def broken_cleanup_logger(record):
        attempted_logs.append(record['event'])
        if record['event'] in ('cleanup_failure', 'fault'):
            raise OSError('log unavailable')
    with pytest.raises(ValueError, match='unsupported or unstable color') as fault:
        run_cycle(config, ev3, arm, broken_cleanup_logger)
    assert type(fault.value) is ValueError
    assert 'ev3.stop' in events and 'arm.stop' in events
    assert arm.stopped
    assert attempted_logs[-2:] == ['cleanup_failure', 'fault']


@pytest.mark.parametrize('log_event', ['cycle_start', 'operation_complete', 'cycle_complete'])
def test_success_logging_failure_stops_both_and_preserves_logger_fault(log_event):
    config, ev3, arm, events = station()
    failure = OSError('cannot write log')
    def broken_logger(record):
        if record['event'] == log_event or record['event'] == 'fault':
            raise failure
    with pytest.raises(OSError) as fault:
        run_cycle(config, ev3, arm, broken_logger)
    assert fault.value is failure
    assert 'ev3.stop' in events and 'arm.stop' in events
    assert ev3.stopped and arm.stopped


@pytest.mark.parametrize('direction', [0, 2, -2, True, 1.0, '1', None])
def test_feeder_home_direction_requires_signed_integer(direction):
    config = example()
    config['ev3']['feeder']['home_direction'] = direction
    with pytest.raises(ValueError, match='home_direction'):
        validate_config(config)


@pytest.mark.parametrize('direction', [-1, 1])
def test_feeder_home_direction_is_independent_of_feed(direction):
    config = example()
    config['ev3']['directions']['feeder'] = -direction
    config['ev3']['feeder']['home_direction'] = direction
    assert validate_config(config) is config


def test_provisional_color_routes_load_until_physical_admission(tmp_path):
    config = example()
    pose = {k: 50 if k == 'gripper' else 0 for k in config['arm']['limits']}
    config['arm']['poses'] = {'pickup': pose, 'red': pose}
    config['arm']['trajectories'] = {'pick_and_place': {'red': ['pickup', 'red']}}
    path = tmp_path/'provisional.json'; path.write_text(json.dumps(config))
    assert load_config(path)['arm']['trajectories']['pick_and_place'] == {'red': ['pickup', 'red']}
    with pytest.raises(ValueError): load_config(path, physical=True)
    config['arm']['trajectories']['pick_and_place']['purple'] = ['pickup', 'red']
    with pytest.raises(ValueError): validate_config(config)
    del config['arm']['trajectories']['pick_and_place']['purple']
    config['arm']['trajectories']['pick_and_place']['red'] = []
    with pytest.raises(ValueError): validate_config(config)


def test_partial_color_routes_cannot_pass_physical_schema(tmp_path):
    config = commissioned(tmp_path)
    paths = config['arm']['trajectories']['pick_and_place']
    saved = paths.pop('blue')
    assert validate_config(config) is config
    with pytest.raises(ValueError, match='all four'): validate_config(config, physical=True)
    paths['blue'] = saved
    assert validate_config(config, physical=True) is config
