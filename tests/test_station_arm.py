import copy
import hashlib
import json
import threading
from types import SimpleNamespace

import pytest
from workshop.station.config import JOINTS
from workshop.station.arm import LeRobotArm


class Bus:
    def __init__(self):
        self.is_connected = False
        self.is_calibrated = True
        self.raw = dict.fromkeys(JOINTS, 2048)
        self.registers = {r: dict.fromkeys(JOINTS, v) for r, v in
                          [('Torque_Enable', 0), ('Acceleration', 20), ('Goal_Velocity', 30), ('Goal_Time', 0), ('Phase', 4)]}
        self.writes = []
        self.motors = {k: SimpleNamespace(id=i) for i, k in enumerate(JOINTS, 1)}
        self.failure = None

    def connect(self, handshake=False): self.is_connected = True
    def disconnect(self, disable_torque=False): self.is_connected = False
    def ping(self, k, **kw): return 777
    def read(self, reg, k, **kw): return self.registers[reg][k]
    def sync_read(self, reg, **kw):
        assert kw['normalize'] is False
        return dict(self.raw)
    def write(self, reg, k, value, **kw):
        self.writes.append((reg, k, value))
        if k == self.failure and reg == 'Torque_Enable': raise RuntimeError('broken motor')
        self.registers[reg][k] = value
    def sync_write(self, reg, values, **kw):
        assert reg == 'Goal_Position' and kw['normalize'] is False
        self.raw.update(values)


@pytest.fixture
def setup(tmp_path):
    calibration = {k: dict(id=i, drive_mode=0, homing_offset=0, range_min=1063, range_max=3033)
                   for i, k in enumerate(JOINTS, 1)}
    path = tmp_path / 'rig.json'
    path.write_text(json.dumps(calibration))
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    pose = dict.fromkeys(JOINTS, 0.)
    pose['gripper'] = 50.
    home = tmp_path / 'home.json'
    home.write_text(json.dumps(dict(schema=1, kind='measured-home', arm_id='rig', calibration_sha256=digest,
                                   units='body degrees; gripper percent', position=pose)))
    arm = dict(id='rig', usb=dict(vendor_id=1, product_id=2, serial='exact'), hz=125, tolerance=2,
               calibration=dict(path=str(path), sha256=digest), home_file=str(home),
               limits={k: dict(min=0 if k == 'gripper' else -80, max=100 if k == 'gripper' else 80,
                               speed=6, acceleration=12) for k in JOINTS},
               poses={k: copy.deepcopy(pose) for k in ['home', 'clearance', 'loading', 'pickup', 'source', 'blue', 'green', 'yellow', 'red']},
               trajectories=dict(home=['home'], clearance=['clearance'], load_piece=['source', 'loading'],
                                 pick_and_place={k: ['pickup', k] for k in ['blue', 'green', 'yellow', 'red']}))
    config = dict(arm=arm, timeouts=dict(operation_s=60, stop_s=2))
    bus = Bus()
    return LeRobotArm(config, bus=bus, output=tmp_path / 'out'), bus, config


def test_constructor_does_not_connect_or_write(setup):
    a, b, _ = setup
    assert not b.is_connected and not b.writes


@pytest.mark.parametrize('fault', ['hash', 'missing', 'drive_mode', 'id', 'range', 'invalid_home', 'missing_color', 'outside_pose'])
def test_preflight_fault_never_arms(setup, fault):
    a, b, c = setup
    if fault == 'hash': c['arm']['calibration']['sha256'] = '0' * 64
    elif fault == 'missing': c['arm']['calibration']['path'] += '-missing'
    elif fault in ('drive_mode', 'id', 'range'):
        p = a.config['arm']['calibration']['path']
        data = json.loads(open(p).read())
        data['gripper'][{'drive_mode': 'drive_mode', 'id': 'id', 'range': 'range_min'}[fault]] = {'drive_mode': 1, 'id': 1, 'range': 4095}[fault]
        open(p, 'w').write(json.dumps(data))
        c['arm']['calibration']['sha256'] = hashlib.sha256(open(p, 'rb').read()).hexdigest()
    elif fault == 'invalid_home':
        p = c['arm']['home_file']; data = json.loads(open(p).read())
        data['position']['shoulder_lift'] = (949-2048)*360/4095
        open(p, 'w').write(json.dumps(data)); c['arm']['poses']['home'] = data['position']
    elif fault == 'missing_color': del c['arm']['trajectories']['pick_and_place']['red']
    else: c['arm']['poses']['pickup']['shoulder_pan'] = 100
    with pytest.raises((ValueError, FileNotFoundError)):
        a.home()
    assert not any(b.registers['Torque_Enable'].values())


def test_raw_gripper_violation_is_not_clamped(setup):
    a, b, _ = setup; b.raw['gripper'] = 949
    with pytest.raises(ValueError, match='raw'): a.preflight()
    assert not any(b.registers['Torque_Enable'].values())


def test_hardware_calibration_mismatch(setup):
    a, b, _ = setup; b.is_calibrated = False
    with pytest.raises(ValueError, match='calibration'): a.preflight()


def test_all_motors_attempted_and_port_closed_after_stop_error(setup):
    a, b, _ = setup; a.connect(); b.failure = 'shoulder_pan'
    with pytest.raises(RuntimeError, match='cleanup'): a.close()
    assert not b.is_connected
    assert set(k for r, k, v in b.writes if r == 'Torque_Enable') == set(JOINTS)


def test_abort_never_arms(setup):
    a, b, _ = setup; a.abort_event.set()
    with pytest.raises(RuntimeError, match='abort'): a.home()
    assert not any(b.registers['Torque_Enable'].values())


def test_plan_saved_before_atomic_startup_failure(setup, monkeypatch):
    a, b, _ = setup
    def failed(*args):
        assert list(a.output.glob('plan-*.json'))
        b.registers['Torque_Enable']['shoulder_pan'] = 1
        raise RuntimeError('atomic failure')
    monkeypatch.setattr('workshop.station.arm.arm_current_pose', failed)
    with pytest.raises(RuntimeError, match='atomic failure'): a.home()
    assert not any(b.registers['Torque_Enable'].values())
    assert not any(r == 'Goal_Position' for r, k, v in b.writes)


def test_excessive_segment_displacement_rejected(setup):
    a, b, c = setup; c['arm']['poses']['source']['shoulder_pan'] = 30
    with pytest.raises(ValueError, match='envelope'): a.load_piece()
    assert not any(b.registers['Torque_Enable'].values())


def test_tracking_fault_and_deadline_stop(setup, monkeypatch):
    a, b, c = setup
    c['arm']['poses']['home']['shoulder_pan'] = 0
    a.preflight()
    move = a._segment(c['arm']['poses']['home'], c['arm']['poses']['home'])
    b.raw['shoulder_pan'] += 50
    monkeypatch.setattr(b, 'sync_write', lambda *args, **kwargs: None)
    with pytest.raises(RuntimeError, match='tracking'):
        a._execute(move, __import__('time').monotonic()+10, [])
    with pytest.raises(TimeoutError): a._execute(move, 0, [])


def test_abort_after_arm_stops_and_restores_ram(setup, monkeypatch):
    a, b, _ = setup
    def armed(bus, acceleration, velocity):
        bus.registers['Torque_Enable'] = dict.fromkeys(JOINTS, 1)
        bus.registers['Acceleration'] = dict(acceleration)
        bus.registers['Goal_Velocity'] = dict(velocity)
        a.abort_event.set()
        return dict(bus.raw)
    monkeypatch.setattr('workshop.station.arm.arm_current_pose', armed)
    with pytest.raises(RuntimeError, match='abort'): a.home()
    assert not any(b.registers['Torque_Enable'].values())
    assert set(b.registers['Acceleration'].values()) == {20}
    assert set(b.registers['Goal_Velocity'].values()) == {30}


def test_commission_capture_never_arms_or_marks_geometry(setup, tmp_path):
    from workshop.station.commission import capture_pose
    a, b, _ = setup
    target = tmp_path/'capture.json'
    target.write_text('{"old": true}')
    result = capture_pose(a, target, 'pickup')
    assert result['geometry_commissioned'] is False
    assert result['grasp_commissioned'] is False
    assert result['torque_at_capture'] == dict.fromkeys(JOINTS, 0)
    assert list(tmp_path.glob('capture.json.backup-*'))
    assert not b.is_connected and not any(b.registers['Torque_Enable'].values())


def test_profile_obeys_configured_limits(setup):
    from workshop.choreography import smooth_state
    a, _, c = setup; a.preflight()
    p = c['arm']['poses']['home']; target = {**p, 'shoulder_pan': 15}
    move = a._segment(p, target); zero = dict.fromkeys(JOINTS, 0.)
    for tick in range(501):
        _, v, acc, jerk = smooth_state(p, target, zero, zero, move['duration'], tick/500)
        assert abs(v['shoulder_pan']) <= 6
        assert abs(acc['shoulder_pan']) <= 12
        assert abs(jerk['shoulder_pan']) <= 48


def test_tracking_failure_preserves_original_and_stops(setup, monkeypatch):
    a, b, _ = setup
    def arm(bus, acceleration, velocity):
        bus.registers['Torque_Enable'] = dict.fromkeys(JOINTS, 1)
        return dict(bus.raw)
    def execute(*args): raise RuntimeError('tracking sentinel')
    monkeypatch.setattr('workshop.station.arm.arm_current_pose', arm)
    monkeypatch.setattr(a, '_execute', execute)
    b.failure = 'shoulder_pan'
    # Initial torque-off failure blocks startup, preserving its cleanup diagnosis.
    with pytest.raises(RuntimeError, match='cleanup'):
        a.home()
    b.failure = None
    with pytest.raises(RuntimeError, match='tracking sentinel'): a.home()
    assert not any(b.registers['Torque_Enable'].values())


def test_no_restore_if_off_readback_fails(setup):
    a, b, _ = setup; a.connect()
    a.saved = {'Acceleration': dict.fromkeys(JOINTS, 99)}
    b.registers['Torque_Enable']['shoulder_pan'] = 1
    b.failure = 'shoulder_pan'
    with pytest.raises(RuntimeError, match='cleanup'): a.stop()
    assert not any(r == 'Acceleration' for r, k, v in b.writes)


def test_success_acknowledges_arrival_and_clearance(setup, monkeypatch):
    a, b, c = setup
    def arm(bus, acceleration, velocity):
        bus.registers['Torque_Enable'] = dict.fromkeys(JOINTS, 1)
        return dict(bus.raw)
    monkeypatch.setattr('workshop.station.arm.arm_current_pose', arm)
    real_segment = a._segment
    def short(start, target):
        plan = real_segment(start, target)
        # All fixture routes have identical poses: stationary duration can be
        # shortened without changing speed/acceleration or the real executor.
        assert start == target
        plan['duration'] = .016
        return plan
    monkeypatch.setattr(a, '_segment', short)
    assert a.clear() is True
    assert a.wait_clear() is True and a.clear_confirmed
    a.close()
    assert not b.is_connected and not any(b.registers['Torque_Enable'].values())


def test_abort_between_startup_read_and_enable_rejected(setup, monkeypatch):
    a, b, _ = setup
    def startup(bus, acceleration, velocity):
        a.abort_event.set()
        bus.sync_read('Present_Position', normalize=False)
        pytest.fail('atomic enable must never be reached')
    monkeypatch.setattr('workshop.station.arm.arm_current_pose', startup)
    with pytest.raises(RuntimeError, match='abort'): a.home()
    assert not any(b.registers['Torque_Enable'].values())


def test_directory_sync_failure_never_arms(setup, monkeypatch):
    a, b, _ = setup
    import stat
    import os
    fsync = os.fsync
    def fail_directory(fd):
        if stat.S_ISDIR(os.fstat(fd).st_mode): raise OSError('directory sync failed')
        fsync(fd)
    monkeypatch.setattr('workshop.station.arm.os.fsync', fail_directory)
    with pytest.raises(OSError, match='directory sync failed'): a.home()
    assert not any(b.registers['Torque_Enable'].values())


def test_commission_exports_pose_home_and_route_config(setup, tmp_path):
    from workshop.station.commission import update_config
    from workshop.station.config import load_config
    a, _, c = setup
    # Use complete public template as the surrounding station schema.
    from pathlib import Path
    complete = json.loads(Path('workshop/station/resources/station.example.json').read_text())
    complete['arm'] = c['arm']
    complete['arm']['poses'] = {}
    complete['arm']['trajectories'] = {}
    a.config = complete; a.settings = complete['arm']
    destination = tmp_path / 'station.json'
    pose = dict.fromkeys(JOINTS, 0.); pose['gripper'] = 50.
    captured = dict(name='home', position=pose, calibration_sha256=a.settings['calibration']['sha256'], arm_id='rig', units='body degrees; gripper percent')
    update_config(a, destination, captured=captured, capture_path=tmp_path/'taught-home.json')
    assert load_config(destination)['arm']['poses']['home'] == pose
    assert load_config(destination)['arm']['home_file'] == str(tmp_path/'taught-home.json')
    update_config(a, destination, route=['home', 'home'])
    exported = load_config(destination)
    assert exported['arm']['trajectories']['home'] == ['home']
    assert exported['commissioned'] is False
    assert list(tmp_path.glob('station.json.backup-*'))


def test_route_export_rejects_large_displacement_and_hash(setup, tmp_path):
    from workshop.station.commission import update_config
    a, _, c = setup
    c['arm']['poses']['source']['shoulder_pan'] = 30
    with pytest.raises(ValueError, match='envelope'):
        update_config(a, tmp_path/'station.json', route=['load_piece', 'source', 'loading'])
    c['arm']['calibration']['sha256'] = '0'*64
    with pytest.raises(ValueError, match='hash'):
        update_config(a, tmp_path/'station.json', route=['home', 'home'])


def test_atomic_startup_overrides_time_without_prearm_write(setup, monkeypatch):
    a, b, _ = setup
    b.registers['Goal_Time'] = dict.fromkeys(JOINTS, 37)
    def startup(bus, acceleration, velocity):
        assert all(bus.read('Goal_Time', k) == 0 for k in JOINTS)
        assert not any(r == 'Goal_Time' for r, k, v in b.writes)
        bus.registers['Torque_Enable'] = dict.fromkeys(JOINTS, 1)
        bus.registers['Goal_Time'] = dict.fromkeys(JOINTS, 0)
        a.abort_event.set()
        return dict(bus.raw)
    monkeypatch.setattr('workshop.station.arm.arm_current_pose', startup)
    with pytest.raises(RuntimeError, match='abort'): a.home()
    assert set(b.registers['Goal_Time'].values()) == {37}
    assert not any(b.registers['Torque_Enable'].values())


def test_commission_cli_exports_taught_pose_and_route(setup, tmp_path, monkeypatch):
    from pathlib import Path
    from workshop.station import commission
    from workshop.station.config import load_config
    a, _, c = setup
    complete = json.loads(Path('workshop/station/resources/station.example.json').read_text())
    complete['arm'] = c['arm']; a.config = complete
    config_file = tmp_path/'input.json'; config_file.write_text(json.dumps(complete))
    exported = tmp_path/'exported.json'; captured = tmp_path/'pickup.json'
    monkeypatch.setattr(commission, 'LeRobotArm', lambda config: a)
    commission.main(['--config', str(config_file), '--capture-pose', 'pickup', '--output', str(captured), '--update-config', str(exported)])
    assert load_config(exported)['arm']['poses']['pickup'] == json.loads(captured.read_text())['position']
    commission.main(['--config', str(exported), '--set-route', 'pick_and_place', 'pickup', 'red', '--color', 'red', '--update-config', str(exported)])
    assert load_config(exported)['arm']['trajectories']['pick_and_place']['red'] == ['pickup', 'red']
    assert load_config(exported)['commissioned'] is False


def test_route_only_cli_builds_first_color_without_device_access(setup, tmp_path, monkeypatch):
    from pathlib import Path
    from workshop.station import commission
    from workshop.station.config import load_config
    a, _, c = setup
    complete = json.loads(Path('workshop/station/resources/station.example.json').read_text())
    complete['arm'] = copy.deepcopy(c['arm'])
    complete['arm']['trajectories'] = {}
    source = tmp_path/'input.json'; source.write_text(json.dumps(complete))
    exported = tmp_path/'routes.json'
    monkeypatch.setattr(LeRobotArm, 'connect', lambda *args: pytest.fail('route export must not open a bus'))
    commission.main(['--config', str(source), '--set-route', 'pick_and_place', 'pickup', 'red', '--color', 'red', '--update-config', str(exported)])
    config = load_config(exported)
    assert config['arm']['trajectories'] == {'pick_and_place': {'red': ['pickup', 'red']}}
    assert not config['commissioned'] and not config['automatic_start']
