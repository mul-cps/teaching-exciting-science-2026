import json
from importlib.resources import files

import pytest

from workshop.station import __main__ as cli


def config_file(tmp_path):
    p = tmp_path / 'station.json'
    p.write_text(files('workshop.station').joinpath('resources/station.example.json').read_text())
    return p


@pytest.mark.parametrize('workflow', ['arm_loads_sorter', 'arm_unloads_sorter'])
@pytest.mark.parametrize('firmware', ['original', 'pybricks'])
def test_simulate(tmp_path, capsys, workflow, firmware):
    path = config_file(tmp_path)
    config = json.loads(path.read_text()); config['ev3']['firmware'] = firmware
    path.write_text(json.dumps(config))
    assert cli.main(['--config', str(path), '--simulate', '--workflow', workflow,
                     '--log', str(tmp_path / 'run.jsonl'), '--lock', str(tmp_path / 'lock')]) == 0
    logs = [json.loads(s) for s in (tmp_path / 'run.jsonl').read_text().splitlines()]
    assert any(r['event'] == 'cycle_complete' and r['workflow'] == workflow for r in logs)


def test_detect_never_constructs_devices(tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(cli, 'make_devices', lambda *a: pytest.fail('device access'))
    assert cli.main(['--detect']) == 0
    assert 'original_ev3' in json.loads(capsys.readouterr().out)


def test_uncommissioned_rejected_before_devices(tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(cli, 'make_devices', lambda *a: pytest.fail('device access'))
    assert cli.main(['--config', str(config_file(tmp_path)), '--activate', '--lock', str(tmp_path / 'lock')]) == 1
    assert 'commissioned' in capsys.readouterr().err


def test_invalid_config(tmp_path, capsys):
    p = tmp_path / 'bad.json'; p.write_text('{')
    assert cli.main(['--config', str(p), '--simulate']) == 1
    assert 'station error' in capsys.readouterr().err


def test_resources():
    root = files('workshop.station').joinpath('resources')
    assert root.joinpath('station.example.json').is_file()
    assert root.joinpath('ev3_agent.py').is_file()


def test_cancellation_and_cleanup_survive_logging_failure(tmp_path):
    events = []
    class Device:
        def __init__(self, name): self.name = name
        def stop(self): events.append(self.name + '.stop')
        def close(self): events.append(self.name + '.close')
    class BrokenLog:
        def __call__(self, record): raise OSError('log full')
    ev3, arm = Device('ev3'), Device('arm')
    with pytest.raises(KeyboardInterrupt):
        cli.execute({}, ev3, arm, BrokenLog(), startup=lambda: (_ for _ in ()).throw(KeyboardInterrupt()))
    assert events == ['ev3.stop', 'arm.stop', 'ev3.close', 'arm.close']


def test_startup_order_and_health(tmp_path):
    events = []
    class Device:
        def __init__(self, name): self.name = name
        def __getattr__(self, name): return lambda: events.append(self.name + '.' + name)
    cli.startup(Device('ev3'), Device('arm'), lambda r: None)
    assert events.index('ev3.wait_stopped') < events.index('arm.home')
    assert events.index('arm.wait_clear') < events.index('ev3.home')


def test_lock_excludes_second_process(tmp_path):
    with cli.station_lock(tmp_path / 'lock'):
        with pytest.raises(RuntimeError, match='owns'):
            with cli.station_lock(tmp_path / 'lock'): pass


def test_automatic_start_false_rejected_before_devices(tmp_path, monkeypatch, capsys):
    # Isolate the CLI flag gate from the independently tested physical schema.
    monkeypatch.setattr(cli, 'validate_config', lambda *a, **k: None)
    monkeypatch.setattr(cli, 'make_devices', lambda *a: pytest.fail('device access'))
    assert cli.main(['--config', str(config_file(tmp_path)), '--activate', '--lock', str(tmp_path / 'lock')]) == 1
    assert 'automatic_start' in capsys.readouterr().err


def test_cancellation_logged_and_cleanup_attempted(tmp_path, monkeypatch):
    events = []
    class Device:
        def stop(self): events.append('stop')
        def close(self): events.append('close')
    monkeypatch.setattr(cli, 'make_devices', lambda *a: (Device(), Device()))
    monkeypatch.setattr(cli, 'startup', lambda *a: (_ for _ in ()).throw(KeyboardInterrupt()))
    log = tmp_path / 'run.jsonl'
    assert cli.main(['--simulate', '--log', str(log), '--lock', str(tmp_path / 'lock')]) == 130
    assert json.loads(log.read_text())['event'] == 'cancelled'
    assert events == ['stop', 'stop', 'close', 'close']


def test_health_failure_prevents_next_operation():
    events = []
    class EV3:
        def check_health(self): raise RuntimeError('heartbeat missing')
    class Arm:
        def load_piece(self): events.append('load')
    with pytest.raises(RuntimeError, match='heartbeat missing'):
        cli.HealthChecked(Arm(), EV3()).load_piece()
    assert events == []


def test_saved_home_below_calibration_rejected_before_factory(tmp_path, monkeypatch, capsys):
    from workshop.station.config import JOINTS
    import hashlib
    p = config_file(tmp_path); config = json.loads(p.read_text())
    calibration = {k: dict(id=i, drive_mode=0, homing_offset=0, range_min=1063, range_max=3000)
                   for i, k in enumerate(JOINTS, 1)}
    calibration_file = tmp_path / (config['arm']['id'] + '.json')
    calibration_file.write_text(json.dumps(calibration))
    digest = hashlib.sha256(calibration_file.read_bytes()).hexdigest()
    config['arm']['calibration'] = dict(path=str(calibration_file), sha256=digest)
    home_file = tmp_path / 'home.json'
    # Represent saved raw shoulder 949 using the same calibrated degree units.
    position = dict.fromkeys(JOINTS, 0.)
    position['shoulder_pan'] = (949 - (1063 + 3000) / 2) * 360 / 4095
    home_file.write_text(json.dumps(dict(schema=1, kind='measured-home', arm_id=config['arm']['id'],
        calibration_sha256=digest, units='body degrees; gripper percent', position=position)))
    config['arm']['home_file'] = str(home_file)
    config['automatic_start'] = True
    p.write_text(json.dumps(config))
    monkeypatch.setattr(cli, 'validate_config', lambda *a, **kw: None)
    monkeypatch.setattr(cli, 'make_devices', lambda *a: pytest.fail('device access'))
    assert cli.main(['--config', str(p), '--activate', '--lock', str(tmp_path / 'lock')]) == 1
    assert 'outside calibrated/configured limits' in capsys.readouterr().err
