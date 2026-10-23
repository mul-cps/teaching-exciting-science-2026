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


def test_commission_capture_observes_activation_lock_before_construction(tmp_path, monkeypatch):
    from workshop.station import commission
    monkeypatch.setattr(commission, 'LeRobotArm', lambda *a: pytest.fail('constructed device while locked'))
    monkeypatch.setattr(commission, 'capture_pose', lambda *a: pytest.fail('capture while locked'))
    with cli.station_lock(tmp_path / 'lock'):
        with pytest.raises(RuntimeError, match='owns'):
            commission.main(['--config', str(config_file(tmp_path)), '--capture-pose', 'home',
                             '--output', str(tmp_path / 'home.json'), '--lock', str(tmp_path / 'lock')])


@pytest.mark.parametrize('fail', [False, True])
def test_commission_holds_lock_through_capture_export_and_releases(tmp_path, monkeypatch, fail):
    from workshop.station import commission
    lock = tmp_path / 'lock'
    monkeypatch.setattr(commission, 'LeRobotArm', lambda *a: object())
    def capture(*args):
        with pytest.raises(RuntimeError, match='owns'):
            with cli.station_lock(lock): pass
        if fail: raise ValueError('capture failed')
        return {'captured': True}
    def export(*args):
        with pytest.raises(RuntimeError, match='owns'):
            with cli.station_lock(lock): pass
    monkeypatch.setattr(commission, 'capture_pose', capture)
    monkeypatch.setattr(commission, 'update_config', export)
    argv = ['--config', str(config_file(tmp_path)), '--capture-pose', 'home', '--output', 'home.json',
            '--update-config', 'export.json', '--lock', str(lock)]
    if fail:
        with pytest.raises(ValueError, match='capture failed'): commission.main(argv)
    else: commission.main(argv)
    with cli.station_lock(lock): pass


def test_commission_default_lock_matches_activation_and_blocks_route_export(tmp_path, monkeypatch):
    from workshop.station import commission
    from workshop.station.locking import state_dir
    monkeypatch.setenv('XDG_STATE_HOME', str(tmp_path))
    monkeypatch.setattr(commission, 'LeRobotArm', lambda *a: pytest.fail('device constructed while locked'))
    with cli.station_lock(state_dir() / 'station.lock'):
        for action in (['--capture-pose', 'home', '--output', 'home.json'],
                       ['--set-route', 'home', 'home', '--update-config', 'export.json']):
            with pytest.raises(RuntimeError, match='owns'):
                commission.main(['--config', str(config_file(tmp_path)), *action])

# Unified launcher tests never invoke ROS, systemd, SSH or physical devices.
def lifecycle(tmp_path, monkeypatch):
    from tools import station_cli
    monkeypatch.setattr(station_cli, 'ROOT', tmp_path)
    monkeypatch.setattr(station_cli.prepared, 'ROOT', tmp_path)
    monkeypatch.setattr(station_cli.prepared, 'STATE', tmp_path / 'state')
    monkeypatch.setattr(station_cli, 'RUNTIME', tmp_path / 'runtime')
    monkeypatch.setattr(station_cli, 'tracked', lambda path: False)
    return station_cli


def test_lifecycle_purge_is_dry_by_default(tmp_path, monkeypatch, capsys):
    entry = lifecycle(tmp_path, monkeypatch)
    (tmp_path / 'build').mkdir()
    (tmp_path / 'build/generated').write_text('cache')
    monkeypatch.setattr(entry, 'stop', lambda args: pytest.fail('dry run stopped services'))
    assert entry.main(['purge']) == 0
    assert (tmp_path / 'build/generated').exists()
    assert 'DRY RUN' in capsys.readouterr().out


def test_lifecycle_purge_stops_before_delete(tmp_path, monkeypatch):
    entry = lifecycle(tmp_path, monkeypatch)
    (tmp_path / 'build').mkdir()
    monkeypatch.setattr(entry, 'stop', lambda args: (_ for _ in ()).throw(RuntimeError('STOP unresolved')))
    assert entry.main(['purge', '--yes']) == 1
    assert (tmp_path / 'build').exists()


def test_lifecycle_purge_rejects_symlink_boundary(tmp_path, monkeypatch):
    entry = lifecycle(tmp_path, monkeypatch)
    outside = tmp_path / 'private'; outside.mkdir()
    (outside / 'secret').write_text('keep')
    (tmp_path / 'build').symlink_to(outside, target_is_directory=True)
    monkeypatch.setattr(entry, 'stop', lambda args: pytest.fail('unsafe purge stopped services'))
    assert entry.main(['purge', '--yes']) == 1
    assert (outside / 'secret').read_text() == 'keep'


def test_lifecycle_purge_preserves_unknown_and_notebooks(tmp_path, monkeypatch):
    entry = lifecycle(tmp_path, monkeypatch)
    for name in ('notebooks', '.ssh', 'private-calibration', 'evidence', 'unknown'):
        (tmp_path / name).mkdir(); (tmp_path / name / 'keep').write_text('user data')
    (tmp_path / '.pytest_cache').mkdir()
    monkeypatch.setattr(entry, 'stop', lambda args: None)
    monkeypatch.setattr(entry, 'tracked', lambda path: False)
    assert entry.main(['purge', '--yes']) == 0
    assert not (tmp_path / '.pytest_cache').exists()
    assert all((tmp_path / name / 'keep').exists() for name in ('notebooks', '.ssh', 'private-calibration', 'evidence', 'unknown'))


def test_lifecycle_physical_blockly_requires_instructor(tmp_path, monkeypatch):
    entry = lifecycle(tmp_path, monkeypatch)
    monkeypatch.setattr(entry.prepared, 'blockly', lambda action: pytest.fail('physical startup'))
    assert entry.main(['blockly', '--physical']) == 1


def test_lifecycle_start_delegates_prepared_gui(tmp_path, monkeypatch):
    entry = lifecycle(tmp_path, monkeypatch)
    calls = []
    monkeypatch.setattr(entry, 'preflight', lambda args: calls.append('preflight'))
    monkeypatch.setattr(entry, 'start_gui', lambda args: calls.append('gui'))
    assert entry.main(['start']) == 0
    assert calls == ['preflight', 'gui']


def test_lifecycle_unknown_record_not_signalled(tmp_path, monkeypatch):
    entry = lifecycle(tmp_path, monkeypatch)
    entry.RUNTIME.mkdir()
    (entry.RUNTIME / 'managed-station.json').write_text(json.dumps({'ros': ['/evil'], 'process': {'pid': 123}}))
    monkeypatch.setattr(entry.prepared, 'blockly', lambda action: None)
    monkeypatch.setattr(entry, 'run', lambda *a, **kw: pytest.fail('unknown process command'))
    assert entry.main(['stop']) == 1


def test_lifecycle_status_unknown_is_not_healthy(tmp_path, monkeypatch, capsys):
    entry = lifecycle(tmp_path, monkeypatch)
    entry.RUNTIME.mkdir()
    (entry.RUNTIME / 'managed-station.json').write_text('{')
    assert entry.main(['status']) == 1
    assert 'station:' in capsys.readouterr().err


def test_lifecycle_stop_order_and_uses_existing_stop_contract(tmp_path, monkeypatch):
    entry = lifecycle(tmp_path, monkeypatch)
    entry.RUNTIME.mkdir()
    path = entry.RUNTIME / 'managed-station.json'; path.write_text('{}')
    gui = entry.RUNTIME / 'station-gui.json'; gui.write_text('{}')
    calls = []
    monkeypatch.setattr(entry, 'owned_station', lambda path: {'managed': True})
    monkeypatch.setattr(entry, 'gui_record', lambda path: {})
    monkeypatch.setattr(entry.prepared, 'blockly', lambda action: calls.append(action))
    monkeypatch.setattr(entry, 'run', lambda cmd: calls.append(cmd))
    monkeypatch.setattr(entry.process, 'cleanup_session', lambda *a, **kw: calls.append('gui-close'))
    assert entry.main(['stop']) == 0
    assert calls[0] == 'blockly-close'
    assert calls[1][2:] == ['stop', '--record', str(path)]
    assert calls[2] == 'gui-close'
    assert not gui.exists()


def test_lifecycle_purge_tracked_content_rejected_before_stop(tmp_path, monkeypatch):
    entry = lifecycle(tmp_path, monkeypatch)
    (tmp_path / 'build').mkdir()
    monkeypatch.setattr(entry, 'tracked', lambda path: True)
    monkeypatch.setattr(entry, 'stop', lambda args: pytest.fail('STOP before invalid target refusal'))
    assert entry.main(['purge', '--yes']) == 1
    assert (tmp_path / 'build').exists()


def test_lifecycle_purge_symlink_ancestor_rejected(tmp_path, monkeypatch):
    entry = lifecycle(tmp_path, monkeypatch)
    private = tmp_path / 'private'; private.mkdir()
    (private / 'install').mkdir()
    (tmp_path / 'ros2').symlink_to(private, target_is_directory=True)
    monkeypatch.setattr(entry, 'stop', lambda args: pytest.fail('STOP for escaped path'))
    assert entry.main(['purge', '--yes']) == 1
    assert (private / 'install').exists()


def test_lifecycle_purge_does_not_follow_nested_symlinks(tmp_path, monkeypatch):
    entry = lifecycle(tmp_path, monkeypatch)
    private = tmp_path / 'private'; private.mkdir()
    (private / 'important').write_text('safe')
    (tmp_path / 'build').mkdir()
    (tmp_path / 'build/link').symlink_to(private, target_is_directory=True)
    monkeypatch.setattr(entry, 'stop', lambda args: None)
    assert entry.main(['purge', '--yes']) == 0
    assert (private / 'important').read_text() == 'safe'


def test_lifecycle_native_custom_manifest_command(tmp_path, monkeypatch):
    entry = lifecycle(tmp_path, monkeypatch)
    params = {key: '/managed/' + key for key in ('station_config', 'station_python', 'joint_limit_contract',
              'state_file', 'station_lock', 'log_dir', 'calibration_dir', 'camera_sdk_prefix')}
    params['use_ev3_tunnel'] = 'false'
    monkeypatch.setattr(entry, 'manifest', lambda args: {'domain': 177, 'parameters': params})
    calls = []
    monkeypatch.setattr(entry, 'preflight', lambda args: None)
    monkeypatch.setattr(entry, 'run', lambda command: calls.append(command))
    assert entry.main(['start', '--headless', '--manifest', str(tmp_path / 'custom.json')]) == 0
    cmd = calls[0]
    assert cmd[cmd.index('--unit-manifest') + 1] == str(tmp_path / 'custom.json')
    assert cmd[cmd.index('--domain') + 1] == '177'
    assert '--bootstrap-owner' in cmd and '--background' in cmd and '--existing-tunnel' in cmd


def test_lifecycle_unknown_uninstall_registration_preserved(tmp_path, monkeypatch):
    entry = lifecycle(tmp_path, monkeypatch)
    directory = tmp_path / 'user-units'; directory.mkdir()
    path = directory / entry.units.UNITS['owner']; path.write_text('other owner')
    monkeypatch.setattr(entry, 'manifest', lambda args: {})
    monkeypatch.setattr(entry.units, 'unit_directory', lambda: directory)
    monkeypatch.setattr(entry.units, 'unit_text', lambda value, role: 'expected')
    monkeypatch.setattr(entry, 'stop', lambda args: pytest.fail('unsafe uninstall STOP'))
    assert entry.main(['purge', '--uninstall', '--yes']) == 1
    assert path.read_text() == 'other owner'


def test_lifecycle_argv_injection_rejected_before_subprocess(tmp_path, monkeypatch):
    entry = lifecycle(tmp_path, monkeypatch)
    entry.RUNTIME.mkdir()
    value = {'ros': [str(tmp_path / 'ros2/conda/so101-conda.sh'), 'run'],
             'command': [str(tmp_path), 'evil']}
    (entry.RUNTIME / 'managed-station.json').write_text(json.dumps(value))
    monkeypatch.setattr(entry, 'run', lambda cmd: pytest.fail('unowned subprocess'))
    monkeypatch.setattr(entry.prepared, 'blockly', lambda action: pytest.fail('unowned STOP'))
    assert entry.main(['stop']) == 1


def test_lifecycle_gui_command_identity_required(tmp_path, monkeypatch):
    import os
    entry = lifecycle(tmp_path, monkeypatch)
    path = tmp_path / 'gui.json'
    path.write_text(json.dumps(dict(root=str(tmp_path), command=entry.gui_command(),
                                   process={'pid': os.getpid()})))
    monkeypatch.setattr(entry.process, 'same_process', lambda record: True)
    with pytest.raises(RuntimeError, match='command identity changed'):
        entry.gui_record(path)


def test_lifecycle_full_physical_start_flags_and_hooks(tmp_path, monkeypatch):
    entry = lifecycle(tmp_path, monkeypatch)
    calls = []
    monkeypatch.setattr(entry, 'preflight', lambda args: calls.append('preflight'))
    monkeypatch.setattr(entry, 'start_gui', lambda args: calls.append(entry.gui_command(args.physical)))
    assert entry.main(['start', '--physical', '--instructor']) == 0
    assert calls == ['preflight', [entry.sys.executable, str(tmp_path / 'station'), '_gui', '--physical', '--instructor']]
    assert entry.main(['start', '--physical']) == 1
    monkeypatch.setattr(entry.prepared, 'blockly', lambda action: calls.append(action))
    def prepared_gui(before_rviz=None, before_stop=None):
        before_rviz(); calls.append('rviz'); before_stop()
    monkeypatch.setattr(entry.prepared, 'start_station', prepared_gui)
    assert entry.main(['_gui', '--physical', '--instructor']) == 0
    assert calls[-3:] == ['blockly-physical', 'rviz', 'blockly-close']


@pytest.mark.parametrize('failure', ['none', 'frontend-start', 'frontend-close'])
def test_lifecycle_prepared_hooks_always_preserve_native_stop(tmp_path, monkeypatch, failure):
    from types import SimpleNamespace
    entry = lifecycle(tmp_path, monkeypatch)
    events = []
    monkeypatch.setattr(entry.prepared, 'station_command', lambda launch_id: ['native-start'])
    monkeypatch.setattr(entry.prepared, 'station_manifest', lambda: (
        SimpleNamespace(component_command=lambda value, role: ['rviz']), {'domain': 76}))
    def command(cmd, **kwargs):
        events.append('native-stop' if 'stop' in cmd else cmd[0])
        return SimpleNamespace(returncode=0)
    monkeypatch.setattr(entry.prepared.subprocess, 'run', command)
    def before():
        events.append('frontend-start')
        if failure == 'frontend-start': raise RuntimeError('frontend startup failed')
    def closing():
        events.append('frontend-close')
        if failure == 'frontend-close': raise RuntimeError('frontend close failed')
    if failure == 'frontend-start':
        with pytest.raises(RuntimeError, match='startup failed'):
            entry.prepared.start_station(before, closing)
    elif failure == 'frontend-close':
        with pytest.raises(SystemExit) as error:
            entry.prepared.start_station(before, closing)
        assert error.value.code == 1
    else:
        entry.prepared.start_station(before, closing)
    assert events[0:2] == ['native-start', 'frontend-start']
    assert events[-2:] == ['frontend-close', 'native-stop']
    assert ('rviz' in events) == (failure != 'frontend-start')


def test_lifecycle_exact_configured_ports_override_usb_defaults(tmp_path, monkeypatch):
    entry = lifecycle(tmp_path, monkeypatch)
    follower = tmp_path / 'follower'; leader = tmp_path / 'leader'
    settings = {'arm': {'port': str(follower), 'usb': {'serial': 'ignored'}}, 'leader': {'port': str(leader)}}
    assert entry.configured_ports(settings) == [follower, leader]
    settings.pop('leader'); settings['leader_port'] = str(leader)
    assert entry.configured_ports(settings) == [follower, leader]


def test_lifecycle_ports_derive_only_from_configured_identity(tmp_path, monkeypatch):
    entry = lifecycle(tmp_path, monkeypatch)
    assert entry.configured_ports({'arm': {'usb': {'serial': 'CONFIGURED'}}}) == [
        entry.Path('/dev/serial/by-id/usb-1a86_USB_Single_Serial_CONFIGURED-if00')]
    with pytest.raises(RuntimeError, match='absolute'):
        entry.configured_ports({'arm': {'port': 'relative-device'}})


def test_lifecycle_runtime_purge_is_explicit_dry_run(tmp_path, monkeypatch, capsys):
    entry = lifecycle(tmp_path, monkeypatch)
    prefixes = ('conda', 'realsense-cpu-texture-bounds', 'rclpy-publisher-gid-7.1.12', 'rmw-cyclonedds-gid-2.2.4')
    for prefix in prefixes + ('private-calibration', 'unknown'):
        path = tmp_path / 'ros2/.cache' / prefix; path.mkdir(parents=True)
        (path / 'keep').write_text(prefix)
    monkeypatch.setattr(entry, 'stop', lambda args: pytest.fail('dry-run STOP'))
    assert entry.main(['purge']) == 0
    output = capsys.readouterr().out
    assert 'ros2/.cache/conda' not in output
    assert entry.main(['purge', '--runtime']) == 0
    output = capsys.readouterr().out
    assert all('ros2/.cache/' + prefix in output for prefix in prefixes)
    assert 'private-calibration' not in output and 'unknown' not in output
    monkeypatch.setattr(entry, 'stop', lambda args: None)
    assert entry.main(['purge', '--runtime', '--yes']) == 0
    assert all(not (tmp_path / 'ros2/.cache' / prefix).exists() for prefix in prefixes)
    assert (tmp_path / 'ros2/.cache/private-calibration/keep').exists()
    assert (tmp_path / 'ros2/.cache/unknown/keep').exists()


def test_lifecycle_runtime_prefix_symlink_preserved(tmp_path, monkeypatch):
    entry = lifecycle(tmp_path, monkeypatch)
    private = tmp_path / 'private'; private.mkdir()
    (tmp_path / 'ros2/.cache').mkdir(parents=True)
    (tmp_path / 'ros2/.cache/conda').symlink_to(private, target_is_directory=True)
    monkeypatch.setattr(entry, 'stop', lambda args: pytest.fail('unsafe runtime STOP'))
    assert entry.main(['purge', '--runtime', '--yes']) == 1
    assert private.is_dir()


def test_private_skills_roundtrip_through_blockly_and_gui_hooks(tmp_path,monkeypatch,capsys):
    entry=lifecycle(tmp_path,monkeypatch);private=tmp_path/'private-library';private.mkdir()
    calls=[]
    monkeypatch.setattr(entry.prepared,'blockly',lambda action,**kw:calls.append((action,kw)))
    assert entry.main(['blockly','--physical','--instructor','--skills',str(private)])==0
    assert calls==[('blockly-physical',{'skills':str(private.resolve())})]
    command=entry.gui_command(True,private)
    assert command[-2:]==['--skills',str(private.resolve())]
    monkeypatch.setattr(entry.prepared,'start_station',lambda before_rviz,before_stop:(before_rviz(),before_stop()))
    assert entry.main(command[2:])==0
    assert calls[-2:]==[('blockly-physical',{'skills':str(private.resolve())}),('blockly-close',{})]
    assert entry.main(['sim','--skills',str(private)])==1
    assert str(private) not in capsys.readouterr().err
    # Stop ownership remains checkable after the configured directory disappears.
    private.rmdir();assert entry.gui_command(True,private)==command
