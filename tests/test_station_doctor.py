"""Offline doctor contract: no devices, services, or motion are started."""
import ast
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from tools import station_cli


def fixture_entry(tmp_path):
    config = tmp_path / 'station.json'
    calibration = tmp_path / 'calibration.json'; calibration.write_text('{}')
    import hashlib
    config.write_text(json.dumps({'arm': {'port': str(tmp_path / 'follower'), 'calibration': {
        'path': str(calibration), 'sha256': hashlib.sha256(b'{}').hexdigest()}}}))
    parsed = SimpleNamespace(config=config, python=tmp_path / 'python', calibration_dir=tmp_path)
    return SimpleNamespace(ROOT=tmp_path, lifecycle_command=lambda *a: ['python', 'tool', 'start'],
        process=SimpleNamespace(parse_arguments=lambda a: parsed, wrapper=lambda a: [str(tmp_path / 'wrapper')]),
        configured_ports=station_cli.configured_ports,
        preflight=lambda a: (_ for _ in ()).throw(RuntimeError('Missing prerequisites')))


def test_aggregates_missing_prerequisites_and_keeps_all_diagnostics(tmp_path):
    from tools import station_doctor as doctor
    result = doctor.collect(SimpleNamespace(), fixture_entry(tmp_path),
        run_probe=lambda argv: '', usb_root=tmp_path / 'usb', status_provider=lambda: {})
    checks = {r['component']: r for r in result['checks']}
    assert result['schema'] == 'so101-station-doctor-v1'
    assert result['qualified_for_motion'] is False
    assert result['prerequisites_ready'] is False
    assert checks['preflight']['state'] == 'blocked'
    assert {'vulkan', 'codecs', 'camera_usb', 'ev3_usb', 'serial_ports', 'follower_calibration',
            'live_api', 'ev3_reference', 'jaw_calibration', 'tcp_calibration', 'recordings',
            'gpu_raw_alignment', 'codec_qualification', 'octomap'} <= checks.keys()
    assert checks['ev3_reference']['state'] == 'unknown'


def test_software_vulkan_rejected_and_encoder_enumeration_unqualified(tmp_path):
    from tools import station_doctor as doctor
    def probe(argv):
        return 'deviceName = llvmpipe (LLVM)' if argv[0] == 'vulkaninfo' else ' V....D h264_vaapi\n V....D h264_vulkan\n V....D libx264'
    result = doctor.collect(SimpleNamespace(), fixture_entry(tmp_path), run_probe=probe,
        usb_root=tmp_path, status_provider=lambda: {})
    checks = {r['component']: r for r in result['checks']}
    assert checks['vulkan']['state'] == 'blocked'
    assert checks['codecs']['state'] == 'available'
    assert 'VAAPI' in checks['codecs']['reason'] and 'CPU' in checks['codecs']['reason']
    assert checks['codec_qualification']['state'] == 'unqualified'


def test_live_api_flags_and_recording_revalidation(tmp_path):
    from tools import station_doctor as doctor
    status = {'backend': {'streams': {'/station/ev3/operator_status': {'age_s': .1, 'data': {
        'fresh': True, 'connected': True, 'references': {'sorter': 'home', 'feeder': 'eject'}}},
        'gripper_calibration': {'age_s': .1, 'data': {'si_gap_calibrated': False}}}},
        'recording_info': {'one': {'state': 'invalid'}, 'two': {'state': 'revalidation_required'}, 'three': {'state': 'usable'}}}
    result = doctor.collect(SimpleNamespace(), fixture_entry(tmp_path), run_probe=lambda a: '',
        usb_root=tmp_path, status_provider=lambda: status)
    checks = {r['component']: r for r in result['checks']}
    assert checks['live_api']['state'] == 'available'
    assert checks['jaw_calibration']['state'] == 'blocked'
    assert checks['tcp_calibration']['state'] == 'unknown'
    assert checks['recordings']['state'] == 'unqualified'
    assert '1 invalid; 2 require' in checks['recordings']['reason']


def test_doctor_json_stdout_is_pure_json(monkeypatch, capsys):
    from tools import station_doctor as doctor
    report = {'schema':'so101-station-doctor-v1', 'checks':[], 'prerequisites_ready':True, 'qualified_for_motion':False}
    monkeypatch.setattr(doctor, 'collect', lambda *a: report)
    assert station_cli.main(['doctor', '--json']) == 0
    assert json.loads(capsys.readouterr().out) == report


@pytest.mark.parametrize('catalog,state', [
    (None, 'unknown'),
    ({'state':'missing','skills':{}}, 'blocked'),
    ({'state':'configured','skills':{}}, 'unqualified'),
    ({'state':'configured','skills':{'fixed_pick_place':{'state':'uncommissioned'}}}, 'unqualified'),
    ({'state':'configured','skills':{'fixed_pick_place':{'state':'revalidation_required'}}}, 'unqualified'),
])
def test_named_skill_catalog_readiness_is_explicit(tmp_path, catalog, state):
    from tools import station_doctor as doctor
    report=doctor.collect(SimpleNamespace(), fixture_entry(tmp_path), run_probe=lambda a:'',
        usb_root=tmp_path, status_provider=lambda:{'taught_skills':catalog})
    row=next(r for r in report['checks'] if r['component']=='taught_skills')
    assert row['state']==state
    assert report['qualified_for_motion'] is False


def test_octomap_config_opt_in_and_gpu_stream():
    import yaml
    root = Path(__file__).resolve().parents[1]
    data = yaml.safe_load((root / 'ros2/config/sensors_3d.yaml').read_text())
    assert data['octomap_resolution'] == .02
    assert data['resident_pointcloud']['max_update_rate'] == 2.
    assert data['resident_pointcloud']['point_cloud_topic'] == '/station/resident/downsampled_points'
    path = root / 'ros2/src/so101_station_control/launch/moveit_resident.launch.py'
    tree = ast.parse(path.read_text())
    helper = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == '_octomap_parameters')
    scope = {'yaml': yaml}
    exec(compile(ast.Module(body=[helper], type_ignores=[]), str(path), 'exec'), scope)
    assert scope['_octomap_parameters'](False, root / 'nonexistent') == {}
    assert scope['_octomap_parameters'](True, root / 'ros2/config/sensors_3d.yaml') == data
    assert 'DeclareLaunchArgument("use_octomap", default_value="false"' in path.read_text()
    assert 'parameters.update(_octomap_parameters(' in path.read_text()


def test_probe_errors_and_malformed_live_status_do_not_abort_report(tmp_path):
    from tools import station_doctor as doctor
    def unavailable(argv): raise TimeoutError('bounded probe')
    result = doctor.collect(SimpleNamespace(), fixture_entry(tmp_path), run_probe=unavailable,
        usb_root=tmp_path, status_provider=lambda: {'streams': 'malformed', 'recording_info': []})
    checks = {r['component']: r for r in result['checks']}
    assert checks['vulkan']['state'] == 'unknown'
    assert checks['codecs']['state'] == 'unknown'
    assert checks['recordings']['state'] == 'unknown'
    assert checks['octomap']['state'] == 'unqualified'


def test_camera_usb_descriptor_is_not_sdk_serial(tmp_path):
    from tools import station_doctor as doctor
    device = tmp_path / '1-1'; device.mkdir()
    (device / 'idVendor').write_text('8086\n'); (device / 'idProduct').write_text('0b3a\n')
    report = doctor.collect(SimpleNamespace(), fixture_entry(tmp_path), run_probe=lambda a: '',
        usb_root=tmp_path, status_provider=lambda: {})
    camera = next(r for r in report['checks'] if r['component'] == 'camera_usb')
    assert camera['state'] == 'available'
    assert 'not SDK serial' in camera['reason']


def test_bounded_probe_commands_are_enumeration_only(monkeypatch):
    from tools import station_doctor as doctor
    calls=[]
    def execute(argv, **kwargs):
        calls.append((argv,kwargs)); return SimpleNamespace(stdout='inventory')
    monkeypatch.setattr(doctor.subprocess, 'run', execute)
    assert doctor.probe(['vulkaninfo', '--summary']) == 'inventory'
    assert calls == [(['vulkaninfo', '--summary'], dict(capture_output=True,text=True,check=True,timeout=5))]


def test_valid_prepared_status_uses_only_bounded_status_service_child(tmp_path, monkeypatch):
    from tools import station_doctor as doctor
    calls=[]
    entry=SimpleNamespace(ROOT=tmp_path, manifest=lambda args: {'domain':177,'wrapper':'/prepared/wrapper'})
    def execute(command, **kwargs):
        calls.append((command,kwargs))
        return SimpleNamespace(stdout=json.dumps({'physical':True,'recording_info':{}}))
    monkeypatch.setattr(doctor.subprocess, 'run', execute)
    assert doctor.live_status(SimpleNamespace(profile='fwmini4'),entry)['physical'] is True
    command,kwargs=calls[0]
    assert command[:4] == ['/prepared/wrapper','ros-vulkan','python','-c']
    assert "client('status','{}',2.0)" in command[-1]
    assert 'ROSBackend' not in command[-1] and 'Application' not in command[-1]
    assert 'client.close()' in command[-1]
    assert kwargs['timeout'] == 8 and kwargs['env']['ROS_DOMAIN_ID'] == '177'


def test_invalid_manifest_never_launches_status_child(monkeypatch):
    from tools import station_doctor as doctor
    def invalid(args): raise ValueError('manifest mismatch')
    entry=SimpleNamespace(manifest=invalid)
    monkeypatch.setattr(doctor.subprocess,'run',lambda *a,**k: pytest.fail('invalid manifest launched child'))
    with pytest.raises(ValueError,match='manifest mismatch'):
        doctor.live_status(SimpleNamespace(profile='fwmini4'),entry)


def prepared_identity_fixture(tmp_path):
    """Real config/calibration bytes; lifecycle preserves the root ownership gate."""
    entry = fixture_entry(tmp_path)
    manifest = tmp_path / 'units.json'
    manifest.write_text(json.dumps({'root': str(tmp_path)}))
    args = SimpleNamespace(profile='fwmini4', manifest=manifest)
    def identity(args):
        value = json.loads(args.manifest.read_text())
        if value['root'] != str(entry.ROOT):
            raise RuntimeError('Unit manifest belongs to another checkout.')
        return value
    entry.manifest = identity
    command = entry.lifecycle_command
    def lifecycle(args, action):
        identity(args)
        return command(args, action)
    entry.lifecycle_command = lifecycle
    return args, entry


def test_foreign_manifest_reports_ownership_first_not_missing_calibration(tmp_path):
    from tools import station_doctor as doctor
    args, entry = prepared_identity_fixture(tmp_path)
    foreign = tmp_path / 'other-checkout'
    args.manifest.write_text(json.dumps({'root': str(foreign)}))
    # Calibration exists and is valid; the ownership refusal occurs before parsing it.
    assert (tmp_path / 'calibration.json').exists()
    def forbidden(*args): pytest.fail('foreign checkout must not be evaluated/contacted')
    entry.preflight = forbidden
    entry.process.parse_arguments = forbidden
    report = doctor.collect(args, entry, run_probe=lambda a: '', usb_root=tmp_path,
                            status_provider=forbidden)
    first = report['checks'][0]
    assert first['component'] == 'manifest_identity' and first['state'] == 'blocked'
    assert str(tmp_path) in first['reason'] and str(foreign) in first['reason']
    assert 'ownership mismatch' in first['reason']
    rows = {r['component']: r for r in report['checks']}
    for component in ('preflight', 'runtime_paths', 'follower_calibration', 'serial_ports',
                      'live_api', 'ev3_reference', 'jaw_calibration', 'tcp_calibration',
                      'recordings', 'taught_skills'):
        assert rows[component]['state'] == 'unknown'
        assert 'Not evaluated' in rows[component]['reason']
        assert 'manifest_identity' in rows[component]['reason']
    assert report['prerequisites_ready'] is False
    assert report['qualified_for_motion'] is False
    with pytest.raises(RuntimeError, match='another checkout'):
        entry.lifecycle_command(args, 'start')


@pytest.mark.parametrize('calibration_state', ['valid', 'missing', 'hash_mismatch'])
def test_matching_manifest_keeps_real_calibration_result(tmp_path, calibration_state):
    from tools import station_doctor as doctor
    args, entry = prepared_identity_fixture(tmp_path)
    if calibration_state == 'missing':
        (tmp_path / 'calibration.json').unlink()
    elif calibration_state == 'hash_mismatch':
        (tmp_path / 'calibration.json').write_text('{"changed":true}')
    report = doctor.collect(args, entry, run_probe=lambda a: '', usb_root=tmp_path,
                            status_provider=lambda: {})
    rows = {r['component']: r for r in report['checks']}
    assert rows['manifest_identity']['state'] == 'available'
    assert rows['follower_calibration']['state'] == ('available' if calibration_state == 'valid' else 'blocked')
    if calibration_state == 'missing':
        assert 'missing' in rows['follower_calibration']['reason']
    elif calibration_state == 'hash_mismatch':
        assert 'hash mismatch' in rows['follower_calibration']['reason']


@pytest.mark.parametrize('contents', ['{', '{}', '{"root":4}'])
def test_malformed_manifest_is_explicit_and_dependent_checks_not_evaluated(tmp_path, contents):
    from tools import station_doctor as doctor
    args, entry = prepared_identity_fixture(tmp_path)
    args.manifest.write_text(contents)
    report = doctor.collect(args, entry, run_probe=lambda a: '', usb_root=tmp_path,
                            status_provider=lambda: pytest.fail('invalid manifest contacted API'))
    rows = {r['component']: r for r in report['checks']}
    assert report['checks'][0]['component'] == 'manifest_identity'
    assert rows['manifest_identity']['state'] == 'blocked'
    assert 'malformed' in rows['manifest_identity']['reason']
    assert rows['follower_calibration']['state'] == 'unknown'
    assert rows['recordings']['state'] == 'unknown'


def test_matching_root_still_requires_authoritative_source_validation(tmp_path):
    from tools import station_doctor as doctor
    args, entry = prepared_identity_fixture(tmp_path)
    def invalid_sources(args): raise ValueError('native unit source cohort changed; render/review again')
    entry.manifest = invalid_sources
    report = doctor.collect(args, entry, run_probe=lambda a: '', usb_root=tmp_path,
                            status_provider=lambda: pytest.fail('unverified source contacted API'))
    rows = {r['component']: r for r in report['checks']}
    assert rows['manifest_identity']['state'] == 'blocked'
    assert 'source' in rows['manifest_identity']['reason']
    assert rows['follower_calibration']['state'] == 'unknown'


@pytest.mark.parametrize('reported_size', [0, 131072], ids=['stale-stat', 'oversized'])
def test_manifest_read_is_bounded_even_when_stat_size_is_stale(tmp_path, reported_size):
    from io import BytesIO
    from tools import station_doctor as doctor
    args, entry = prepared_identity_fixture(tmp_path)
    payload = json.dumps({'root': str(tmp_path)}).encode() + b' ' * 131072
    read_sizes = []
    class Reader(BytesIO):
        def read(self, size=-1):
            read_sizes.append(size)
            return super().read(size)
    class Manifest:
        def stat(self): return SimpleNamespace(st_size=reported_size)
        def read_text(self): return payload.decode()
        def open(self, mode):
            assert mode == 'rb'
            return Reader(payload)
    args.manifest = Manifest()
    def forbidden(*args): pytest.fail('oversized manifest evaluated dependent checkout')
    entry.manifest = entry.preflight = entry.process.parse_arguments = forbidden
    report = doctor.collect(args, entry, run_probe=lambda a: '', usb_root=tmp_path,
                            status_provider=forbidden)
    rows = {r['component']: r for r in report['checks']}
    assert read_sizes == [65537]
    assert rows['manifest_identity']['state'] == 'blocked'
    assert rows['preflight']['state'] == 'unknown'
    assert rows['follower_calibration']['state'] == 'unknown'
    assert rows['live_api']['state'] == 'unknown'
