"""Offline systemd inspection and restart admission regressions."""
import importlib.util
from pathlib import Path
import threading
import time
from types import SimpleNamespace

import pytest


@pytest.fixture
def restart(monkeypatch, tmp_path):
    path = Path(__file__).resolve().parents[1] / 'ros2/src/so101_station_control/so101_station_control/operator_restart.py'
    spec = importlib.util.spec_from_file_location('operator_restart_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    node = module.OperatorRestartMixin()
    node._operator_restart_lock = node._active_lock = threading.RLock()
    node._operator_restart_units = {'so101-camera.service'}
    node._operator_external_job = None
    node._operator_maintenance_token = 'a' * 32
    node._active_goal_is = lambda goal: goal == 'operator_maintenance'
    node._closed = node._stop_latched = node._owner_faulted = False
    node._owner_started = node._cached_owner_generation = time.monotonic() - .01
    node._process = SimpleNamespace(poll=lambda: None)
    node._expected_arm_id = 'arm'
    node._expected_calibration_sha256 = 'b' * 64
    node._owner_source_expected = {'arm_session': 'c' * 64}
    node._operator_restart_source_sha256 = {'operator_restart': 'd' * 64}
    node._operator_restart_journal = tmp_path / 'pending.json'
    node._cached_owner_state = dict(arm_id='arm', calibration_sha256='b' * 64,
        owner_source_sha256=node._owner_source_expected, torque=dict.fromkeys(module.JOINTS, 0),
        monotonic=time.monotonic(), torque_monotonic=time.monotonic())
    monkeypatch.setattr(module.threading, 'Thread', lambda **kwargs: SimpleNamespace(start=lambda: None))
    return module, node


def mock_show(monkeypatch, module, job, **changes):
    row = dict(Id='so101-camera.service', LoadState='loaded', ActiveState='active',
        SubState='running', Job=job, InvocationID='1' * 32, StateChangeTimestampMonotonic='1')
    row.update(changes)
    monkeypatch.setattr(module.subprocess, 'run', lambda *args, **kwargs:
        SimpleNamespace(returncode=0, stderr='', stdout='\n'.join(f'{k}={v}' for k, v in row.items())))


@pytest.mark.parametrize('job', ['', '0'])
def test_no_job_canonicalized_before_journal_and_completion(restart, monkeypatch, job):
    module, node = restart
    mock_show(monkeypatch, module, job)
    node._operator_restart_begin('a' * 32, 'so101-camera.service')
    record = node._operator_external_job
    assert record['before']['Job'] == '0'
    assert '"Job": "0"' in node._operator_restart_journal.read_text()
    mock_show(monkeypatch, module, job, InvocationID='2' * 32,
              StateChangeTimestampMonotonic=str(record['requested_monotonic_us'] + 1))
    proof = node._operator_restart_completion(record)
    assert proof['success'] and proof['after']['Job'] == '0'


@pytest.mark.parametrize('job', ['42', ' ', 'garbage'])
def test_existing_or_malformed_job_rejects_begin_and_completion(restart, monkeypatch, job):
    module, node = restart
    mock_show(monkeypatch, module, job)
    assert module.read_unit('so101-camera.service')['Job'] == job
    with pytest.raises(RuntimeError, match='external manager job'):
        node._operator_restart_begin('a' * 32, 'so101-camera.service')
    assert not node._operator_restart_journal.exists()
    with pytest.raises(RuntimeError, match='still pending'):
        node._operator_restart_completion({'unit': 'so101-camera.service'})


def test_empty_job_does_not_bypass_complete_state_validation(restart, monkeypatch):
    module, _ = restart
    mock_show(monkeypatch, module, '', StateChangeTimestampMonotonic='bad')
    with pytest.raises(RuntimeError, match='timestamp'):
        module.read_unit('so101-camera.service')
