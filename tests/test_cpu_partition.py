"""Pure lane construction tests; never collect the full suite or initialize ROS."""
from types import SimpleNamespace as NS
import pytest
from tools import test_cpu as lane


def test_canonical_lists_preserve_pure_and_mandatory_overlap():
    native, excluded = lane.partition()
    assert len(native) == 39 and len(excluded) == 19
    for name in ('test_supervisor_policy.py', 'test_supervisor_static.py',
                 'test_supervisor_journal.py', 'test_supervisor_adoption_ack.py',
                 'test_supervisor_private_availability.py',
                 'test_supervisor_consumer_reconciliation.py',
                 'test_supervisor_consumer_reconciliation_cli.py'):
        assert name in native and name not in excluded
    for name in ('test_completion_event_protocol.py', 'test_object_timing_diagnostic.py'):
        assert name not in excluded
    assert 'test_completion_event_final_gates.py' in excluded
    assert 'test_object_timing_diagnostic_ros.py' in native


def test_cpu_command_and_environment(monkeypatch):
    monkeypatch.setenv('PYTHONPATH', '/untrusted')
    monkeypatch.setenv('ROS_DOMAIN_ID', '76')
    monkeypatch.setenv('PYTEST_ADDOPTS', '--collect-only')
    monkeypatch.setenv('PYTEST_PLUGINS', 'untrusted')
    env = lane.environment()
    assert 'PYTHONPATH' not in env
    assert 'PYTEST_ADDOPTS' not in env and 'PYTEST_PLUGINS' not in env
    assert all(env[k] == v for k, v in lane.ENVIRONMENT.items())
    cmd = lane.command(False, ['--junitxml=result.xml'])
    assert cmd[:5] == [lane.sys.executable, '-m', 'pytest', '-q', 'tests']
    assert sum(v.startswith('--ignore=') for v in cmd) == 19
    assert cmd[-1] == '--junitxml=result.xml'


@pytest.mark.parametrize('extra', [['-k', 'one'], ['--collect-only'], ['tests/test_one.py'],
                                 ['-p', 'untrusted'], ['--ignore=tests/test_one.py'],
                                 ['--junitxml'], ['--basetemp=x', '--basetemp=y']])
def test_selection_and_plugin_overrides_rejected_before_launch(monkeypatch, extra):
    monkeypatch.setattr(lane.subprocess, 'run', lambda *a, **k: pytest.fail('forbidden launch'))
    assert lane.main(['--native'] + extra) == 2


def test_separate_output_values_and_clean_child_environment(monkeypatch):
    monkeypatch.setenv('PYTEST_ADDOPTS', '-k one')
    monkeypatch.setenv('PYTEST_PLUGINS', 'untrusted')
    monkeypatch.setattr(lane, 'discovery', lambda: dict.fromkeys(lane.PACKAGES, False))
    calls = []
    monkeypatch.setattr(lane.subprocess, 'run', lambda cmd, **kw: calls.append((cmd, kw)) or NS(returncode=0))
    assert lane.main(['--basetemp', 'work', '--junitxml', 'result.xml']) == 0
    assert calls[0][0][-4:] == ['--basetemp', 'work', '--junitxml', 'result.xml']
    assert all(n not in calls[0][1]['env'] and n not in lane.os.environ for n in lane.UNTRUSTED_ENV)


@pytest.mark.parametrize('present', lane.PACKAGES)
def test_each_discoverable_runtime_blocks_before_pytest(monkeypatch, present):
    monkeypatch.setattr(lane.importlib.util, 'find_spec', lambda n: NS() if n == present else None)
    monkeypatch.setattr(lane.subprocess, 'run', lambda *a, **k: pytest.fail('must not run tests'))
    assert lane.main(['--preflight']) == 2


def test_absent_runtime_preflight_does_not_launch(monkeypatch):
    monkeypatch.setattr(lane.importlib.util, 'find_spec', lambda n: None)
    monkeypatch.setattr(lane.subprocess, 'run', lambda *a, **k: pytest.fail('preflight must not launch'))
    assert lane.main(['--preflight']) == 0


def test_native_delegates_all_modules_and_requires_no_skips(monkeypatch):
    monkeypatch.setattr(lane, 'discovery', lambda: dict.fromkeys(lane.PACKAGES, True))
    calls = []
    monkeypatch.setattr(lane.subprocess, 'run', lambda cmd, **kw: calls.append((cmd, kw)) or NS(returncode=0))
    assert lane.main(['--native']) == 0
    cmd, kw = calls[0]
    assert cmd[0].endswith('/tools/test_perception_supervisor.sh')
    assert sum(s.startswith('tests/test_') for s in cmd) == 39
    assert kw['env']['SO101_REQUIRE_NATIVE_NO_SKIPS'] == '1'


@pytest.mark.parametrize('skips,expected', [([], 0), ([object()], 1)])
def test_native_skip_is_negative(monkeypatch, skips, expected):
    monkeypatch.setenv('SO101_REQUIRE_NATIVE_NO_SKIPS', '1')
    session = NS(exitstatus=0, config=NS(pluginmanager=NS(get_plugin=lambda n: NS(stats={'skipped': skips}))))
    lane.pytest_sessionfinish(session, 0)
    assert session.exitstatus == expected
