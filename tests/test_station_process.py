"""Managed startup/shutdown without ROS, SSH or physical devices."""
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('station_process', ROOT/'tools/station_process.py')
station = importlib.util.module_from_spec(spec)
spec.loader.exec_module(station)


def test_duplicate_owner_refuses_before_prerequisites(tmp_path, monkeypatch):
    monkeypatch.setattr(station, 'duplicate_owners', lambda: ['123: trajectory_server'])
    monkeypatch.setattr(station, 'prerequisites', lambda *a: pytest.fail('prerequisites after duplicate'))
    with pytest.raises(RuntimeError, match='Existing station'):
        station.start(SimpleNamespace(), tmp_path/'record.json')


def test_failed_prerequisite_never_launches(tmp_path, monkeypatch):
    monkeypatch.setattr(station, 'duplicate_owners', lambda: [])
    monkeypatch.setattr(station, 'wrapper', lambda a: ['/missing-wrapper', 'run'])
    monkeypatch.setattr(station, 'prerequisites', lambda *a: (_ for _ in ()).throw(RuntimeError('missing device')))
    monkeypatch.setattr(station.subprocess, 'Popen', lambda *a, **k: pytest.fail('must not launch'))
    with pytest.raises(RuntimeError, match='missing device'):
        station.start(SimpleNamespace(), tmp_path/'record.json')


def test_stale_or_reused_pid_sends_no_rpc_or_signal(tmp_path, monkeypatch):
    monkeypatch.setattr(station, 'same_process', lambda r: False)
    monkeypatch.setattr(station, 'run', lambda *a, **k: pytest.fail('must not call ROS'))
    monkeypatch.setattr(station.os, 'killpg', lambda *a: pytest.fail('must not signal'))
    with pytest.raises(RuntimeError, match='identity'):
        station.stop_record(tmp_path/'record.json', {'process': {'pid': 123}})


def test_no_record_stop_is_idempotent_and_has_no_calls(tmp_path, monkeypatch):
    monkeypatch.setattr(station, 'run', lambda *a, **k: pytest.fail('must not call ROS'))
    for _ in range(2):
        station.main(['stop', '--record', str(tmp_path/'absent.json')])


def test_stop_calls_both_bounded_rpcs_even_when_first_times_out(tmp_path, monkeypatch):
    child = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(90)'], start_new_session=True)
    record = {'process': station.process_identity(child.pid), 'ros': ['/fake-wrapper', 'run'], 'environment': {}}
    path = tmp_path/'record.json'
    path.write_text(json.dumps(record))
    calls = []
    def fake_run(command, **kwargs):
        calls.append((command, kwargs['timeout']))
        raise subprocess.TimeoutExpired(command, kwargs['timeout'])
    monkeypatch.setattr(station, 'run', fake_run)
    try:
        station.stop_record(path, record)
        child.wait(timeout=3)
    finally:
        if child.poll() is None:
            os.killpg(child.pid, signal.SIGKILL)
            child.wait()
    assert [call[0][-3] for call in calls] == ['/station/stop', '/follower/stop']
    assert all(timeout == 8 for _, timeout in calls)
    assert not path.exists()


def test_record_identity_includes_start_time_and_own_group():
    child = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(90)'], start_new_session=True)
    try:
        record = station.process_identity(child.pid)
        assert station.same_process(record)
        assert not station.same_process({**record, 'starttime': 'wrong'})
        assert not station.same_process({**record, 'pgrp': child.pid+1})
    finally:
        os.killpg(child.pid, signal.SIGKILL)
        child.wait()


def test_start_tracks_group_connects_once_and_never_requests_motion(tmp_path, monkeypatch):
    contract = tmp_path/'limits.json'
    contract.write_text('{}')
    args = SimpleNamespace(profile='local', wrapper=None, wrapper_command=None,
        python=Path(sys.executable), config=tmp_path/'config.json', contract=contract,
        state_file=tmp_path/'state.json', station_lock=tmp_path/'station.lock',
        log_dir=tmp_path/'logs', bootstrap_owner=False, calibration_dir=tmp_path,
        camera_sdk=tmp_path, color_mask_backend='cpu', existing_tunnel=True, background=True)
    monkeypatch.setattr(station, 'duplicate_owners', lambda: [])
    monkeypatch.setattr(station, 'prerequisites', lambda *a: None)
    real_popen = subprocess.Popen
    commands = []
    def fake_popen(command, **kwargs):
        commands.append(command)
        return real_popen([sys.executable, '-c', 'import time; time.sleep(90)'], **kwargs)
    monkeypatch.setattr(station.subprocess, 'Popen', fake_popen)
    calls = []
    def fake_run(command, **kwargs):
        calls.append(command)
        return SimpleNamespace(stdout='success=True\n')
    monkeypatch.setattr(station, 'run', fake_run)
    path = tmp_path/'record.json'
    station.start(args, path)
    record = json.loads(path.read_text())
    try:
        assert station.same_process(record['process'])
        assert 'use_ev3_tunnel:=false' in commands[0]
        assert f'joint_limit_contract:={contract}' in commands[0]
        assert calls[0][-3] == '/station/ev3/connect'
        assert len(calls) == 1
        assert not any(item.endswith('/home') or item.endswith('/teleop_enable') for command in calls for item in command)
        assert not any('startup_observe_only:=false' in item for item in commands[0])
    finally:
        station.stop_record(path, record)


def test_passive_export_failure_never_launches(tmp_path, monkeypatch):
    args = SimpleNamespace(profile='local', wrapper=None, wrapper_command=None,
        python=Path(sys.executable), config=tmp_path/'config.json', contract=tmp_path/'limits.json',
        state_file=tmp_path/'state.json', station_lock=tmp_path/'station.lock',
        log_dir=tmp_path/'logs', bootstrap_owner=True)
    monkeypatch.setattr(station, 'duplicate_owners', lambda: [])
    monkeypatch.setattr(station, 'prerequisites', lambda *a: None)
    def fail_export(command, **kwargs):
        assert '--bootstrap-owner' in command
        assert kwargs['timeout'] == 40
        raise subprocess.CalledProcessError(2, command)
    monkeypatch.setattr(station, 'run', fail_export)
    monkeypatch.setattr(station.subprocess, 'Popen', lambda *a, **k: pytest.fail('must not launch'))
    with pytest.raises(subprocess.CalledProcessError):
        station.start(args, tmp_path/'record.json')


def _tree(tmp_path, *, ignore_term=False, exit_leader=False):
    """Harmless owned session with child/grandchild, including another pgrp."""
    ready = tmp_path/'tree.json'
    grandchild = ('import os,signal,time; os.setpgid(0,0); '
                  'signal.signal(signal.SIGINT,signal.SIG_IGN); '
                  + ('signal.signal(signal.SIGTERM,signal.SIG_IGN); ' if ignore_term else '') + 'time.sleep(90)')
    child = ('import os,signal,subprocess,sys,time; from pathlib import Path; '
             'signal.signal(signal.SIGINT,signal.SIG_IGN); '
             + ('signal.signal(signal.SIGTERM,signal.SIG_IGN); ' if ignore_term else '') +
             'g=subprocess.Popen([sys.executable,"-c",'+repr(grandchild)+']); '
             'Path('+repr(str(ready))+').write_text(str(os.getpid())+","+str(g.pid)); time.sleep(90)')
    parent = ('import subprocess,sys,time; from pathlib import Path; '
              'subprocess.Popen([sys.executable,"-c",'+repr(child)+']); '
              'time.sleep(90)')
    process = subprocess.Popen([sys.executable,'-c',parent],start_new_session=True,
                               stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    import time
    deadline=time.monotonic()+3
    while not ready.exists() and time.monotonic()<deadline: time.sleep(.01)
    assert ready.exists()
    identities=[station.process_identity(int(pid)) for pid in ready.read_text().split(',')]
    assert all(identities)
    # Give the grandchild time to install its own signal dispositions/group.
    time.sleep(.05)
    identities=[station.process_identity(member['pid']) for member in identities]
    return process,identities


def _clean_tree(process, identities):
    for identity in identities:
        if identity and station.process_identity(identity['pid'])==identity:
            station.signal_member(identity,signal.SIGKILL)
    if process.poll() is None:
        station.signal_member(station.process_identity(process.pid),signal.SIGKILL)
    process.wait(timeout=3)


def test_stop_escalates_verified_children_after_leader_exits(tmp_path, monkeypatch):
    process,members=_tree(tmp_path,ignore_term=True)
    record={'process':station.process_identity(process.pid),'ros':['fake-wrapper'],'environment':{}}
    path=tmp_path/'record.json'; path.write_text(json.dumps(record))
    monkeypatch.setattr(station,'run',lambda *a,**k:SimpleNamespace(stdout='success=True\n'))
    monkeypatch.setattr(station,'STOP_STEPS',((signal.SIGINT,.1),(signal.SIGTERM,.1),(signal.SIGKILL,.5)))
    try:
        station.stop_record(path,record)
        assert not path.exists()
        assert all(station.process_identity(member['pid']) is None for member in members)
    finally:
        _clean_tree(process,members)


def test_rpc_timeout_kills_wrapper_child_and_grandchild(tmp_path):
    ready=tmp_path/'rpc-tree.json'
    grandchild='import signal,time; signal.signal(signal.SIGTERM,signal.SIG_IGN); time.sleep(90)'
    child=('import os,signal,subprocess,sys,time; from pathlib import Path; '
           'signal.signal(signal.SIGTERM,signal.SIG_IGN); '
           'g=subprocess.Popen([sys.executable,"-c",'+repr(grandchild)+']); '
           'Path('+repr(str(ready))+').write_text(str(os.getpid())+","+str(g.pid)); time.sleep(90)')
    wrapper='import subprocess,sys,time; subprocess.Popen([sys.executable,"-c",'+repr(child)+']); time.sleep(90)'
    import time
    before=time.monotonic()
    try:
        with pytest.raises(subprocess.TimeoutExpired):
            station.run([sys.executable,'-c',wrapper],timeout=.3,
                        stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
        assert time.monotonic()-before<3
        assert ready.exists()
        assert all(station.process_identity(int(pid)) is None for pid in ready.read_text().split(','))
    finally:
        if ready.exists():
            for pid in ready.read_text().split(','):
                identity=station.process_identity(int(pid))
                if identity:station.signal_member(identity,signal.SIGKILL)


def test_pidfd_identity_recheck_never_signals_reused_pid(monkeypatch):
    identity={'pid':123,'pgrp':123,'session':123,'starttime':'old'}
    monkeypatch.setattr(station,'open_pidfd',lambda pid:42)
    monkeypatch.setattr(station,'process_identity',lambda pid:{**identity,'starttime':'new'})
    monkeypatch.setattr(station,'send_pidfd',lambda *a:pytest.fail('must not signal reused PID'))
    closed=[];monkeypatch.setattr(station.os,'close',closed.append)
    station.signal_member(identity,signal.SIGTERM)
    assert closed==[42]


def test_unverifiable_survivors_retain_ownership_and_send_no_signal(tmp_path,monkeypatch):
    identity={'pid':123,'pgrp':123,'session':123,'starttime':'old'}
    survivor={'pid':456,'pgrp':123,'session':123,'starttime':'new'}
    record={'process':identity,'members':[identity],'ros':['fake'],'environment':{}}
    path=tmp_path/'record.json';path.write_text(json.dumps(record))
    monkeypatch.setattr(station,'same_process',lambda value:True)
    monkeypatch.setattr(station,'process_identity',lambda *a,**k:None)
    monkeypatch.setattr(station,'session_members',lambda session:[survivor])
    monkeypatch.setattr(station,'run',lambda *a,**k:pytest.fail('unverified identity must not call ROS'))
    monkeypatch.setattr(station,'signal_member',lambda *a:pytest.fail('unverified identity must not signal'))
    with pytest.raises(RuntimeError,match='unverifiable'):
        station.stop_record(path,record)
    assert path.exists()
    assert json.loads(path.read_text())['unverified_members']==[survivor]


def test_bounded_run_preserves_output_and_nonzero_returncode():
    result=station.run([sys.executable,'-c','print("wrapper result")'],capture_output=True,text=True,timeout=2)
    assert result.stdout=='wrapper result\n' and result.returncode==0
    with pytest.raises(subprocess.CalledProcessError) as error:
        station.run([sys.executable,'-c','import sys; print("rejected"); sys.exit(3)'],
                    capture_output=True,text=True,timeout=2)
    assert error.value.returncode==3 and error.value.stdout=='rejected\n'


def test_stop_can_resume_saved_descendants_after_leader_already_exited(tmp_path,monkeypatch):
    process,members=_tree(tmp_path)
    identity=station.process_identity(process.pid)
    record={'process':identity,'members':[identity]+members,'ros':['fake'],'environment':{}}
    path=tmp_path/'record.json';path.write_text(json.dumps(record))
    monkeypatch.setattr(station,'run',lambda *a,**k:SimpleNamespace(stdout='success=True\n'))
    monkeypatch.setattr(station,'STOP_STEPS',((signal.SIGINT,.05),(signal.SIGTERM,.2),(signal.SIGKILL,.5)))
    try:
        station.signal_member(identity,signal.SIGTERM);process.wait(timeout=2)
        station.stop_record(path,record)
        assert not path.exists()
        assert all(station.process_identity(member['pid']) is None for member in members)
    finally:
        _clean_tree(process,members)


def test_known_descendant_session_change_is_retained_as_unknown(monkeypatch):
    identity={'pid':123,'pgrp':123,'session':123,'starttime':'known'}
    changed={**identity,'session':456,'pgrp':456}
    monkeypatch.setattr(station,'process_identity',lambda *a,**k:changed)
    record={'process':identity,'members':[identity]}
    with pytest.raises(RuntimeError,match='changed session'):
        station.refresh_members(record)
    assert record['unverified_members']==[changed]


def test_rpc_cleanup_runtimeerror_still_stops_follower_and_managed_tree(tmp_path,monkeypatch):
    process,members=_tree(tmp_path,ignore_term=True)
    identity=station.process_identity(process.pid)
    record={'process':identity,'ros':['fake-wrapper','run'],'environment':{}}
    path=tmp_path/'record.json';path.write_text(json.dumps(record))
    calls=[]
    unknown='RPC cleanup incomplete; ownership retained in /tmp/unknown-rpc-survivors.json'
    def fake_run(command,**kwargs):
        calls.append(command[-3])
        if command[-3]=='/station/stop':
            raise RuntimeError(unknown)
        return SimpleNamespace(stdout='success=True\n')
    monkeypatch.setattr(station,'run',fake_run)
    monkeypatch.setattr(station,'STOP_STEPS',((signal.SIGINT,.05),(signal.SIGTERM,.05),(signal.SIGKILL,.5)))
    try:
        with pytest.raises(RuntimeError,match='RPC cleanup incomplete; managed session stopped'):
            station.stop_record(path,record)
        assert calls==['/station/stop','/follower/stop']
        assert station.process_identity(identity['pid']) is None
        assert all(station.process_identity(member['pid']) is None for member in members)
        saved=json.loads(path.read_text())
        assert saved['managed_cleanup_complete'] is True
        assert saved['rpc_cleanup_errors']==[{'service':'/station/stop','error':unknown}]
        assert saved['members']==[]
    finally:
        _clean_tree(process,members)


@pytest.mark.parametrize('retained_marker', ('rpc_cleanup_errors', 'unverified_members'))
def test_foreground_interrupt_preserves_uncertainty_record_after_managed_exit(tmp_path,monkeypatch,retained_marker):
    contract=tmp_path/'limits.json';contract.write_text('{}')
    args=SimpleNamespace(profile='local',wrapper=None,wrapper_command=None,
        python=Path(sys.executable),config=tmp_path/'config.json',contract=contract,
        state_file=tmp_path/'state.json',station_lock=tmp_path/'station.lock',
        log_dir=tmp_path/'logs',bootstrap_owner=False,calibration_dir=tmp_path,
        camera_sdk=tmp_path,color_mask_backend='cpu',existing_tunnel=True,background=False)
    monkeypatch.setattr(station,'duplicate_owners',lambda:[])
    monkeypatch.setattr(station,'prerequisites',lambda *a:None)
    real_popen=subprocess.Popen
    children=[]
    class InterruptedForeground:
        def __init__(self,child):self.child=child;self.pid=child.pid
        def wait(self):raise KeyboardInterrupt
        def poll(self):return self.child.poll()
    def fake_popen(command,**kwargs):
        child=real_popen([sys.executable,'-c','import time; time.sleep(90)'],**kwargs)
        children.append(child)
        return InterruptedForeground(child)
    monkeypatch.setattr(station.subprocess,'Popen',fake_popen)
    calls=[]
    unknown='RPC cleanup incomplete; ownership retained in /tmp/foreground-rpc-survivors.json'
    def fake_run(command,**kwargs):
        calls.append(command[-3])
        if command[-3]=='/station/stop' and retained_marker=='rpc_cleanup_errors':raise RuntimeError(unknown)
        return SimpleNamespace(stdout='success=True\n')
    monkeypatch.setattr(station,'run',fake_run)
    if retained_marker=='unverified_members':
        real_stop=station.stop_record
        def retain_unverified(path,record):
            real_stop(path,record)
            # Simulate a retained escaped/unanchored descendant observation
            # after the original managed session is empty. No PID is signaled.
            record['unverified_members']=[{**record['process'],'session':record['process']['session']+1}]
            record['managed_cleanup_complete']=True
            station.save(path,record)
            raise RuntimeError('Managed session contains unverifiable survivors; retained ownership record')
        monkeypatch.setattr(station,'stop_record',retain_unverified)
    monkeypatch.setattr(station,'STOP_STEPS',((signal.SIGINT,.2),(signal.SIGTERM,.2),(signal.SIGKILL,.5)))
    path=tmp_path/'record.json'
    try:
        expected='RPC cleanup incomplete; managed session stopped' if retained_marker=='rpc_cleanup_errors' else 'unverifiable survivors'
        with pytest.raises(RuntimeError,match=expected):
            station.start(args,path)
        assert calls==['/station/ev3/connect','/station/stop','/follower/stop']
        saved=json.loads(path.read_text())
        assert saved['managed_cleanup_complete'] is True
        if retained_marker=='rpc_cleanup_errors':
            assert saved['rpc_cleanup_errors']==[{'service':'/station/stop','error':unknown}]
        else:
            assert saved['unverified_members']
        assert station.process_identity(children[0].pid) is None
    finally:
        for child in children:
            identity=station.process_identity(child.pid)
            if identity:station.signal_member(identity,signal.SIGKILL)
            child.wait(timeout=3)
