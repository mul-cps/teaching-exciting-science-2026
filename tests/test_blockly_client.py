"""Owned wrapper subprocess/terminal receipt checks, without ROS or devices."""
import json
import os
from pathlib import Path
import selectors
import signal
import subprocess
import sys
import threading
import time

import pytest
from workshop import blockly_client as client

ROWS=[{'op':'log','message':'Grüße'}]
RECEIPT={'schema':'so101-program-terminal-v1','physical':False,'stop_verified':True,'dispatched':True}
TAG='SO101_STATION_PROGRAM_TERMINAL '


def launcher(tmp_path, monkeypatch, body):
    script=tmp_path/'client.py'
    script.write_text('import json,os,signal,sys,time\nfrom pathlib import Path\n'+body)
    monkeypatch.setenv('STATION_PROGRAM_LAUNCHER',json.dumps([sys.executable,'-u',str(script)]))


def cancel_on_ready(path):
    cancel=threading.Event()
    def ready():
        until=time.monotonic()+5
        while not path.exists() and time.monotonic()<until:time.sleep(.005)
        cancel.set()
    thread=threading.Thread(target=ready);thread.start()
    return cancel,thread


def assert_reaped(pid):
    with pytest.raises(ChildProcessError):os.waitpid(pid,os.WNOHANG)
    assert not Path('/proc',str(pid)).exists()


def test_wrapper_normal_json_and_pid_pinned_through_final_group_signal(tmp_path,monkeypatch):
    pid_file=tmp_path/'pid'
    launcher(tmp_path,monkeypatch,f"Path({str(pid_file)!r}).write_text(str(os.getpid()))\nprint(json.dumps(json.load(sys.stdin)))\n")
    calls=[];real=os.killpg
    def kill(group,value):
        assert group!=os.getpgrp()
        observed=os.waitid(os.P_PID,group,os.WEXITED|os.WNOHANG|os.WNOWAIT)
        calls.append((value,observed is not None));real(group,value)
    monkeypatch.setattr(os,'killpg',kill)
    assert client._wrapper_run(ROWS,False,1)=={'program':ROWS}
    assert calls==[(signal.SIGKILL,True)]
    assert_reaped(int(pid_file.read_text()))


def test_partial_pipe_setup_failure_kills_and_reaps_owned_client(tmp_path,monkeypatch):
    launcher(tmp_path,monkeypatch,'time.sleep(30)\n')
    real_selector=selectors.DefaultSelector
    class BrokenSelector:
        def __init__(self):self.inner=real_selector()
        def register(self,*args):raise OSError('synthetic registration failure')
        def close(self):self.inner.close()
    monkeypatch.setattr(selectors,'DefaultSelector',BrokenSelector)
    spawned=[];real_popen=subprocess.Popen
    def popen(*args,**kwargs):
        p=real_popen(*args,**kwargs);spawned.append(p);return p
    monkeypatch.setattr(subprocess,'Popen',popen)
    with pytest.raises(OSError,match='registration failure'):client._wrapper_run(ROWS,False,1)
    assert len(spawned)==1 and spawned[0].returncode is not None
    assert all(p.closed for p in (spawned[0].stdin,spawned[0].stdout,spawned[0].stderr))
    assert_reaped(spawned[0].pid)


@pytest.mark.parametrize('receipt,exit_code',[
    ('',0),('',1),('',7),('',-signal.SIGKILL),
    (TAG+'not-json',1),(TAG+json.dumps(RECEIPT)+'\n'+TAG+json.dumps(RECEIPT),1),
    (TAG+json.dumps({**RECEIPT,'physical':True}),1),
    (TAG+json.dumps({**RECEIPT,'stop_verified':1}),1),
    (TAG+'{"schema":"so101-program-terminal-v1","physical":false,"stop_verified":false,"stop_verified":true,"dispatched":true}',1),
    (TAG+json.dumps(RECEIPT),0),
])
def test_cancel_without_one_strict_matching_receipt_is_unverified(tmp_path,monkeypatch,receipt,exit_code):
    ready=tmp_path/'ready';pid_file=tmp_path/'pid'
    ending=f'os.kill(os.getpid(),{signal.SIGKILL})' if exit_code<0 else f'raise SystemExit({exit_code})'
    launcher(tmp_path,monkeypatch,f'''json.load(sys.stdin)
def interrupted(*_):
    print({receipt!r},file=sys.stderr,flush=True)
    {ending}
signal.signal(signal.SIGINT,interrupted)
Path({str(pid_file)!r}).write_text(str(os.getpid()))
Path({str(ready)!r}).touch()
while True:time.sleep(.01)
''')
    cancel,thread=cancel_on_ready(ready)
    try:
        with pytest.raises(RuntimeError,match='STOP unverified'):client._wrapper_run(ROWS,False,1,cancel=cancel)
    finally:thread.join(6)
    assert_reaped(int(pid_file.read_text()))


def test_verified_receipt_preserves_original_cancel_and_defers_repeated_interrupt(tmp_path,monkeypatch):
    ready=tmp_path/'ready';pid_file=tmp_path/'pid'
    launcher(tmp_path,monkeypatch,f'''json.load(sys.stdin)
def interrupted(*_):
    print({(TAG+json.dumps(RECEIPT))!r},file=sys.stderr,flush=True)
    raise SystemExit(1)
signal.signal(signal.SIGINT,interrupted)
Path({str(pid_file)!r}).write_text(str(os.getpid()))
Path({str(ready)!r}).touch()
while True:time.sleep(.01)
''')
    cancel,thread=cancel_on_ready(ready);real=os.killpg;handler=signal.getsignal(signal.SIGINT)
    def kill(group,value):
        real(group,value)
        if value==signal.SIGINT:os.kill(os.getpid(),signal.SIGINT)
    monkeypatch.setattr(os,'killpg',kill)
    try:
        with pytest.raises(InterruptedError,match='ROS bridge cancelled'):client._wrapper_run(ROWS,False,1,cancel=cancel)
    finally:thread.join(6)
    assert signal.getsignal(signal.SIGINT)==handler
    assert_reaped(int(pid_file.read_text()))


def test_unresponsive_owned_client_escalates_and_reaps_without_stop_claim(tmp_path,monkeypatch):
    ready=tmp_path/'ready';pid_file=tmp_path/'pid'
    launcher(tmp_path,monkeypatch,f'''json.load(sys.stdin)
signal.signal(signal.SIGINT,signal.SIG_IGN)
signal.signal(signal.SIGTERM,signal.SIG_IGN)
Path({str(pid_file)!r}).write_text(str(os.getpid()))
Path({str(ready)!r}).touch()
while True:time.sleep(.01)
''')
    cancel,thread=cancel_on_ready(ready);calls=[];real=os.killpg
    def kill(group,value):calls.append(value);real(group,value)
    monkeypatch.setattr(os,'killpg',kill)
    try:
        with pytest.raises(RuntimeError,match='did not verify STOP'):client._wrapper_run(ROWS,False,1,cancel=cancel)
    finally:thread.join(6)
    assert calls==[signal.SIGINT,signal.SIGTERM,signal.SIGKILL]
    assert_reaped(int(pid_file.read_text()))


@pytest.mark.parametrize('stop_verified',[False,True])
def test_original_interruption_metadata_and_cli_receipt_require_verified_stop(monkeypatch,capsys,stop_verified):
    import io
    from tools import run_station_program as cli
    cancel=threading.Event();calls=[]
    def transport(operation,body,timeout):
        calls.append(operation)
        if operation=='status':return {'physical':False,'admission':{'session':'session','epoch':0}}
        if operation=='run':cancel.set();return {'id':'exact','state':'running'}
        return {'acknowledged':stop_verified,'simulation':True}
    def execute(*args,**kwargs):
        try:client.run_program(ROWS,physical=False,transport=transport,cancel=cancel)
        except InterruptedError as error:
            assert getattr(error,'_station_program_verified_stop',None)=={'physical':False,'stop_verified':True,'dispatched':True}
            raise
        except RuntimeError as error:
            assert not hasattr(error,'_station_program_verified_stop')
            raise
    monkeypatch.setattr(cli,'run_program',execute)
    monkeypatch.setattr(sys,'argv',['run_station_program.py','--stdin-json','--simulation'])
    monkeypatch.setattr(sys,'stdin',io.StringIO(json.dumps({'program':ROWS})))
    assert cli.main()==1
    assert calls==['status','run','stop']
    tagged=[x for x in capsys.readouterr().err.splitlines() if x.startswith(TAG)]
    assert tagged==([TAG+json.dumps(RECEIPT,separators=(',',':'))] if stop_verified else [])
