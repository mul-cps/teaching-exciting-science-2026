"""External controls retain STOP ordering through admission and ROS delivery."""
import threading
from concurrent.futures import ThreadPoolExecutor
import pytest
from tools.run_blockly_station import Application

@pytest.fixture
def app(tmp_path):
    app=Application(tmp_path);app.backend.physical=True;app.instructor=True
    yield app
    app.backend.physical=False;app.close()

@pytest.mark.parametrize('operation,args',[('operator',{'command':'start'}),('recover',{}),('record_start',{'name':'move'}),('record_stop',{}),('simulation_color',{'color':'red'})])
def test_pre_stop_stamp_never_dispatches_external_control(app,operation,args):
    stamp=app.status()['admission'];app.stop()
    with pytest.raises(RuntimeError,match='admission'):
        app.control_external(operation,args,stamp)
    assert app.recorder is None
    assert app.operator_busy is False

@pytest.mark.parametrize('stamp',[None,{}, {'session':'valid','epoch':True},{'session':'valid','epoch':-1},{'session':'x'*129,'epoch':0},{'session':'valid','epoch':2**63}])
def test_malformed_stamp_never_dispatches(app,stamp):
    app.backend.instructor=lambda *a,**kw:pytest.fail('dispatched')
    with pytest.raises(ValueError,match='admission'):
        app.control_external('operator',{'command':'start'},stamp)


def test_stop_cancels_admitted_operator_before_backend_delivery(app):
    entered=threading.Event();release=threading.Event();delivered=[]
    def instructor(command,*,cancel):
        entered.set();assert release.wait(2)
        if cancel.is_set():raise InterruptedError('cancelled before delivery')
        delivered.append(command)
    app.backend.instructor=instructor
    with ThreadPoolExecutor(max_workers=1) as pool:
        future=pool.submit(app.control_external,'operator',{'command':'start'},app.status()['admission'])
        assert entered.wait(2);app.stop();release.set()
        with pytest.raises(InterruptedError):future.result(2)
    assert delivered==[]
    assert app.recovery_required is True


def test_closing_cancellation_is_composed_with_stop_cancellation(app):
    closing=threading.Event();closing.set()
    app.backend.instructor=lambda *a,**kw:pytest.fail('dispatched')
    with pytest.raises(InterruptedError):
        app.control_external('operator',{'command':'start'},app.status()['admission'],cancel=closing)


def test_stop_pending_blocks_new_control_even_with_fresh_stamp(app):
    entered=threading.Event();release=threading.Event()
    def stop():entered.set();assert release.wait(2);return {'acknowledged':True,'simulation':True}
    app.backend.stop=stop
    with ThreadPoolExecutor(max_workers=1) as pool:
        future=pool.submit(app.stop);assert entered.wait(2)
        with pytest.raises(RuntimeError,match='STOP'):
            app.control_external('operator',{'command':'start'},app.status()['admission'])
        release.set();future.result(2)


def test_restart_session_never_dispatches_old_control(app,tmp_path):
    old=Application(tmp_path/'old').status()['admission']
    with pytest.raises(RuntimeError,match='admission'):
        app.control_external('operator',{'command':'start'},old)


def test_http_delayed_operator_requires_stamp_and_stop_still_works(app):
    import json,urllib.request
    from urllib.error import HTTPError
    from tools.run_blockly_station import make_server
    server=make_server(app,0);worker=threading.Thread(target=server.serve_forever,daemon=True);worker.start()
    root=f'http://127.0.0.1:{server.server_port}'
    calls=[];app.backend.instructor=lambda *args,**kw:calls.append(args)
    def request(path,body):
        return urllib.request.Request(root+'/api/'+path,data=json.dumps(body).encode(),headers={'X-Station-Token':app.token})
    delayed=request('instructor',{'command':'start','admission':app.status()['admission']})
    try:
        with urllib.request.urlopen(request('program/stop',{})) as response:assert response.status==200
        for mutation in (delayed,request('instructor',{'command':'start'})):
            with pytest.raises(HTTPError):urllib.request.urlopen(mutation)
        assert calls==[]
    finally:server.shutdown();server.server_close();worker.join(2)
