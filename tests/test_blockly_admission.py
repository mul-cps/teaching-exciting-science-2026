"""Late delivery cannot admit an old command after STOP or process restart."""
import pytest
from tools.run_blockly_station import Application

PROGRAM=[{'op':'log','message':'late'}]

def test_stop_invalidates_delayed_run(tmp_path):
    app=Application(tmp_path)
    stamp=app.status()['admission']
    app.stop()
    with pytest.raises(RuntimeError,match='admission'):
        app.run(PROGRAM,admission=stamp)
    assert app.thread is None
    assert 'late' not in app.backend.events

def test_restart_invalidates_old_session(tmp_path):
    first=Application(tmp_path/'first'); second=Application(tmp_path/'second')
    with pytest.raises(RuntimeError,match='admission'):
        second.run(PROGRAM,admission=first.status()['admission'])

@pytest.mark.parametrize('stamp',[None,{}, {'session':'x','epoch':True}, {'session':1,'epoch':0}, {'session':'x','epoch':-1}])
def test_external_admission_required(tmp_path,stamp):
    app=Application(tmp_path)
    with pytest.raises((ValueError,RuntimeError)):
        app.run_external(PROGRAM,stamp)
    assert app.thread is None


def test_http_requires_same_admission_envelope(tmp_path):
    import json
    import threading
    import urllib.request
    from urllib.error import HTTPError
    from tools.run_blockly_station import make_server
    app=Application(tmp_path);server=make_server(app,0)
    thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    root=f'http://127.0.0.1:{server.server_port}'
    def run(body):
        request=urllib.request.Request(root+'/api/program/run',data=json.dumps(body).encode(),headers={'X-Station-Token':app.token})
        return urllib.request.urlopen(request)
    try:
        with pytest.raises(HTTPError):run({'program':PROGRAM})
        stamp=app.status()['admission'];app.stop()
        with pytest.raises(HTTPError):run({'program':PROGRAM,'admission':stamp})
        assert app.thread is None
        with run({'program':PROGRAM,'admission':app.status()['admission']}) as response:
            assert json.load(response)['state']=='running'
        app.thread.join(2)
    finally:
        server.shutdown();server.server_close();app.close()
