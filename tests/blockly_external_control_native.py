"""Real ROS callbacks, synthetic backend: STOP defeats delayed external controls."""
import json,os,sys,tempfile,threading
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
if os.environ.get('ROS_DOMAIN_ID')!='177':raise SystemExit('Use isolated ROS_DOMAIN_ID=177')
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from tools.run_blockly_station import Application,strict_json
from workshop.blockly_ros_api import ROSProgramAPI
from workshop.blockly_client import ROSProgramTransport

with tempfile.TemporaryDirectory() as directory:
    app=Application(directory);api=ROSProgramAPI(app,strict_json)
    client=ROSProgramTransport();stop_client=ROSProgramTransport()
    app.backend.physical=True;app.instructor=True
    delivered=[]
    try:
        def call(transport,operation,args):return transport(operation,json.dumps(args),4.)
        original=app.control_external;entered=threading.Event();release=threading.Event()
        def delayed(*args,**kwargs):entered.set();assert release.wait(3);return original(*args,**kwargs)
        app.control_external=delayed
        stamp=app.status()['admission']
        with ThreadPoolExecutor(max_workers=1) as pool:
            pending=pool.submit(call,client,'operator',{'command':'start','admission':stamp})
            assert entered.wait(2)
            assert call(stop_client,'stop',{})['acknowledged']
            release.set()
            try:pending.result(3)
            except RuntimeError as error:assert 'admission' in str(error),error
            else:raise AssertionError('stale operator accepted')
        app.control_external=original;entered.clear();release.clear()
        def instructor(command,*,cancel):
            entered.set();assert release.wait(3)
            if cancel.is_set():raise InterruptedError('cancelled before backend delivery')
            delivered.append(command);return {'ok':True}
        app.backend.instructor=instructor
        with ThreadPoolExecutor(max_workers=1) as pool:
            pending=pool.submit(call,client,'operator',{'command':'start','admission':app.status()['admission']})
            assert entered.wait(2)
            assert call(stop_client,'stop',{})['acknowledged']
            release.set()
            try:pending.result(3)
            except RuntimeError as error:assert 'cancel' in str(error),error
            else:raise AssertionError('cancelled admitted operator accepted')
        assert delivered==[]
        assert app.recovery_required
        print(json.dumps({'native_service':True,'delayed_callback_rejected':True,'admitted_delivery_cancelled':True,'unstamped_STOP':True,'hardware':False,'domain':177}))
    finally:
        release.set();client.close();stop_client.close();api.close();app.backend.physical=False;app.close()
