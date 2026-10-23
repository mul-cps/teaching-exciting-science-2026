"""Shared Python client integration, isolated domain177; no physical devices."""
import json, os, sys, tempfile, threading, time
from pathlib import Path
if os.environ.get('ROS_DOMAIN_ID') != '177':
    raise SystemExit('Use isolated ROS_DOMAIN_ID=177')
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from tools.run_blockly_station import Application, strict_json
from workshop.blockly_ros_api import ROSProgramAPI
from workshop.blockly_client import Program, run_program, simulate_program, export_python
program=Program().log('Ready café').repeat(2,Program().ev3('eject').ev3('left_end')).program
expected=simulate_program(program)
with tempfile.TemporaryDirectory() as directory:
    app=Application(Path(directory)); api=ROSProgramAPI(app,strict_json)
    try:
        result=run_program(program,physical=False,timeout=10,wrapper_bridge=False)
        assert result['complete'] is True
        assert app.backend.events==expected['events'], (app.backend.events,expected)
        exported=export_python(program,physical=False)
        exec(compile(exported,'exported-student.py','exec'),{'__name__':'__main__'})
        assert app.backend.events==expected['events']*2
        cancel=threading.Event()
        timer=threading.Timer(.4,cancel.set);timer.start()
        try:
            try: run_program(Program().wait(10),physical=False,timeout=10,cancel=cancel,wrapper_bridge=False)
            except InterruptedError: pass
            else: raise AssertionError('cancel was ignored')
        finally: timer.cancel()
        assert app.backend.events[-1]=='STOP'
        print(json.dumps({'native_service':True,'same_trace':True,'executable_export':True,'cancel_STOP':True,'hardware':False,'domain':177}))
    finally:
        api.close();app.close()
