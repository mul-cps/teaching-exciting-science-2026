"""Shared Python client integration, isolated domain177; no physical devices."""
import argparse, json, os, sys, tempfile, threading, time
from pathlib import Path
if os.environ.get('ROS_DOMAIN_ID') != '177':
    raise SystemExit('Use isolated ROS_DOMAIN_ID=177')
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from tools.run_blockly_station import Application, strict_json
from workshop.blockly_ros_api import ROSProgramAPI
from workshop.blockly_client import Program, run_program, simulate_program, export_python
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--wrapper-overlay',type=Path,help='Isolated generated workflow-interface install prefix')
args=parser.parse_args()

def check_wrapper_clients(app, directory, overlay):
    """Real no-ROS outer Python -> Conda wrapper -> real ROS service client."""
    import signal
    import subprocess
    import resource
    root = Path(__file__).resolve().parents[1]
    overlay = Path(overlay).resolve(strict=True)
    assert (overlay/'local_setup.bash').is_file(), overlay
    pid_file = Path(directory)/'wrapper-client-pids'
    launcher = [str(root/'ros2/conda/so101-conda.sh'), 'run', 'bash',
                '--noprofile', '--norc', '-c',
                'source "$1/local_setup.bash"; printf "%s\\n" "$$" >> "$2"; '
                'shift 2; exec python "$@"', 'fixture-client', str(overlay), str(pid_file)]
    environment = dict(os.environ, STATION_PROGRAM_LAUNCHER=json.dumps(launcher))
    outer_code = r'''
import importlib.util,json,sys,threading
assert importlib.util.find_spec('rclpy') is None, 'outer interpreter unexpectedly has ROS'
sys.path.insert(0,sys.argv[1])
from workshop.blockly_client import run_program,export_python
mode=sys.argv[2]; program=json.loads(sys.argv[3])
print(json.dumps({'outer_no_ros':True,'mode':mode}),flush=True)
if mode=='export':
    exec(compile(export_python(program,physical=False),'wrapper-export.py','exec'),{'__name__':'__main__'})
    print(json.dumps({'export_complete':True}),flush=True)
elif mode=='cancel':
    cancel=threading.Event()
    def control():
        assert sys.stdin.readline().strip()=='cancel'
        cancel.set()
    control_thread=threading.Thread(target=control,daemon=True);control_thread.start()
    try:run_program(program,physical=False,timeout=25,cancel=cancel,wrapper_bridge=True)
    except InterruptedError:
        assert cancel.is_set()
        control_thread.join(2);assert not control_thread.is_alive()
        print(json.dumps({'cancel_interrupted':True}),flush=True)
    else:raise AssertionError('wrapper cancellation ignored')
elif mode=='mismatch':
    try:run_program(program,physical=True,timeout=20,wrapper_bridge=True)
    except RuntimeError as error:
        assert 'mode differs' in str(error),str(error)
        print(json.dumps({'mode_rejected':True}),flush=True)
    else:raise AssertionError('wrong physical mode accepted')
else:
    result=run_program(program,physical=False,timeout=20,wrapper_bridge=True)
    assert result['complete'] is True
    print(json.dumps({'run_complete':True}),flush=True)
'''
    def group_members(group):
        members=[]
        for entry in Path('/proc').iterdir():
            if not entry.name.isdigit():continue
            try:fields=(entry/'stat').read_text().rsplit(')',1)[1].split()
            except (OSError,IndexError):continue
            if int(fields[2])==group:members.append(int(entry.name))
        return members
    receipts=[]
    cancel_triggered=[]
    def outer(mode, rows, admitted=None):
        started=time.monotonic()
        self_before=resource.getrusage(resource.RUSAGE_SELF)
        children_before=resource.getrusage(resource.RUSAGE_CHILDREN)
        process=subprocess.Popen([sys.executable,'-I','-S','-c',outer_code,str(root),mode,json.dumps(rows)],
            env=environment,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,
            text=True,start_new_session=True)
        try:
            if admitted is not None:admitted(process)
            output, errors=process.communicate(timeout=40)
            assert process.returncode==0,(mode,process.returncode,output,errors)
            messages=[json.loads(line) for line in output.splitlines()]
            assert messages[0]=={'outer_no_ros':True,'mode':mode},messages
            assert messages[-1]=={dict(run='run_complete',export='export_complete',cancel='cancel_interrupted',mismatch='mode_rejected')[mode]:True},messages
            deadline=time.monotonic()+3
            while group_members(process.pid) and time.monotonic()<deadline:time.sleep(.02)
            assert group_members(process.pid)==[],(mode,'unreaped process group',group_members(process.pid))
            self_after=resource.getrusage(resource.RUSAGE_SELF)
            children_after=resource.getrusage(resource.RUSAGE_CHILDREN)
            receipts.append({'mode':mode,'outer_pid':process.pid,'exit':process.returncode,
                'outer_no_ros':True,'group_empty_after_reap':True,
                'wall_seconds':time.monotonic()-started,
                'self_CPU_seconds':self_after.ru_utime+self_after.ru_stime-self_before.ru_utime-self_before.ru_stime,
                'reaped_children_CPU_seconds':children_after.ru_utime+children_after.ru_stime-children_before.ru_utime-children_before.ru_stime,
                'cancel_trigger_to_reap_seconds':time.monotonic()-cancel_triggered[0] if mode=='cancel' else None})
            print(json.dumps({'wrapper_process':receipts[-1]}),flush=True)
        finally:
            if process.poll() is None:
                try:os.killpg(process.pid,signal.SIGKILL)
                except ProcessLookupError:pass
            process.communicate(timeout=5)
    app.thread.join(timeout=2)
    assert not app.thread.is_alive()
    baseline=list(app.backend.events)
    outer('run',program)
    assert app.backend.events==baseline+expected['events'],app.backend.events
    outer('export',program)
    assert app.backend.events==baseline+expected['events']*2,app.backend.events
    baseline=list(app.backend.events);old_job=app.status()['job']['id']
    old_epoch=app.status()['admission']['epoch'];cancel_job=[]
    marker='wrapper-cancel-admitted-'+str(time.monotonic_ns())
    def admitted(process):
        deadline=time.monotonic()+15
        while time.monotonic()<deadline:
            status=app.status();job=status['job']
            if job.get('id')!=old_job and job['state']=='running' and marker in app.backend.events:
                assert app.thread.is_alive();cancel_job.append(job['id']);break
            assert process.poll() is None,'outer exited before new wait was admitted'
            time.sleep(.01)
        assert cancel_job,'new wrapper long job was never admitted'
        print(json.dumps({'wrapper_cancel_new_job':cancel_job[0],
            'new_wait_marker':marker,'pre_cancel_epoch':old_epoch,
            'outer_process_group':process.pid,'process_group_members':group_members(process.pid)}),flush=True)
        cancel_triggered.append(time.monotonic())
        process.stdin.write('cancel\n');process.stdin.flush()
    outer('cancel',[{'op':'log','message':marker},{'op':'wait','seconds':10},
                    {'op':'log','message':'forbidden-after-wrapper-cancel'}],admitted)
    app.thread.join(timeout=3)
    status=app.status()
    assert status['job']['id']==cancel_job[0] and status['job']['state']=='failed',status
    assert 'cancel' in status['job']['error'] and not app.thread.is_alive(),status
    delta=app.backend.events[len(baseline):]
    assert delta[0]==marker and delta.count('STOP')>=1 and 'forbidden-after-wrapper-cancel' not in delta,delta
    assert app.simulation_stop_verified and status['admission']['epoch']>old_epoch,status
    before=(list(app.backend.events),dict(status['job']),dict(status['admission']))
    outer('mismatch',program)
    status=app.status()
    assert before==(list(app.backend.events),dict(status['job']),dict(status['admission']))
    pids=[int(line) for line in pid_file.read_text().splitlines()]
    assert len(pids)==4 and len(set(pids))==4,pids
    assert not any(Path('/proc',str(pid)).exists() for pid in pids),pids
    print(json.dumps({'wrapper_native_service':True,'real_no_ros_outer':True,
        'same_trace':True,'executable_export':True,'new_cancel_job':cancel_job[0],
        'distinct_terminal_STOP':True,'no_active_job':True,'physical_mode_mismatch_no_mutation':True,
        'wrapper_client_pids':pids,'processes':receipts,'hardware':False,'simulation':True,'domain':177}))

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
        if args.wrapper_overlay is not None:
            check_wrapper_clients(app,directory,args.wrapper_overlay)
    finally:
        api.close();app.close()
