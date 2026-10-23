"""Readable Python programs submitted once to the shared station Application.

Importing this module needs no ROS. A notebook outside the ROS environment uses
this checkout's wrapper to start a client process, never a second robot owner.
"""
import copy
import json
import math
import os
from pathlib import Path
import subprocess
import threading
import time
from .blockly_program import validate_program
from .blockly_ros_api import MAX_JSON_BYTES, SERVICE


class Program:
    """Build the same bounded AST Blockly sends; loops run on the server."""
    def __init__(self): self._rows=[]
    @property
    def program(self): return copy.deepcopy(validate_program(self._rows))
    def add(self, op, **fields):
        rows=self._rows+[{'op':op,**fields}]
        validate_program(rows); self._rows=rows
        return self
    def wait(self, seconds): return self.add('wait',seconds=seconds)
    def log(self, message): return self.add('log',message=message)
    def ev3(self, command): return self.add('ev3',command=command)
    def station(self, command): return self.add('station',command=command)
    def arm(self, command): return self.add('arm',command=command)
    def jaw(self, gap_mm): return self.add('jaw',gap_mm=gap_mm)
    def record_stroke(self, name, seconds): return self.add('record_stroke',name=name,seconds=seconds)
    def replay(self, name, speed=.5, timing='original'): return self.add('replay',name=name,speed=speed,timing=timing)
    def approach_recording(self, name, speed=.5): return self.add('approach_recording',name=name,speed=speed)
    def pick_place_recording(self, name, speed=.5): return self.add('pick_place_recording',name=name,speed=speed)
    def sort_cycle(self): return self.add('sort_cycle')
    def stop(self): return self.add('stop')
    def repeat(self, count, body): return self.add('repeat',count=count,body=body.program if isinstance(body,Program) else body)
    def if_color(self, color, then, otherwise):
        return self.add('if_color',color=color,then=then.program if isinstance(then,Program) else then,
                        **{'else':otherwise.program if isinstance(otherwise,Program) else otherwise})


def simulate_program(program, *, recordings=None, cancel=None):
    """Explicit in-process simulation for an offline notebook; never opens ROS."""
    from .blockly_program import ProgramRunner, SimulationBackend
    if isinstance(program,Program): program=program.program
    backend=SimulationBackend(recordings)
    result=ProgramRunner(backend).execute(program,cancel or threading.Event())
    return {**result,'simulation':True,'events':list(backend.events),'status':backend.status()}


def export_python(program, *, physical=True):
    """An executable, syntax-safe JSON literal, not a second interpreter."""
    validate_program(program)
    literal=repr(json.dumps(program,ensure_ascii=False,allow_nan=False))
    return ('from workshop.blockly_client import run_program\nimport json\n\n'
            f'program = json.loads({literal})\n\n'
            'if __name__ == "__main__":\n'
            f'    run_program(program, physical={physical!r})\n')


def _json(body):
    encoded=json.dumps(body,allow_nan=False,separators=(',',':'))
    if not 0<len(encoded.encode('utf-8'))<=MAX_JSON_BYTES: raise ValueError('arguments_json exceeds 65536 UTF-8 bytes')
    return encoded


def _verified_stop(reply, physical):
    if not isinstance(reply,dict): return False
    if physical:
        return (isinstance(reply.get('request_id'),str) and bool(reply['request_id']) and
                reply.get('pending') is False and reply.get('ev3_stopped') is True and reply.get('arm_disarmed') is True)
    return reply.get('acknowledged') is True and reply.get('simulation') is True


def run_program(program, *, physical=True, transport=None, timeout=190., cancel=None,
                clock=time.monotonic, poll_interval=.1, wrapper_bridge=True):
    """Submit once, correlate completion, and verify STOP on uncertain outcomes.

    ``physical=False`` explicitly selects a simulation endpoint. Transport is a
    callable (operation, arguments_json, timeout_seconds) returning a dict.
    """
    if isinstance(program,Program): program=program.program
    validate_program(program); _json({'program':program})
    if (type(physical) is not bool or type(timeout) not in (int,float) or not 0<timeout<=600
            or type(poll_interval) not in (int,float) or not math.isfinite(poll_interval) or not 0<poll_interval<=5):
        raise ValueError('explicit mode, bounded timeout and positive poll interval required')
    cancel=cancel or threading.Event()
    owned=None
    if transport is None:
        try: owned=ROSProgramTransport()
        except ImportError:
            if not wrapper_bridge: raise RuntimeError('ROS client unavailable; launch through ros2/conda/so101-conda.sh')
            return _wrapper_run(program,physical,timeout,cancel=cancel)
        transport=owned
    cancel=cancel or threading.Event(); deadline=clock()+timeout
    def call(op,body):
        remaining=deadline-clock()
        if remaining<=0: raise TimeoutError('station program deadline expired')
        return transport(op,_json(body),min(5.,remaining))
    dispatched=False
    try:
        status=call('status',{})
        if not isinstance(status,dict) or status.get('physical') is not physical:
            raise RuntimeError('station endpoint mode differs from requested mode')
        admission=status.get('admission')
        if (not isinstance(admission,dict) or set(admission)!={'session','epoch'}
                or not isinstance(admission['session'],str) or not 1<=len(admission['session'])<=128
                or type(admission['epoch']) is not int or admission['epoch']<0):
            raise RuntimeError('station admission stamp unavailable')
        if cancel.is_set(): raise InterruptedError('cancelled before dispatch')
        dispatched=True  # Unknown run delivery still requires STOP, never retry.
        reply=call('run',{'program':program,'admission':admission})
        job=reply.get('id') if isinstance(reply,dict) else None
        if not isinstance(job,str) or not 1<=len(job)<=128: raise RuntimeError('run acknowledgement lacks exact job ID')
        while True:
            if cancel.is_set(): raise InterruptedError('station program cancelled')
            snapshot=call('status',{})
            if not isinstance(snapshot,dict) or snapshot.get('physical') is not physical:
                raise RuntimeError('station endpoint mode changed')
            state=snapshot.get('job')
            if not isinstance(state,dict) or state.get('id')!=job: raise RuntimeError('station job identity changed or unavailable')
            if state.get('state')=='complete':
                result=state.get('result')
                if not isinstance(result,dict) or result.get('complete') is not True: raise RuntimeError('station result incomplete')
                return result
            if state.get('state')=='stopped':
                result=state.get('result')
                if not isinstance(result,dict) or result.get('complete') is not False or result.get('stopped') is not True:
                    raise RuntimeError('station STOP result incomplete')
                return result
            if state.get('state')!='running': raise RuntimeError('station program failed: '+str(state.get('error',state.get('state'))))
            cancel.wait(min(poll_interval,max(0.,deadline-clock())))
    except BaseException as error:
        if dispatched:
            try:
                stop=transport('stop',_json({}),8.)
                if not _verified_stop(stop,physical): raise RuntimeError('terminal correlated STOP acknowledgement missing')
            except BaseException as stop_error:
                raise RuntimeError(f'{error}; STOP unverified: {stop_error}') from error
        raise
    finally:
        if owned is not None: owned.close()


class ROSProgramTransport:
    """Only a service client. Does not instantiate ROSBackend or Application."""
    def __init__(self):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from so101_station_workflow.srv import StationCommand
        self.ros=rclpy; self.context=Context(); self.node=None; self.executor=None
        try:
            rclpy.init(context=self.context)
            self.node=Node('station_python_program_client',context=self.context)
            self.executor=SingleThreadedExecutor(context=self.context)
            self.executor.add_node(self.node)
            self.service_type=StationCommand
            self.client=self.node.create_client(StationCommand,SERVICE)
        except BaseException:
            self.close(); raise
    def __call__(self, operation, body, timeout):
        deadline=time.monotonic()+timeout
        if not self.client.wait_for_service(timeout_sec=timeout): raise RuntimeError(SERVICE+' unavailable; start the shared station server')
        request=self.service_type.Request(); request.operation=operation; request.arguments_json=body
        future=self.client.call_async(request)
        while not future.done():
            remaining=deadline-time.monotonic()
            if remaining<=0: raise TimeoutError('station response deadline; delivery may be unknown')
            self.executor.spin_once(timeout_sec=min(.05,remaining))
        response=future.result()
        if response is None or response.success is not True: raise RuntimeError('station command failed: '+str(getattr(response,'message','missing response')))
        if len(response.result_json.encode('utf-8'))>MAX_JSON_BYTES: raise RuntimeError('station response too large')
        from tools.run_blockly_station import strict_json
        result=strict_json(response.result_json)
        if not isinstance(result,dict): raise RuntimeError('station response must be an object')
        return result
    def close(self):
        if self.executor is not None: self.executor.shutdown(timeout_sec=2.)
        if self.node is not None: self.node.destroy_node()
        if self.context.ok(): self.ros.shutdown(context=self.context)


def _launcher_prefix():
    configured=os.environ.get('STATION_PROGRAM_LAUNCHER')
    if configured is None:
        return [str(Path(__file__).resolve().parents[1]/'ros2/conda/so101-conda.sh'),'run','python']
    prefix=json.loads(configured)
    if (not isinstance(prefix,list) or not prefix or len(prefix)>16 or
            any(not isinstance(value,str) or not value or '\0' in value for value in prefix)):
        raise ValueError('STATION_PROGRAM_LAUNCHER must be a JSON argv prefix')
    prefix[0]=str(Path(prefix[0]).expanduser())
    return prefix


def _wrapper_run(program,physical,timeout,*,cancel=None):
    root=Path(__file__).resolve().parents[1]
    entry=root/'tools/run_station_program.py'
    process=subprocess.Popen(_launcher_prefix()+[str(entry),'--stdin-json',
                              '--physical' if physical else '--simulation','--timeout',str(timeout)],
                             stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
    deadline=time.monotonic()+timeout+20
    body=_json({'program':program})
    try:
        while True:
            if cancel is not None and cancel.is_set(): raise InterruptedError('ROS bridge cancelled')
            if time.monotonic()>=deadline: raise TimeoutError('ROS bridge deadline expired')
            try:
                output,errors=process.communicate(body,timeout=.1)
                break
            except subprocess.TimeoutExpired: body=None
    except BaseException as error:
        # SIGINT lets the child run the same terminal STOP verification path.
        # A missing/failed child response stays explicit; never resubmit a run.
        import signal
        process.send_signal(signal.SIGINT)
        try: _,errors=process.communicate(timeout=12)
        except subprocess.TimeoutExpired: raise RuntimeError('ROS bridge did not verify STOP; retain station ownership and inspect') from error
        if 'STOP unverified' in errors: raise RuntimeError(errors[-2000:]) from error
        raise
    if process.returncode: raise RuntimeError('ROS program bridge failed: '+errors[-2000:])
    return json.loads(output)
