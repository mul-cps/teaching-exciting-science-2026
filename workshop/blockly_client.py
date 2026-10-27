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


def export_python(program, *, physical=True, offline=False):
    """An executable, syntax-safe JSON literal, not a second interpreter."""
    validate_program(program)
    if type(physical) is not bool or type(offline) is not bool:
        raise ValueError('explicit boolean physical and offline modes required')
    if physical and offline: raise ValueError('physical programs cannot use offline simulation')
    literal=repr(json.dumps(program,ensure_ascii=False,allow_nan=False))
    if offline:
        return ('# Synthetic offline simulation; no hardware or shared station connection.\n'
                'from workshop.blockly_client import simulate_program\nimport json\n\n'
                f'program = json.loads({literal})\n\n'
                '# Built-in demo is invented/synthetic, never a physical recording.\n'
                '# Other named strokes require separate synthetic resources or the shared station route.\n'
                "recordings = {'demo': {'schema': 'simulation-only'}}\n\n"
                'if __name__ == "__main__":\n'
                '    result = simulate_program(program, recordings=recordings)\n'
                '    print(json.dumps(result, ensure_ascii=False, indent=2))\n')
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
                error._station_program_verified_stop={'physical':physical,'stop_verified':True,'dispatched':True}
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
    import selectors
    import signal
    root=Path(__file__).resolve().parents[1]
    entry=root/'tools/run_station_program.py'
    body=memoryview(_json({'program':program}).encode('utf-8'))
    selector=selectors.DefaultSelector()
    try:
        process=subprocess.Popen(_launcher_prefix()+[str(entry),'--stdin-json',
                              '--physical' if physical else '--simulation','--timeout',str(timeout)],
                             stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,
                                 start_new_session=True)
    except BaseException:
        selector.close()
        raise
    output=[]; errors=[]
    def close_pipe(pipe):
        selector.unregister(pipe)
        pipe.close()
    def pump(seconds):
        nonlocal body
        for key,_ in selector.select(seconds):
            pipe=key.fileobj
            if key.data=='input':
                try:written=os.write(pipe.fileno(),body)
                except BrokenPipeError:close_pipe(pipe);continue
                body=body[written:]
                if not body:close_pipe(pipe)
            else:
                chunk=os.read(pipe.fileno(),65536)
                if chunk:(output if key.data=='output' else errors).append(chunk)
                else:close_pipe(pipe)
    def leader_exited():
        # Observe without reaping: keep this owned PID/PGID reserved until the
        # final group signal, even if the wrapper exits before its descendants.
        return os.waitid(os.P_PID,process.pid,os.WEXITED|os.WNOHANG|os.WNOWAIT) is not None
    def signal_group(value):
        try:os.killpg(process.pid,value)
        except ProcessLookupError:pass
    def drain(seconds):
        until=time.monotonic()+seconds
        while time.monotonic()<until:
            if leader_exited() and not selector.get_map():return True
            pump(min(.1,max(0.,until-time.monotonic())))
        return leader_exited() and not selector.get_map()
    primary=None; grace_failed=False; previous_interrupt=None
    def defer_interrupts():
        nonlocal previous_interrupt
        if previous_interrupt is None and threading.current_thread() is threading.main_thread():
            previous_interrupt=signal.signal(signal.SIGINT,signal.SIG_IGN)
    try:
        for pipe,label,event in ((process.stdin,'input',selectors.EVENT_WRITE),
                                 (process.stdout,'output',selectors.EVENT_READ),
                                 (process.stderr,'errors',selectors.EVENT_READ)):
            os.set_blocking(pipe.fileno(),False)
            selector.register(pipe,event,label)
        deadline=time.monotonic()+timeout+20
        try:
            while not (leader_exited() and not selector.get_map()):
                if cancel is not None and cancel.is_set():raise InterruptedError('ROS bridge cancelled')
                if time.monotonic()>=deadline:raise TimeoutError('ROS bridge deadline expired')
                pump(.1)
        except BaseException as error:
            primary=error
            defer_interrupts()
            # Signal the owned group, including the ROS child behind Bash/Conda.
            # Allow an in-flight 5s RPC, 8s STOP and 2s close, plus margin.
            signal_group(signal.SIGINT)
            grace_failed=not drain(17.)
        if grace_failed:
            signal_group(signal.SIGTERM)
            drain(2.)
        # The leader is still unreaped here. Reap only after the last signal so
        # a recycled PGID can never receive cleanup intended for this client.
        signal_group(signal.SIGKILL)
        if not drain(2.):raise RuntimeError('ROS bridge client cleanup incomplete; STOP unverified') from primary
        process.wait(timeout=2.)
    finally:
        # Also cover setup/read failures; defer repeated Ctrl-C until reaped.
        defer_interrupts()
        try:
            if process.returncode is None:
                signal_group(signal.SIGKILL)
                process.wait(timeout=2.)
        finally:
            for pipe in (process.stdin,process.stdout,process.stderr):pipe.close()
            selector.close()
            if previous_interrupt is not None:signal.signal(signal.SIGINT,previous_interrupt)
    stderr=b''.join(errors).decode('utf-8',errors='replace')
    if primary is not None:
        if grace_failed:raise RuntimeError('ROS bridge did not verify STOP; client process group reaped; inspect station ownership') from primary
        tag='SO101_STATION_PROGRAM_TERMINAL '
        receipts=[]
        def receipt_pairs(pairs):
            value=dict(pairs)
            if len(value)!=len(pairs):raise ValueError('duplicate terminal field')
            return value
        for line in stderr.splitlines():
            if line.startswith(tag):
                try:
                    if len(line)>512:raise ValueError('oversized terminal receipt')
                    receipts.append(json.loads(line[len(tag):],object_pairs_hook=receipt_pairs))
                except ValueError:receipts.append(None)
        expected={'schema':'so101-program-terminal-v1','physical':physical,
                  'stop_verified':True,'dispatched':True}
        strict_receipt=(len(receipts)==1 and isinstance(receipts[0],dict) and
                        all(type(receipts[0].get(key)) is bool for key in ('physical','stop_verified','dispatched')))
        if process.returncode!=1 or not strict_receipt or receipts!=[expected] or 'STOP unverified' in stderr:
            raise RuntimeError('ROS bridge STOP unverified: missing or invalid terminal receipt; '+stderr[-2000:]) from primary
        raise primary
    if process.returncode:raise RuntimeError('ROS program bridge failed: '+stderr[-2000:])
    return json.loads(b''.join(output).decode('utf-8'))
