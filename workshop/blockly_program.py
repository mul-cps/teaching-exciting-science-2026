"""Bounded Blockly station programs; no generated code is executed."""
import math
import re
import threading
import time

NAME = re.compile(r'^[A-Za-z][A-Za-z0-9_-]{0,39}$')
STATION = frozenset({'emergency_reset','reset','start','base_release','base_confirm','teleop',
                     'teleop_relative','teleop_absolute','moveit','damping','zero_torque',
                     'restart_moveit','restart_camera','restart_depth','restart_tags','restart_all',
                     'verify_restart','reinitialize','verify_recovery'})
RECOVERY_STATION = frozenset({'reinitialize','verify_recovery','emergency_reset','reset','zero_torque',
                             'restart_moveit','restart_camera','restart_depth','restart_tags',
                             'restart_all','verify_restart'})
ARM = frozenset({'home', 'moveit', 'teleop_relative', 'teleop_absolute', 'torque_off'})
EV3 = {'home', 'left_end', 'right_end', 'eject', 'wake', 'sleep', 'connect', 'reset_fault'}
COLORS = {'red', 'blue', 'green', 'yellow', 'unknown'}


def number(value, low, high):
    if type(value) not in (float, int) or not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f'number must be finite in {low}..{high}')
    return value


def name(value):
    if not isinstance(value, str) or not NAME.fullmatch(value):
        raise ValueError('name must start with a letter; use letters, numbers, _ and - only')
    return value


def validate_program(program):
    """Reject extra fields, deep nesting and >100 expanded commands."""
    def walk(rows, depth=0):
        if depth > 4 or not isinstance(rows, list) or len(rows) > 100:
            raise ValueError('program must be a list, maximum nesting 4 and 100 commands')
        count = 0
        for row in rows:
            if not isinstance(row, dict): raise ValueError('command must be an object')
            op = row.get('op')
            fields = {'wait':{'seconds'}, 'ev3':{'command'}, 'replay':{'name','speed'},
                      'station':{'command'}, 'arm':{'command'}, 'record_stroke':{'name','seconds'}, 'jaw':{'gap_mm'}, 'repeat':{'count','body'},
                      'if_color':{'color','then','else'}, 'stop':set(), 'sort_cycle':set(), 'log':{'message'}}
            if not isinstance(op,str) or op not in fields or (set(row) != fields[op] | {'op'} and not (op=='replay' and set(row)==fields[op] | {'op','timing'})):
                raise ValueError('unknown command or unsupported fields')
            count += 1
            if op == 'wait': number(row['seconds'],0,10)
            elif op == 'ev3':
                if not isinstance(row['command'],str) or row['command'] not in EV3: raise ValueError('unknown EV3 command')
            elif op == 'station':
                if not isinstance(row['command'],str) or row['command'] not in STATION: raise ValueError('unknown station command')
            elif op == 'arm':
                if not isinstance(row['command'],str) or row['command'] not in ARM: raise ValueError('unknown arm command')
            elif op == 'record_stroke':
                name(row['name']); number(row['seconds'],1,30)
            elif op == 'replay':
                name(row['name']); number(row['speed'],.1,1)
                if row.get('timing','original') not in ('original','skip_stationary_pauses'): raise ValueError('unknown replay timing mode')
            elif op == 'jaw': number(row['gap_mm'],0,200)
            elif op == 'log':
                if not isinstance(row['message'],str) or len(row['message']) > 200: raise ValueError('message too long')
            elif op == 'repeat':
                if type(row['count']) is not int or not 1 <= row['count'] <= 5: raise ValueError('repeat must be 1..5')
                count += row['count'] * walk(row['body'],depth+1)
            elif op == 'if_color':
                if not isinstance(row['color'],str) or row['color'] not in COLORS: raise ValueError('unknown color')
                count += max(walk(row['then'],depth+1),walk(row['else'],depth+1))
            if count > 100: raise ValueError('program exceeds 100 expanded commands')
        return count
    if not walk(program): raise ValueError('connect at least one block')
    return program


class _ProgramStopped(Exception):
    """Internal control flow after an intentionally verified STOP."""


class ProgramRunner:
    def __init__(self, backend, *, clock=time.monotonic, dispatch=None):
        self.backend, self.clock = backend, clock
        self.dispatch = dispatch if dispatch is not None else backend.command

    def execute(self, program, cancel):
        validate_program(program)
        deadline = self.clock()+180
        sort_results=[]
        def check():
            if cancel.is_set(): raise InterruptedError('program cancelled')
            if self.clock() >= deadline: raise TimeoutError('180 second program deadline')
        def run(rows):
            for row in rows:
                check(); op=row['op']
                if op == 'repeat':
                    for _ in range(row['count']): run(row['body'])
                elif op == 'if_color': run(row['then'] if self.backend.color() == row['color'] else row['else'])
                elif op == 'wait':
                    end=min(deadline,self.clock()+row['seconds'])
                    while self.clock()<end:
                        check(); cancel.wait(min(.05,end-self.clock()))
                elif op == 'stop':
                    reply=self.backend.stop()
                    if self.backend.physical:
                        acknowledged=(isinstance(reply,dict) and isinstance(reply.get('request_id'),str)
                                      and bool(reply['request_id']) and reply.get('pending') is False
                                      and reply.get('ev3_stopped') is True and reply.get('arm_disarmed') is True)
                    else:
                        acknowledged=isinstance(reply,dict) and reply.get('acknowledged') is True
                    if not acknowledged: raise RuntimeError('STOP acknowledgement missing')
                    check()
                    raise _ProgramStopped()
                elif op == 'log': self.backend.log(row['message'])
                elif op == 'sort_cycle': sort_results.append(self.dispatch(row,cancel,deadline))
                else: self.dispatch(row, cancel, deadline)
                check()
        try:
            run(program)
            result={'complete':True}
            if sort_results: result['sort_cycles']=sort_results
            return result
        except _ProgramStopped:
            return {'complete':False,'stopped':True}
        except BaseException:
            try: self.backend.stop()
            except Exception as error: raise RuntimeError(f'program failed; STOP acknowledgement missing: {error}')
            raise


class SimulationBackend:
    """Explicit simulator: never imports ROS or accesses physical devices."""
    physical=False
    def __init__(self, recordings=None):
        self.events=[]; self.recordings=recordings if recordings is not None else {}; self.observed_color='red'
    def color(self): return self.observed_color
    def log(self,message): self.events.append(message); self.events=self.events[-100:]
    def identity(self): return {'arm_id':'simulation','calibration_sha256':'0'*64}
    def sample(self):
        return {'identity':self.identity(),'positions':[0.]*5,'jaw':0.,'stamp':time.monotonic()}
    def command(self,row,cancel,deadline):
        if row['op']=='record_stroke': raise ValueError('record_stroke requires Application dispatch')
        if row['op']=='station':
            self.log('station:'+row['command']+' (simulation)')
            if row['command'] in {'reinitialize','verify_recovery'}: return self.recover(cancel=cancel,deadline=deadline)
            return {'acknowledged':True,'simulation':True}
        if row['op']=='replay' and row['name'] not in self.recordings: raise RuntimeError('recording not found')
        if row['op']=='sort_cycle':
            color=self.color()
            if color not in {'red','blue','green','yellow'}: raise RuntimeError('simulation has no commissioned destination for unknown color')
            result={'completed':1,'colors':{color:1}}
            self.log('sort_cycle: '+color+' → commissioned destination (simulation)')
            return result
        self.log(row['op']+':'+str(row.get('command',row.get('name',row.get('gap_mm')))))
    def recover(self,*,cancel=None,deadline=None):
        if cancel is not None and cancel.is_set(): raise InterruptedError('simulation recovery cancelled')
        if deadline is not None and time.monotonic()>=deadline: raise TimeoutError('simulation recovery deadline expired')
        self.log('station recovery verified (simulation)'); return {'recovered':True,'simulation':True}
    def stop(self): self.log('STOP'); return {'acknowledged':True,'simulation':True}
    def status(self): return {'physical':False,'color':self.color(),'events':self.events[-20:]}
    def close(self): pass
