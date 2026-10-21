#!/usr/bin/env python3
"""Serve the offline Blockly station. Simulation unless --physical is explicit."""
import argparse
import hmac
import json
import os
from pathlib import Path
import secrets
import socket
import signal
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from workshop.blockly_program import ProgramRunner, SimulationBackend, name, validate_program
from workshop.blockly_recording import Recorder, recording_timing


class Application:
    def __init__(self, directory, *, physical=False, instructor=False):
        self.physical_lock=None
        if physical:
            import fcntl
            lock_path=Path.home()/'.local/state/exciting-science/blockly-physical.lock'
            lock_path.parent.mkdir(parents=True,exist_ok=True)
            self.physical_lock=lock_path.open('a')
            try: fcntl.flock(self.physical_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            except OSError:
                self.physical_lock.close(); raise RuntimeError('another physical Blockly adapter already owns the program lock')
        self.directory=Path(directory); self.directory.mkdir(parents=True,exist_ok=True)
        self.recordings={}
        for index,path in enumerate(self.directory.glob('*.json')):
            if index>=64: break
            if path.stat().st_size<=250000:
                try:
                    data=strict_json(path.read_text()); name(data['name'])
                    from workshop.blockly_recording import replay_points
                    replay_points(data,data['identity'],.5)
                    self.recordings[data['name']]=data
                except (ValueError,KeyError,TypeError): pass
        if not physical:
            self.recordings.setdefault('demo',{'schema':'simulation-only'})
        self.recording_info={name:recording_timing(value) for name,value in self.recordings.items() if value.get('points')}
        if physical:
            from workshop.blockly_ros import ROSBackend
            self.backend=ROSBackend(self.recordings)
        else: self.backend=SimulationBackend(self.recordings)
        self.instructor=instructor; self.token=secrets.token_urlsafe(32)
        self.lock=threading.RLock(); self.cancel=threading.Event(); self.job={'state':'idle'}
        self.thread=None; self.recorder=None; self.record_thread=None; self.record_cancel=threading.Event()
        self.record_error=None; self.operator_busy=False; self.recovery_required=False

    def status(self):
        with self.lock:
            return {'job':dict(self.job),'recording':self.recorder.name if self.recorder else None,
                    'record_error':self.record_error,'recordings':sorted(self.recordings),'recording_info':dict(self.recording_info),
                    'recovery_required':self.recovery_required,'instructor':self.instructor,'operator_busy':self.operator_busy,**self.backend.status()}

    def idle(self,*,recovery=False):
        if self.recovery_required and not recovery: raise RuntimeError('unknown/failed physical operation: inspect and recover explicitly')
        if self.thread and self.thread.is_alive(): raise RuntimeError('program still active')
        if self.recorder: raise RuntimeError('finish recording first')
        if self.operator_busy: raise RuntimeError('instructor operation active')

    def _check_recordings(self,rows):
        missing=set()
        def walk(commands):
            for row in commands:
                if row['op']=='replay' and row['name'] not in self.recordings:
                    missing.add(row['name'])
                elif row['op']=='repeat': walk(row['body'])
                elif row['op']=='if_color':
                    walk(row['then']); walk(row['else'])
        walk(rows)
        if missing: raise ValueError('recording not found: '+', '.join(sorted(missing)))

    def run(self,program):
        validate_program(program)
        with self.lock:
            self.idle()
            self._check_recordings(program)
            self.cancel=threading.Event(); job=secrets.token_hex(8)
            self.job={'id':job,'state':'running'}
            def work():
                try:
                    result=ProgramRunner(self.backend).execute(program,self.cancel)
                    terminal={'id':job,'state':'complete','result':result}
                except BaseException as error:
                    terminal={'id':job,'state':'failed','error':str(error)[:2000]}
                    with self.lock: self.recovery_required=self.backend.physical
                with self.lock: self.job=terminal
            self.thread=threading.Thread(target=work,daemon=True); self.thread.start()
            return {'id':job,'state':'running'}

    def record_start(self,recording_name):
        with self.lock:
            self.idle(); name(recording_name)
            if len(self.recordings)>=64: raise ValueError('recording library limit 64; archive old recordings while server is stopped')
            if recording_name in self.recordings: raise ValueError('choose a new recording name; existing strokes are preserved')
            if not self.backend.physical: raise RuntimeError('recording requires explicit physical mode; simulation has a demo stroke')
            self.operator_busy=True
        try:
            identity=self.backend.identity(); recorder=Recorder(recording_name,identity)
            with self.lock:
                self.recorder=recorder; self.record_error=None; self.record_cancel=threading.Event()
                def capture():
                    last=None
                    try:
                        while not self.record_cancel.wait(.1):
                            sample=self.backend.sample()
                            if sample['identity']!=identity: raise RuntimeError('owner generation/source changed during recording')
                            if sample['stamp']==last: raise RuntimeError('follower joint feedback stopped advancing')
                            recorder.add(sample['positions'],sample['jaw'],sample['stamp']); last=sample['stamp']
                    except Exception as error:
                        with self.lock: self.record_error=str(error); recorder.error=str(error)
                self.record_thread=threading.Thread(target=capture,daemon=True); self.record_thread.start()
                return {'recording':recording_name,'read_only':True}
        finally:
            with self.lock: self.operator_busy=False

    def record_stop(self):
        with self.lock:
            if self.recorder is None: raise RuntimeError('no recording active')
            recorder=self.recorder; thread=self.record_thread; self.record_cancel.set()
        thread.join(timeout=2)
        if thread.is_alive(): raise RuntimeError('recording collector did not stop')
        with self.lock: self.recorder=None; self.operator_busy=True
        try:
            result=recorder.finish()
            if self.backend.identity()!=result['identity']: raise RuntimeError('follower identity changed during recording')
            path=self.directory/(result['name']+'.json')
            with path.open('x') as stream: json.dump(result,stream,allow_nan=False)
            with self.lock:
                self.recordings[result['name']]=result
                self.recording_info[result['name']]=recording_timing(result)
            return {'name':result['name'],'samples':len(result['points'])}
        finally:
            with self.lock: self.operator_busy=False

    def stop(self):
        self.cancel.set(); self.record_cancel.set()
        try: return self.backend.stop()
        except BaseException:
            with self.lock: self.recovery_required=self.backend.physical
            raise

    def operator(self,command):
        if not self.instructor or not self.backend.physical: raise RuntimeError('instructor controls require --physical --instructor')
        with self.lock: self.idle(recovery=True); self.operator_busy=True
        try:
            if self.recovery_required and command not in {'zero_torque','emergency_reset','reset','restart_moveit','restart_camera','restart_depth','restart_tags','restart_all','verify_restart'}:
                raise RuntimeError('motion/mode changes blocked until explicit recovery')
            return self.backend.instructor(command)
        except BaseException:
            with self.lock: self.recovery_required=True
            try: self.backend.stop()
            except Exception: pass
            raise
        finally:
            with self.lock: self.operator_busy=False

    def recover(self):
        if not self.instructor or not self.backend.physical: raise RuntimeError('instructor physical recovery required')
        with self.lock: self.idle(recovery=True); self.operator_busy=True
        try:
            result=self.backend.recover()
            with self.lock: self.recovery_required=False
            return result
        finally:
            with self.lock: self.operator_busy=False

    def close(self):
        self.cancel.set(); self.record_cancel.set()
        if self.backend.physical:
            try: self.backend.stop()
            except Exception as error: print('STOP acknowledgement missing: '+str(error),file=sys.stderr)
        if self.thread: self.thread.join(timeout=16)
        if self.record_thread: self.record_thread.join(timeout=2)
        self.backend.close()
        if self.physical_lock: self.physical_lock.close()


def strict_json(body):
    def pairs(items):
        result={}
        for key,value in items:
            if key in result: raise ValueError('duplicate JSON field')
            result[key]=value
        return result
    def invalid(_): raise ValueError('nonfinite JSON value')
    return json.loads(body,object_pairs_hook=pairs,parse_constant=invalid)


class BoundedServer(ThreadingHTTPServer):
    daemon_threads=True
    def __init__(self,*args,**kwargs):
        self.slots=threading.BoundedSemaphore(8)
        super().__init__(*args,**kwargs)
    def process_request(self,request,address):
        if not self.slots.acquire(blocking=False):
            self.shutdown_request(request); return
        try: super().process_request(request,address)
        except BaseException:
            self.slots.release(); raise
    def process_request_thread(self,request,address):
        try: super().process_request_thread(request,address)
        finally: self.slots.release()
    def get_request(self):
        request,address=super().get_request(); request.settimeout(2)
        return request,address


def make_server(app,port,*,control_only=False,frontend_port=None):
    assets=ROOT/'web/blockly-station'
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*_): pass
        def send_json(self,status,data):
            body=json.dumps(data,allow_nan=False).encode(); self.send_response(status)
            self.send_header('Content-Type','application/json'); self.send_header('Content-Length',str(len(body)))
            self.send_header('Cache-Control','no-store'); self.end_headers(); self.wfile.write(body)
        def do_OPTIONS(self):
            if not self.valid_host(): self.send_json(403,{'error':'invalid host'}); return
            origin=self.headers.get('Origin')
            if origin!=f'http://127.0.0.1:{frontend_port}': self.send_json(403,{'error':'invalid origin'}); return
            self.send_response(204)
            self.send_header('Access-Control-Allow-Headers','X-Station-Token, Content-Type')
            self.send_header('Access-Control-Allow-Methods','POST'); self.end_headers()
        def end_headers(self):
            if control_only and self.headers.get('Origin')==f'http://127.0.0.1:{frontend_port}':
                self.send_header('Access-Control-Allow-Origin',self.headers['Origin'])
            super().end_headers()
        def valid_host(self): return self.headers.get('Host') in {f'127.0.0.1:{self.server.server_port}',f'localhost:{self.server.server_port}'}
        def authenticated(self):
            origin=self.headers.get('Origin')
            allowed={f'http://127.0.0.1:{self.server.server_port}',f'http://localhost:{self.server.server_port}'}
            if control_only: allowed.add(f'http://127.0.0.1:{frontend_port}')
            return self.valid_host() and (origin is None or origin in allowed) and hmac.compare_digest(self.headers.get('X-Station-Token',''),app.token)
        def do_GET(self):
            if control_only: self.send_json(404,{'error':'STOP listener only'}); return
            if not self.valid_host(): self.send_json(403,{'error':'loopback host required'}); return
            path=urlsplit(self.path).path
            if path.startswith('/api/'):
                if not self.authenticated(): self.send_json(403,{'error':'station token required'}); return
                if path=='/api/status': self.send_json(200,app.status())
                elif path=='/api/recordings': self.send_json(200,{'names':sorted(app.recordings)})
                else: self.send_json(404,{'error':'unknown endpoint'})
                return
            target=(assets/('index.html' if path=='/' else path.lstrip('/'))).resolve()
            if not target.is_relative_to(assets) or not target.is_file(): self.send_json(404,{'error':'not found'}); return
            body=target.read_bytes()
            if target.name=='index.html': body=body.replace(b'__STATION_TOKEN__',app.token.encode())
            types={'.html':'text/html','.js':'text/javascript','.css':'text/css','.svg':'image/svg+xml','.png':'image/png'}
            self.send_response(200); self.send_header('Content-Type',types.get(target.suffix,'application/octet-stream'))
            self.send_header('Content-Length',str(len(body))); self.send_header('Cache-Control','no-store')
            self.send_header('X-Content-Type-Options','nosniff'); self.end_headers(); self.wfile.write(body)
        def do_POST(self):
            if not self.authenticated(): self.send_json(403,{'error':'station token or origin rejected'}); return
            try:
                size=int(self.headers.get('Content-Length','0'))
                if not 0<size<=262144: raise ValueError('request size must be 1..262144 bytes')
                payload=strict_json(self.rfile.read(size))
                if not isinstance(payload,dict): raise ValueError('request must be an object')
                path=urlsplit(self.path).path
                if control_only and path!='/api/program/stop': raise ValueError('STOP listener only')
                if path=='/api/program/run' and set(payload)=={'program'}: result=app.run(payload['program'])
                elif path=='/api/program/stop' and not payload: result=app.stop()
                elif path=='/api/record/start' and set(payload)=={'name'}: result=app.record_start(payload['name'])
                elif path=='/api/record/stop' and not payload: result=app.record_stop()
                elif path=='/api/simulation/color' and set(payload)=={'color'}:
                    from workshop.blockly_program import COLORS
                    if app.backend.physical or not isinstance(payload['color'],str) or payload['color'] not in COLORS: raise ValueError('simulation color is unavailable or invalid')
                    app.backend.observed_color=payload['color']; result={'color':payload['color']}
                elif path=='/api/recover' and not payload: result=app.recover()
                elif path=='/api/instructor' and set(payload)=={'command'}: result=app.operator(payload['command'])
                else: raise ValueError('unsupported endpoint or fields')
                self.send_json(200,result)
            except (ValueError,RuntimeError,KeyError,TimeoutError,OSError,RecursionError) as error: self.send_json(400,{'error':str(error)[:2000]})
    server=BoundedServer(('127.0.0.1',port),Handler); server.daemon_threads=True
    return server


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--physical',action='store_true'); parser.add_argument('--instructor',action='store_true')
    parser.add_argument('--port',type=int,default=8877)
    parser.add_argument('--recordings',type=Path,default=Path.home()/'.local/share/exciting-science/blockly-recordings')
    args=parser.parse_args()
    app=Application(args.recordings,physical=args.physical,instructor=args.instructor)
    server=make_server(app,args.port)
    control=make_server(app,args.port+1,control_only=True,frontend_port=server.server_port)
    control_thread=threading.Thread(target=control.serve_forever,daemon=True); control_thread.start()
    def stop(*_):
        threading.Thread(target=server.shutdown,daemon=True).start()
        threading.Thread(target=control.shutdown,daemon=True).start()
    signal.signal(signal.SIGINT,stop); signal.signal(signal.SIGTERM,stop)
    print(f'Blockly {"PHYSICAL" if args.physical else "SIMULATION"}: http://127.0.0.1:{server.server_port}',flush=True)
    try: server.serve_forever(poll_interval=.1)
    finally: control.shutdown(); control.server_close(); server.server_close(); app.close()

if __name__=='__main__': main()
