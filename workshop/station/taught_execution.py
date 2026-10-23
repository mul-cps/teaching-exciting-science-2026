"""One existing grasp CLI session, supervised by the shared Application owner."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading
import time
from workshop.blockly_skills import validate_skill_result
from workshop.station.ros_sorting import StableTargetSelector

ROOT=Path(__file__).resolve().parents[2]


class TaughtSession:
    def __init__(self,backend,library):
        from std_msgs.msg import String
        self.backend,self.library=backend,library
        self.lock=threading.Lock(); self.stop_lock=threading.Lock(); self.child_stop_requested=False; self.child=None; self.stopped=False; self.shutdown_error=None; self.terminated=False; self.lifetime_w=None
        self.status=None; self.received=0.
        self.subscription=backend.node.create_subscription(String,'/so101/grasp_status',self.observe,10)

    def observe(self,msg):
        if not isinstance(msg.data,str) or len(msg.data)>32768: return
        try: status=json.loads(msg.data)
        except (ValueError,TypeError,RecursionError): return
        if (not isinstance(status,dict) or status.get('state')!='tracking' or
                not isinstance(status.get('objects'),list) or len(status['objects'])>64 or
                not isinstance(status.get('grasp_candidates'),list) or len(status['grasp_candidates'])>128): return
        with self.lock: self.status=status; self.received=time.monotonic()

    def request_stop(self):
        """Stop producing goals immediately without delaying resident STOP."""
        with self.lock:
            self.stopped=True; child=self.child
            lifetime_w=self.lifetime_w; self.lifetime_w=None
            signal_child=child is not None and child.poll() is None and not self.child_stop_requested
            if signal_child: self.child_stop_requested=True
        if lifetime_w is not None: os.close(lifetime_w)
        if signal_child:
            try: child.send_signal(signal.SIGINT)
            except ProcessLookupError: pass

    def wait_stopped(self):
        # Serialize program cleanup and external STOP; never interrupt the
        # child's cancellation cleanup with a second SIGINT.
        with self.stop_lock:
            with self.lock: child=self.child; previous_error=self.shutdown_error
            if previous_error: raise RuntimeError(previous_error)
            if child is not None and child.poll() is None:
                try: child.wait(timeout=5.)
                except subprocess.TimeoutExpired:
                    child.kill()
                    try: child.wait(timeout=2.)
                    finally:
                        self.terminated=child.poll() is not None
                        self.shutdown_error='taught producer forced down; resident STOP verification required'
                    raise RuntimeError(self.shutdown_error)
            if child is not None: self.terminated=child.poll() is not None

    def stop(self):
        self.request_stop()
        self.wait_stopped()

    def reconcile_terminated(self):
        """Explicit inspected recovery may clear only proven-dead producer state."""
        with self.lock:
            proven=self.child.poll() is not None if self.child is not None else self.terminated
            if self.shutdown_error and not proven:
                raise RuntimeError('taught producer termination still unresolved')
            if proven: self.shutdown_error=None

    def run(self,name,speed,cancel,deadline):
        def check():
            if cancel.is_set() or self.stopped: raise InterruptedError('taught skill cancelled')
            if time.monotonic()>=deadline: raise TimeoutError('taught skill deadline')
        check()
        identity=self.backend.identity()
        skill=self.library.load(name,physical=True,identity=identity)
        from workshop.blockly_skills import require_taught_calibration_status
        require_taught_calibration_status(self.backend.trigger('/follower/gripper_calibration_status',cancel,deadline),skill)
        selector=StableTargetSelector(after_stamp=self.backend.node.get_clock().now().nanoseconds*1e-9,excluded_ids=())
        while True:
            check()
            with self.lock: status=self.status; received=self.received
            target=None
            if status is not None and time.monotonic()-received<=.5:
                target=selector.update(status,now=self.backend.node.get_clock().now().nanoseconds*1e-9)
            if target is not None: break
            cancel.wait(.05)
        # A private temporary artifact is not reusable authorization.
        with tempfile.TemporaryDirectory(prefix='station-taught-') as directory:
            output=Path(directory)/'result.json'
            command=[sys.executable,str(ROOT/'tools/ros_plan_grasp.py'),'--execute',
                     '--track-id',target.track_id,'--frame',skill['frame_id'],
                     '--tip-link',skill['tip_link'],'--execution-config',str(skill['artifact_paths']['execution_config']),
                     '--place-pose',*(str(v) for v in skill['checkpoints']['drop']),
                     '--taught-skill',skill['manifest_path'],'--taught-speed',str(speed),
                     '--timeout',str(min(180.,max(.1,deadline-time.monotonic()))),'--output',str(output)]
            with (Path(directory)/'session.log').open('w+') as log:
                with self.lock:
                    check()
                    lifetime_r,lifetime_w=os.pipe()
                    self.lifetime_w=lifetime_w
                    try:
                        self.child=subprocess.Popen(command+['--taught-lifetime-fd',str(lifetime_r)],
                            stdout=log,stderr=subprocess.STDOUT,pass_fds=(lifetime_r,))
                    except BaseException:
                        os.close(lifetime_w);self.lifetime_w=None;raise
                    finally: os.close(lifetime_r)
                    child=self.child
                try:
                    while child.poll() is None: check(); cancel.wait(.05)
                    check()
                    if child.returncode!=0:
                        log.seek(0); details=log.read(2000)
                        raise RuntimeError('taught grasp transaction failed: '+details)
                    return validate_skill_result(json.loads(output.read_text()),target.track_id)
                finally:
                    self.stop()
                    with self.lock: self.child=None
