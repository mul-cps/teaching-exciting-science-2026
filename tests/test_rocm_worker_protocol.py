"""Real private child/FD ownership with a CPU test factory, never a GPU context."""
import os
import sys
from pathlib import Path
import subprocess
import signal
import json
import shutil
import time

import numpy as np
import pytest

from workshop.perception.rocm_client import ProjectionWorker

ROOT = Path(__file__).resolve().parents[1]


def configuration():
    return dict(schema='realsense-cpu-projection-oracle-v1', qualified=False, align_name='Align',
        pointcloud_name='Pointcloud', occlusion_option=2., depth_scale_m=.001,
        depth_size=[2, 2], color_size=[3, 2], depth_K=[4., 4., 1., 1.], color_K=[4., 4., 1., 1.],
        rotation_column_major=[1., 0., 0., 0., 1., 0., 0., 0., 1.], translation_m=[.02, 0., 0.])


def cpu_private_child(monkeypatch, mode='normal'):
    """Replace the command only inside tests; production wire has no factory flag."""
    original = subprocess.Popen
    script = r'''
import sys,socket,time
from types import SimpleNamespace as NS
sys.path.insert(0,ROOT)
import numpy as np
from perception.rocm_worker import serve
from perception.rocm_transport import SharedSlots,SlotLayout,decode_packet,encode_packet,source_hashes
c=socket.socket(fileno=SOCKET)
conf=decode_packet(c.recv(65537))['calibration']
s=SharedSlots(SlotLayout(tuple(conf['depth_size']),tuple(conf['color_size'])),INPUT,OUTPUT)
class Backend:
 hardware={'test_only_CPU_factory':True}
 def process(self,d,r):
  if MODE=='stall' and d.any():time.sleep(.8)
  xyz=np.zeros((2,2,3),np.float32);xyz[...,2]=d.astype(np.float32)*.001
  return NS(aligned_z16=np.ones((2,3),np.uint16),xyz=xyz,rgb_packed=np.ones((2,2),np.uint32),valid=(d>0).astype(np.uint8),complete_ms=.1)
try:
 if MODE=='orphan':
  import subprocess,os
  descendant=subprocess.Popen([sys.executable,'-c','import time;time.sleep(60)'],close_fds=True)
  with open(CHILDPATH+'.tmp','w') as pidfile:pidfile.write(str(descendant.pid))
  os.replace(CHILDPATH+'.tmp',CHILDPATH)
  os._exit(0)
 elif MODE=='malformed':
  c.send(encode_packet(dict(kind='ready',protocol=1,hardware=Backend.hardware,source_sha256=source_hashes(),startup_ms=0.,qualified=False)))
  job=decode_packet(c.recv(65537));c.send(encode_packet(dict(kind='result',slot=True,sequence=job['sequence'],metadata=job['metadata'],cloud_bytes=0,backend_ms=.1,worker_ms=.1,qualified=False)))
  c.recv(65537)
 else:serve(c,s,conf,lambda conf:Backend())
finally:s.close();c.close()
'''
    def create(args, **kwargs):
        def arg(name): return int(args[args.index(name)+1])
        prefix = ('ROOT='+repr(str(ROOT/'workshop'))+'\nMODE='+repr(mode)+'\nCHILDPATH='+repr(os.environ.get('TEST_ONLY_CHILD_PID_PATH', ''))+'\n'
                  +'SOCKET='+str(arg('--socket-fd'))+'\nINPUT='+str(arg('--input-fd'))+'\nOUTPUT='+str(arg('--output-fd'))+'\n')
        assert not any(key in kwargs['env'] for key in ('PYTHONPATH', 'PYTHONHOME', 'LD_LIBRARY_PATH',
            'LD_PRELOAD', 'AMENT_PREFIX_PATH', 'CONDA_PREFIX', 'CONDA_DEFAULT_ENV', 'CMAKE_PREFIX_PATH'))
        assert len(kwargs['pass_fds']) == 3 and kwargs['cwd'] == '/'
        assert kwargs['start_new_session'] is True
        assert kwargs['env']['HOME'] != '/foreign/ROS3.12/home'
        return original([args[0], '-c', prefix+script], **kwargs)
    monkeypatch.setattr(subprocess, 'Popen', create)


def ready(worker):
    end = time.monotonic()+3.
    while not worker.ready and time.monotonic() < end:
        worker.poll(); time.sleep(.003)
    assert worker.ready


@pytest.mark.parametrize('mode', ['normal', 'malformed', 'stall'])
def test_real_private_process_ownership_finite_failure_and_cleanup(tmp_path, monkeypatch, mode):
    for key in ('PYTHONPATH', 'PYTHONHOME', 'LD_LIBRARY_PATH', 'LD_PRELOAD', 'AMENT_PREFIX_PATH',
                'CONDA_PREFIX', 'CONDA_DEFAULT_ENV', 'CMAKE_PREFIX_PATH', 'HOME'):
        monkeypatch.setenv(key, '/foreign/ROS3.12/home' if key == 'HOME' else '/foreign/ROS3.12/ABI')
    cpu_private_child(monkeypatch, mode)
    library = tmp_path/'test-only-CPU-library'; library.write_bytes(b'not loaded by CPU test factory')
    worker = ProjectionWorker(configuration(), sys.executable, library, enabled=True)
    pid = worker.process.pid
    try:
        ready(worker)
        assert worker.process.poll() is None and worker.process.pid == pid
        assert os.getpgid(pid) == pid and os.getpgid(pid) != os.getpgrp()
        metadata = dict(depth_stamp_ns=100, color_stamp_ns=100, producer_generation='c'*32)
        worker.offer(np.ones((2, 2), np.uint16), np.zeros((2, 3, 3), np.uint8), metadata)
        result = None; end = time.monotonic()+2.
        error = None
        while time.monotonic() < end and result is None:
            try: result = worker.poll()
            except (ValueError, TimeoutError) as failure: error = failure; break
            time.sleep(.003)
        if mode == 'normal':
            assert error is None and result is not None
            assert result['metadata'] == metadata and result['cloud_bytes'] == 80
        elif mode == 'malformed': assert isinstance(error, ValueError)
        else: assert isinstance(error, TimeoutError)
    finally:
        worker.close()
        assert worker.process.pid == pid and worker.process.poll() is not None
        assert worker.closed and worker.scheduler.active is None
        with pytest.raises(OSError): os.fstat(worker.slots.input_fd)


def test_worker_disabled_never_spawns_or_loads_gpu(tmp_path, monkeypatch):
    monkeypatch.setattr(subprocess, 'Popen', lambda *a, **k: pytest.fail('disabled must not spawn'))
    with pytest.raises(ValueError, match='explicit'):
        ProjectionWorker(configuration(), '/no/python', '/no/library')


def test_failed_offer_cannot_pair_new_metadata_with_old_slot_bytes(tmp_path, monkeypatch):
    cpu_private_child(monkeypatch)
    library = tmp_path/'test-only-library'; library.write_bytes(b'CPU stub only')
    worker = ProjectionWorker(configuration(), sys.executable, library, enabled=True)
    try:
        ready(worker)
        worker.offer(np.ones((2, 2), np.uint16), np.zeros((2, 3, 3), np.uint8), {'valid': 'old'})
        with pytest.raises(ValueError): worker.offer(np.ones((1, 1), np.uint16), np.zeros((2, 3, 3), np.uint8), {'invalid': 'new'})
        assert worker.scheduler.pending is None
        assert worker.poll() is None and worker.scheduler.active is None
    finally: worker.close()


def test_pidfd_group_retires_orphan_after_leader_exit_sentinel_untouched(tmp_path, monkeypatch):
    pid_path = tmp_path/'owned-child.pid'; monkeypatch.setenv('TEST_ONLY_CHILD_PID_PATH', str(pid_path))
    # The sentinel is a separately owned test session, not station hardware.
    original = subprocess.Popen
    sentinel = original([sys.executable, '-c', 'import time;time.sleep(10)'], start_new_session=True)
    cpu_private_child(monkeypatch, 'orphan')
    library = tmp_path/'test-only-library'; library.write_bytes(b'CPU stub only')
    worker = ProjectionWorker(configuration(), sys.executable, library, enabled=True)
    try:
        deadline = time.monotonic()+3.
        while time.monotonic() < deadline and not pid_path.exists(): time.sleep(.003)
        assert pid_path.exists()
        descendant = int(pid_path.read_text()); worker.process.wait(timeout=1.)  # leader fully reaped
        assert Path('/proc', str(descendant)).exists()
        worker.close()
        deadline = time.monotonic()+1.
        while time.monotonic() < deadline:
            try: state = Path('/proc', str(descendant), 'stat').read_text().split(') ', 1)[1][0]
            except FileNotFoundError: break
            if state == 'Z': break
            time.sleep(.003)
        else: pytest.fail('owned orphan descendant remained running')
        assert sentinel.poll() is None
    finally:
        worker.close()
        sentinel.terminate(); sentinel.wait(timeout=1.)


def test_source_hash_failure_acquires_no_shared_descriptors(monkeypatch, tmp_path):
    import workshop.perception.rocm_client as client
    library = tmp_path/'test-only-library'; library.write_bytes(b'CPU stub only')
    monkeypatch.setattr(client, 'source_hashes', lambda: (_ for _ in ()).throw(OSError('hash read fixture')))
    monkeypatch.setattr(client.SharedSlots, 'create', lambda layout: pytest.fail('hash failure must precede FD acquisition'))
    with pytest.raises(OSError): client.ProjectionWorker(configuration(), sys.executable, library, enabled=True)


def test_pidfd_probe_failure_cannot_send_configuration_or_initialize_gpu(tmp_path, monkeypatch):
    import workshop.perception.rocm_client as client
    cpu_private_child(monkeypatch)
    library = tmp_path/'test-only-library'; library.write_bytes(b'CPU stub only')
    processes = []; original = client.subprocess.Popen
    def record(*args, **kwargs):
        process = original(*args, **kwargs); processes.append(process); return process
    monkeypatch.setattr(client.subprocess, 'Popen', record)
    def unsupported(group): raise OSError('unsupported pidfd group fixture')
    monkeypatch.setattr(client.OwnedProcessGroup, 'probe', unsupported)
    with pytest.raises(OSError, match='unsupported'): client.ProjectionWorker(configuration(), sys.executable, library, enabled=True)
    assert len(processes) == 1 and processes[0].poll() is not None


def test_installed_source_bootstrap_does_not_add_foreign_ros_abi_parent(tmp_path):
    foreign = tmp_path/'python3.12/site-packages'; package = foreign/'so101_station_projection'
    package.mkdir(parents=True)
    for name in ('rocm_worker.py', 'rocm_projection.py', 'rocm_transport.py'):
        shutil.copyfile(ROOT/'workshop/perception'/name, package/name)
    # run_name avoids main/GPU; this exercises the exact direct-file bootstrap.
    script = 'import runpy,sys,json;runpy.run_path('+repr(str(package/'rocm_worker.py'))+',run_name="inspection_only");print(json.dumps(sys.path))'
    result = subprocess.run([sys.executable, '-I', '-c', script], check=True,
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    assert str(foreign) not in json.loads(result.stdout)
