"""Hardware-free timing diagnostic, never physical qualification.

Run with PYTHONPATH=.:tests python tests/benchmark_recovery_runtime.py.
"""
import json
from pathlib import Path
import statistics
import tempfile
import time
import xml.etree.ElementTree as ET

import numpy as np

from test_recovery_runtime import runtime, record
from test_recovery_depth_scene import calibration
from workshop.station.recovery_collision_model import URDFCollisionModel
from workshop.station.recovery_runtime import RecoveryRuntime
from workshop.station.arm_session import CameraRecoveryPolicy


def stats(values):
    return dict(samples=len(values),mean_ms=statistics.mean(values)*1000,
        p95_ms=sorted(values)[min(len(values)-1,int(len(values)*.95))]*1000,max_ms=max(values)*1000)


def benchmark():
    with tempfile.TemporaryDirectory(prefix='recovery-runtime-bench-') as folder:
        value,_,_=runtime(Path(folder))
        started=time.monotonic();scene=value.prepare(record(value),np.ones((100,100),np.float32));cold=time.monotonic()-started
        lookup=[];proof=[]
        for index in range(100):
            if index%10==0:scene=value.prepare(record(value),np.ones((100,100),np.float32))
            raw=dict(scene.arm_raw,elbow_flex=2993-index%4)
            started=time.monotonic();bound=value(raw,started);lookup.append(time.monotonic()-started)
            started=time.monotonic()
            assert bound.minimum_clearance_m(raw)==.01
            assert bound.minimum_segment_clearance_m(raw,dict(raw,elbow_flex=raw['elbow_flex']-2))==.01
            proof.append(time.monotonic()-started)
        synthetic=dict(cold_prepare_ms=cold*1000,provider=stats(lookup),candidate_and_segment=stats(proof))
    # Checked-in effective SO101 geometry, with SYNTHETIC joint mapping, pose,
    # camera and contacts. Report timing/rejection only; never live authority.
    source=Path(__file__).resolve().parents[1]/'workshop/digital_twin/assets/so101_new_calib.urdf'
    xml=ET.parse(source).getroot()
    for mesh in xml.findall('.//mesh'):
        mesh.set('filename','package://station_cad/'+Path(mesh.get('filename')).name)
    mapping=calibration()
    for name in mapping:mapping[name]['urdf_joint']=name
    started=time.monotonic()
    model=URDFCollisionModel(ET.tostring(xml).decode(),joint_calibration=mapping,calibration_qualified=True,
        base_frame='base_link',package_paths={'station_cad':source.parent/'cad'})
    model_s=time.monotonic()-started
    policy=CameraRecoveryPolicy(model.model_sha256,model.calibration_sha256,'synthetic-qualified','a'*64)
    registration=dict(base_from_camera=np.eye(4),base_frame='base_link',camera_frame='camera_optical_frame',
        image_width=640,image_height=480,intrinsics=dict(fx=100.,fy=100.,ppx=319.5,ppy=239.5),distortion=[0.]*5,distortion_model='plumb_bob')
    real=RecoveryRuntime(model=model,registration=registration,policy=policy,
        settings=dict(arm_id='rig',depth_uncertainty_m=.002,geometry_uncertainty_m=.001,support_planes=[]),cache_root=Path('/nonexistent'),start_worker=False)
    attempts=[]
    prewarm=None
    for label in ('cold','warm'):
        if label=='warm':
            started=time.monotonic()
            try:
                real.prewarm_model({name:2000 for name in mapping})
                prewarm=dict(elapsed_ms=(time.monotonic()-started)*1000,rejection=None)
            except ValueError as error:
                prewarm=dict(elapsed_ms=(time.monotonic()-started)*1000,rejection=str(error))
        frame=record(real,{name:2000 for name in mapping});frame['intrinsics']=registration['intrinsics']
        started=time.monotonic()
        try:
            scene=real.prepare(frame,np.full((480,640),2.,np.float32))
            prepared=time.monotonic()-started
            bound=real(dict(scene.arm_raw),time.monotonic())
            checked=time.monotonic()
            clearance=None if bound is None else bound.minimum_clearance_m(dict(scene.arm_raw))
            attempts.append(dict(label=label,prepare_ms=prepared*1000,proof_ms=(time.monotonic()-checked)*1000,
                clearance=clearance,rejection=scene.last_rejection))
        except ValueError as error:attempts.append(dict(label=label,elapsed_ms=(time.monotonic()-started)*1000,rejection=str(error)))
    return dict(hardware_access=False,physical_qualification=False,synthetic=synthetic,
        checked_in_geometry=dict(source=str(source),model_sha256=model.model_sha256,model_load_ms=model_s*1000,
            actual_calibration_used=False,camera_registration_qualified=False,contact_qualification=False,model_only_prewarm=prewarm,attempts=attempts))


if __name__=='__main__':print(json.dumps(benchmark(),indent=2))
