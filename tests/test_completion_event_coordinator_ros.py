"""Actual isolated DDS metadata recovery, no camera/GPU/SDK or motion resources."""
import json
import os
from pathlib import Path
import time
import pytest


@pytest.mark.parametrize('exposed',[False,True],ids=['exact-legacy-default','opted-in-event'])
def test_actual_dds_opt_in_event_and_default_off_exact_legacy(tmp_path,monkeypatch,exposed):
    assert os.environ.get('ROS_DOMAIN_ID')=='177' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
    import rclpy
    from rclpy.parameter import Parameter
    from so101_station_perception import perception_recovery as module
    from so101_station_perception.completion_event import CONTEXT,validate_completion_event,validate_current_readiness,validate_status_event
    from test_perception_recovery_ros import test_actual_dds_private_pinner_full_recovery_and_original_expiry as actual
    assert Path(module.__file__).resolve()==Path(__file__).resolve().parents[1]/'ros2/src/so101_station_perception/so101_station_perception/perception_recovery.py'
    original_init=rclpy.init;original_class=module.PerceptionRecovery
    samples=[];instances=[]
    class EvidenceCoordinator(original_class):
        def __init__(self):
            super().__init__();instances.append(self)
            assert self.expose_completion_evidence is exposed
            rejected=self.set_parameters_atomically([Parameter('expose_completion_evidence',value=not exposed)])
            assert not rejected.successful and self.expose_completion_evidence is exposed
        def _status(self,request,response):
            response=super()._status(request,response)
            samples.append(dict(value=json.loads(response.message),mono=time.monotonic(),ns=self.get_clock().now().nanoseconds))
            return response
    def init(*,args=None,**kwargs):
        assert os.environ['ROS_DOMAIN_ID']=='177' and os.environ['ROS_LOCALHOST_ONLY']=='1'
        return original_init(args=[*args,*(['-p','expose_completion_evidence:=true'] if exposed else [])],**kwargs)
    monkeypatch.setattr(rclpy,'init',init);monkeypatch.setattr(module,'PerceptionRecovery',EvidenceCoordinator)
    evidence=tmp_path/'dds-event.json';monkeypatch.setenv('RECOVERY_COORDINATOR_DDS_EVIDENCE',str(evidence))
    actual(tmp_path,monkeypatch,0.)
    result=json.loads(evidence.read_text());completed=result['completed'];expired=result['expired']
    if not exposed:
        assert not hasattr(instances[0],'_event_witnesses') and not hasattr(instances[0],'_completion_event')
        assert all(key not in completed and key not in expired for key in ('completion_event','completion_event_unavailable','current_readiness'))
        return
    event=completed.get('completion_event')
    assert event is not None,completed.get('completion_event_unavailable')
    assert event==expired['completion_event'] and expired['operation_success'] is True and expired['ready_now'] is False
    assert completed['current_readiness']['ready'] is True and expired['current_readiness']['ready'] is False
    assert len(json.dumps(completed,separators=(',',':')))<=16384
    expected={k:event[k] for k in CONTEXT}
    expected['boot_id']=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    expected['coordinator_instance_id']=instances[0]._evidence_instance
    expected['artifact_sha256']=completed['artifact_sha256']
    expected['job_id']=completed['job_id']
    check=validate_completion_event(event,expected,received_mono=time.monotonic(),
        received_ros_ns=time.time_ns(),job_deadline=completed['deadline_monotonic'])
    assert check['historical_completion_valid'] and check['observed_within_job_budget'] and check['current_readiness'] is False
    verified=validate_status_event(expired,expected,received_mono=time.monotonic(),received_ros_ns=time.time_ns())
    assert verified['historical_completion_valid'] and not verified['current_readiness']
    assert not validate_current_readiness(expired['current_readiness'],take_mono=time.monotonic(),take_ros_ns=time.time_ns())
    if os.environ.get('COMPLETION_EVENT_DDS_EVIDENCE'):
        Path(os.environ['COMPLETION_EVENT_DDS_EVIDENCE']).write_text(json.dumps(dict(scope='syntheticDDS177 only',
            original=result,status_samples=samples,source_origin=str(Path(module.__file__).resolve()),source_sha256=__import__('hashlib').sha256(Path(module.__file__).read_bytes()).hexdigest(),event_bytes=len(json.dumps(event,separators=(',',':')))),indent=2))
