"""Endpoint routing preserves every existing single-phase admission/terminal gate."""
import threading
import pytest
from workshop.station.sorter_endpoint import move_endpoint, move_phase


class Proxy:
    session = 'one-session'
    last_call_timing = {}
    def __init__(self, angle=0.):
        self.angle = angle
        self.stamp = 1000
        self.targets = []
        self.last = None
        self.next_id = 10
        self.stops = 0
        self.reference = 'home'
        self.corrupt_phase = None
    def call(self, operation, arguments=None):
        self.stamp += 20
        if operation == 'stop':
            self.stops += 1
            return {'stopped':True}
        if operation == 'motor_target':
            assert arguments['speed']==50. and arguments['acceleration']==400. and arguments['actuation']==30.
            assert abs(arguments['target']-self.angle)<=200.
            self.targets.append(arguments['target'])
            self.angle = arguments['target']
            self.next_id += 1
            self.last = dict(id=self.next_id, operation='motor_target', motor='sorter', state='complete')
            return dict(accepted=True,job=dict(self.last,state='running'))
        assert operation=='critical_status'
        last = dict(self.last) if self.last else None
        if last and len(self.targets)==self.corrupt_phase:
            last['id'] += 1
        return dict(session=self.session,connected=True,busy=False,power_mode='awake',last_error=None,
            references={'sorter':self.reference}, angles={'sorter':self.angle}, sampled_ms=self.stamp,
            last_job=last,job=None,motor_control={'sorter':dict(angle_sample_ms=self.stamp,angle_read_ms=1,
                speed_sample=dict(speed_dps=0.,sampled_ms=self.stamp,duration_ms=1))})


def test_long_endpoint_three_phases_unique_terminal_ids_one_session():
    proxy = Proxy()
    result = move_endpoint(proxy,threading.Event(),530.,lambda event:None,{})
    assert proxy.targets==[200.,400.,530.]
    assert result['phase_count']==3 and result['measured_deg']==530.
    assert [phase['native_job_id'] for phase in result['phases']]==[11,12,13]
    assert all(phase['session']=='one-session' for phase in result['phases'])
    assert proxy.stops==0


def test_reverse_endpoint_three_phases():
    proxy = Proxy(530.)
    result = move_endpoint(proxy,threading.Event(),10.,lambda event:None,{})
    assert proxy.targets==[330.,130.,10.]
    assert result['measured_deg']==10.


def test_typed_target_remains_single_phase_and_rejects_long_distance():
    proxy = Proxy()
    with pytest.raises(ValueError,match='six-second'):
        move_phase(proxy,threading.Event(),530.,lambda event:None,{})
    assert proxy.targets==[] and proxy.stops==1


def test_cancel_between_phases_brakes_and_never_dispatches_next():
    proxy = Proxy(); cancel = threading.Event()
    def record(event):
        if event['event']=='sorter_position_complete': cancel.set()
    with pytest.raises(InterruptedError,match='between phases'):
        move_endpoint(proxy,cancel,530.,record,{})
    assert proxy.targets==[200.] and proxy.stops>=1


def test_each_phase_rechecks_reference_and_terminal_job_identity():
    proxy = Proxy(); proxy.corrupt_phase=2
    with pytest.raises(ValueError,match='did not complete cleanly'):
        move_endpoint(proxy,threading.Event(),530.,lambda event:None,{})
    assert proxy.targets==[200.,400.] and proxy.stops>=1
    proxy = Proxy()
    def record(event):
        if event['event']=='sorter_position_complete': proxy.reference=None
    with pytest.raises(ValueError,match='exact home reference'):
        move_endpoint(proxy,threading.Event(),530.,record,{})
    assert proxy.targets==[200.] and proxy.stops>=1


def test_overall_timeout_blocks_native_dispatch():
    proxy=Proxy()
    with pytest.raises(TimeoutError):
        move_phase(proxy,threading.Event(),200.,lambda event:None,{},overall_deadline=0.)
    assert proxy.targets==[] and proxy.stops==1
