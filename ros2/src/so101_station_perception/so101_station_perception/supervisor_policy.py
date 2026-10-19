"""Bounded automatic recovery policy. No ROS, pixels, files or motor authority."""
from collections import deque
from copy import deepcopy
from .derived_lineage import require,finite,hex_value

TRANSIENT_CODES=frozenset(('private_source_receipt_gap','private_source_stream_lost'))


def transient_fault(status,expected_generation,expected_node_sha,*,now_mono,now_ns):
    require(status.get('state')=='unavailable','producer is not unavailable')
    fault=status.get('fault')
    require(type(fault) is dict and fault.get('code') in TRANSIENT_CODES and
        fault.get('automatic_recovery_class')=='transient_private_availability',
        'fault is not positively classified transient private availability')
    require(hex_value(fault.get('fault_id'),32) and fault.get('producer_generation')==expected_generation==status.get('producer_generation') and
        fault.get('producer_node_sha256')==expected_node_sha,'fault identity/code provenance changed')
    require(finite(fault.get('first_observed_monotonic')) and 0<=fault['first_observed_monotonic']<=now_mono and
        type(fault.get('first_observed_ros_ns')) is int and 0<fault['first_observed_ros_ns']<=now_ns,
        'fault event clock is future/invalid')
    reset=status.get('reset') or {}
    require(reset.get('state') not in ('resetting','preparing','reset_failed'),'producer reset active/failed')
    return deepcopy(fault)


class SupervisorPolicy:
    """One attempt per outage. Rolling caps survive disable and explicit reset."""
    def __init__(self,history=None):
        history=history or {}
        self.attempts=deque(history.get('attempts',()),maxlen=3)
        require(len(history.get('attempts',()))<=3 and all(finite(t) and t>=0 for t in self.attempts),'invalid persisted quota')
        require(list(self.attempts)==sorted(self.attempts),'persisted quota not ordered')
        self.last_utc=history.get('last_utc',0.)
        require(finite(self.last_utc) and self.last_utc>=0,'persisted clock invalid')
        self.enabled=False;self.state='disabled';self.latched=None
        self.events=deque(maxlen=128);self.handled=deque(history.get('handled',()),maxlen=128)
        require(len(history.get('handled',()))<=128 and all(hex_value(x,32) for x in self.handled),'invalid outage journal')
        self.fault=None;self.job=deepcopy(history.get('inflight'));self.stable_since=None;self.last_mono=history.get('last_mono')
        if history.get('inflight'):self.fail('persisted in-flight job completion unknown')
        if history.get('latched'):self.fail(str(history['latched'])[:512])

    def clocks(self,now,utc):
        require(finite(now) and finite(utc) and now>=0 and utc>=self.last_utc and
            (self.last_mono is None or now>=self.last_mono),'supervisor clock rollback/invalid')
        if self.last_mono is not None:
            require(abs((utc-self.last_utc)-(now-self.last_mono))<=2.,'supervisor wall/monotonic clock discontinuity')
        self.last_mono=now;self.last_utc=utc
        while self.attempts and utc-self.attempts[0]>=3600.:self.attempts.popleft()
        require(not self.attempts or utc>=self.attempts[-1],'attempt clock future/invalid')

    def event(self,name,now,**data):
        self.events.append(dict(event=name,monotonic=now,**data))

    def enable(self,now):
        require(not self.latched and self.job is None,'supervisor latch/job requires explicit resolution')
        self.enabled=True;self.state='warming';self.stable_since=None;self.event('enabled',now)

    def disable(self,now):
        self.enabled=False;self.stable_since=None
        if self.job is None:self.state='disabled' if not self.latched else 'latched'
        self.event('disabled',now,accepted_job_continues=self.job is not None)

    def healthy(self,now,valid):
        if not self.enabled or self.latched or self.job is not None or self.state=='verifying_native':return
        if not valid:self.stable_since=None;self.state='warming';return
        if self.stable_since is None:self.stable_since=now
        if now-self.stable_since>=5.:
            if len(self.attempts)>=3:self.fail('automatic recovery three-attempt hourly quota exhausted');return
            if not self.attempts or self.last_utc-self.attempts[-1]>=30.:
                self.state='armed'

    def outage(self,fault,now):
        require(self.enabled and self.state=='armed' and not self.latched and self.job is None,'automatic supervisor not armed')
        require(fault['fault_id'] not in self.handled,'same outage cannot be automatically retried')
        self.handled.append(fault['fault_id']);self.fault=deepcopy(fault)
        self.state='verifying_native';self.stable_since=None;self.event('outage_observed',now,fault_id=fault['fault_id'])

    def reserve(self,now,utc):
        self.clocks(now,utc)
        require(self.enabled and self.state=='verifying_native' and self.job is None and not self.latched,'automatic attempt not eligible')
        require(len(self.attempts)<3 and (not self.attempts or utc-self.attempts[-1]>=30.),'automatic quota/cooldown exhausted')
        self.attempts.append(utc);self.job={'state':'dispatch_unknown','reserved_monotonic':now,'outage_id':self.fault['fault_id']}
        self.state='recovering';self.event('attempt_reserved',now,outage_id=self.fault['fault_id'])

    def accepted(self,job_id,deadline,now):
        require(self.job is not None and self.job['state']=='dispatch_unknown' and hex_value(job_id,32), 'unknown recovery acceptance')
        require(finite(deadline) and now<deadline<=self.job['reserved_monotonic']+42.,'coordinator deadline invalid')
        self.job.update(state='accepted',job_id=job_id,deadline=deadline)

    def completed(self,now):
        require(self.job is not None and self.job['state']=='accepted','no owned accepted job')
        self.event('own_job_completed',now,job_id=self.job['job_id']);self.job=None;self.fault=None
        self.state='warming' if self.enabled else 'disabled';self.stable_since=None

    def fail(self,reason):
        self.latched=str(reason)[:512];self.state='latched';self.enabled=False;self.stable_since=None

    def reset_fault(self,now,*,terminal_proven):
        require(terminal_proven and self.job is None,'active or unknown job cannot reset latch')
        self.latched=None;self.state='disabled';self.enabled=False;self.stable_since=None;self.event('fault_reset',now)

    def journal(self):
        return dict(schema='so101-perception-supervisor-journal-v1',attempts=list(self.attempts),
            last_utc=self.last_utc,last_mono=self.last_mono,handled=list(self.handled),inflight=deepcopy(self.job),latched=self.latched)

    def status(self):
        return dict(schema='so101-perception-supervisor-v1',qualified=False,execution_enabled=False,
            enabled=self.enabled,state=self.state,latched=self.latched,job=deepcopy(self.job),
            attempts_last_hour=len(self.attempts),stable_since=self.stable_since,events=list(self.events)[-64:])
