"""Volatile diagnostic eligibility history; never current readiness or motion proof."""
from copy import deepcopy
from .derived_lineage import require,finite,integer
from .supervisor_policy import CLOCK_CONTINUITY_S


class PreOutageHistory:
    def __init__(self):self.clear()

    def clear(self):
        self.token=None;self.active=None

    def _clock(self,token,now,ns):
        require(finite(now) and integer(ns) and now>=token['observed_monotonic'] and ns>=token['observed_ros_ns'],
            'pre-outage history clock rollback/invalid')
        require(abs((ns-token['observed_ros_ns'])/1e9-(now-token['observed_monotonic']))<=CLOCK_CONTINUITY_S,
            'pre-outage history ROS/monotonic clock discontinuity')

    def validate(self,context,*,now,ns):
        for token in (self.token,self.active):
            if token is not None:
                self._clock(token,now,ns)
                require(context==token['context'],'pre-outage history source/epoch context changed')

    def observe(self,context,*,now,ns,coverage_end,armed):
        if self.token is not None:
            self._clock(self.token,now,ns)
            require(context==self.token['context'],'pre-outage history source/epoch context changed')
        if not armed:return  # A short healthy blip cannot renew the original deadline.
        require(finite(coverage_end) and coverage_end>now and finite(now) and integer(ns),
            'pre-outage history requires current genuine armed coverage')
        self.token=dict(context=deepcopy(context),observed_monotonic=now,observed_ros_ns=ns,
            original_coverage_end_monotonic=coverage_end,expires_monotonic=coverage_end+2.)

    def claim(self,context,fault,*,now,ns):
        token=self.token
        require(token is not None and self.active is None,'pre-outage history absent/already used')
        self._clock(token,now,ns)
        require(context==token['context'],'pre-outage history source/epoch context changed')
        require(now<=token['expires_monotonic'] and
            token['observed_monotonic']<=fault['first_observed_monotonic']<=now and
            token['observed_ros_ns']<=fault['first_observed_ros_ns']<=ns,
            'typed outage outside original pre-outage history')
        self.active=deepcopy(token);self.active['fault_id']=fault['fault_id'];self.token=None
        return self.active['expires_monotonic']

    def check_active(self,context,*,now,ns):
        require(self.active is not None,'claimed pre-outage history absent')
        self._clock(self.active,now,ns)
        require(context==self.active['context'] and now<=self.active['expires_monotonic'],
            'original pre-outage eligibility expired/context changed')
        return self.active['expires_monotonic']

    def status(self,*,now):
        token=self.token
        return dict(pre_outage_history_available=token is not None and now<=token['expires_monotonic'],
            pre_outage_history=None if token is None else {k:v for k,v in token.items() if k!='context'},
            claimed_pre_outage_expiry_monotonic=None if self.active is None else self.active['expires_monotonic'])
