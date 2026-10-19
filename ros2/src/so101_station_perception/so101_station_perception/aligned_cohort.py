"""Bounded exact aligned-image references; never decodes or copies pixels.

A token is local trusted producer provenance, not independent pixel attestation.
Callers serialize access and recheck tokens immediately before observable commits.
"""
from collections import deque
from .derived_lineage import (LineageBuffer, canonical_gid, require, validate_statement,
                              validate_graph, validate_camera)

STREAMS=('color','aligned','color_info','depth_info')


def stamp_ns(message):
    stamp=message.header.stamp
    require(type(stamp.sec) is int and type(stamp.nanosec) is int and stamp.sec>=0 and
            0<=stamp.nanosec<1_000_000_000,'invalid original image timestamp')
    return stamp.sec*1_000_000_000+stamp.nanosec


class AlignedCohorts:
    """Four references/stream, eight statements, no fallback to old statements."""
    def __init__(self,pin):
        require(pin is not None,'explicit source pins required')
        self.ledger=LineageBuffer(pin)
        self.streams={kind:deque(maxlen=4) for kind in STREAMS}
        self.last={kind:0 for kind in STREAMS}
        self.emitted=0;self.closed=False;self.epoch=object()

    def _open(self):
        require(not self.closed,'aligned admission closed')
        if self.ledger.invalid_reason:raise ValueError(self.ledger.invalid_reason)

    def clear_observations(self):
        for items in self.streams.values():items.clear()
        self.ledger.entries.clear()

    def close(self):
        self.closed=True;self.clear_observations()

    def receive(self,kind,message,gid,*,now_ns,now_mono):
        self._open();require(kind in self.streams,'unknown image stream')
        # Validate exposure before identity/high-water mutation: future input must
        # not poison recovery to the next genuinely current acquisition.
        stamp=stamp_ns(message)
        require(0<=now_ns-stamp<=500_000_000,'image exposure future/stale')
        try:self.ledger.observe_gid(kind,gid)
        except ValueError:
            self.clear_observations();raise
        pin=self.ledger.pin;cal=pin['calibration'];camera='depth' if kind=='depth_info' else 'color'
        frame=pin['outputs']['cloud_frame' if camera=='depth' else 'aligned_frame']
        valid=(message.header.frame_id==frame and [message.width,message.height]==cal[camera+'_size'])
        if kind in ('color','aligned'):
            channels=3 if kind=='color' else 2
            valid=valid and message.encoding==('rgb8' if kind=='color' else '16UC1') and not message.is_bigendian
            valid=valid and message.step==message.width*channels and len(message.data)==message.step*message.height
        if not valid:
            self.clear_observations();self.ledger.invalidate('aligned source profile/frame changed; explicit repin/reset required')
        if stamp==self.last[kind]:return False  # duplicates cannot renew receipts
        if stamp<self.last[kind]:
            self.clear_observations();self.ledger.invalidate('aligned source exposure rollback; explicit repin/reset required')
        self.last[kind]=stamp
        self.streams[kind].append((message,canonical_gid(gid),now_mono,stamp))
        return True

    def lineage(self,raw,gid,*,now_ns,now_mono):
        self._open()
        try:return self.ledger.add(raw,gid,now_ns=now_ns,now_mono=now_mono)
        except ValueError:
            self.clear_observations();raise

    def take(self,*,now_ns,now_mono):
        self._open()
        if not self.ledger.entries:return None
        value,gid,receipt=self.ledger.entries[-1]
        if value['sequence']<=self.emitted:return None
        validate_statement(value,now_ns=now_ns,now_mono=now_mono,receipt_mono=receipt)
        token={'epoch':self.epoch,'statement':value,'lineage_gid':gid,'lineage_receipt':receipt,'records':{}}
        for kind in STREAMS:
            key='depth_stamp_ns' if kind=='aligned' else kind+'_stamp_ns'
            target=value['original'][key]
            record=next((item for item in reversed(self.streams[kind]) if item[3]==target),None)
            if record is None:return None
            require(0<=now_mono-record[2]<=.250,'received image lease expired/future')
            token[kind]=record[0];token['records'][kind]=record
        self.emitted=value['sequence']
        return token

    def check(self,token,*,now_ns,now_mono,lookup,color_from_depth):
        self._open();require(token.get('epoch') is self.epoch,'token predates explicit reset');value=token['statement']
        require(self.ledger.pin==self._pin(value),'token immutable source changed')
        validate_statement(value,now_ns=now_ns,now_mono=now_mono,receipt_mono=token['lineage_receipt'])
        for kind,record in token['records'].items():
            self.ledger.observe_gid(kind,record[1])
            require(0<=now_mono-record[2]<=.250,'received image lease expired/future')
            key='depth_stamp_ns' if kind=='aligned' else kind+'_stamp_ns'
            require(stamp_ns(record[0])==value['original'][key],'token original exposure changed')
        try:
            validate_graph(value,lookup,lineage_gid=token['lineage_gid'],aligned_gid=token['records']['aligned'][1])
            validate_camera(value,token['depth_info'],token['color_info'],color_from_depth)
        except ValueError:
            self.clear_observations();self.ledger.invalidate('aligned graph/calibration changed; explicit repin/reset required')
        return True

    @staticmethod
    def _pin(value):
        from .derived_lineage import statement_pin
        return statement_pin(value)

    def retained_bytes(self):
        return sum(len(item[0].data) for kind in ('color','aligned') for item in self.streams[kind])
