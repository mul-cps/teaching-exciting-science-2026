"""Bounded scalar timing observations; never an admission or freshness proof."""
from collections import deque
import json
import math
import time

LIMIT = (1 << 63) - 1
SITES = ('process_entry', 'pre_submit', 'result_take', 'pre_tracker', 'pre_publish')
# Compact row: id,outcome,site,error,generation,sequence,epoch,headers4,
# source_receipts4,identity4,lineage_receipt,consumer_receipts4,clocks22,analyze_calls.


class TimingRecorder:
    def __init__(self):
        self.rows = deque(maxlen=64)
        self.counts = dict(observed=0, emitted=0, ring_dropped=0, invalid=0,
                           pending_replaced=0, result_overwritten=0, shutdown_lost=0, notification_lost=0)
        self.latest = [None] * 5
        self.sequence = 0
        self.startup = {}
        self.closed = False
        self.counter_overflow = False

    def _add(self, key, amount=1):
        total = self.counts[key] + amount
        if total > LIMIT:self.counter_overflow = True
        self.counts[key] = min(LIMIT, total)

    def begin(self, token, *, consumer_epoch=''):
        """Copy only fixed scalar metadata, never token/message/image references."""
        try:
            if self.sequence == LIMIT:self.counter_overflow = True
            self.sequence = min(LIMIT, self.sequence + 1)
            row = dict(id=self.sequence, generation='', sequence=0, epoch=consumer_epoch,
                       headers=[None]*4, source=[None]*4, identity=[None]*4,
                       lineage=None, consumer=[None]*4, clocks=[None]*22, analyze_calls=0)
            row['clocks'][0] = time.monotonic()
            if token is not None:
                value = token['statement']; original = value['original']
                kinds = ('depth', 'color', 'depth_info', 'color_info')
                row.update(generation=value['producer_generation'], sequence=value['sequence'],
                           headers=[original[k+'_stamp_ns'] for k in kinds],
                           source=[value['health']['source_receipts_monotonic'][k] for k in kinds],
                           identity=[value['identity'][k] for k in ('request_started_monotonic',
                                     'confirmed_monotonic', 'expires_monotonic', 'confirmed_ros_ns')],
                           lineage=token['lineage_receipt'],
                           consumer=[token['records'][k][2] for k in ('color','aligned','color_info','depth_info')])
            return row
        except Exception:
            self._add('invalid')
            return None

    @staticmethod
    def clock(row, index, value=None):
        if row is not None:
            try:row['clocks'][index] = time.monotonic() if value is None else value
            except Exception:row['_diagnostic_error'] = True

    def finish(self, row, *, outcome, site, error=''):
        """No serialization/IO; diagnostic errors cannot escape into publication."""
        try:
            if row is None or self.closed:
                return
            if row.get('_finished'):return
            row['_finished'] = True
            self._add('observed')
            if outcome not in (0,1) or type(site) is not int or not 0<=site<=17 or row.get('_diagnostic_error'):
                raise ValueError('invalid diagnostic outcome/site')
            if any(len(row[k])!=n for k,n in (('headers',4),('source',4),('identity',4),('consumer',4),('clocks',22))):
                raise ValueError('invalid fixed diagnostic shape')
            numeric = row['headers'] + row['source'] + row['identity'] + [row['lineage']] + row['consumer'] + row['clocks']
            if any(v is not None and (type(v) not in (float,int) or not math.isfinite(v) or v<0 or (type(v) is int and v>LIMIT)) for v in numeric):
                raise ValueError('invalid diagnostic scalar')
            if any(type(row[k]) is not int or not 0<=row[k]<=LIMIT for k in ('id','sequence','analyze_calls')):
                raise ValueError('invalid diagnostic counter')
            if type(row['epoch']) is not str or len(row['epoch']) not in (0,32) or any(c not in '0123456789abcdef' for c in row['epoch']):
                raise ValueError('invalid diagnostic epoch')
            if type(row['generation']) is not str or len(row['generation']) not in (0,32) or any(c not in '0123456789abcdef' for c in row['generation']):
                raise ValueError('invalid diagnostic generation')
            # ASCII20 => encoded JSON string <=122 bytes, even all control chars.
            reason = str(error).encode('ascii','replace')[:20].decode('ascii')
            encoded = [row['id'],outcome,site,reason,row['generation'],row['sequence'],row['epoch'],
                       list(row['headers']),list(row['source']),list(row['identity']),row['lineage'],
                       list(row['consumer']),list(row['clocks']),row['analyze_calls']]
            if len(self.rows)==64:self._add('ring_dropped')
            self.rows.append(encoded)
            if outcome==0 and 1<=site<=5:
                self.latest[site-1] = [row['id'],row['clocks'][0],row['sequence']]
        except Exception:
            self._add('invalid')

    def losses(self, pending, overwritten, shutdown=0):
        for key,value in (('pending_replaced',pending),('result_overwritten',overwritten),('shutdown_lost',shutdown)):
            self.counts[key] = min(LIMIT, max(self.counts[key], int(value)))

    def drain(self):
        """Called at2Hz on the ROS thread, outside acquisition/worker locks."""
        rows=[]
        for _ in range(min(4,len(self.rows))):
            item=self.rows.popleft()
            raw=json.dumps(item,separators=(',',':'),allow_nan=False)
            if len(raw.encode())>1536:self._add('invalid')
            else:rows.append(item)
        self._add('emitted',len(rows))
        message=dict(schema='so101-object-analysis-timing-v1',qualified=False,execution_enabled=False,startup=dict(self.startup),source_check_sites=SITES,
                     rows=rows,counts=dict(self.counts),pending=len(self.rows),latest_rejections=list(self.latest),counter_overflow=self.counter_overflow,
                     coverage_scope="partial terminal buffer only; not active-job/acquisition/transport completeness",
                     locally_drained_without_known_loss=not self.rows and not self.counter_overflow and not any(self.counts[k] for k in ('ring_dropped','invalid','pending_replaced','result_overwritten','shutdown_lost','notification_lost')))
        raw=json.dumps(message,separators=(',',':'),allow_nan=False)
        if len(raw.encode())>8192:
            self._add('invalid');return '{"schema":"so101-object-analysis-timing-v1","diagnostic_error":"oversize","qualified":false,"execution_enabled":false}'
        return raw

    def close(self):
        self.closed=True
        self._add('shutdown_lost',len(self.rows))
        self.rows.clear()


def begin_node(node, token):
    recorder = getattr(node, 'timing_recorder', None)
    if recorder is None:return None
    row = recorder.begin(token, consumer_epoch=getattr(getattr(node,'derived',None),'consumer_epoch',''))
    node._timing_current = row
    return row


def site(row, number, clock=None):
    if row is not None:
        try:
            row['_site'] = number
            if clock is not None:TimingRecorder.clock(row, clock)
        except Exception:row['_diagnostic_error'] = True


def finish_node(node, outcome, reason=''):
    recorder = getattr(node, 'timing_recorder', None)
    row = getattr(node, '_timing_current', None)
    if recorder is not None and row is not None:
        # Cold faults/failed notifications have no proven acquisition association.
        if outcome == 0 and row.get('_site') in (15,17):return
        recorder.finish(row, outcome=outcome, site=row.get('_site',0), error=reason)


def worker_timing(row, timing):
    if row is not None and timing is not None:
        try:
            _worker_timing(row,timing)
        except Exception:row['_diagnostic_error'] = True


def _worker_timing(row,timing):
        for index,key in ((4,'submit'),(5,'start'),(6,'analysis_enter'),(7,'analysis_return'),(8,'store_enter')):
            row['clocks'][index] = timing[key]
        row['clocks'][9] = time.monotonic()


def count_analyze(row):
    try:row['analyze_calls'] += 1
    except Exception:row['_diagnostic_error'] = True
