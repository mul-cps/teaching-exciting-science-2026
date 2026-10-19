"""Bounded recovery metadata. No ROS, pixels, device or process resources."""
from collections import deque
from copy import deepcopy
import json
import math

from .derived_lineage import (TOPICS, canonical_gid, finite, hex_value, integer,
                              require, statement_pin)
from .admission_proof import pin_digest


def bounded_json(value):
    try: raw = json.dumps(value, allow_nan=False, separators=(',', ':'))
    except (TypeError, ValueError, RecursionError) as error:
        raise ValueError('finite recovery JSON required') from error
    require(len(raw) <= 16384, 'recovery status exceeds bound')
    return raw


class Budget:
    def __init__(self, started):
        require(finite(started), 'finite recovery start required')
        self.started = started; self.deadline = started + 40.

    def check(self, now):
        require(finite(now) and self.started <= now <= self.deadline,
                'absolute recovery40s deadline expired/clock invalid')

    def rpc_deadline(self, now):
        self.check(now)
        return min(self.deadline, now + 2.)


def stable_pin(pin):
    """Operator-reviewed code/calibration/context; runtime GIDs may rotate."""
    result = deepcopy(pin)
    result.pop('producer_generation'); result.pop('source_generation')
    for stream in TOPICS: result['source'].pop(stream + '_publisher_gid')
    for kind in ('cloud', 'status', 'aligned'):
        result['outputs'].pop(kind + '_publisher_gid')
    return result


def native_graph(lookup, check=lambda: None):
    endpoints = {}
    for kind, topic in TOPICS.items():
        check(); values = lookup(topic); check()
        require(len(values) == 1, 'native source graph is not unique')
        item = values[0]; node = item.node_namespace.rstrip('/') + '/' + item.node_name
        require(0 < len(node) <= 128, 'invalid native camera node')
        endpoints[kind] = dict(node=node, gid=canonical_gid(item.endpoint_gid))
    names = {v['node'] for v in endpoints.values()}
    require(len(names) == 1 and len({v['gid'] for v in endpoints.values()}) == 4,
            'four distinct native endpoints from one camera required')
    return dict(node=next(iter(names)), endpoints=endpoints)


class NativePairs:
    def __init__(self, *, start_ns, start_mono):
        self.start_ns = start_ns; self.start_mono = start_mono
        self.infos = {k: deque(maxlen=4) for k in ('depth', 'color')}
        self.high = dict(depth=0, color=0); self.pairs = deque(maxlen=2)
        self.graph = None; self.invalid = None

    def add(self, kind, message, gid, *, now_ns, now_mono):
        require(self.invalid is None, self.invalid)
        stamp = message.header.stamp
        require(type(stamp.sec) is int and type(stamp.nanosec) is int and
                stamp.sec >= 0 and 0 <= stamp.nanosec < 10**9, 'native Info stamp invalid')
        ns = stamp.sec * 10**9 + stamp.nanosec; gid = canonical_gid(gid)
        require(integer(ns) and 0 <= now_ns-ns <= 500_000_000, 'native Info future/stale')
        require(type(message.header.frame_id) is str and 0 < len(message.header.frame_id) <= 128
            and 0 < message.width <= (1280 if kind == 'depth' else 1920)
            and 0 < message.height <= (720 if kind == 'depth' else 1080)
            and len(message.k) == 9 and all(math.isfinite(float(x)) for x in message.k)
            and message.k[0] > 0 and message.k[4] > 0 and len(message.d) == 5
            and all(float(x) == 0 for x in message.d) and message.distortion_model == 'plumb_bob',
            'invalid native Info profile')
        if self.graph is not None and gid != self.graph['endpoints'][kind + '_info']['gid']:
            self.invalid = 'native Info publisher generation changed'
            raise ValueError(self.invalid)
        if ns <= self.high[kind]: return False  # never renew duplicate receipts
        self.high[kind] = ns
        self.infos[kind].append(dict(stamp=ns, gid=gid, receipt=now_mono))
        return True

    def select(self, graph, *, now_ns, now_mono):
        require(self.invalid is None, self.invalid)
        if self.graph is not None and graph != self.graph:
            self.invalid = 'native graph changed during preflight'
            raise ValueError(self.invalid)
        self.graph = deepcopy(graph)
        options = [[r for r in self.infos[k] if r['gid'] == graph['endpoints'][k+'_info']['gid']
                    and r['stamp'] > self.start_ns and r['receipt'] >= self.start_mono
                    and 0 <= now_mono-r['receipt'] <= .250 and 0 <= now_ns-r['stamp'] <= 500_000_000]
                   for k in ('depth', 'color')]
        matches = [(d, c) for d in options[0] for c in options[1]
                   if abs(d['stamp']-c['stamp']) <= 40_000_000]
        if matches:
            pair = max(matches, key=lambda p: (min(p[0]['stamp'],p[1]['stamp']),p[0]['stamp'],p[1]['stamp']))
            if not self.pairs or all(pair[i]['stamp'] > self.pairs[-1][i]['stamp'] for i in (0,1)):
                self.pairs.append(pair)
        return len(self.pairs) == 2 and all(0 <= now_mono-r['receipt'] <= .250
            and 0 <= now_ns-r['stamp'] <= 500_000_000 for pair in self.pairs for r in pair)


class ConsumerProgress:
    def __init__(self, old_epoch, pin, *, start_ns):
        require(hex_value(old_epoch,32), 'consumer baseline epoch unavailable')
        self.old_epoch = old_epoch; self.epoch = None; self.pin = deepcopy(pin)
        self.digest = pin_digest(pin); self.start_ns = start_ns
        self.witnesses = deque(maxlen=2); self.current = None

    @property
    def complete(self): return len(self.witnesses) == 2 and self.current is not None

    def clear_current(self): self.current = None

    def _validate(self, proof, statement, *, now_ns, now_mono):
        require(proof.get('schema') == 'so101-derived-admission-status-v1'
            and proof.get('qualified') is False and proof.get('execution_enabled') is False
            and proof.get('state') == 'ready', 'source admission status is not ready')
        require(hex_value(proof.get('consumer_epoch'),32) and proof['consumer_epoch'] != self.old_epoch,
                'consumer epoch did not change')
        require(self.epoch is None or proof['consumer_epoch'] == self.epoch, 'consumer epoch changed again')
        require(proof.get('source_pin_sha256') == self.digest and
                proof.get('producer_generation') == self.pin['producer_generation'] and
                statement_pin(statement) == self.pin, 'consumer generation/pin changed')
        original = statement['original']
        require(integer(proof.get('sequence')) and proof['sequence'] == statement['sequence'] and
            all(integer(proof.get(k)) and self.start_ns < proof[k] <= now_ns and proof[k] == original[k]
                for k in ('depth_stamp_ns','color_stamp_ns')), 'consumer original lineage mismatch')
        require(integer(proof.get('checked_ros_ns')) and max(proof['depth_stamp_ns'],proof['color_stamp_ns'])
            <= proof['checked_ros_ns'] <= now_ns, 'consumer checked clock invalid')
        expiry = proof.get('original_receipt_expiry_monotonic')
        require(finite(expiry) and now_mono < expiry <= min(statement['health']['source_expiry_monotonic'],
            statement['identity']['expires_monotonic']) + 1e-9, 'consumer original receipt lease expired/invalid')
        require(type(proof.get('original_header_expiry_ros_ns')) is int and now_ns <
            proof['original_header_expiry_ros_ns'] == min(original[k] for k in
                ('depth_stamp_ns','color_stamp_ns','depth_info_stamp_ns','color_info_stamp_ns')) + 500_000_000,
            'consumer original header lease expired/invalid')

    def observe(self, proof, statement, *, now_ns, now_mono):
        self._validate(proof, statement, now_ns=now_ns, now_mono=now_mono)
        if self.witnesses:
            previous = self.witnesses[-1]
            if proof['sequence'] == previous['sequence']:
                require(all(proof[k] == previous[k] for k in ('depth_stamp_ns','color_stamp_ns',
                    'original_receipt_expiry_monotonic','original_header_expiry_ros_ns')),
                    'repeated consumer cohort cannot renew original proof')
            else:
                require(all(proof[k] > previous[k] for k in ('sequence','depth_stamp_ns','color_stamp_ns')),
                        'consumer acquisitions did not independently advance')
                self.witnesses.append(deepcopy(proof))
        else: self.witnesses.append(deepcopy(proof))
        self.epoch = proof['consumer_epoch']; self.current = deepcopy(proof)

    def check(self, statement, *, now_ns, now_mono):
        require(self.current is not None, 'current consumer proof unavailable')
        self._validate(self.current, statement, now_ns=now_ns, now_mono=now_mono)
        return True
