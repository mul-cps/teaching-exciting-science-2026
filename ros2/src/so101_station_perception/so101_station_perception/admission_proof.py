"""Small snapshots of checked admission metadata, never image payloads."""
from copy import deepcopy
import hashlib
import json
from types import SimpleNamespace as NS

from .derived_lineage import (require, statement_pin, validate_statement,
                              validate_graph, validate_camera)


def pin_digest(pin):
    return hashlib.sha256(json.dumps(pin, sort_keys=True, separators=(',', ':'),
                                     allow_nan=False).encode()).hexdigest()


def require_current_lineage(core, *, now_ns, now_mono):
    core._open()
    require(bool(core.ledger.entries), 'current newest lineage unavailable')
    value, _, receipt = core.ledger.entries[-1]
    validate_statement(value, now_ns=now_ns, now_mono=now_mono,
                       receipt_mono=receipt)
    require(statement_pin(value) == core.ledger.pin,
            'current newest lineage source changed')


def _info_snapshot(message):
    stamp = message.header.stamp
    return NS(header=NS(frame_id=str(message.header.frame_id),
                        stamp=NS(sec=int(stamp.sec), nanosec=int(stamp.nanosec))),
              width=int(message.width), height=int(message.height),
              distortion_model=str(getattr(message, 'distortion_model', 'plumb_bob')),
              k=[float(x) for x in message.k], d=[float(x) for x in message.d])


class AdmissionProof:
    """Proof of a prior checked source admission, not a detector/grasp result."""
    @classmethod
    def from_token(cls, token):
        result = cls()
        result.epoch = token['epoch']
        result.statement = deepcopy(token['statement'])
        result.pin = statement_pin(result.statement)
        result.lineage_gid = token['lineage_gid']
        result.lineage_receipt = token['lineage_receipt']
        result.records = {}
        for kind, (message, gid, receipt, stamp) in token['records'].items():
            actual = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
            require(actual == stamp, 'admission header changed during snapshot')
            result.records[kind] = (gid, receipt, stamp)
        result.depth_info = _info_snapshot(token['depth_info'])
        result.color_info = _info_snapshot(token['color_info'])
        return result

    def check_leases(self, core, *, now_ns, now_mono):
        require_current_lineage(core, now_ns=now_ns, now_mono=now_mono)
        require(core.epoch is self.epoch, 'admission predates consumer reset')
        require(core.ledger.pin == self.pin, 'admission source pin changed')
        validate_statement(self.statement, now_ns=now_ns, now_mono=now_mono,
                           receipt_mono=self.lineage_receipt)
        for kind, (gid, receipt, stamp) in self.records.items():
            require(0 <= now_mono - receipt <= .250,
                    'original received image lease expired/future')
            key = 'depth_stamp_ns' if kind == 'aligned' else kind + '_stamp_ns'
            require(stamp == self.statement['original'][key],
                    'admission original header changed')
            expected = (self.statement['outputs']['aligned_publisher_gid']
                        if kind == 'aligned' else
                        self.statement['source'][kind + '_publisher_gid'])
            require(gid == expected, 'admission publisher generation changed')
        return True

    def check(self, core, *, now_ns, now_mono, lookup, color_from_depth):
        self.check_leases(core, now_ns=now_ns, now_mono=now_mono)
        try:
            validate_graph(self.statement, lookup, lineage_gid=self.lineage_gid,
                           aligned_gid=self.records['aligned'][0])
            validate_camera(self.statement, self.depth_info, self.color_info,
                            color_from_depth)
        except ValueError:
            core.clear_observations()
            core.ledger.invalidate('admission graph/calibration changed; explicit repin/reset required')
        return True

    def status(self, *, consumer_epoch, source_pin_sha256, now_ns):
        value = self.statement
        original = value['original']
        expiry = min([self.lineage_receipt + .250,
                      value['identity']['request_started_monotonic'] + 1.]
                     + [r + .250 for r in value['health']['source_receipts_monotonic'].values()]
                     + [r[1] + .250 for r in self.records.values()])
        return {'schema': 'so101-derived-admission-status-v1',
                'state': 'ready', 'qualified': False, 'execution_enabled': False,
                'scope': 'checked source admission; not detection or physical qualification',
                'consumer_epoch': consumer_epoch,
                'source_pin_sha256': source_pin_sha256,
                'producer_generation': value['producer_generation'],
                'sequence': value['sequence'],
                'depth_stamp_ns': original['depth_stamp_ns'],
                'color_stamp_ns': original['color_stamp_ns'],
                'original_receipt_expiry_monotonic': expiry,
                'original_header_expiry_ros_ns': min(original[k] for k in
                    ('depth_stamp_ns', 'color_stamp_ns', 'depth_info_stamp_ns',
                     'color_info_stamp_ns')) + 500_000_000,
                'checked_ros_ns': now_ns}
