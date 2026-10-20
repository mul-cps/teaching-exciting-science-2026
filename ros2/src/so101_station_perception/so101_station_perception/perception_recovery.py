"""Explicit bounded metadata-only perception recovery. Never motion authority."""
from copy import deepcopy
from collections import deque
import hashlib
import json
import os
import struct
from pathlib import Path
import threading
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterDescriptor
from rcl_interfaces.srv import SetParameters, GetParameters
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .camera_source import CameraSourceCapture
from .derived_aligned import received_gid
from .derived_lineage import (TOPICS, LINEAGE_TOPIC, LineageBuffer, canonical_gid,
    configure_ros_domain, hex_value, parse_lineage, require, statement_pin,
    validate_graph, validate_statement)
from .admission_proof import pin_digest
from .recovery_core import Budget, NativePairs, ConsumerProgress, bounded_json, native_graph, stable_pin

PRODUCER = '/so101_rocm_rgbd'
PRODUCER_STATUS = '/so101_perception/rocm/status'
CONSUMERS = ('/so101_charuco_world', '/so101_colored_objects')


class ProducerStatusPending(ValueError):
    """An unchanged endpoint has no current heartbeat; never readiness proof."""


def read_artifact(path):
    with Path(path).open('rb') as stream: raw = stream.read(131073)
    require(len(raw) <= 131072, 'source artifact exceeds bound')
    def pairs(items):
        result = {}
        for key,item in items:
            require(key not in result, 'duplicate artifact JSON key'); result[key] = item
        return result
    value = json.loads(raw, object_pairs_hook=pairs)
    json.dumps(value, allow_nan=False)
    require(value.get('schema') == 'so101-camera-source-v1' and value.get('qualified') is False
        and value.get('execution_enabled') is False and value.get('metadata_only') is True
        and value.get('independent_pixels_observed') is False, 'camera-only diagnostic artifact required')
    require(value['derived_source'].get('schema') == 'so101-derived-rocm-rgbd-cohort-v1'
        and value['derived_source'].get('qualified') is False, 'explicit diagnostic derived pin required')
    pin = value['derived_source']['pin']; LineageBuffer(pin)
    require(statement_pin(value['camera_evidence']['lineage']['statement']) == pin,
            'artifact statement/pin differ')
    return value, hashlib.sha256(raw).hexdigest()


class PerceptionRecovery(Node):
    def __init__(self):
        super().__init__('so101_perception_recovery')
        readonly = ParameterDescriptor(read_only=True)
        for key, default in (('enabled', False), ('expose_completion_evidence', False), ('artifact_directory', ''),
                ('expected_source_cohort', ''), ('expected_serial', '243322071783')):
            self.declare_parameter(key, default, readonly)
        self.expose_completion_evidence = self.get_parameter('expose_completion_evidence').value
        require(type(self.expose_completion_evidence) is bool, 'evidence exposure requires strict bool')
        if self.expose_completion_evidence:
            self._evidence_boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
            self._evidence_instance = uuid.uuid4().hex
            self._completion_event = None; self._evidence_error = None
            self._event_witnesses = {}; self._event_high = {}; self._rpc_trace = None
        require(self.get_parameter('enabled').value is True, 'explicit enabled=true required')
        self.directory = Path(self.get_parameter('artifact_directory').value).expanduser().resolve()
        require(bool(self.get_parameter('artifact_directory').value) and self.directory.is_dir(),
                'explicit existing trusted artifact directory required')
        self.baseline_path = Path(self.get_parameter('expected_source_cohort').value).expanduser().resolve()
        require(bool(self.get_parameter('expected_source_cohort').value), 'explicit reviewed baseline required')
        baseline, self.baseline_sha = read_artifact(self.baseline_path)
        self.baseline_pin = baseline['derived_source']['pin']
        self.serial = self.get_parameter('expected_serial').value
        require(type(self.serial) is str and 0 < len(self.serial) <= 128 and
                self.baseline_pin['source']['serial_number'] == self.serial, 'expected camera serial mismatch')
        self.lock = threading.RLock(); self.cancel = threading.Event()
        self.thread = None; self.closed = False; self.last_start = float('-inf')
        self.budget = None; self.native = None; self.ledger = LineageBuffer()
        self.expected_generation = None; self.old_generation = None; self.producer = None
        self.progress = {}; self.pin = None; self.artifact_path = None; self.artifact_sha = None
        self.watch_until = 0.; self.lineage_error = None; self.native_error = None
        self.result = dict(schema='so101-perception-recovery-v1', state='idle', job_id=None,
            operation_success=False, ready_now=False, qualified=False, execution_enabled=False,
            scope='source admission only; not detected objects, calibration or motion')
        self.rpc_clients = {PRODUCER+'/reset_source': self.create_client(Trigger, PRODUCER+'/reset_source')}
        for consumer in CONSUMERS:
            for suffix, typ in (('/derived_source_status', Trigger), ('/reset_derived_source', Trigger),
                                 ('/set_parameters', SetParameters), ('/get_parameters', GetParameters)):
                self.rpc_clients[consumer+suffix] = self.create_client(typ, consumer+suffix)
        self.create_subscription(String, PRODUCER_STATUS, self._producer, qos_profile_sensor_data)
        self.create_subscription(String, LINEAGE_TOPIC, self._lineage, qos_profile_sensor_data)
        for kind in ('depth', 'color'):
            self.create_subscription(CameraInfo, TOPICS[kind+'_info'], self._info_callback(kind), qos_profile_sensor_data)
        self.create_service(Trigger, '~/start_recovery', self._start)
        self.create_service(Trigger, '~/recovery_status', self._status)

    def _active(self): return self.thread is not None and self.thread.is_alive()

    def _guard(self):
        require(not self.closed and not self.cancel.is_set(), 'recovery cancelled/closed')
        self.budget.check(time.monotonic())

    def _watching(self): return self._active() or time.monotonic() < self.watch_until

    def _info_callback(self, kind):
        def callback(message, info):
            receipt = time.monotonic(); now_ns = self.get_clock().now().nanoseconds
            with self.lock:
                if self.native is None or not self._active(): return
                try:
                    cal = self.baseline_pin['calibration']
                    require([message.width,message.height] == cal[kind+'_size'] and
                        message.header.frame_id == self.baseline_pin['outputs']['cloud_frame' if kind=='depth' else 'aligned_frame'] and
                        [struct.pack('f',float(message.k[i])) for i in (0,4,2,5)] ==
                        [struct.pack('f',v) for v in cal[kind+'_K']], 'native camera profile differs from reviewed calibration')
                    self.native.add(kind, message, received_gid(info), now_ns=now_ns, now_mono=receipt)
                    self.native_error = None
                except (ValueError, TypeError, KeyError, AttributeError, IndexError, OverflowError) as error:
                    self.native.infos[kind].clear(); self.native.pairs.clear()
                    self.native_error = str(error)[:512]
                    if 'profile differs from reviewed' in str(error): self.native.invalid = self.native_error
        return callback

    def _producer(self, message, info):
        receipt = time.monotonic()
        try:
            value = parse_lineage(message.data)
            require(value.get('schema') == 'so101-derived-rocm-rgbd-v1' and value.get('qualified') is False
                and value.get('execution_enabled') is False and hex_value(value.get('producer_generation'),32),
                'invalid producer status')
            record = (value, canonical_gid(received_gid(info)), receipt)
        except (ValueError, TypeError, KeyError): record = None
        with self.lock: self.producer = record

    def _lineage(self, message, info):
        receipt = time.monotonic()
        with self.lock:
            if not self._watching() or self.expected_generation is None: return
            try:
                value = parse_lineage(message.data)
                if (self.ledger.pin is None and value.get('producer_generation') == self.old_generation): return
                require(value.get('producer_generation') == self.expected_generation, 'unexpected producer generation')
                require(stable_pin(statement_pin(value)) == stable_pin(self.baseline_pin), 'reviewed code/calibration/source context changed')
                added = self.ledger.add(message.data, received_gid(info), now_ns=self.get_clock().now().nanoseconds,
                                        now_mono=time.monotonic())
                if added:
                    value, gid, _ = self.ledger.entries[-1]
                    self.ledger.entries[-1] = (value, gid, receipt)
                self.lineage_error = None
            except (ValueError, TypeError, KeyError) as error:
                self.ledger.entries.clear(); self.lineage_error = str(error)[:512]
                # A different generation/code/context cannot silently recover.
                if 'generation' in str(error) or 'context changed' in str(error): self.ledger.invalid_reason = self.lineage_error

    def _start(self, request, response):
        with self.lock:
            reason = None
            if self.closed: reason = 'coordinator closed'
            elif self._active(): reason = 'one recovery job already active'
            elif time.monotonic()-self.last_start < 30.: reason = 'recovery cooldown30s'
            if reason:
                response.success = False; response.message = bounded_json(dict(accepted=False, reason=reason)); return response
            started = time.monotonic(); self.last_start = started; self.budget = Budget(started)
            if getattr(self, 'expose_completion_evidence', False):
                self._completion_event = None; self._evidence_error = None
                self._event_witnesses = {}; self._event_high = {}; self._rpc_trace = None
            self.cancel.clear(); self.expected_generation = self.old_generation = None
            self.native = NativePairs(start_ns=self.get_clock().now().nanoseconds, start_mono=started)
            self.ledger = LineageBuffer(); self.progress = {}; self.pin = None
            self.artifact_path = None; self.artifact_sha = None; self.watch_until = 0.
            self.lineage_error = self.native_error = None
            self.result = dict(schema='so101-perception-recovery-v1', state='native_preflight',
                job_id=uuid.uuid4().hex, operation_success=False, ready_now=False, qualified=False,
                execution_enabled=False, scope='source admission only; not objects/calibration/motion',
                deadline_monotonic=self.budget.deadline, reason=None, configured_consumers=[], rebound_consumers=[])
            self.thread = threading.Thread(target=self._run, name='perception-recovery-job', daemon=False)
            try: self.thread.start()
            except RuntimeError as error:
                self.result.update(state='failed', reason=str(error)[:512])
                response.success = False
                response.message = bounded_json(dict(accepted=False, job_id=self.result['job_id'], reason=self.result['reason']))
                return response
            response.success = True
            response.message = bounded_json(dict(accepted=True, job_id=self.result['job_id'],
                state=self.result['state'], ready_now=False, deadline_monotonic=self.budget.deadline))
            return response

    def _phase(self, phase):
        self._guard()
        with self.lock: self.result['state'] = phase

    def _pause(self, seconds=.025):
        self._guard(); self.cancel.wait(min(seconds, max(0., self.budget.deadline-time.monotonic()))); self._guard()

    def _rpc(self, endpoint, request):
        self._guard(); started = time.monotonic(); deadline = self.budget.rpc_deadline(started); client = self.rpc_clients[endpoint]
        self.rpc_deadline = deadline
        future = None
        try:
            while not client.service_is_ready():
                self._guard(); require(time.monotonic() <= deadline, 'RPC service unavailable: '+endpoint)
                self.cancel.wait(.005)
            self._guard(); require(time.monotonic() <= deadline, 'RPC deadline before dispatch')
            future = client.call_async(request)
            while not future.done():
                self._guard(); require(time.monotonic() <= deadline, 'RPC timeout; completion unknown, no adoption: '+endpoint)
                self.cancel.wait(.005)
            if getattr(self, 'expose_completion_evidence', False): observed_done = time.monotonic()
            response = future.result()
            if getattr(self, 'expose_completion_evidence', False):
                self._rpc_trace = (started, observed_done, time.monotonic(), deadline)
            self._guard(); require(time.monotonic() <= deadline and response is not None,
                                   'RPC response expired/unknown: '+endpoint)
            return response
        except Exception:
            if future is not None:
                try: client.remove_pending_request(future)
                except (KeyError, RuntimeError): pass
                future.cancel()
            raise

    def _rpc_guard(self):
        self._guard(); require(time.monotonic() <= self.rpc_deadline, 'RPC post-access deadline expired; no adoption')

    def _json_response(self, response):
        require(type(response.message) is str and len(response.message) <= 4096, 'bounded service status required')
        value = parse_lineage(response.message); self._rpc_guard(); return value

    def _check_baseline(self):
        self._guard(); _, digest = read_artifact(self.baseline_path); self._guard()
        require(digest == self.baseline_sha, 'reviewed baseline file changed; reconfigure explicitly')

    def _producer_status(self, *, job=True):
        check = self._guard if job else lambda: None
        check()
        with self.lock: record = self.producer
        require(record is not None, 'current producer status unavailable')
        value, gid, receipt = record
        endpoints = self.get_publishers_info_by_topic(PRODUCER_STATUS); check()
        require(len(endpoints) == 1 and endpoints[0].node_namespace.rstrip('/')+'/'+endpoints[0].node_name == PRODUCER
            and canonical_gid(endpoints[0].endpoint_gid) == gid,
            'producer status endpoint changed')
        age = time.monotonic()-receipt
        require(0 <= age, 'producer status receipt is future/invalid')
        if age > .250:
            raise ProducerStatusPending('producer status receipt expired; awaiting new heartbeat')
        with self.lock:
            current = self.producer
            require(current is not None and current[1] == gid, 'current producer status unavailable/changed')
            age = time.monotonic()-current[2]
            require(0 <= age, 'current producer status receipt is future/invalid')
            if age > .250:
                raise ProducerStatusPending('current producer status receipt expired; awaiting new heartbeat')
            return current[0]

    def _statement(self, proof=None, *, job=True):
        check = self._guard if job else lambda: None
        check()
        with self.lock:
            require(not self.ledger.invalid_reason and bool(self.ledger.entries), self.lineage_error or 'current newest lineage unavailable')
            latest = self.ledger.entries[-1]
            matched = latest
            if proof is not None:
                candidates = [r for r in self.ledger.entries if r[0]['sequence'] == proof['sequence']
                    and r[0]['original']['depth_stamp_ns'] == proof['depth_stamp_ns']
                    and r[0]['original']['color_stamp_ns'] == proof['color_stamp_ns']]
                require(bool(candidates), 'exact consumer lineage no longer retained')
                matched = candidates[-1]
            def validate(record):
                require(0 <= time.monotonic()-record[2] <= .250, 'own original lineage receipt expired/future')
                validate_statement(record[0], now_ns=self.get_clock().now().nanoseconds,
                    now_mono=time.monotonic(), receipt_mono=record[2])
            validate(latest); validate(matched)
            require(stable_pin(statement_pin(latest[0])) == stable_pin(self.baseline_pin), 'reviewed immutable context changed')
            def lookup(topic):
                check(); result = self.get_publishers_info_by_topic(topic); check(); return result
            try: validate_graph(latest[0], lookup, lineage_gid=latest[1])
            except ValueError as error:
                self.ledger.invalid_reason = str(error); self.ledger.entries.clear(); raise
            validate(latest); validate(matched); check()
            return matched[0]

    def _capture(self):
        self._phase('capture_source_pins')
        output = self.directory / (self.result['job_id'] + '.json')
        executor = SingleThreadedExecutor(context=self.context); capture = None
        try:
            self._guard()
            capture = CameraSourceCapture(serial=self.serial, context=self.context, use_global_arguments=False)
            executor.add_node(capture); self._guard()
            artifact = capture.capture(timeout=min(4., self.budget.deadline-time.monotonic()), output=output, executor=executor)
            self._guard()
        finally:
            if capture is not None: executor.remove_node(capture)
            executor.shutdown(timeout_sec=.5)
            if capture is not None: capture.destroy_node()
        self._guard(); stored, digest = read_artifact(output); self._guard()
        require(stored == artifact, 'published source artifact changed after capture')
        pin = artifact['derived_source']['pin']
        require(pin['producer_generation'] == self.expected_generation and
            stable_pin(pin) == stable_pin(self.baseline_pin), 'captured generation/reviewed pins differ')
        with self.lock:
            require(self.ledger.pin == pin, 'captured pin differs from actual current lineage')
            self.pin = pin; self.artifact_path = output; self.artifact_sha = digest
            self.result.update(artifact=str(output), artifact_sha256=digest, source_pin_sha256=pin_digest(pin))
        self._check_baseline(); self._statement()

    def _artifact_unchanged(self):
        _, digest = read_artifact(self.artifact_path)
        require(digest == self.artifact_sha, 'new source artifact changed')

    def _rebind(self):
        epochs = {}
        for consumer in CONSUMERS:
            response = self._rpc(consumer+'/derived_source_status', Trigger.Request())
            value = self._json_response(response)
            require(value.get('schema') == 'so101-derived-admission-status-v1' and value.get('qualified') is False
                and value.get('execution_enabled') is False and hex_value(value.get('consumer_epoch'),32), 'consumer baseline epoch unavailable')
            epochs[consumer] = value['consumer_epoch']
        self._phase('rebind_consumers')
        for consumer in CONSUMERS:
            self._artifact_unchanged(); self._guard()
            parameter = Parameter(name='derived_source_cohort', value=ParameterValue(type=4, string_value=str(self.artifact_path)))
            response = self._rpc(consumer+'/set_parameters', SetParameters.Request(parameters=[parameter]))
            require(len(response.results) == 1 and response.results[0].successful, 'consumer artifact parameter rejected')
            self._rpc_guard()
            with self.lock: self.result['configured_consumers'].append(consumer)
            response = self._rpc(consumer+'/get_parameters', GetParameters.Request(names=['derived_source_cohort']))
            require(len(response.values) == 1 and response.values[0].type == 4 and
                response.values[0].string_value == str(self.artifact_path), 'consumer artifact readback differs')
            self._rpc_guard()
            response = self._rpc(consumer+'/reset_derived_source', Trigger.Request())
            require(response.success is True, 'consumer reset rejected; no rollback/retry')
            self._rpc_guard()
            self.progress[consumer] = ConsumerProgress(epochs[consumer], self.pin, start_ns=self.native.start_ns)
            with self.lock: self.result['rebound_consumers'].append(consumer)

    def _proofs_ready(self, *, job=True):
        status = self._producer_status(job=job); reset = status.get('reset') or {}
        require(status.get('state') == 'ready' and status.get('producer_generation') == self.expected_generation
            and reset.get('accepted_reset_id') == self.result.get('accepted_reset_id')
            and reset.get('new_producer_generation') == self.expected_generation and reset.get('ready') is True,
            'current producer reset/generation is not ready')
        if len(self.progress) != len(CONSUMERS): return False
        for progress in self.progress.values():
            require(progress.complete, 'two advancing consumer admissions unavailable')
            statement = self._statement(progress.current, job=job)
            progress.check(statement, now_ns=self.get_clock().now().nanoseconds, now_mono=time.monotonic())
        return True

    def _run(self):
        try:
            self._check_baseline()
            native_deadline = min(self.budget.deadline, time.monotonic()+2.)
            while True:
                self._guard()
                require(time.monotonic() <= native_deadline, 'native camera unhealthy; camera-only supervisor required: '+str(self.native_error))
                missing = [False]
                def lookup(topic):
                    values = self.get_publishers_info_by_topic(topic)
                    missing[0] = missing[0] or not values
                    return values
                try: graph = native_graph(lookup, self._guard)
                except ValueError:
                    if self.native.graph is not None or not missing[0]: raise
                    self._pause(); continue
                with self.lock:
                    if self.native.select(graph, now_ns=self.get_clock().now().nanoseconds, now_mono=time.monotonic()):
                        self.result['native_pairs'] = deepcopy(list(self.native.pairs)); break
                self._pause()
            self._phase('reset_producer')
            status_deadline = min(self.budget.deadline,time.monotonic()+2.)
            while True:
                self._guard()
                try: self.old_generation = self._producer_status()['producer_generation']; break
                except ValueError:
                    require(time.monotonic() <= status_deadline, 'producer status unavailable before reset')
                    self._pause()
            response = self._rpc(PRODUCER+'/reset_source', Trigger.Request())
            value = self._json_response(response)
            require(response.success is True and value.get('accepted') is True and value.get('ready') is False
                and hex_value(value.get('accepted_reset_id'),32) and hex_value(value.get('new_producer_generation'),32)
                and value['new_producer_generation'] != self.old_generation, 'producer reset not accepted/new generation unavailable')
            self._rpc_guard()
            with self.lock:
                self.expected_generation = value['new_producer_generation']; self.ledger = LineageBuffer()
                self.result.update(accepted_reset_id=value['accepted_reset_id'], producer_generation=self.expected_generation)
            self._phase('wait_producer_generation')
            while True:
                self._guard()
                require(not self.ledger.invalid_reason, self.lineage_error or 'lineage generation invalidated')
                try: status = self._producer_status()
                except ProducerStatusPending:
                    require(not self.ledger.invalid_reason, self.lineage_error or 'lineage generation invalidated')
                    # Only after this job's explicit accepted reset: cold worker
                    # setup may pause status publication. Never reuse its lease.
                    self._pause(); continue
                reset = status.get('reset') or {}
                require(not self.ledger.invalid_reason, self.lineage_error or 'lineage generation invalidated')
                require(status['producer_generation'] in (self.expected_generation,self.old_generation), 'unexpected producer generation after reset')
                require(reset.get('state') != 'reset_failed', 'producer reset permanently failed: '+str(reset.get('reason')))
                if (status['producer_generation'] == self.expected_generation and reset.get('accepted_reset_id') == value['accepted_reset_id']
                        and reset.get('new_producer_generation') == self.expected_generation and reset.get('ready') is True):
                    with self.lock: witnesses = list(self.ledger.entries)[-2:]
                    if len(witnesses) == 2:
                        for record in witnesses:
                            validate_statement(record[0], now_ns=self.get_clock().now().nanoseconds,
                                now_mono=time.monotonic(), receipt_mono=record[2])
                        self._statement()
                        with self.lock:
                            self.result['producer_witnesses'] = [dict(sequence=r[0]['sequence'],
                                depth_stamp_ns=r[0]['original']['depth_stamp_ns'],color_stamp_ns=r[0]['original']['color_stamp_ns'],
                                original_received_monotonic=r[2]) for r in witnesses]
                        break
                self._pause()
            self._capture(); self._rebind(); self._phase('wait_consumer_admissions')
            while True:
                for consumer, progress in self.progress.items():
                    response = self._rpc(consumer+'/derived_source_status', Trigger.Request()); proof = self._json_response(response)
                    with self.lock:
                        self.result.setdefault('consumer_status', {})[consumer] = dict(
                            state=str(proof.get('state') or '')[:64], reason=str(proof.get('reason') or '')[:512],
                            sequence=proof.get('sequence') if type(proof.get('sequence')) is int else None, independent_observations=len(progress.witnesses))
                    require(proof.get('source_pin_sha256') == progress.digest and proof.get('producer_generation') == self.expected_generation
                        and hex_value(proof.get('consumer_epoch'),32) and proof['consumer_epoch'] != progress.old_epoch,
                        'consumer reset epoch/generation/pin mismatch')
                    if not response.success:
                        progress.clear_current(); self._rpc_guard(); continue
                    statement = None
                    try: statement = self._statement(proof)
                    except ValueError:
                        progress.clear_current()
                        if self.ledger.invalid_reason: raise
                    if statement is not None:
                        now_ns = self.get_clock().now().nanoseconds; now_mono = time.monotonic()
                        # A genuine lease race is unavailable; malformed/mutated
                        # proofs fail the operation rather than being retried.
                        if (type(proof.get('original_receipt_expiry_monotonic')) in (int,float)
                                and type(proof.get('original_header_expiry_ros_ns')) is int
                                and (now_mono >= proof['original_receipt_expiry_monotonic']
                                     or now_ns >= proof['original_header_expiry_ros_ns'])):
                            progress.clear_current()
                        else:
                            progress.observe(proof, statement, now_ns=now_ns, now_mono=now_mono)
                            if getattr(self, 'expose_completion_evidence', False):
                                self._capture_event_witness(consumer, proof, statement)
                    self._rpc_guard()
                try:
                    if self._proofs_ready(): break
                except ValueError:
                    if self.ledger.invalid_reason: raise
                self._pause(.05)
            self._check_baseline(); self._artifact_unchanged(); self._guard()
            with self.lock:
                self._proofs_ready()
                completed = dict(self.result, state='completed', operation_success=True, ready_now=True,
                    consumers={k:dict(latest=deepcopy(v.current), witnesses=list(v.witnesses)) for k,v in self.progress.items()})
                event = None
                if getattr(self, 'expose_completion_evidence', False): event = self._prepare_event()
                bounded_json(completed); self._guard(); self._proofs_ready()
                if getattr(self, 'expose_completion_evidence', False):
                    event = self._event_commit_gate(event)
                    # New metadata access cannot rescue expired legacy completion.
                    self._guard(); self._proofs_ready()
                    self._completion_event = PerceptionRecovery._completion_final(self, event)
                self.result = completed
                self.watch_until = min(p.current['original_receipt_expiry_monotonic'] for p in self.progress.values())
        except Exception as error:
            with self.lock: self.result.update(state='failed', operation_success=False, ready_now=False, reason=str(error)[:512])

    def _legacy_status(self, request, response):
        with self.lock:
            if self._active() and not self.result['operation_success'] and time.monotonic() > self.budget.deadline:
                self.cancel.set(); self.result.update(state='failed', operation_success=False, ready_now=False,
                    reason='absolute40s deadline expired; unfinished work completion unknown')
            result = deepcopy(self.result); result['ready_now'] = False
            try:
                if result['operation_success']:
                    self._artifact_unchanged(); result['ready_now'] = self._proofs_ready(job=False)
                message = bounded_json(result)
                if result['ready_now']:
                    self._artifact_unchanged()
                    require(self._proofs_ready(job=False), 'current final source proofs unavailable')
            except (ValueError, OSError, KeyError, TypeError) as error:
                result['ready_now'] = False; result['readiness_reason'] = str(error)[:512]
                message = bounded_json(result)
            response.success = True  # status retrieval, not readiness
            response.message = message
            return response

    def _capture_event_witness(self, consumer, proof, statement):
        """Evidence-only, no change to ConsumerProgress or recovery outcome."""
        from .completion_event import record_witness
        sequence = proof['sequence']
        high = self._event_high.get(consumer, 0)
        if sequence <= high: return  # Never reconstruct missing original evidence.
        self._event_high[consumer] = sequence
        records = self._event_witnesses.setdefault(consumer, deque(maxlen=2))
        try:
            matched = next(r for r in reversed(self.ledger.entries) if r[0] is statement)
            request, done, take, deadline = self._rpc_trace
            ns = self.get_clock().now().nanoseconds; now = time.monotonic()
            record = record_witness(proof, statement, request_mono=request, observed_done_mono=done,
                take_mono=take, consumed_mono=now, consumed_ros_ns=ns, rpc_deadline=deadline,
                lineage_receipt=matched[2], lineage_gid=canonical_gid(matched[1]))
            # Hash/copy/validation must finish inside the original leases too.
            final_ns = self.get_clock().now().nanoseconds; final = time.monotonic()
            require(final <= deadline and final < proof['original_receipt_expiry_monotonic'] and
                final_ns < proof['original_header_expiry_ros_ns'] and now <= final and ns <= final_ns,
                'witness evidence materialization crossed original budget')
            records.append(record)
        except (ValueError, TypeError, KeyError, StopIteration) as error:
            self._evidence_error = str(error)[:256]
        wanted = [p['sequence'] for p in self.progress[consumer].witnesses]
        self._event_witnesses[consumer] = deque((r for r in records if r['proof']['sequence'] in wanted), maxlen=2)

    def _prepare_event(self):
        from .completion_event import prepare_completion_event
        try:
            consumers = {}
            for name, progress in self.progress.items():
                records = list(self._event_witnesses.get(name, ()))
                require([r['proof']['sequence'] for r in records] == [p['sequence'] for p in progress.witnesses],
                    'original event witness evidence unavailable')
                consumers[name] = dict(old_epoch=progress.old_epoch, new_epoch=progress.epoch, witnesses=records)
            context = dict(boot_id=self._evidence_boot, monotonic_clock_domain='CLOCK_MONOTONIC',
                coordinator_instance_id=self._evidence_instance, job_id=self.result['job_id'],
                accepted_reset_id=self.result['accepted_reset_id'], producer_generation=self.expected_generation,
                source_pin_sha256=pin_digest(self.pin), artifact_sha256=self.artifact_sha, baseline_sha256=self.baseline_sha,
                job_started_monotonic=self.budget.started, job_deadline_monotonic=self.budget.deadline,
                job_start_ros_ns=self.native.start_ns)
            ns = self.get_clock().now().nanoseconds; now = time.monotonic()
            return prepare_completion_event(context, consumers, checked_mono=now, checked_ros_ns=ns)
        except (ValueError, TypeError, KeyError, IndexError) as error:
            self._evidence_error = str(error)[:256]; return None

    def _completion_final(self, event):
        """Opt-in only: prepare ORIGINAL legacy scalar/ref limits, then no IO."""
        ledger = self.ledger; producer = self.producer; budget = self.budget
        generation = self.expected_generation; job = self.result['job_id']
        currents = [(p, p.current) for p in self.progress.values()]
        records = list(ledger.entries)
        require(bool(records) and bool(currents) and producer is not None, 'legacy final context unavailable')
        matched = [records[-1]]
        for _, proof in currents:
            matched.append(next(r for r in reversed(records) if r[0]['sequence'] == proof['sequence'] and
                all(r[0]['original'][k] == proof[k] for k in ('depth_stamp_ns','color_stamp_ns'))))
        mono_limit = min([budget.deadline, producer[2]+.250] +
            [p['original_receipt_expiry_monotonic'] for _,p in currents] +
            [min(r[0]['health']['source_expiry_monotonic'], r[0]['identity']['expires_monotonic'], r[2]+.250) for r in matched])
        ros_limit = min([p['original_header_expiry_ros_ns'] for _,p in currents] +
            [min(r[0]['original'][k] for k in ('depth_stamp_ns','color_stamp_ns','depth_info_stamp_ns','color_info_stamp_ns'))+500_000_000 for r in matched])
        # All tuple walks, metadata reads, graph validation and event construction
        # precede these clocks. Keep original references; do not refetch receipts.
        ns = self.get_clock().now().nanoseconds; now = time.monotonic()
        require(not self.closed and not self.cancel.is_set() and self.ledger is ledger and self.producer is producer
            and self.budget is budget and self.expected_generation == generation and self.result['job_id'] == job
            and all(progress.current is proof for progress,proof in currents), 'legacy final context changed')
        require(budget.started <= now < mono_limit and self.native.start_ns <= ns < ros_limit,
            'legacy original deadline expired after evidence access')
        if event is not None and (now < event['event_checked_monotonic'] or ns < event['event_checked_ros_ns']
                or now >= event['latest_valid_until_monotonic'] or ns >= event['latest_header_valid_until_ros_ns']):
            self._evidence_error = 'completion evidence unavailable at final legacy gate'; return None
        return event

    def _event_commit_gate(self, event):
        if event is None: return None
        # All preparation/hash/serialization/graph IO is complete; only original
        # scalar clocks and the single prepared-reference assignment remain.
        ns = self.get_clock().now().nanoseconds; now = time.monotonic()
        if (self.closed or self.cancel.is_set() or now < event['event_checked_monotonic']
                or ns < event['event_checked_ros_ns'] or now > self.budget.deadline
                or now >= event['latest_valid_until_monotonic'] or ns >= event['latest_header_valid_until_ros_ns']):
            self._evidence_error = 'completion evidence final original clock gate unavailable'; return None
        return event

    def _readiness_snapshot(self, ready, reason):
        from .completion_event import READINESS_SCHEMA
        receipt = identity = local = effective = header = None
        if ready:
            records = list(self.ledger.entries)
            proofs = [p.current for p in self.progress.values()]
            matched = [records[-1]]
            for proof in proofs:
                matched.append(next(r for r in reversed(records) if r[0]['sequence'] == proof['sequence'] and
                    all(r[0]['original'][k] == proof[k] for k in ('depth_stamp_ns','color_stamp_ns'))))
            receipt = min([self.producer[2]+.250] + [p['original_receipt_expiry_monotonic'] for p in proofs] +
                [r[0]['health']['source_expiry_monotonic'] for r in matched])
            identity = min(r[0]['identity']['expires_monotonic'] for r in matched)
            local = min(r[2]+.250 for r in matched)
            effective = min(receipt, identity, local)
            if self._active(): effective = min(effective, self.budget.deadline)
            header = min([p['original_header_expiry_ros_ns'] for p in proofs] +
                [min(r[0]['original'][k] for k in ('depth_stamp_ns','color_stamp_ns','depth_info_stamp_ns','color_info_stamp_ns'))+500_000_000 for r in matched])
        ns = self.get_clock().now().nanoseconds; now = time.monotonic()
        if ready and (now >= effective or ns >= header): ready=False; reason='current original evidence expired during snapshot'
        return dict(schema=READINESS_SCHEMA, ready=ready, checked_monotonic=now, checked_ros_ns=ns, reason=reason,
            original_receipt_valid_until_monotonic=receipt, original_identity_valid_until_monotonic=identity,
            original_local_lineage_valid_until_monotonic=local, effective_valid_until_monotonic=effective,
            original_header_valid_until_ros_ns=header, qualified=False, execution_enabled=False)

    def _evidence_status(self, legacy):
        result = deepcopy(legacy)
        result['completion_event'] = deepcopy(getattr(self, '_completion_event', None))
        result['completion_event_unavailable'] = None if result['completion_event'] is not None else (getattr(self, '_evidence_error', None) or 'future owned completion evidence unavailable')
        try:
            result['current_readiness'] = self._readiness_snapshot(legacy['ready_now'], legacy.get('readiness_reason'))
            result['ready_now'] = result['current_readiness']['ready']
            if not result['ready_now'] and result['current_readiness']['reason'] is not None:
                result['readiness_reason'] = result['current_readiness']['reason']
            bounded_json(result)
            return result
        except (ValueError, TypeError, KeyError, StopIteration):
            # New metadata availability can never change legacy operation outcome.
            return legacy

    def _status(self, request, response):
        if not getattr(self, 'expose_completion_evidence', False):
            return PerceptionRecovery._legacy_status(self, request, response)
        from .completion_event import validate_current_readiness
        with self.lock:
            response = PerceptionRecovery._legacy_status(self, request, response)
            legacy = json.loads(response.message)
            result = self._evidence_status(legacy)
            message = bounded_json(result)
            if result['ready_now']:
                try:
                    self._artifact_unchanged(); require(self._proofs_ready(job=False), 'current proofs unavailable after extension materialization')
                    ns = self.get_clock().now().nanoseconds; now = time.monotonic()
                    if 'current_readiness' in result:
                        require(validate_current_readiness(result['current_readiness'], take_mono=now, take_ros_ns=ns), 'original current limits expired after materialization')
                except (ValueError, OSError, KeyError, TypeError):
                    legacy['ready_now'] = False; legacy['readiness_reason'] = 'current final evidence unavailable after extension materialization'
                    result = self._evidence_status(legacy); message = bounded_json(result)
            response.message = message
            return response

    def close(self):
        self.closed = True; self.cancel.set()
        if self._active(): self.thread.join(timeout=2.)
        if self._active():
            self.get_logger().error('Recovery thread did not retire; retaining ROS context/node. External supervisor required.')
            return False
        return True


def main(args=None):
    configure_ros_domain(os.environ); rclpy.init(args=args); node = None; executor = None
    try:
        node = PerceptionRecovery()
        executor = SingleThreadedExecutor(context=node.context); node.main_executor = executor
        executor.add_node(node); executor.spin()
    finally:
        if node is None or node.close():
            if executor is not None: executor.shutdown(timeout_sec=2.)
            if node is not None: node.destroy_node()
            rclpy.shutdown()
