"""Read-only ROS attestation for the fixed camera; no queries on image paths."""
from __future__ import annotations

import hashlib
import json
import time

CAMERA_NODE = '/camera'
LEASE_S = 1.0
REFRESH_S = .5
QUERY_TIMEOUT_S = .4
STREAM_GAP_S = .250


class RecoveryCameraIdentity:
    def __init__(self, node, *, depth_topic, camera_info_topic, clock=time.monotonic):
        # The optional vendor interface is needed only for an enabled producer.
        try:
            from realsense2_camera_msgs.srv import DeviceInfo
        except ImportError as error:
            raise RuntimeError('RealSense DeviceInfo ROS interface is unavailable') from error
        from rcl_interfaces.msg import ParameterEvent
        from rcl_interfaces.srv import GetParameters
        from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
        from rclpy.qos import QoSProfile, QoSReliabilityPolicy
        self.node = node
        self.clock = clock
        self.depth_topic, self.info_topic = depth_topic, camera_info_topic
        self._device_type, self._parameter_type = DeviceInfo, GetParameters
        self._group = MutuallyExclusiveCallbackGroup()
        self._collector = None
        self._epoch = 0
        self._identity = None
        self._pending = None
        self._last_request = float('-inf')
        self._last_graph_check = float('-inf')
        self._last_received = {}
        self._last_header = {}
        self._closed = False
        self._diagnostics = {'valid': False, 'reason': 'no producer runtime'}
        clients, subscription = [], None
        try:
            self._device = node.create_client(DeviceInfo, CAMERA_NODE+'/device_info', callback_group=self._group)
            clients.append(self._device)
            self._parameters = node.create_client(GetParameters, CAMERA_NODE+'/get_parameters', callback_group=self._group)
            clients.append(self._parameters)
            subscription = node.create_subscription(
                ParameterEvent, '/parameter_events', self._parameter_event,
                QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE), callback_group=self._group)
            self._parameter_subscription = subscription
            self._timer = node.create_timer(.05, self._tick, callback_group=self._group)
        except Exception:
            if subscription is not None:
                node.destroy_subscription(subscription)
            for client in clients:
                node.destroy_client(client)
            raise

    def set_collector(self, collector):
        if collector is self._collector:
            return
        # RPC construction/retirement belongs exclusively to the timer group.
        self._revoke('owner runtime changed')
        self._collector = collector
        self._last_request = float('-inf')

    def _revoke(self, reason):
        self._epoch += 1
        self._identity = None
        self._last_received = {}
        self._last_header = {}
        self._diagnostics = {'valid': False, 'reason': reason}
        collector = self._collector
        if collector is not None:
            collector.invalidate_camera_identity()

    @staticmethod
    def _gid(value):
        try:
            result = bytes(value)
        except (TypeError, ValueError):
            return None
        return result if 0 < len(result) <= 32 and any(result) else None

    def _publishers(self):
        result = []
        for topic in (self.depth_topic, self.info_topic):
            publishers = self.node.get_publishers_info_by_topic(topic)
            if len(publishers) != 1:
                return None
            endpoint = publishers[0]
            name = endpoint.node_namespace.rstrip('/')+'/'+endpoint.node_name
            gid = self._gid(endpoint.endpoint_gid)
            if name != CAMERA_NODE or gid is None:
                return None
            result.append(gid)
        return tuple(result)

    @staticmethod
    def _generation(gids):
        # This runs only in the query timer, never per image or status request.
        encoded = json.dumps([CAMERA_NODE, *(gid.hex() for gid in gids)]).encode()
        return hashlib.sha256(encoded).hexdigest()

    def _discard_pending(self):
        pending = self._pending
        if pending is None:
            return True
        for client, future in zip((self._device, self._parameters), pending['futures']):
            try:
                client.remove_pending_request(future)
            except KeyError:
                pass  # A completed request may already have been removed by rclpy.
            except RuntimeError:
                pending['expired'] = True
                self._revoke('camera pending request could not be retired')
                return False
            future.cancel()
        self._pending = None
        return True

    def _tick(self):
        if self._closed or self._collector is None:
            return
        now = self.clock()
        identity = self._identity
        if identity is not None:
            invalid_context = (identity['epoch'] != self._epoch or
                               identity['collector'] is not self._collector)
            expired = not 0 <= now-identity['record']['request_started_monotonic'] <= LEASE_S
            gap = any(now-self._last_received.get(stream, identity['record']['confirmed_monotonic']) > STREAM_GAP_S
                      for stream in ('depth', 'info'))
            if invalid_context or expired or gap:
                self._revoke('camera identity lease/context expired' if expired or invalid_context else 'camera stream gap')
        try:
            pending = self._pending
            if pending is not None and (pending['collector'] is not self._collector or
                    pending['nonce'] != self._collector.cache_generation):
                if not self._discard_pending():
                    return
                pending = None
            if pending is not None:
                done = all(future.done() for future in pending['futures'])
                expired = now-pending['started'] > QUERY_TIMEOUT_S
                if expired and not pending['expired']:
                    pending['expired'] = True
                    self._revoke('camera identity query timed out')
                if not done:
                    # Quarantine a slow pair: never pile more requests onto it.
                    # A changed DDS generation proves it cannot authorize this stream.
                    if now-self._last_graph_check >= REFRESH_S:
                        self._last_graph_check = now
                        current = self._publishers()
                        if current is not None and current != pending['gids']:
                            if not self._discard_pending():
                                return
                            self._revoke('camera publishers changed while query was pending')
                            self._last_request = float('-inf')
                        return
                    else:
                        return
                else:
                    if not self._discard_pending():
                        return
                    current = self._publishers()
                    context_matches = (len(pending['futures']) == 2 and not pending['expired'] and pending['epoch'] == self._epoch and
                        pending['collector'] is self._collector and
                        pending['nonce'] == self._collector.cache_generation and current == pending['gids'])
                    if context_matches:
                        device, parameters = [future.result() for future in pending['futures']]
                        values = parameters.values
                        raw = values[0].string_value if len(values) == 1 and values[0].type == 4 else None
                        configured = raw[1:] if isinstance(raw, str) and raw.startswith('_') else raw
                        expected = self._collector.registration['camera_serial']
                        if device.serial_number != expected or configured != expected:
                            self._revoke('camera serial or configured serial_no mismatch')
                        else:
                            confirmed = self.clock()
                            target = pending['collector']
                            if (not 0 <= confirmed-pending['started'] <= QUERY_TIMEOUT_S or
                                    pending['epoch'] != self._epoch or target is not self._collector or
                                    pending['nonce'] != target.cache_generation):
                                self._revoke('camera query confirmation context expired')
                            else:
                                record = dict(schema='realsense-camera-identity-v1', serial_number=device.serial_number,
                                    serial_no_parameter=configured, generation=self._generation(current),
                                    request_started_monotonic=pending['started'], confirmed_monotonic=confirmed)
                                target.observe_camera_identity(record)
                                if pending['epoch'] == self._epoch and target is self._collector:
                                    self._identity = {'record': record, 'gids': current,
                                                      'epoch': pending['epoch'], 'collector': target}
                                    self._diagnostics = dict(record, valid=True, reason=None,
                                                             serial_no_parameter_raw=raw)
                                else:
                                    target.invalidate_camera_identity()
            if self._pending is not None or now-self._last_request < REFRESH_S:
                return
            self._last_request = now
            self._last_graph_check = now
            gids = self._publishers()
            if self._identity is not None and gids != self._identity['gids']:
                self._revoke('camera publishers changed')
            if gids is None or not self._device.service_is_ready() or not self._parameters.service_is_ready():
                self._revoke('fixed camera publishers or identity services unavailable')
                return
            target = self._collector
            if target is None or self._closed:
                return
            self._pending = dict(started=now, epoch=self._epoch, collector=target,
                nonce=target.cache_generation, gids=gids, expired=False, futures=[])
            self._pending['futures'].append(self._device.call_async(self._device_type.Request()))
            self._pending['futures'].append(self._parameters.call_async(
                self._parameter_type.Request(names=['serial_no'])))
        except (AttributeError, IndexError, RuntimeError, TypeError, ValueError) as error:
            if self._pending is not None:
                self._pending['expired'] = True
            self._revoke('camera identity query failed: '+str(error))

    def accept(self, stream, message, message_info, *, collector=None):
        """Check cached authority and DDS metadata only; no graph/service calls."""
        identity = self._identity
        collector = self._collector if collector is None else collector
        if (self._closed or identity is None or collector is None or
                identity["epoch"] != self._epoch or identity["collector"] is not collector or
                collector is not self._collector):
            return None
        now = self.clock()
        index = 0 if stream == 'depth' else 1
        gid = self._gid(message_info.get('publisher_gid') if isinstance(message_info, dict)
                        else getattr(message_info, 'publisher_gid', None))
        if gid is None:
            self._revoke('ROS message metadata publisher GID unavailable')
            return None
        if gid != identity['gids'][index]:
            self._revoke('camera publisher GID changed')
            return None
        if not 0 <= now-identity['record']['request_started_monotonic'] <= LEASE_S:
            self._revoke('camera identity lease expired')
            return None
        try:
            header = message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
        except (AttributeError, TypeError, ValueError):
            self._revoke('camera header is invalid')
            return None
        previous = self._last_header.get(stream)
        received = self._last_received.get(stream)
        if ((previous is not None and header < previous) or
                (received is not None and now-received > STREAM_GAP_S)):
            self._revoke('camera header rollback or stream restart')
            return None
        if previous == header:
            return None
        self._last_header[stream] = header
        self._last_received[stream] = now
        if identity['epoch'] != self._epoch or collector is not self._collector:
            return None
        return identity['record']['generation']

    def _parameter_event(self, event):
        if event.node == CAMERA_NODE and any(parameter.name == 'serial_no' for parameter in
                (*event.changed_parameters, *event.new_parameters, *event.deleted_parameters)):
            self._revoke('camera serial_no parameter changed')

    def snapshot(self):
        result = dict(self._diagnostics)
        identity = self._identity
        result['valid'] = bool(not self._closed and identity is not None and
            identity['epoch'] == self._epoch and identity['collector'] is self._collector and
            0 <= self.clock()-identity['record']['request_started_monotonic'] <= LEASE_S)
        return result

    def close(self):
        self._closed = True
        self._revoke('bridge closed')
        self._collector = None
