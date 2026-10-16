"""Producer lifecycle/admission wiring, fake owner only on domain75."""
import ast
import inspect
import textwrap
from types import SimpleNamespace

import pytest
pytest.importorskip('rclpy')
from test_ros_follower_recovery_status import ready as owner_ready, state, status
from test_ros_stop_service import bridge
from so101_station_control.trajectory_server import TrajectoryBridge


NONCE = '1'*32


def producer_state(node, **changes):
    value = state(node, camera_recovery_cache_generation=NONCE,
                  camera_recovery_runtime_started_monotonic=node._owner_started+.001)
    value.update(changes)
    return value


@pytest.fixture
def ready(owner_ready):
    node, process = owner_ready
    node._read_state = lambda **kwargs: producer_state(node)
    node._cache_owner_state(producer_state(node))
    node._recovery_camera_identity = SimpleNamespace(set_collector=lambda value: None,
        close=lambda: None, snapshot=lambda: {}, accept=lambda *a, **k: "generation")
    yield node, process


class Collector:
    def __init__(self):
        self.samples = []
        self.cache_generation = NONCE
        self.runtime_started_monotonic = None
        self.generations = []
        self.closed = False
    def observe_owner_sample(self, sample, generation):
        if not self.closed:
            self.samples.append((sample, generation))
    def invalidate_generation(self, generation):
        self.generations.append(generation)
    def close(self):
        self.closed = True
    def snapshot(self):
        return {'state': 'testing', 'published': 0}


def test_only_admitted_cache_samples_reach_producer(ready):
    node, process = ready
    collector = Collector()
    collector.runtime_started_monotonic = node._owner_started+.001
    node._recovery_depth_cache = collector
    sample = producer_state(node, timestamp_utc='2026-10-16T00:00:00+00:00')
    node._cache_owner_state(sample, generation=node._owner_started, process=process)
    assert len(collector.samples) == 1
    retained, token = collector.samples[0]
    assert token == (process, node._owner_started)
    assert retained['timestamp_utc'] == sample['timestamp_utc']
    sample['raw']['gripper'] = 9999
    assert retained['raw']['gripper'] != 9999
    assert collector.generations == []


@pytest.mark.parametrize('fault', ['source', 'identity', 'old_stamp', 'future', 'raw', 'torque',
                                   'mode', 'dead', 'closed'])
def test_untrusted_or_disconnected_sample_revokes_producer_generation(ready, fault):
    node, process = ready
    collector = Collector()
    collector.runtime_started_monotonic = node._owner_started+.001
    node._recovery_depth_cache = collector
    sample = producer_state(node)
    if fault == 'source': sample['owner_source_sha256']['recovery_runtime_config'] = 'f'*64
    if fault == 'identity': sample['arm_id'] = 'other'
    if fault == 'old_stamp': sample['monotonic'] = node._owner_started-.001
    if fault == 'future': sample['monotonic'] += 1
    if fault == 'raw': sample['raw']['gripper'] = 9999
    if fault == 'torque': sample['torque']['gripper'] = 128
    if fault == 'mode': sample['observe_only'] = None
    if fault == 'dead': process.alive = False
    if fault == 'closed': node._closed = True
    node._cache_owner_state(sample, generation=node._owner_started, process=process)
    assert collector.samples == []
    assert collector.generations == [None]


def test_late_old_reader_cannot_revoke_or_feed_current_collector(ready):
    node, process = ready
    collector = Collector()
    collector.runtime_started_monotonic = node._owner_started+.001
    node._recovery_depth_cache = collector
    node._cache_owner_state(producer_state(node), generation=node._owner_started-.001,
                            process=SimpleNamespace(poll=lambda: None))
    assert collector.samples == [] and collector.generations == []


def test_failed_read_revokes_pending_depth_cohort(ready):
    node, process = ready
    collector = Collector()
    collector.runtime_started_monotonic = node._owner_started+.001
    node._recovery_depth_cache = collector
    assert TrajectoryBridge._read_state(node) is None
    assert collector.generations == [None]


def test_status_observes_only_cached_producer_diagnostics(ready, monkeypatch):
    node, process = ready
    collector = Collector()
    collector.runtime_started_monotonic = node._owner_started+.001
    node._recovery_depth_cache = collector
    monkeypatch.setattr(node, '_read_state', lambda **kwargs: pytest.fail('status performed I/O'))
    assert status(node)['recovery_depth_cache'] == collector.snapshot()
    assert collector.samples == [] and process.commands == []


def test_collector_constructor_and_generation_replay(ready, monkeypatch):
    from so101_station_control import recovery_depth_cache
    node, process = ready
    node._recovery_cache_config = {'synthetic': True}
    collector = Collector()
    calls = []
    def construct(node_arg, **kwargs):
        calls.append((node_arg, kwargs))
        collector.cache_generation = kwargs["cache_generation"]
        collector.runtime_started_monotonic = kwargs["runtime_started_monotonic"]
        return collector
    monkeypatch.setattr(recovery_depth_cache, 'RecoveryDepthCache', construct)
    node._configure_recovery_depth_cache()
    assert calls[0][0] is None
    assert calls[0][1]['expected_owner_sources'] == node._owner_source_expected
    assert calls[0][1]['state_file'] == node._state_file
    assert calls[0][1]['depth_topic'] == '/camera/aligned_depth_to_color/image_raw'
    assert collector.generations == [(process, node._owner_started)]
    assert len(collector.samples) == 1
    assert node._recovery_depth_cache is collector
    node._close_recovery_depth_cache()
    assert collector.closed and node._recovery_depth_cache is None
    node._configure_recovery_depth_cache()
    assert len(calls) == 1  # An asynchronous retired writer owns its old nonce forever.


@pytest.mark.parametrize('fault', ['disabled', 'external', 'dead', 'closed'])
def test_producer_never_constructs_without_live_owned_authority(ready, monkeypatch, fault):
    from so101_station_control import recovery_depth_cache
    node, process = ready
    node._recovery_cache_config = {'synthetic': True}
    if fault == 'disabled': node._recovery_cache_config = None
    if fault == 'external': node._external_state_only = True
    if fault == 'dead': process.alive = False
    if fault == 'closed': node._closed = True
    monkeypatch.setattr(recovery_depth_cache, 'RecoveryDepthCache',
                        lambda *args, **kwargs: pytest.fail('unauthorized producer'))
    node._configure_recovery_depth_cache()
    assert node._recovery_depth_cache is None


def test_constructor_shutdown_race_cannot_leave_worker_attached(ready, monkeypatch):
    from so101_station_control import recovery_depth_cache
    node, process = ready
    node._recovery_cache_config = {'synthetic': True}
    collector = Collector()
    def construct(*args, **kwargs):
        node._closed = True
        return collector
    monkeypatch.setattr(recovery_depth_cache, 'RecoveryDepthCache', construct)
    node._configure_recovery_depth_cache()
    assert collector.closed and node._recovery_depth_cache is None


def test_producer_hook_never_moves_into_teleop_or_hashes_sources():
    tick = inspect.getsource(TrajectoryBridge._teleop_tick)
    assert 'observe_owner_sample' not in tick and '_cache_owner_state' not in tick
    tree = ast.parse(textwrap.dedent(inspect.getsource(TrajectoryBridge._cache_owner_state)))
    calls = {item.func.attr for item in ast.walk(tree)
             if isinstance(item, ast.Call) and isinstance(item.func, ast.Attribute)}
    assert not calls.intersection({'sha256', 'read_text', 'read_bytes', 'open', 'write_text', 'write_bytes'})


def test_subscriptions_are_bounded_and_route_only_to_current_collector(ready, monkeypatch):
    node, process = ready
    subscriptions = []
    def subscribe(message_type, topic, callback, qos, **kwargs):
        subscription = SimpleNamespace(topic=topic, callback=callback, qos=qos)
        subscriptions.append(subscription)
        return subscription
    monkeypatch.setattr(node, 'create_subscription', subscribe)
    node._ensure_recovery_depth_subscriptions()
    node._ensure_recovery_depth_subscriptions()
    assert len(subscriptions) == 2
    assert all(subscription.qos.depth == 1 for subscription in subscriptions)
    first, second = Collector(), Collector()
    seen = []
    first._on_depth = lambda message, **kwargs: seen.append(('first', message))
    second._on_depth = lambda message, **kwargs: seen.append(('second', message))
    second._on_info = lambda message, **kwargs: seen.append(('info', message))
    node._recovery_depth_cache = first
    subscriptions[0].callback('a', None)
    node._close_recovery_depth_cache()
    node._recovery_depth_cache = second
    node._ensure_recovery_depth_subscriptions()
    subscriptions[0].callback('b', None)
    subscriptions[1].callback('c', None)
    assert seen == [('first', 'a'), ('second', 'b'), ('info', 'c')]
    assert first.closed and len(subscriptions) == 2
    node._closed = True
    subscriptions[0].callback('ignored', None)
    assert len(seen) == 3


def test_partial_subscription_failure_stays_tracked_and_bounded(ready, monkeypatch):
    node, process = ready
    live = []
    attempts = []
    def subscribe(message_type, topic, callback, qos, **kwargs):
        attempts.append(topic)
        if len(attempts) == 2:
            raise ValueError('synthetic second-subscription failure')
        subscription = SimpleNamespace(topic=topic, callback=callback)
        live.append(subscription)
        return subscription
    monkeypatch.setattr(node, 'create_subscription', subscribe)
    with pytest.raises(ValueError, match='second-subscription'):
        node._ensure_recovery_depth_subscriptions()
    assert len(live) == len(node._recovery_depth_subscriptions) == 1
    node._ensure_recovery_depth_subscriptions()
    node._ensure_recovery_depth_subscriptions()
    assert len(live) == len(node._recovery_depth_subscriptions) == 2
    assert attempts.count('/camera/aligned_depth_to_color/image_raw') == 1


@pytest.mark.parametrize('fault', ['missing_nonce', 'bad_nonce', 'runtime_before_owner',
                                   'runtime_after_acquisition', 'source', 'identity'])
def test_collector_requires_current_runtime_nonce_authority(ready, monkeypatch, fault):
    from so101_station_control import recovery_depth_cache
    node, process = ready
    node._recovery_cache_config = {'synthetic': True}
    sample = producer_state(node)
    if fault == 'missing_nonce': sample.pop('camera_recovery_cache_generation')
    if fault == 'bad_nonce': sample['camera_recovery_cache_generation'] = 'z'*32
    if fault == 'runtime_before_owner': sample['camera_recovery_runtime_started_monotonic'] = node._owner_started-1
    if fault == 'runtime_after_acquisition': sample['camera_recovery_runtime_started_monotonic'] = sample['monotonic']+1
    if fault == 'source': sample['owner_source_sha256']['arm'] = 'f'*64
    if fault == 'identity': sample['arm_id'] = 'other'
    node._cache_owner_state(sample)
    monkeypatch.setattr(recovery_depth_cache, 'RecoveryDepthCache',
                        lambda *args, **kwargs: pytest.fail('untrusted runtime producer'))
    node._configure_recovery_depth_cache()
    assert node._recovery_depth_cache is None
