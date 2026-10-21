"""Fresh-cohort scheduling fences, replayed without ROS, images, GPU or devices."""
import ast
from pathlib import Path
from threading import Event, Lock, RLock, Thread
from types import SimpleNamespace as NS
from unittest.mock import Mock
import sys
import time

import pytest

ROOT = Path(__file__).resolve().parents[1]
OBJECTS = ROOT / 'ros2/src/so101_station_objects/so101_station_objects/objects.py'
DERIVED = ROOT / 'ros2/src/so101_station_perception/so101_station_perception/derived_aligned.py'
sys.path.insert(0, str(ROOT / 'ros2/src/so101_station_objects'))
from so101_station_objects.analysis_worker import LatestOnlyAnalysisWorker


def production_class(path, names, scope, base=object):
    """Compile the actual production methods, retaining their lock/fence ordering."""
    tree = ast.parse(path.read_text())
    methods = [n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name in names]
    assert {n.name for n in methods} == set(names)
    cls = ast.ClassDef(name='Replay', bases=[ast.Name(id='Base', ctx=ast.Load())],
                       keywords=[], body=methods, decorator_list=[])
    module = ast.fix_missing_locations(ast.Module(body=[cls], type_ignores=[]))
    scope = dict(scope, Base=base)
    exec(compile(module, str(path), 'exec'), scope)
    return scope['Replay']


def scheduler():
    clock = [10.]
    cls = production_class(OBJECTS, {'_offer_derived', '_fresh_current', '_process_offered_derived'},
                           {'time': NS(monotonic=lambda: clock[0])})
    node = cls()
    node._fresh_lock = Lock(); node._fresh_signal_gate = Lock()
    node._fresh_pending = None; node._fresh_generation = 0; node._fresh_closed = False
    node._fresh_next_attempt = 0.; node.period = .2
    node._fresh_guard = NS(trigger=Mock())
    node.derived = NS(token_is_current=lambda token: token.get('current', True))
    node.analysis_worker = NS(availability=lambda: {'available': True, 'ready_after': 9.})
    node._process = Mock()
    return node, clock


def token(receipt=10.):
    return dict(color=object(), aligned=object(), color_info=object(), lineage_receipt=receipt)


def test_worker_availability_covers_active_pending_and_unconsumed_result():
    entered, release, ready = Event(), Event(), Event()
    def analyze(value):
        entered.set(); assert release.wait(2); return value
    worker = LatestOnlyAnalysisWorker(analyze, on_ready=ready.set, min_start_interval_s=.5)
    try:
        initial = worker.availability()
        assert initial['available'] and initial['ready_after'] <= time.monotonic()
        worker.submit('first'); assert entered.wait(1)
        assert not worker.availability()['available']
        worker.submit('pending')
        assert not worker.availability()['available']
        # Keep the pending slot deterministic until the active result is consumed.
        with worker._condition:
            worker._pending = None
        release.set(); assert ready.wait(1)
        assert not worker.availability()['available']
        assert worker.take_result() == ('first', 'first', None)
        snapshot = worker.availability()
        assert snapshot['available']
        assert snapshot['ready_after'] >= worker._last_start + .5
        assert snapshot['ready_after'] >= worker._result_consumed_monotonic
    finally:
        release.set(); assert worker.close(1)
    assert not worker.availability()['available']


def test_worker_notifier_fault_never_becomes_available_after_result_consumed():
    worker = LatestOnlyAnalysisWorker(lambda value: value,
                                      on_ready=lambda: (_ for _ in ()).throw(RuntimeError('lost wakeup')))
    try:
        worker.submit(object())
        deadline = time.monotonic() + 1
        while worker.notification_error is None and time.monotonic() < deadline:
            time.sleep(.001)
        assert worker.notification_error
        assert worker.take_result() is not None
        assert not worker.availability()['available']
    finally:
        assert worker.close(1)


def test_offer_holds_only_latest_reference_and_invalidates_generation():
    node, _ = scheduler(); first, latest = token(), token()
    node._offer_derived(first); node._offer_derived(latest)
    assert node._fresh_pending == (latest, 0)
    assert node._fresh_pending[0] is latest
    node._offer_derived(None)
    assert node._fresh_pending is None and node._fresh_generation == 1
    assert not node._fresh_current(latest, 0)
    node._fresh_closed = True
    node._offer_derived(first)
    assert node._fresh_pending is None


@pytest.mark.parametrize('reason', ['busy', 'not_ready', 'old_receipt', 'cancelled', 'epoch', 'backoff'])
def test_scheduler_drops_ineligible_offer_before_any_processing(reason):
    node, clock = scheduler(); value = token()
    node._offer_derived(value)
    if reason == 'busy': node.analysis_worker.availability = lambda: {'available': False, 'ready_after': 9.}
    if reason == 'not_ready': node.analysis_worker.availability = lambda: {'available': True, 'ready_after': 11.}
    if reason == 'old_receipt': value['lineage_receipt'] = 8.
    if reason == 'cancelled': node._fresh_generation += 1
    if reason == 'epoch': value['current'] = False
    if reason == 'backoff': node._fresh_next_attempt = 11.
    node._process_offered_derived()
    node._process.assert_not_called()
    assert node._fresh_pending is None
    assert value['lineage_receipt'] == (8. if reason == 'old_receipt' else 10.)


def test_scheduler_accepts_fresh_cohort_after_readiness_without_renewing_receipts():
    node, clock = scheduler(); value = token()
    node._offer_derived(value); node._process_offered_derived()
    node._process.assert_called_once_with(value['color'], value['aligned'], value['color_info'], value, fresh_generation=0)
    assert node._fresh_next_attempt == pytest.approx(10.2)
    assert value['lineage_receipt'] == 10.
    clock[0] = 10.21
    stale = token(10.1); node._offer_derived(stale); node._process_offered_derived()
    assert node._process.call_count == 1
    fresh = token(10.21); node._offer_derived(fresh); node._process_offered_derived()
    assert node._process.call_count == 2


def test_metadata_offer_and_intake_have_no_image_copy_decode_tf_or_gpu_work():
    banned = {'copy', 'asarray', 'imgmsg_to_cv2', 'lookup_transform', 'analyze_rgbd',
              'publish', 'cuda', 'get', 'wait', 'submit', '_process', 'take'}
    for path, name in ((OBJECTS, '_offer_derived'), (DERIVED, '_intake')):
        tree = ast.parse(path.read_text())
        method = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == name)
        calls = {n.func.attr if isinstance(n.func, ast.Attribute) else n.func.id
                 for n in ast.walk(method) if isinstance(n, ast.Call) and isinstance(n.func, (ast.Attribute, ast.Name))}
        # Legacy non-metadata intake retains take(); metadata mode explicitly bypasses it.
        assert not (calls & (banned - {'take'}))
        if name == '_offer_derived':
            assert 'take' not in calls
            assert not any(isinstance(n, (ast.For, ast.While, ast.ListComp, ast.DictComp)) for n in ast.walk(method))
        else:
            take_if = next(n for n in ast.walk(method) if isinstance(n, ast.If)
                           and any(isinstance(x, ast.Attribute) and x.attr == 'take' for x in ast.walk(n)))
            assert 'self.metadata_offer is None' in ast.unparse(take_if.test)


def test_callback_groups_separate_metadata_intake_from_expensive_default_group():
    tree = ast.parse(OBJECTS.read_text())
    derived = next(n for n in ast.walk(tree) if isinstance(n, ast.Call) and isinstance(n.func, ast.Name) and n.func.id == 'DerivedAligned')
    kwargs = {k.arg: ast.unparse(k.value) for k in derived.keywords}
    assert kwargs['callback_group'] == kwargs['health_callback_group'] == 'self._derived_group'
    assert kwargs['metadata_offer'] == 'self._offer_derived'
    guards = [n for n in ast.walk(tree) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) and n.func.attr == 'create_guard_condition']
    assert {n.args[0].attr for n in guards} == {'_process_offered_derived', '_consume_analysis'}
    assert all(not n.keywords for n in guards)


def test_shutdown_joins_metadata_trigger_before_guard_destruction():
    entered, release, destroyed, done = Event(), Event(), Event(), Event()
    class Base:
        def destroy_node(self): destroyed.set(); return True
    cls = production_class(OBJECTS, {'_offer_derived', 'destroy_node'}, {}, Base)
    node, _ = scheduler()
    node.__class__ = cls
    node._fresh_guard = NS(trigger=lambda: (entered.set(), release.wait(2)))
    node.derived = None; node.analysis_worker = None; node.timing_recorder = None
    offer = Thread(target=node._offer_derived, args=(token(),)); offer.start()
    assert entered.wait(1)
    shutdown = Thread(target=lambda: (node.destroy_node(), done.set())); shutdown.start()
    try:
        assert not destroyed.wait(.02)
        release.set(); assert done.wait(1)
        assert destroyed.is_set() and node._fresh_closed and node._fresh_pending is None
    finally:
        release.set(); offer.join(1); shutdown.join(1)


@pytest.mark.parametrize('stage', ['before_copy', 'before_submit'])
def test_generation_cancellation_fences_actual_process_copy_and_submit(monkeypatch, stage):
    node, _ = scheduler(); copies = []; warnings = []; unavailable = []
    class Array:
        shape = (2, 2)
        def copy(self):
            copies.append(self)
            if stage == 'before_submit': node._offer_derived(None)
            return self
    array = Array()
    tag_helpers = NS(camera_intrinsics=lambda *args: {'fx': 1.},
                     depth_scale_for_encoding=lambda *args: .001,
                     stamp_is_fresh=lambda now, stamp, limit: 0 <= now-stamp <= limit,
                     stamp_seconds=lambda stamp: stamp.sec + stamp.nanosec * 1e-9)
    monkeypatch.setitem(sys.modules, 'so101_station_perception.tag_observations', tag_helpers)
    scope = dict(time=time, np=NS(asarray=lambda *args, **kwargs: array),
                 Time=NS(from_msg=lambda stamp: stamp), Duration=lambda **kwargs: None,
                 transform_from_msg=lambda tf: array, cv2=NS(error=RuntimeError),
                 TransformException=RuntimeError)
    cls = production_class(OBJECTS, {'_offer_derived', '_fresh_current', '_process'}, scope)
    node.__class__ = cls; del node._process
    checked = []
    def check(value):
        checked.append(value)
        if len(checked) == 2 and stage == 'before_copy': node._offer_derived(None)
        return True
    node.derived = NS(check=check, token_is_current=lambda value: True)
    node.timing_recorder = None; node.require_registration_tracking = False
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=10_120_000_000))
    node.bridge = NS(imgmsg_to_cv2=lambda *args, **kwargs: array)
    node.tf_buffer = NS(lookup_transform=lambda *args, **kwargs: object())
    node.board_frame = 'board'; node.depth_scale = .001; node.last_support_plane = None
    node.last_warning = 0.; node.get_logger = lambda: NS(warning=warnings.append)
    node._publish_unavailable = lambda *args: unavailable.append(args)
    node.analysis_worker = NS(submit=Mock())
    msg = NS(header=NS(stamp=NS(sec=10, nanosec=0), frame_id='camera'), encoding='16UC1')
    info = NS(header=msg.header, d=[0.]*5)
    value = token(); node._process(msg, msg, info, value, fresh_generation=0)
    node.analysis_worker.submit.assert_not_called()
    assert len(checked) == 2
    assert unavailable and 'cancelled before' in unavailable[-1][-1]
    assert bool(copies) == (stage == 'before_submit')
    assert value['lineage_receipt'] == 10.


def test_derived_token_epoch_pin_and_shutdown_fences_do_not_admit_sources():
    cls = production_class(DERIVED, {'token_is_current', '_invalidate_offers', 'close'}, {})
    adapter = cls(); epoch = object(); invalidations = []
    adapter.lock = RLock(); adapter.closed = False
    ledger = NS(invalid_reason=None, pin={'producer_generation': 'current'})
    adapter.core = NS(epoch=epoch, ledger=ledger, close=lambda: None)
    adapter.metadata_offer = invalidations.append; adapter.timer = NS(cancel=Mock())
    value = {'epoch': epoch, 'statement': {'producer_generation': 'current'}}
    assert adapter.token_is_current(value)
    assert not adapter.token_is_current(dict(value, epoch=object()))
    value['statement']['producer_generation'] = 'old'
    assert not adapter.token_is_current(value)
    value['statement']['producer_generation'] = 'current'; ledger.invalid_reason = 'expired'
    assert not adapter.token_is_current(value)
    adapter.close()
    assert invalidations == [None] and adapter.closed
    adapter.timer.cancel.assert_called_once()
    assert not adapter.token_is_current(value)


def test_derived_metadata_intake_defers_take_and_offers_drain_reference_only():
    cls = production_class(DERIVED, {'_intake', 'drain', '_invalidate_offers'},
                           {'time': NS(monotonic=lambda: 10.)})
    adapter = cls(); calls = []; value = token()
    adapter.lock = RLock(); adapter.closed = False; adapter.fault = None
    adapter.period = .2; adapter.last_take = 10.
    adapter.metadata_offer = calls.append; adapter.callback = Mock()
    adapter._clock = lambda: {'now_ns': 10_000_000_000, 'now_mono': 10.}
    adapter.node = NS(get_clock=lambda: NS(now=lambda: NS(nanoseconds=10_000_000_000)))
    adapter._validate_received_info = Mock()
    adapter.core = NS(receive=Mock(), lineage=Mock(), take=Mock(return_value=value),
                      clear_observations=Mock())
    message = object()
    adapter._intake('color', message, b'gid')
    adapter.core.receive.assert_called_once_with('color', message, b'gid', now_ns=10_000_000_000, now_mono=10.)
    adapter.core.take.assert_not_called()
    adapter.callback.assert_not_called()
    adapter.drain()
    assert calls == [value] and calls[0] is value
    adapter.callback.assert_not_called()
    adapter.core.receive.side_effect = ValueError('binding invalidated')
    adapter._intake('color', message, b'gid')
    assert calls[-1] is None and adapter.fault == 'binding invalidated'
    adapter.core.clear_observations.assert_called_once()


def test_derived_metadata_timer_uses_intake_group_without_changing_legacy_timer():
    tree = ast.parse(DERIVED.read_text())
    method = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == '__init__')
    choice = next(n for n in method.body if isinstance(n, ast.If)
                  and ast.unparse(n.test) == 'self.metadata_offer is not None')
    metadata_timer = next(n for n in ast.walk(ast.Module(body=choice.body, type_ignores=[]))
                          if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) and n.func.attr == 'create_timer')
    assert ast.literal_eval(metadata_timer.args[0].left) / ast.literal_eval(metadata_timer.args[0].right) == 1./30.
    assert {k.arg: ast.unparse(k.value) for k in metadata_timer.keywords} == {'callback_group': 'callback_group'}
    legacy_timer = next(n for n in ast.walk(ast.Module(body=choice.orelse, type_ignores=[]))
                        if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) and n.func.attr == 'create_timer')
    assert legacy_timer.args[0].value == .01 and not legacy_timer.keywords


def test_worker_availability_is_bounded_scalar_metadata_only():
    import inspect
    import textwrap
    method = ast.parse(textwrap.dedent(inspect.getsource(LatestOnlyAnalysisWorker.availability)))
    assert not any(isinstance(n, (ast.For, ast.While, ast.ListComp, ast.DictComp,
                                  ast.SetComp, ast.GeneratorExp)) for n in ast.walk(method))
    calls = [n for n in ast.walk(method) if isinstance(n, ast.Call)]
    assert len(calls) == 1 and isinstance(calls[0].func, ast.Name) and calls[0].func.id == 'max'
    result = next(n for n in ast.walk(method) if isinstance(n, ast.Dict))
    assert {key.value for key in result.keys} == {'available', 'ready_after'}
