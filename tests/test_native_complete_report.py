"""Synthetic complete-report admission; no ROS, devices, or live capability."""
import copy
from dataclasses import replace
import json
from pathlib import Path
import sys
from threading import Event, Thread

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros2/src/so101_station_objects'))
from so101_station_objects.native_complete_report import (
    NativeIdentity, ReportLimits, NativeCompleteReport, NativeCompleteIngress)
from test_complete_report_admission import decode_baseline, encode_baseline


def retained(index=0):
    fixture = json.loads((Path(__file__).parent / 'fixtures/complete_report_admission/cpu_baseline.json').read_text())
    report = decode_baseline(fixture['cases'][index]['analyzer_report'])
    for key in ('inlier_mask', 'residuals_m'):
        report['support_plane'].pop(key, None)
    for obj in report['objects']:
        obj.pop('contour', None)
    return report


def identity():
    return NativeIdentity(object(), 1, 10, 10, 100., 100., 1, 1, 'color', 'depth',
                          object(), object(), object(), 10_000_000_000, object())


class Provider:
    """Explicit fixture-only source/clock/capability registry, never production."""
    def __init__(self, ident):
        self.limits = ReportLimits(1, 1)
        self.frame_ids = ('color', 'depth')
        self.backend_status = retained()['color_mask_backend']
        self.originals = [ident]
        self.handles = {}
        self.seen = set()
        self.revoked = False

    def admit(self, ident, report):
        assert not self.revoked and any(ident == record for record in self.originals)
        capture = (ident.session_token, ident.color_frame_number, ident.depth_frame_number)
        assert capture not in self.seen
        self.seen.add(capture)
        handle = object()
        self.handles[handle] = ident
        return handle

    def check(self, handle):
        assert not self.revoked and handle in self.handles

    def publish_if_current(self, handle, action):
        self.check(handle)
        action()
        return True


def setup(notify=lambda: None):
    ident = identity()
    provider = Provider(ident)
    ingress = NativeCompleteIngress(provider, notify)
    return ident, provider, ingress


@pytest.mark.parametrize('index', range(5))
def test_retained_projection_roundtrips_exactly(index):
    ident, provider, ingress = setup()
    report = retained(index)
    expected = encode_baseline(report)
    assert ingress.offer(NativeCompleteReport(ident, report))
    admitted = ingress.take()
    assert admitted.identity is ident and admitted.admission in provider.handles
    assert encode_baseline(admitted.report) == expected
    assert ingress.take() is None


def test_visible_only_and_optional_grasp_absence():
    ident, _, ingress = setup()
    report = retained()
    obj = report['objects'][0]
    obj.update(height_m=None, grasps=[], pickup_eligible=False,
               eligibility_reason='not_support_plane_qualified',
               observation_geometry='visible_surface_centroid', dimensions_semantics='visible_surface_extent_only')
    assert ingress.offer(NativeCompleteReport(ident, report))
    assert encode_baseline(ingress.take().report) == encode_baseline(report)


@pytest.mark.parametrize('change', [
    lambda r: r.update(stamp=10),
    lambda r: r.update(qualified=True),
    lambda r: r.update(objects=r['objects']*2),
    lambda r: r['support_plane'].update(residuals_m=np.zeros(10000)),
    lambda r: r['support_plane'].update(normal=np.zeros(3000)),
    lambda r: r['objects'][0].update(track_id=1),
    lambda r: r['objects'][0].update(acquisition_state='confirmed'),
    lambda r: r['objects'][0].update(execution_eligible=True),
    lambda r: r['objects'][0].update(position=np.array([0., 0., np.nan])),
    lambda r: r['objects'][0].update(orientation_matrix=np.ones((3, 3))),
    lambda r: r['objects'][0].update(depth_points=True),
    lambda r: r['objects'][0].update(grasps=r['objects'][0]['grasps']*2),
    lambda r: r['objects'][0]['grasps'][0].update(candidate_id=3),
    lambda r: r['objects'][0]['grasps'][0].update(contacts_camera=np.zeros((3, 3))),
    lambda r: r['objects'][0]['depth_validation']['metrics'].update(dense=np.ones((100, 100))),
    lambda r: r['color_mask_backend'].update(device={'dense': np.ones(10000)}),
    lambda r: r['color_mask_backend'].update(selected='vulkan'),
    lambda r: r['validation_diagnostics'].update(rejected_samples=[{}]*6),
    lambda r: r['objects'][0].update(image_boundary_sides=['right', 'left']),
    lambda r: r['objects'][0]['silhouette_validation']['metrics'].pop('observed_pixels'),
])
def test_closed_schema_rejects_whole_report_and_clears_pending(change):
    ident, provider, ingress = setup()
    assert ingress.offer(NativeCompleteReport(ident, retained()))
    changed = retained()
    change(changed)
    assert not ingress.offer(NativeCompleteReport(ident, changed))
    assert ingress.take() is None and ingress.rejected == 1


def test_no_custom_conversion_or_container_hooks():
    class Evil:
        def __array__(self, *args):
            pytest.fail('untrusted conversion')
    class Custom(dict):
        def items(self):
            pytest.fail('custom mapping')
    for value in (Evil(), Custom()):
        ident, _, ingress = setup()
        report = retained()
        report['objects'][0]['position'] = value
        assert not ingress.offer(NativeCompleteReport(ident, report))


@pytest.mark.parametrize('field,value', [
    ('session_token', True), ('session_token', 0), ('depth_frame_number', -1),
    ('color_clock_domain', 2**32), ('observation_ros_ns', 0),
    ('color_hardware_time_ms', float('inf')), ('depth_frame_id', 'x'*10000),
])
def test_identity_scalar_contract(field, value):
    ident, _, ingress = setup()
    assert not ingress.offer(NativeCompleteReport(replace(ident, **{field: value}), retained()))


@pytest.mark.parametrize('field,value', [
    ('producer_epoch', object()), ('session_token', 2), ('configuration_epoch', object()),
    ('transform_epoch', object()), ('clock_mapping_epoch', object()), ('calibration_epoch', object()),
    ('observation_ros_ns', 10_000_000_001), ('color_hardware_time_ms', 101.),
])
def test_registry_rejects_changed_epochs_and_restamps(field, value):
    ident, _, ingress = setup()
    assert not ingress.offer(NativeCompleteReport(replace(ident, **{field: value}), retained()))


def test_duplicate_original_capture_is_not_independent_observation():
    ident, _, ingress = setup()
    assert ingress.offer(NativeCompleteReport(ident, retained()))
    assert not ingress.offer(NativeCompleteReport(ident, retained()))
    assert ingress.take() is None


def test_owned_arrays_lists_and_dicts_and_replacement():
    ident, provider, ingress = setup()
    report = retained()
    expected = encode_baseline(report)
    assert ingress.offer(NativeCompleteReport(ident, report))
    report['objects'][0]['position'][:] = 99
    report['objects'][0]['depth_validation']['metrics']['centroid_camera_m'][0] = 99
    report['validation_diagnostics']['color_parent_components'] = 99
    admitted = ingress.take()
    assert encode_baseline(admitted.report) == expected
    for index in (11, 12):
        next_id = replace(ident, color_frame_number=index, depth_frame_number=index)
        provider.originals.append(next_id)
        assert ingress.offer(NativeCompleteReport(next_id, retained()))
    assert ingress.replaced == 1 and ingress.take().identity.color_frame_number == 12


@pytest.mark.parametrize('method', ['invalidate', 'close'])
def test_lifetime_fences_inflight_admission(method):
    ident, provider, ingress = setup()
    entered, release = Event(), Event()
    original = provider.admit
    def admit(*args):
        entered.set()
        assert release.wait(2)
        return original(*args)
    provider.admit = admit
    accepted = []
    thread = Thread(target=lambda: accepted.append(ingress.offer(NativeCompleteReport(ident, retained()))))
    thread.start()
    assert entered.wait(2)
    getattr(ingress, method)()
    release.set()
    thread.join(2)
    assert not thread.is_alive() and accepted == [False] and ingress.take() is None
    ingress.close()
    ingress.close()


def test_revocation_and_notification_failure():
    ident, provider, ingress = setup()
    assert ingress.offer(NativeCompleteReport(ident, retained()))
    provider.revoked = True
    assert ingress.take() is None
    def broken():
        raise ValueError('arbitrary untrusted exception payload')
    ident, _, ingress = setup(broken)
    assert not ingress.offer(NativeCompleteReport(ident, retained()))
    assert ingress.take() is None and ingress.notification_failures == 1
    assert ingress.take_fault() == 'native report notification failed'
    assert ingress.take_fault() is None


@pytest.mark.parametrize('kind', ['missing', 'no_limits', 'no_fence', 'bool_handle', 'none_handle', 'bool_check'])
def test_provider_capabilities_and_handles_are_not_packet_proofs(kind):
    ident = identity()
    provider = Provider(ident)
    if kind == 'missing': provider = None
    if kind == 'no_limits': provider.limits = None
    if kind == 'no_fence': provider.publish_if_current = None
    if kind == 'bool_handle': provider.admit = lambda *args: True
    if kind == 'none_handle': provider.admit = lambda *args: None
    if kind == 'bool_check': provider.check = lambda *args: True
    ingress = NativeCompleteIngress(provider, lambda: None)
    assert not ingress.offer(NativeCompleteReport(ident, retained()))
    assert not ingress.offer({'valid': True, 'objects': []})
    assert ingress.take() is None


@pytest.mark.parametrize('stage', [0, 1, 2])
def test_rejected_depth_reached_stage_metrics_and_five_sample_limit(stage):
    ident, _, ingress = setup()
    report = retained(4)
    sample = report['validation_diagnostics']['rejected_samples'][0]
    keys = ['color_pixels', 'local_support_pixels', 'semantic_identity', 'workspace_gate_applied', 'support_plane_gate_applied']
    if stage >= 1:
        keys += ['supported_fraction', 'invalid_local_depth_fraction', 'core_supported_fraction']
    if stage < 2:
        sample['metrics'] = {key: sample['metrics'][key] for key in keys}
        sample['reason'] = 'no_color_support' if stage == 0 else 'insufficient_depth_support'
        report['validation_diagnostics']['rejection_reasons'] = {sample['reason']: 5}
    report['validation_diagnostics']['rejected_samples'] = [copy.deepcopy(sample) for _ in range(5)]
    assert ingress.offer(NativeCompleteReport(ident, report))
    assert encode_baseline(ingress.take().report) == encode_baseline(report)


def test_retained_analytic_scene_visible_geometry_uses_original_cpu_source():
    from test_ros2_objects import tabletop_scene, analyze_rgbd
    rgb, depth, intrinsics, distortion, transform = tabletop_scene()
    original = analyze_rgbd(rgb, depth, intrinsics, distortion, .001, transform,
                            max_object_height_m=.01)
    assert original['objects'][0]['observation_geometry'] == 'visible_surface_centroid'
    original['support_plane'].pop('inlier_mask')
    original['support_plane'].pop('residuals_m')
    for obj in original['objects']:
        obj.pop('contour', None)
    ident, _, ingress = setup()
    assert ingress.offer(NativeCompleteReport(ident, original))
    assert encode_baseline(ingress.take().report) == encode_baseline(original)


@pytest.mark.parametrize('stage', [0, 1, 2])
def test_rejected_silhouette_reached_stage_exact_absence(stage):
    report = retained(4)
    metrics = retained()['objects'][0]['silhouette_validation']['metrics']
    initial = ['min_rectangle_fill_required', 'semantic_identity', 'applied_to', 'workspace_gate_applied']
    if stage >= 1:
        initial += ['min_rectangle_fill', 'contour_area_px2', 'convex_hull_fill', 'shape_evidence']
    if stage < 2:
        metrics = {key: metrics[key] for key in initial}
        if stage == 1:
            metrics['shape_evidence'] = 'contour_geometry_only'
    sample = report['validation_diagnostics']['rejected_samples'][0]
    sample.update(metrics=metrics, reason='insufficient_silhouette_geometry' if stage == 0
                  else 'insufficient_observed_silhouette' if stage == 1
                  else 'silhouette_inconsistent_with_convex_brick')
    report['validation_diagnostics']['rejection_reasons'] = {sample['reason']: 1}
    ident, _, ingress = setup()
    assert ingress.offer(NativeCompleteReport(ident, report))
    assert encode_baseline(ingress.take().report) == encode_baseline(report)


def test_optional_grasp_fields_remain_absent_and_provider_metadata_is_snapshotted():
    ident, provider, ingress = setup()
    report = retained()
    grasp = report['objects'][0]['grasps'][0]
    del grasp['axis_rank'], grasp['contacts_camera']
    provider.backend_status['selected'] = 'changed later'
    provider.frame_ids = ('changed', 'changed')
    assert ingress.offer(NativeCompleteReport(ident, report))
    grasp = ingress.take().report['objects'][0]['grasps'][0]
    assert set(grasp) == {'position', 'closing_axis', 'approach_direction', 'opening_m'}


def test_notification_runs_outside_pending_lock_and_invalidation_cannot_revive_work():
    entered, release = Event(), Event()
    def notify():
        entered.set()
        assert release.wait(2)
        raise RuntimeError('late callback')
    ident, _, ingress = setup(notify)
    results = []
    thread = Thread(target=lambda: results.append(ingress.offer(NativeCompleteReport(ident, retained()))))
    thread.start()
    assert entered.wait(2)
    ingress.invalidate()
    assert ingress._pending is None
    release.set()
    thread.join(2)
    assert not thread.is_alive() and results == [False]
    assert ingress.take() is None and ingress.take_fault() is None


def test_two_concurrent_offers_keep_one_owned_pending_report():
    ident, provider, ingress = setup()
    second = replace(ident, color_frame_number=11, depth_frame_number=11)
    provider.originals.append(second)
    entered, release = Event(), Event()
    original = provider.admit
    def admit(identity, report):
        if identity is ident:
            entered.set()
            assert release.wait(2)
        return original(identity, report)
    provider.admit = admit
    results = []
    thread = Thread(target=lambda: results.append(ingress.offer(NativeCompleteReport(ident, retained()))))
    thread.start()
    assert entered.wait(2)
    assert ingress.offer(NativeCompleteReport(second, retained()))
    release.set()
    thread.join(2)
    assert not thread.is_alive() and results == [True]
    assert ingress.replaced == 1
    assert ingress.take() is not None and ingress.take() is None


@pytest.mark.parametrize('value', [-1, True, 1.0])
def test_provider_limits_require_exact_nonnegative_ints(value):
    with pytest.raises(ValueError):
        ReportLimits(value, 1)
    with pytest.raises(ValueError):
        ReportLimits(1, value)


def test_invalidate_during_successful_notify_fences_offer_result():
    entered, release = Event(), Event()
    def notify():
        entered.set()
        assert release.wait(2)
    ident, _, ingress = setup(notify)
    results = []
    thread = Thread(target=lambda: results.append(ingress.offer(NativeCompleteReport(ident, retained()))))
    thread.start()
    assert entered.wait(2)
    ingress.invalidate()
    release.set()
    thread.join(2)
    assert not thread.is_alive() and results == [False] and ingress.take() is None


@pytest.mark.parametrize('failure', [False, True])
def test_take_serializes_with_notify_outcome(failure):
    entered, release, taken = Event(), Event(), Event()
    def notify():
        entered.set()
        assert release.wait(2)
        if failure:
            raise RuntimeError('failed signal')
    ident, _, ingress = setup(notify)
    offered, delivered = [], []
    producer = Thread(target=lambda: offered.append(ingress.offer(NativeCompleteReport(ident, retained()))))
    producer.start()
    assert entered.wait(2)
    def take():
        delivered.append(ingress.take())
        taken.set()
    executor = Thread(target=take)
    executor.start()
    assert not taken.wait(.05)
    release.set()
    producer.join(2)
    executor.join(2)
    assert not producer.is_alive() and not executor.is_alive()
    assert offered == [not failure]
    assert (delivered[0] is None) == failure


def test_synchronous_notify_take_is_deferred_without_lost_pending():
    ident = identity()
    provider = Provider(ident)
    seen = []
    ingress = NativeCompleteIngress(provider, lambda: seen.append(ingress.take()))
    assert ingress.offer(NativeCompleteReport(ident, retained()))
    assert seen == [None]
    assert ingress.take() is not None


def test_close_waits_until_notifier_has_returned_before_guard_destruction():
    entered, release, closed = Event(), Event(), Event()
    def notify():
        entered.set()
        assert release.wait(2)
    ident, _, ingress = setup(notify)
    producer = Thread(target=lambda: ingress.offer(NativeCompleteReport(ident, retained())))
    producer.start()
    assert entered.wait(2)
    closer = Thread(target=lambda: (ingress.close(), closed.set()))
    closer.start()
    assert not closed.wait(.05)
    # Close already fenced/discarded while joining the notifier.
    assert ingress._closed and ingress._pending is None
    release.set()
    producer.join(2)
    closer.join(2)
    assert not producer.is_alive() and not closer.is_alive() and closed.is_set()
    assert ingress.take() is None


def test_reentrant_notify_close_is_explicitly_closed_without_self_deadlock():
    ident = identity()
    provider = Provider(ident)
    errors = []
    def notify():
        with pytest.raises(RuntimeError, match='native report close during notification'):
            ingress.close()
        errors.append(True)
    ingress = NativeCompleteIngress(provider, notify)
    assert not ingress.offer(NativeCompleteReport(ident, retained()))
    assert errors == [True] and ingress.take() is None
    assert ingress.take_fault() == 'native report close during notification'


def test_metadata_strings_are_bounded_by_provider_pins_without_global_cap():
    ident = identity()
    provider = Provider(ident)
    provider.frame_ids = ('c'*300, 'd'*300)
    ident = replace(ident, color_frame_id=provider.frame_ids[0], depth_frame_id=provider.frame_ids[1])
    provider.originals = [ident]
    provider.backend_status['device'] = {'name': 'synthetic fixture '*100, 'api': 'fixture'}
    provider.backend_status['fallback_reason'] = 'fixture reason '*200
    report = retained()
    report['color_mask_backend'] = copy.deepcopy(provider.backend_status)
    ingress = NativeCompleteIngress(provider, lambda: None)
    assert ingress.offer(NativeCompleteReport(ident, report))
    assert ingress.take() is not None


@pytest.mark.parametrize('field', ['position', 'closing_axis', 'orientation_matrix'])
def test_producer_mutation_at_array_copy_is_validated_on_owned_storage(monkeypatch, field):
    from so101_station_objects import native_complete_report as module
    ident, _, ingress = setup()
    report = retained()
    obj = report['objects'][0]
    target = obj['grasps'][0][field] if field == 'closing_axis' else obj[field]
    original_array, original_copyto = np.array, np.copyto
    def mutate(value):
        if value is target:
            target.flat[0] = np.nan if field == 'position' else 2.
    def array(value, *args, **kwargs):
        mutate(value)
        return original_array(value, *args, **kwargs)
    def copyto(destination, value, *args, **kwargs):
        mutate(value)
        return original_copyto(destination, value, *args, **kwargs)
    monkeypatch.setattr(module.np, 'array', array)
    monkeypatch.setattr(module.np, 'copyto', copyto)
    assert not ingress.offer(NativeCompleteReport(ident, report))
    assert ingress.take() is None


def test_concurrent_shape_change_cannot_allocate_dense_owned_storage(monkeypatch):
    from so101_station_objects import native_complete_report as module
    ident, _, ingress = setup()
    report = retained()
    target = report['objects'][0]['position'].copy()
    report['objects'][0]['position'] = target
    original_array, original_copyto = np.array, np.copyto
    allocations = []
    def expand(value):
        if value is target:
            target.resize((100_000,), refcheck=False)
    def array(value, *args, **kwargs):
        expand(value)
        owned = original_array(value, *args, **kwargs)
        allocations.append(owned.size)
        return owned
    def copyto(destination, value, *args, **kwargs):
        expand(value)
        allocations.append(destination.size)
        return original_copyto(destination, value, *args, **kwargs)
    monkeypatch.setattr(module.np, 'array', array)
    monkeypatch.setattr(module.np, 'copyto', copyto)
    assert not ingress.offer(NativeCompleteReport(ident, report))
    assert max(allocations) <= 9


@pytest.mark.parametrize('field', ['objects', 'grasps'])
def test_live_list_growth_at_first_vector_copy_rejects_whole_report(monkeypatch, field):
    from so101_station_objects import native_complete_report as module
    ident, _, ingress = setup()
    report = retained()
    sequence = report['objects'] if field == 'objects' else report['objects'][0]['grasps']
    target = sequence[0]['position']
    extra = copy.deepcopy(sequence[0])
    original_array, original_copyto = np.array, np.copyto
    mutated = []
    def grow(value):
        if value is target and not mutated:
            sequence.append(extra)
            mutated.append(True)
    def array(value, *args, **kwargs):
        grow(value)
        return original_array(value, *args, **kwargs)
    def copyto(destination, value, *args, **kwargs):
        grow(value)
        return original_copyto(destination, value, *args, **kwargs)
    monkeypatch.setattr(module.np, 'array', array)
    monkeypatch.setattr(module.np, 'copyto', copyto)
    assert not ingress.offer(NativeCompleteReport(ident, report))
    assert mutated and ingress.take() is None


@pytest.mark.parametrize('minimum,maximum', [(3, 3), (0, 5)])
def test_list_snapshot_traversal_is_bounded_during_growth(minimum, maximum):
    from so101_station_objects import native_complete_report as module
    sequence = [0]*maximum
    visited = []
    def item(value):
        visited.append(value)
        if len(visited) == 1:
            sequence.extend([0]*10000)
        return value
    with pytest.raises(ValueError):
        module._list(sequence, maximum, item, minimum)
    assert len(visited) <= maximum


def test_dictionary_growth_before_key_validation_has_bounded_traversal(monkeypatch):
    from so101_station_objects import native_complete_report as module
    import builtins
    source = {'allowed': 1}
    visited = []
    mutated = []
    def length(value):
        count = builtins.len(value)
        if value is source and not mutated:
            mutated.append(True)
            source.update({f'extra{index}': 0 for index in range(10000)})
        return count
    def exact_type(value):
        if builtins.type(value) is str:
            visited.append(value)
        return builtins.type(value)
    monkeypatch.setattr(module, 'len', length, raising=False)
    monkeypatch.setattr(module, 'type', exact_type, raising=False)
    with pytest.raises(ValueError):
        module._record(source, {'allowed': module._integer})
    assert len(visited) <= 2


@pytest.mark.parametrize('value', [2**64, 1 << 1_000_000], ids=['uint64_overflow', 'million_bit'])
def test_native_report_counters_fit_uint64(value):
    ident, _, ingress = setup()
    report = retained()
    report['validation_diagnostics']['color_parent_components'] = value
    assert not ingress.offer(NativeCompleteReport(ident, report))
    assert ingress.take() is None


def test_float32_scalar_is_normalized_for_real_status_json():
    ident, _, ingress = setup()
    report = retained()
    report['support_plane']['d'] = np.float32(0)
    assert ingress.offer(NativeCompleteReport(ident, report))
    admitted = ingress.take()
    assert type(admitted.report['support_plane']['d']) is float
    json.dumps(admitted.report['support_plane'], default=lambda value: value.tolist()
               if type(value) is np.ndarray else (_ for _ in ()).throw(TypeError()))


@pytest.mark.parametrize('field', ['metric_vector', 'rejected_samples'])
def test_nested_live_list_growth_rejects_before_unbounded_traversal(monkeypatch, field):
    from so101_station_objects import native_complete_report as module
    ident, _, ingress = setup()
    report = retained(4 if field == 'rejected_samples' else 0)
    if field == 'metric_vector':
        sequence = report['objects'][0]['depth_validation']['metrics']['centroid_camera_m']
        trigger = sequence[0]
        extra = 0.
    else:
        sequence = report['validation_diagnostics']['rejected_samples']
        sequence[:] = [copy.deepcopy(sequence[0]) for _ in range(5)]
        trigger = sequence[0]['metrics']['centroid_camera_m'][0]
        extra = copy.deepcopy(sequence[0])
    original = module._number
    visited, mutated = [], []
    def number(value):
        visited.append(value)
        if value is trigger and not mutated:
            sequence.extend([extra]*10000)
            mutated.append(True)
        return original(value)
    monkeypatch.setattr(module, '_number', number)
    assert not ingress.offer(NativeCompleteReport(ident, report))
    assert mutated and ingress.take() is None
    assert len(visited) < 100


def test_dictionary_replacement_during_value_validation_is_rejected():
    from so101_station_objects import native_complete_report as module
    value = object()
    report = {'allowed': value}
    def replace_value(item):
        assert item is value
        report['allowed'] = object()
        return item
    with pytest.raises(ValueError):
        module._record(report, {'allowed': replace_value})
