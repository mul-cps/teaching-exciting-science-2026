"""Closed compact numerical reports and a provider-fenced single pending slot.

This module grants no source, clock, geometry or production resource capability.
Those are the trusted provider's responsibility. Packets carry data only.
"""
from dataclasses import dataclass
import math
from threading import Lock, RLock, get_ident

import numpy as np


@dataclass(frozen=True, slots=True)
class NativeIdentity:
    producer_epoch: object
    session_token: int
    color_frame_number: int
    depth_frame_number: int
    color_hardware_time_ms: float
    depth_hardware_time_ms: float
    color_clock_domain: int
    depth_clock_domain: int
    color_frame_id: str
    depth_frame_id: str
    calibration_epoch: object
    configuration_epoch: object
    transform_epoch: object
    observation_ros_ns: int
    clock_mapping_epoch: object


@dataclass(frozen=True, slots=True)
class ReportLimits:
    max_objects: int
    max_tracks: int

    def __post_init__(self):
        _integer(self.max_objects)
        _integer(self.max_tracks)


@dataclass(frozen=True, slots=True)
class NativeCompleteReport:
    identity: NativeIdentity
    report: dict


@dataclass(frozen=True, slots=True)
class AdmittedNativeReport:
    identity: NativeIdentity
    report: dict
    admission: object


def _require(condition):
    if not condition:
        raise ValueError('invalid compact report')


def _integer(value, maximum=2**64-1, minimum=0):
    _require(type(value) is int and value >= minimum and
             (maximum is None or value <= maximum))
    return value


def _number(value):
    _require(type(value) in (int, float, np.float64, np.float32) and math.isfinite(value))
    return float(value) if type(value) is np.float32 else value


def _text(value, maximum=None):
    _require(type(value) is str and (maximum is None or len(value) <= maximum))
    return value


def _enum(value, choices):
    _text(value, max(map(len, choices)))
    _require(value in choices)
    return value


def _bool(value):
    _require(type(value) is bool)
    return value


def _false(value):
    _require(value is False)
    return value


def _fixed(expected):
    def check(value):
        _require(type(value) is type(expected))
        if type(expected) is str:
            _require(len(value) == len(expected))
        _require(value == expected)
        return value
    return check


def _dict_snapshot(value, maximum, minimum=0):
    """Take at most the declared number of shallow entries, never a live scan."""
    _require(type(value) is dict)
    count = len(value)
    _require(minimum <= count <= maximum)
    entries = []
    iterator = iter(value.items())
    for _ in range(count):
        key, item = next(iterator)
        _require(type(key) is str)
        entries.append((key, item))
    sentinel = object()
    _require(next(iterator, sentinel) is sentinel and len(value) == count)
    return entries


def _dict_coherent(value, entries, maximum):
    current = _dict_snapshot(value, maximum, len(entries))
    _require(len(current) == len(entries) and all(
        key is old_key and item is old_item
        for (key, item), (old_key, old_item) in zip(current, entries)))


def _record(value, required, optional=None):
    """Snapshot bounded references, validate owned values, then check coherence."""
    optional = {} if optional is None else optional
    maximum = len(required)+len(optional)
    entries = _dict_snapshot(value, maximum, len(required))
    keys = {key for key, _ in entries}
    _require(required.keys() <= keys <= required.keys() | optional.keys())
    result = {key: (required[key] if key in required else optional[key])(item)
              for key, item in entries}
    _dict_coherent(value, entries, maximum)
    return result


def _list(value, maximum, item, minimum=0):
    _require(type(value) is list)
    count = len(value)
    _require(minimum <= count <= maximum)
    # Index only the initially admitted count. Never traverse a growing iterator.
    snapshot = [value[index] for index in range(count)]
    _require(len(value) == count)
    result = []
    for index, entry in enumerate(snapshot):
        result.append(item(entry))
        _require(len(value) == count and value[index] is entry)
    _require(len(result) == count and minimum <= len(result) <= maximum)
    _require(len(value) == count and all(value[index] is entry
                                       for index, entry in enumerate(snapshot)))
    return result


def _vector(value, shape=(3,), unit=False, rotation=False):
    # Destination allocation/copy size derives only from this fixed schema,
    # including if a producer changes the source shape after the first check.
    _require(type(value) is np.ndarray and value.shape == shape and
             value.dtype in (np.dtype('float64'), np.dtype('float32')))
    dtype = value.dtype
    owned = np.empty(shape, dtype=np.float64)
    np.copyto(owned, value, casting='safe')
    _require(value.shape == shape and value.dtype == dtype)
    _require(bool(np.isfinite(owned).all()))
    if unit:
        _require(abs(float(np.linalg.norm(owned))-1.) <= 1e-9)
    if rotation:
        _require(bool(np.all(np.abs(owned.T @ owned-np.eye(3)) <= 1e-9)) and
                 abs(float(np.linalg.det(owned))-1.) <= 1e-9)
    owned.flags.writeable = False
    return owned


def _metric_vector(value):
    return _list(value, 3, _number, minimum=3)


_DEPTH_INITIAL = dict(color_pixels=_integer, local_support_pixels=_integer,
                      semantic_identity=_fixed('unqualified_colored_surface'),
                      workspace_gate_applied=_false, support_plane_gate_applied=_false)
_DEPTH_FRACTIONS = dict(supported_fraction=_number, invalid_local_depth_fraction=_number,
                        core_supported_fraction=lambda v: None if v is None else _number(v))
_DEPTH_GEOMETRY = dict(centroid_camera_m=_metric_vector, visible_extents_m=_metric_vector,
                       **{key: _number for key in ('plane_residual_p95_m', 'largest_depth_gap_m',
                          'split_minority_fraction', 'depth_neighbor_jump_fraction',
                          'saturation_p10', 'silhouette_rectangularity')})
_SIL_INITIAL = dict(min_rectangle_fill_required=_number,
                    semantic_identity=_fixed('unqualified_convex_brick_shape_prior'),
                    applied_to=_fixed('final_child_contour'), workspace_gate_applied=_false)
_SIL_GEOMETRY = dict(min_rectangle_fill=_number, contour_area_px2=_number,
                     convex_hull_fill=_number, shape_evidence=_fixed('contour_geometry_only'))
_SIL_OBSERVED = dict(reconstructed_min_rectangle_fill=_number, observed_enclosing_pixel_area=_number,
                     observed_pixel_fill=_number, observed_boundary_fill=_number,
                     observed_pixels=_integer, observed_components=_integer)
_DEPTH_STAGES = (_DEPTH_INITIAL, _DEPTH_INITIAL | _DEPTH_FRACTIONS,
                 _DEPTH_INITIAL | _DEPTH_FRACTIONS | _DEPTH_GEOMETRY)
_SIL_STAGES = (_SIL_INITIAL, _SIL_INITIAL | _SIL_GEOMETRY,
               _SIL_INITIAL | _SIL_GEOMETRY | _SIL_OBSERVED |
               {'shape_evidence': _fixed('original_color_depth_pixels')})
_REASONS = ('outside_observation_depth_range', 'no_depth_coherent_component',
            'no_observation_contour', 'outside_observation_roi', 'insufficient_roi_depth_pixels',
            'visible_geometry_out_of_range', 'no_color_support', 'insufficient_depth_support',
            'locally_unsupported_color_depth', 'multiple_disconnected_depth_surfaces',
            'incoherent_neighbor_depth', 'insufficient_silhouette_geometry',
            'insufficient_observed_silhouette', 'silhouette_inconsistent_with_convex_brick')


def _metrics(value, stages):
    _require(type(value) is dict)
    for fields in stages:
        if len(value) == len(fields):
            return _record(value, fields)
    raise ValueError('invalid compact metrics')


def _depth_validation(value):
    return _record(value, dict(accepted=_fixed(True), reason=_fixed('depth_supported_colored_surface'),
                               metrics=lambda v: _record(v, _DEPTH_STAGES[-1]), depth_cut_pixels=_integer))


def _sil_validation(value):
    return _record(value, dict(accepted=_fixed(True),
                               reason=_fixed('silhouette_consistent_with_convex_brick_prior'),
                               metrics=lambda v: _record(v, _SIL_STAGES[-1])))


def _grasp(value):
    return _record(value, dict(position=_vector, closing_axis=lambda v: _vector(v, unit=True),
                               approach_direction=lambda v: _vector(v, unit=True), opening_m=_number),
                   dict(contacts_camera=lambda v: _vector(v, (2, 3)), axis_rank=_number))


def _boundary(value):
    sides = _list(value, 4, lambda v: _enum(v, ('left', 'right', 'top', 'bottom')))
    _require(sides == [side for side in ('left', 'right', 'top', 'bottom') if side in sides])
    return sides


def _object(value):
    result = _record(value, dict(
        color=lambda v: _enum(v, ('red', 'green', 'blue', 'yellow')),
        **{key: _vector for key in ('position', 'surface_centroid', 'surface_centroid_spread_m',
                                    'dimensions_m', 'dimension_uncertainty_m')},
        orientation_matrix=lambda v: _vector(v, (3, 3), rotation=True),
        height_m=lambda v: None if v is None else _number(v), depth_points=_integer, pixel_area=_integer,
        grasps=lambda v: _list(v, 1, _grasp), geometric_only=_fixed(True),
        pickup_eligible=_bool, acquisition_eligible=_bool,
        **{key: _false for key in ('execution_eligible', 'object_center_qualified', 'ik_validated', 'collision_checked')},
        eligibility_reason=lambda v: _enum(v, ('inside_pickup_roi', 'outside_pickup_roi', 'not_support_plane_qualified')),
        execution_eligibility_reason=_fixed('not_ik_or_collision_checked'),
        observation_geometry=lambda v: _enum(v, ('support_plane_box_estimate', 'visible_surface_centroid')),
        dimensions_semantics=lambda v: _enum(v, ('support_plane_box_extent_estimate', 'visible_surface_extent_only')),
        acquisition_eligibility_reason=lambda v: _enum(v, ('observed_silhouette_within_image', 'image_boundary_truncated')),
        image_boundary_sides=_boundary, depth_validation=_depth_validation, silhouette_validation=_sil_validation))
    visible = result['observation_geometry'] == 'visible_surface_centroid'
    if visible:
        _require(result['height_m'] is None and not result['grasps'] and not result['pickup_eligible'] and
                 result['eligibility_reason'] == 'not_support_plane_qualified' and
                 result['dimensions_semantics'] == 'visible_surface_extent_only')
    else:
        _require(result['height_m'] is not None and result['dimensions_semantics'] == 'support_plane_box_extent_estimate' and
                 result['eligibility_reason'] == ('inside_pickup_roi' if result['pickup_eligible'] else 'outside_pickup_roi'))
    _require(result['acquisition_eligible'] == (not result['image_boundary_sides']) and
             result['acquisition_eligibility_reason'] == ('observed_silhouette_within_image'
             if result['acquisition_eligible'] else 'image_boundary_truncated'))
    return result


def _plane(value):
    base = dict(normal=lambda v: _vector(v, unit=True), d=_number,
                plane_source=lambda v: _enum(v, ('measured_table_consistent_with_board_z0',
                    'measured_table_plane', 'cached_measured_support_plane')),
                support_points=_integer, support_inliers=_integer)
    measured = {key: _number for key in ('median_m', 'p95_m', 'inlier_p95_m', 'all_p95_m')}
    result = _record(value, base, measured | {'support_plane_age_s': _number})
    _require(result['normal'][2] >= .85 and result['support_inliers'] <= result['support_points'])
    if result['plane_source'] == 'cached_measured_support_plane':
        _require('support_plane_age_s' in result and 0 <= result['support_plane_age_s'] <= 2 and
                 not any(key in result for key in measured))
    else:
        _require(all(key in result for key in measured) and 'support_plane_age_s' not in result)
    return result


def _rejected(value):
    def metrics(v):
        entries = _dict_snapshot(v, max(len(_DEPTH_STAGES[-1]), len(_SIL_STAGES[-1])), 1)
        stages = _DEPTH_STAGES if any(key == 'color_pixels' for key, _ in entries) else _SIL_STAGES
        result = _metrics(v, stages)
        _dict_coherent(v, entries, len(stages[-1]))
        return result
    return _record(value, dict(color=lambda v: _enum(v, ('red', 'green', 'blue', 'yellow')),
                               reason=lambda v: _enum(v, _REASONS), parent_pixels=_integer,
                               component_pixels=_integer), {'metrics': metrics})


def _diagnostics(value):
    def reasons(v):
        return _record(v, {}, {key: _integer for key in _REASONS})
    return _record(value, dict(
        **{key: _integer for key in ('color_parent_components', 'depth_components',
            'depth_accepted_components', 'rejected_components', 'silhouette_checks', 'silhouette_accepted')},
        rejection_reasons=reasons, rejected_samples=lambda v: _list(v, 5, _rejected)))


def _backend(value, pinned=None):
    if pinned is not None:
        device_pin = pinned['device']
        device_check = (_fixed(None) if device_pin is None else
                        lambda v: _record(v, {key: _fixed(item) for key, item in device_pin.items()}))
        return _record(value, {key: device_check if key == 'device' else _fixed(item)
                               for key, item in pinned.items()})
    def device(v):
        if v is None:
            return None
        _require(type(v) is dict and len(v) <= 3)
        allowed = {'name', 'vendor', 'version', 'api', 'slangpy'}
        return _record(v, {}, {key: _text for key in allowed})
    return _record(value, dict(requested=_text, selected=_text,
        gpu_available=lambda v: None if v is None else _bool(v), device=device,
        fallback_reason=lambda v: None if v is None else _text(v)))


def _identity(value, frame_ids):
    _require(type(value) is NativeIdentity)
    _integer(value.session_token, 2**64-1, 1)
    for counter in (value.color_frame_number, value.depth_frame_number):
        _integer(counter, 2**64-1)
    for domain in (value.color_clock_domain, value.depth_clock_domain):
        _integer(domain, 2**32-1)
    _integer(value.observation_ros_ns, 2**63-1, 1)
    for timestamp in (value.color_hardware_time_ms, value.depth_hardware_time_ms):
        _number(timestamp)
        _require(timestamp >= 0)
    for actual, pinned in zip((value.color_frame_id, value.depth_frame_id), frame_ids):
        _text(actual, len(pinned))
        _require(actual == pinned)


class NativeCompleteIngress:
    """Validate/copy outside the slot lock; all lifecycle fences are generation based.

    ``check`` must raise on invalidation and return exactly None on success.
    The executor must use the provider's ``publish_if_current`` for final commit.
    A wake callback may run after invalidation, but can never revive its data.
    """
    def __init__(self, provider, notify_ready):
        self._provider = provider
        self._notify = notify_ready
        self._lock = Lock()
        self._delivery_lock = RLock()
        self._notify_owner = None
        self._generation = 0
        self._closed = False
        self._pending = None
        self._fault = None
        self.replaced = 0
        self.rejected = 0
        self.notification_failures = 0
        self._available = False
        try:
            _require(type(provider.limits) is ReportLimits)
            self.limits = provider.limits
            _integer(self.limits.max_objects)
            _integer(self.limits.max_tracks)
            _require(type(provider.frame_ids) is tuple and len(provider.frame_ids) == 2)
            self.frame_ids = tuple(_text(v) for v in provider.frame_ids)
            self.backend_status = _backend(provider.backend_status)
            _require(all(callable(getattr(provider, method, None)) for method in
                         ('admit', 'check', 'publish_if_current')) and callable(notify_ready))
            self._available = True
        except Exception:
            self._fault = 'native report provider unavailable'

    def _reject(self, generation):
        with self._lock:
            self.rejected += 1
            if generation == self._generation:
                self._generation += 1
                self._pending = None
        return False

    def offer(self, packet):
        with self._lock:
            generation = self._generation
            if self._closed or not self._available:
                self.rejected += 1
                return False
        try:
            _require(type(packet) is NativeCompleteReport)
            _identity(packet.identity, self.frame_ids)
            owned = _record(packet.report, dict(
                support_plane=_plane, objects=lambda v: _list(v, self.limits.max_objects, _object),
                qualified=_false, frame_id=_fixed('charuco_board'),
                observation_depth_range_m=lambda v: _list(v, 2, _number, minimum=2),
                validation_diagnostics=_diagnostics,
                color_mask_backend=lambda v: _backend(v, self.backend_status)))
            _require(owned['observation_depth_range_m'][0] == .1 and
                     owned['observation_depth_range_m'][1] > .1)
            _require(owned['color_mask_backend'] == self.backend_status)
            admission = self._provider.admit(packet.identity, owned)
            _require(admission is not None and type(admission) is not bool)
            _require(self._provider.check(admission) is None)
            admitted = AdmittedNativeReport(packet.identity, owned, admission)
        except Exception:
            return self._reject(generation)
        # The trusted callback must be a bounded, nonblocking signal (e.g.
        # GuardCondition.trigger). It must not await executor consumption.
        # This separate gate prevents transfer before notification succeeds;
        # lifecycle invalidation still only needs the short pending lock.
        with self._delivery_lock:
            with self._lock:
                if self._closed or self._generation != generation:
                    self.rejected += 1
                    return False
                if self._pending is not None:
                    self.replaced += 1
                self._pending = admitted
            self._notify_owner = get_ident()
            try:
                self._notify()
            except Exception:
                with self._lock:
                    self.notification_failures += 1
                    if self._generation == generation:
                        self._generation += 1
                        self._pending = None
                        self._fault = 'native report notification failed'
                return False
            finally:
                self._notify_owner = None
            with self._lock:
                return not self._closed and self._generation == generation

    def take(self):
        # Another executor thread waits for the bounded signal to return. A
        # synchronous callback cannot consume its own provisional delivery.
        with self._delivery_lock:
            if self._notify_owner == get_ident():
                return None
            with self._lock:
                generation, pending = self._generation, self._pending
                if self._closed or pending is None:
                    return None
            try:
                _require(self._provider.check(pending.admission) is None)
            except Exception:
                self._reject(generation)
                return None
            with self._lock:
                if self._closed or generation != self._generation or self._pending is not pending:
                    return None
                self._pending = None
                return pending

    def take_fault(self):
        with self._lock:
            fault, self._fault = self._fault, None
            return fault

    def invalidate(self):
        with self._lock:
            self._generation += 1
            self._pending = None

    def close(self):
        # Fence immediately, then join notifications before guard destruction.
        with self._lock:
            self._closed = True
            self._generation += 1
            self._pending = None
            if self._notify_owner == get_ident():
                self._fault = 'native report close during notification'
                raise RuntimeError('native report close during notification')
        with self._delivery_lock:
            pass
