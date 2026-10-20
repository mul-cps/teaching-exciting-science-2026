"""Small, position-only tag alignment primitives; never modify control calibration.

Printed code IDs are not instance identities. Association requires an explicit
image seed and remains lost after any failed match until an operator resets it.
Transforms use column vectors: camera_point = camera_from_link @ link_point.
"""
from copy import deepcopy
import math
import numbers
import re

import numpy as np


def _vector(value, length, name):
    array = np.asarray(value, dtype=float)
    if array.shape != (length,) or not np.isfinite(array).all():
        raise ValueError(f'{name} must contain {length} finite numbers')
    return array


def _positive(value, name):
    if isinstance(value, bool) or not isinstance(value, numbers.Real):
        raise ValueError(f'{name} must be a positive finite number')
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f'{name} must be a positive finite number')
    return value


def _transform(value, name):
    matrix = np.asarray(value, dtype=float)
    if (matrix.shape != (4, 4) or not np.isfinite(matrix).all()
            or not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-8, rtol=0)):
        raise ValueError(f'{name} must be a finite rigid 4x4 transform')
    rotation = matrix[:3, :3]
    if (not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6, rtol=0)
            or not math.isclose(float(np.linalg.det(rotation)), 1., abs_tol=1e-6)):
        raise ValueError(f'{name} must contain a proper rotation')
    return matrix


def validate_bindings(config):
    """Validate and return a detached, JSON-compatible binding configuration.

    Mount provenance is retained, but no mount is promoted to qualified here.
    A seed radius is explicit; temporal tracking subsequently uses 60 pixels.
    """
    if not isinstance(config, dict) or config.get('schema') != 'so101-tag-bindings-v1':
        raise ValueError('schema must be so101-tag-bindings-v1')
    result = deepcopy(config)
    if not isinstance(result.get('model_sha256'), str) or not re.fullmatch(r'[0-9a-fA-F]{64}', result['model_sha256']):
        raise ValueError('model_sha256 must be a SHA256 hex digest')
    if not isinstance(result.get('camera_serial'), str) or not result['camera_serial'].strip():
        raise ValueError('camera_serial must be a nonempty string')
    for key in ('image_width', 'image_height'):
        if type(result.get(key)) is not int or result[key] <= 0:
            raise ValueError(f'{key} must be a positive integer')
    if 'camera_from_base_reference' in result:
        result['camera_from_base_reference'] = _transform(result['camera_from_base_reference'], 'camera_from_base_reference').tolist()
    bindings = result.get('bindings')
    if not isinstance(bindings, list) or not bindings:
        raise ValueError('bindings must be a nonempty list')
    names = set()
    for binding in bindings:
        if not isinstance(binding, dict):
            raise ValueError('each binding must be an object')
        name = binding.get('instance_name')
        if not isinstance(name, str) or not name.strip() or name in names:
            raise ValueError('instance_name must be nonempty and unique')
        names.add(name)
        if type(binding.get('code_id')) is not int or binding['code_id'] < 0:
            raise ValueError('code_id must be a nonnegative integer')
        binding['size_m'] = _positive(binding.get('size_m'), 'size_m')
        binding['seed_radius_px'] = _positive(binding.get('seed_radius_px'), 'seed_radius_px')
        seed = _vector(binding.get('seed_px'), 2, 'seed_px')
        if not (0 <= seed[0] < result['image_width'] and 0 <= seed[1] < result['image_height']):
            raise ValueError('seed_px must lie inside the configured image')
        binding['seed_px'] = seed.tolist()
        if not isinstance(binding.get('link'), str) or not binding['link'].strip():
            raise ValueError('link must be a nonempty string')
        binding['link_point_m'] = _vector(binding.get('link_point_m'), 3, 'link_point_m').tolist()
        if type(binding.get('moving')) is not bool:
            raise ValueError('moving must be an explicit boolean')
        if binding.get('qualified', False) is not False:
            raise ValueError('live tag bindings must remain qualified=false')
        binding['qualified'] = False
    return result


class SeededTagAssociator:
    """Seeded local tracking of repeated-code instances, with latched loss."""

    def __init__(self, config):
        self.config = validate_bindings(config)
        self.reset()

    def reset(self):
        """Explicitly allow acquisition around the original configured seeds."""
        self._centers = {}
        self._last_valid_sizes = {}
        self._latched = {}
        self._missed = {}
        self._last_seen_stamp = {}
        self._last_stamp = None

    def _result(self, binding, association, reason, observation=None):
        result = deepcopy(binding)
        if observation is not None:
            result.update(deepcopy(observation))
            # An observation cannot override binding identity or qualification.
            result.update({key: deepcopy(binding[key]) for key in (
                'instance_name', 'link', 'link_point_m', 'moving', 'qualified')})
        result.update(association=association, reason=reason)
        return result

    def update(self, observations, *, stamp):
        """Associate a single admitted exposure; reject out-of-order frames.

        Initialization requires a valid measured size. After acquisition, an
        explicitly missing size (None) can preserve pixel continuity using the
        existing identity prior, but never supplies a valid 3D measurement.
        Malformed or conflicting measured sizes are rejected, not downgraded to
        missing sizes. Pixel-only observations report depth_unavailable and
        cannot enter a fit or residual calculation.
        Acquired identities tolerate at most three missed frames within .8 s
        of their last observation, without predicting a 3D measurement. After
        that grace period, loss remains latched until explicit reset.
        """
        bindings = self.config['bindings']
        if (isinstance(stamp, bool) or not isinstance(stamp, numbers.Real)
                or not math.isfinite(float(stamp)) or stamp < 0
                or (self._last_stamp is not None and stamp <= self._last_stamp)):
            return [self._result(b, 'unavailable', 'stale_or_repeated_stamp') for b in bindings]
        self._last_stamp = float(stamp)
        if not isinstance(observations, (list, tuple)):
            observations = []
        valid = []
        for index, obs in enumerate(observations):
            if not isinstance(obs, dict) or type(obs.get('code_id')) is not int:
                continue
            try:
                center = _vector(obs.get('center_px'), 2, 'center_px')
                size = None if obs.get('size_m') is None else _positive(obs['size_m'], 'size_m')
            except (ValueError, TypeError):
                continue
            if not (0 <= center[0] < self.config['image_width'] and 0 <= center[1] < self.config['image_height']):
                continue
            observation = deepcopy(obs)
            observation['size_m'] = size
            try:
                position = _vector(obs.get('center_camera_m'), 3, 'center_camera_m')
                measurement_valid = size is not None and obs.get('measurement_valid') is True and position[2] > 0
            except (ValueError, TypeError):
                measurement_valid = False
            observation['measurement_valid'] = measurement_valid
            if not measurement_valid:
                observation.pop('center_camera_m', None)
            valid.append((index, observation, center, size))
        selected = {}
        waiting = {}
        for binding in bindings:
            name = binding['instance_name']
            if name in self._latched:
                continue
            initialized = name in self._centers
            previous = self._centers.get(name, np.asarray(binding['seed_px']))
            radius = 60. if initialized else binding['seed_radius_px']
            candidates = []
            for index, observation, center, size in valid:
                if observation['code_id'] != binding['code_id']:
                    continue
                if size is None:
                    if not initialized or name not in self._last_valid_sizes:
                        continue
                elif abs(size / binding['size_m'] - 1) > .20 + 1e-12:
                    continue
                distance = float(np.linalg.norm(center - previous))
                if distance <= radius:
                    candidates.append((distance, index, observation, center))
            candidates.sort(key=lambda item: item[0])
            if not candidates:
                if initialized:
                    self._missed[name] = self._missed.get(name, 0) + 1
                    if (self._missed[name] <= 3
                            and stamp - self._last_seen_stamp[name] <= .8):
                        waiting[name] = ('temporarily_missing', 'brief_detection_gap')
                    else:
                        self._latched[name] = ('lost', 'no_candidate_in_gate')
                else:
                    waiting[name] = ('waiting', 'awaiting_initial_seed')
            elif len(candidates) > 1 and candidates[1][0] - candidates[0][0] <= 10.:
                self._latched[name] = ('ambiguous', 'nearest_candidates_within_10px')
            else:
                selected[name] = candidates[0]
        # Global one-to-one use is checked before committing any center update.
        owners = {}
        for name, choice in selected.items():
            owners.setdefault(choice[1], []).append(name)
        for names in owners.values():
            if len(names) > 1:
                for name in names:
                    self._latched[name] = ('ambiguous', 'duplicate_observation_assignment')
        results = []
        for binding in bindings:
            name = binding['instance_name']
            if name in waiting:
                state, reason = waiting[name]
                results.append(self._result(binding, state, reason))
            elif name in self._latched:
                state, reason = self._latched[name]
                results.append(self._result(binding, state, reason))
            else:
                _, _, observation, center = selected[name]
                self._centers[name] = center.copy()
                self._missed[name] = 0
                self._last_seen_stamp[name] = float(stamp)
                if observation.get('size_m') is not None:
                    self._last_valid_sizes[name] = float(observation['size_m'])
                observation['measurement_state'] = 'available' if observation['measurement_valid'] else 'depth_unavailable'
                reason = 'pixel_and_depth' if observation['measurement_valid'] else 'pixel_only_no_valid_depth_or_size'
                results.append(self._result(binding, 'tracked', reason, observation))
        return results


def fit_tag_alignment(model_points, camera_points, *, previous_camera_from_base=None,
                      max_translation_m=.03, max_rotation_deg=5.,
                      max_residual_m=.008, min_baseline_m=.04, min_shape_ratio=.05):
    """Fit camera-from-base from corresponding positions, without promotion.

    Two independent spread directions are required; planar tags are sufficient.
    A supplied previous/reference transform bounds even the first fitted update.
    """
    result = {'status': 'unavailable', 'qualified': False}
    try:
        for name, value in (('max_translation_m', max_translation_m), ('max_rotation_deg', max_rotation_deg),
                            ('max_residual_m', max_residual_m), ('min_baseline_m', min_baseline_m),
                            ('min_shape_ratio', min_shape_ratio)):
            _positive(value, name)
        source = np.asarray(model_points, dtype=float)
        target = np.asarray(camera_points, dtype=float)
        if (source.ndim != 2 or source.shape[1:] != (3,) or source.shape != target.shape
                or len(source) < 3 or not np.isfinite(source).all() or not np.isfinite(target).all()):
            return dict(result, reason='need_at_least_three_finite_paired_points')
        previous = None if previous_camera_from_base is None else _transform(previous_camera_from_base, 'previous_camera_from_base')
        source_center = source.mean(axis=0)
        target_center = target.mean(axis=0)
        x = source - source_center
        y = target - target_center
        source_s = np.linalg.svd(x, compute_uv=False)
        target_s = np.linalg.svd(y, compute_uv=False)
        shape_ratio = min(float(source_s[1] / source_s[0]) if source_s[0] else 0.,
                          float(target_s[1] / target_s[0]) if target_s[0] else 0.)
        # N is the compact marker count, never the image/cloud point count.
        baseline = min(float(np.linalg.norm(source[:, None] - source[None, :], axis=2).max()),
                       float(np.linalg.norm(target[:, None] - target[None, :], axis=2).max()))
        result.update(shape_ratio=shape_ratio, baseline_m=baseline, point_count=len(source))
        if baseline < min_baseline_m:
            return dict(result, reason='insufficient_baseline')
        if shape_ratio < min_shape_ratio:
            return dict(result, reason='collinear_or_poorly_observable_geometry')
        u, _, vt = np.linalg.svd(x.T @ y)
        correction = np.eye(3)
        correction[2, 2] = np.linalg.det(vt.T @ u.T)
        rotation = vt.T @ correction @ u.T
        translation = target_center - rotation @ source_center
        matrix = np.eye(4)
        matrix[:3, :3] = rotation
        matrix[:3, 3] = translation
        residual = np.linalg.norm(source @ rotation.T + translation - target, axis=1)
        result.update(camera_from_base=matrix.tolist(), residual_m=residual.tolist(),
                      max_residual_m=float(residual.max()), rms_residual_m=float(np.sqrt(np.mean(residual ** 2))))
        if residual.max() > max_residual_m:
            return dict(result, reason='excessive_point_residual')
        if previous is not None:
            delta_translation = float(np.linalg.norm(translation - previous[:3, 3]))
            delta_rotation = math.degrees(math.acos(float(np.clip((np.trace(rotation @ previous[:3, :3].T) - 1) / 2, -1, 1))))
            result.update(delta_translation_m=delta_translation, delta_rotation_deg=delta_rotation)
            if delta_translation > max_translation_m or delta_rotation > max_rotation_deg:
                return dict(result, reason='excessive_transform_jump')
        result.update(status='usable_candidate', reason='position_fit_only_unqualified', reference_bounded=previous is not None)
        return result
    except (TypeError, ValueError, np.linalg.LinAlgError) as error:
        return dict(result, reason='invalid_input', detail=str(error))


def point_residual(link_point_m, camera_from_link, center_camera_m):
    """Report a position residual; a single marker point gives no orientation."""
    try:
        point = _vector(link_point_m, 3, 'link_point_m')
        transform = _transform(camera_from_link, 'camera_from_link')
        measured = _vector(center_camera_m, 3, 'center_camera_m')
        predicted = transform[:3, :3] @ point + transform[:3, 3]
        residual = measured - predicted
        return {'status': 'available', 'predicted_camera_m': predicted.tolist(),
                'measured_camera_m': measured.tolist(), 'residual_vector_m': residual.tolist(),
                'residual_m': float(np.linalg.norm(residual)), 'position_only': True, 'qualified': False}
    except (TypeError, ValueError) as error:
        return {'status': 'unavailable', 'reason': 'invalid_input', 'detail': str(error), 'qualified': False}
