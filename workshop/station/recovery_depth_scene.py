"""Conservative depth free-space certificates for a calibrated URDF robot.

This is an offline collision oracle, not a live camera/owner adapter. A qualified
caller must supply aligned metric depth, calibration, exact paired encoders and
registration uncertainty. Invalid, occluded and out-of-view space stays unknown.
Returned clearance is the checked lower bound, never an estimated nearest gap.
"""
from __future__ import annotations

from collections import OrderedDict
import math
import time
from types import MappingProxyType

import numpy as np

from .recovery_collision_model import URDFCollisionModel, validate_transform, voxel_keys


_CORNERS = np.array([[x, y, z] for x in (-.5, .5) for y in (-.5, .5) for z in (-.5, .5)])


def _mul(a, b):
    values = np.stack((a[0]*b[0], a[0]*b[1], a[1]*b[0], a[1]*b[1]))
    return values.min(axis=0), values.max(axis=0)


def _scale(a, value):
    return (a[0]*value, a[1]*value) if value >= 0 else (a[1]*value, a[0]*value)


def _add(*intervals):
    return sum(value[0] for value in intervals), sum(value[1] for value in intervals)


def _square(interval):
    low, high = interval
    return np.where((low <= 0) & (high >= 0), 0, np.minimum(low*low, high*high)), np.maximum(low*low, high*high)


class DepthRecoveryScene:
    """Immutable acquisition metadata plus full-volume camera proofs.

    ``support_planes`` entries have ``name``, unit ``normal`` and ``offset_m``
    defining free half-space n·x+d >= 0 in base_frame, plus optional explicit
    ``stationary_contact_links``. Such contacts are allowed only when the link
    transform is unchanged from the acquisition pose. They never exempt unknown
    camera space. No link, adjacent pair or table contact is guessed.
    """
    def __setattr__(self, name, value):
        if getattr(self, '_sealed', False) and not name.startswith('_') and name != 'last_rejection':
            raise AttributeError('camera recovery acquisition is immutable')
        object.__setattr__(self, name, value)

    @property
    def last_rejection(self):
        return self._last_rejection

    @last_rejection.setter
    def last_rejection(self, value):
        self._last_rejection = value

    def __init__(self, *, depth_m, intrinsics, distortion, base_from_camera,
                 base_frame, camera_frame, camera_serial, model_sha256,
                 calibration_sha256, registration_qualified, stamp_monotonic,
                 arm_raw, arm_stamp_monotonic, model, clearance_m=.010,
                 voxel_size_m=.005, depth_uncertainty_m=.005,
                 geometry_uncertainty_m=.002, distortion_model='plumb_bob',
                 support_planes=(), max_check_time_s=.45, min_depth_m=.05, max_depth_m=2.0):
        if not isinstance(model, URDFCollisionModel):
            raise ValueError('effective calibrated URDF collision model required')
        if (registration_qualified is not True or base_frame != model.base_frame
                or not isinstance(camera_frame, str) or not camera_frame.strip()
                or not isinstance(camera_serial, str) or not camera_serial.strip()
                or model_sha256 != model.model_sha256 or calibration_sha256 != model.calibration_sha256):
            raise ValueError('identity-matched qualified camera, frame, model and calibration required')
        values = (stamp_monotonic, arm_stamp_monotonic, clearance_m, voxel_size_m,
                  depth_uncertainty_m, geometry_uncertainty_m, max_check_time_s, min_depth_m, max_depth_m)
        if (any(type(value) not in (int, float) or not math.isfinite(value) for value in values)
                or stamp_monotonic < 0 or arm_stamp_monotonic < 0
                or abs(stamp_monotonic-arm_stamp_monotonic) > .100
                or clearance_m < .010 or not 0 < voxel_size_m <= .005
                or depth_uncertainty_m <= 0 or geometry_uncertainty_m <= 0
                or not 0 < max_check_time_s <= .45 or not 0 < min_depth_m < max_depth_m):
            raise ValueError('finite synchronized acquisition, clearance and bounded uncertainties required')
        if any(not 0 <= time.monotonic()-stamp <= .5 for stamp in (stamp_monotonic, arm_stamp_monotonic)):
            raise ValueError('fresh acquisition timestamps from the local monotonic clock required')
        if not isinstance(intrinsics, dict) or set(intrinsics) != {'fx', 'fy', 'ppx', 'ppy'}:
            raise ValueError('aligned optical pinhole intrinsics required')
        if any(type(value) not in (int, float) or not math.isfinite(value) for value in intrinsics.values()) or min(intrinsics['fx'], intrinsics['fy']) <= 0:
            raise ValueError('finite positive focal lengths required')
        depth = np.asarray(depth_m)
        if depth.ndim != 2 or not np.issubdtype(depth.dtype, np.floating) or not depth.size or depth.size > 4_194_304:
            raise ValueError('bounded aligned metric floating depth image required')
        distortion = np.asarray(distortion, dtype=float)
        if distortion_model != 'plumb_bob' or distortion.shape not in {(4,), (5,)} or not np.isfinite(distortion).all():
            raise ValueError('explicit finite Brown-Conrady plumb_bob 4/5 coefficient distortion required')
        model.joint_positions(arm_raw)
        self.model = model
        self.model_sha256 = model_sha256
        self.calibration_sha256 = calibration_sha256
        self.registration_qualified = True
        self.camera_serial = camera_serial
        self.camera_frame = camera_frame
        self.base_frame = base_frame
        self.stamp_monotonic = float(stamp_monotonic)
        self.arm_stamp_monotonic = float(arm_stamp_monotonic)
        self.arm_raw = MappingProxyType(dict(arm_raw))
        self.clearance_m = float(clearance_m)
        self.voxel_size_m = float(voxel_size_m)
        self.depth_uncertainty_m = float(depth_uncertainty_m)
        self.geometry_uncertainty_m = float(geometry_uncertainty_m)
        self.max_check_time_s = float(max_check_time_s)
        self.intrinsics = MappingProxyType(dict(intrinsics))
        self.distortion = np.pad(distortion, (0, 5-len(distortion)))
        self.distortion.setflags(write=False)
        self.base_from_camera = validate_transform(base_from_camera)
        self.base_from_camera.setflags(write=False)
        self.camera_from_base = np.linalg.inv(self.base_from_camera)
        self.camera_from_base.setflags(write=False)
        valid = np.isfinite(depth) & (depth >= min_depth_m) & (depth <= max_depth_m)
        # Invalid samples are negative infinity in range-min tables; a single
        # invalid pixel anywhere in a projected footprint prevents certification.
        self.depth_m = np.where(valid, depth, -np.inf).astype(np.float64)
        self.depth_m.setflags(write=False)
        self._depth_min_tables = {(0, 0): self.depth_m}
        self._depth_table_bytes = self.depth_m.nbytes
        self._proofs = OrderedDict()
        self._check_elapsed_s = 0.
        self.last_rejection = None
        self._current_transforms = model.forward_kinematics(dict(self.arm_raw))
        current, interior = model.voxel_volume(dict(self.arm_raw), voxel_size_m=self.voxel_size_m,
            padding_m=0., interior_margin_m=self.geometry_uncertainty_m,
            deadline_monotonic=min(self.stamp_monotonic+.5, self.arm_stamp_monotonic+.5,
                                   time.monotonic()+self.max_check_time_s))
        self._current_interior = np.unique(np.concatenate([voxel_keys(indices) for indices in interior.values()]))
        self._current_interior.setflags(write=False)
        self.support_planes = []
        plane_names = set()
        for plane in support_planes:
            if not isinstance(plane, dict) or set(plane)-{'name', 'normal', 'offset_m', 'stationary_contact_links'}:
                raise ValueError('explicit named base-frame support planes required')
            normal = np.asarray(plane.get('normal'), dtype=float)
            offset = plane.get('offset_m')
            name = plane.get('name')
            links = plane.get('stationary_contact_links', ())
            if (not isinstance(name, str) or not name or name in plane_names or normal.shape != (3,)
                    or not np.isfinite(normal).all() or not np.isclose(np.linalg.norm(normal), 1., atol=1e-8)
                    or type(offset) not in (int, float) or not math.isfinite(offset)
                    or not isinstance(links, (tuple, list)) or any(link not in model.links for link in links)):
                raise ValueError('finite unit support plane and explicit contact links required')
            normal = normal.copy()
            normal.setflags(write=False)
            self.support_planes.append((name, normal, float(offset), frozenset(links)))
            plane_names.add(name)
        # Check acquisition self geometry too. A collision-free candidate cannot
        # turn an already ambiguous measured/self-intersecting start into proof.
        self._current_self_valid = self._self_clear(current)
        self._current_self_rejection = self.last_rejection
        self.support_planes = tuple(self.support_planes)
        self._sealed = True

    def _fresh(self):
        now = time.monotonic()
        return (0 <= now-self.stamp_monotonic <= .5 and
                0 <= now-self.arm_stamp_monotonic <= .5 and
                self._check_elapsed_s < self.max_check_time_s)

    def _minimum_table(self, ky, kx):
        key = (ky, kx)
        if key not in self._depth_min_tables:
            if kx:
                previous = self._minimum_table(ky, kx-1)
                shift = 2**(kx-1)
                value = np.minimum(previous[:, :-shift], previous[:, shift:])
            else:
                previous = self._minimum_table(ky-1, kx)
                shift = 2**(ky-1)
                value = np.minimum(previous[:-shift, :], previous[shift:, :])
            if self._depth_table_bytes+value.nbytes > 128*1024*1024:
                raise ValueError('depth footprint proof exceeds bounded memory budget')
            value.setflags(write=False)
            self._depth_min_tables[key] = value
            self._depth_table_bytes += value.nbytes
        return self._depth_min_tables[key]

    def _rectangle_minima(self, low, high):
        size = high-low+1
        powers = np.floor(np.log2(size)).astype(int)
        result = np.full(len(low), -np.inf)
        for ky, kx in np.unique(powers, axis=0):
            select = np.flatnonzero(np.all(powers == (ky, kx), axis=1))
            table = self._minimum_table(int(ky), int(kx))
            y0, x0 = low[select].T
            y1, x1 = (high[select]-(np.array([2**ky, 2**kx])-1)).T
            result[select] = np.minimum.reduce((table[y0, x0], table[y0, x1], table[y1, x0], table[y1, x1]))
        return result

    def _projected_bounds(self, indices):
        centers = ((indices+.5)*self.voxel_size_m - self.base_from_camera[:3, 3]) @ self.base_from_camera[:3, :3]
        corners = (_CORNERS*self.voxel_size_m) @ self.base_from_camera[:3, :3]
        minimum_z = centers[:, 2]+corners[:, 2].min()
        maximum_z = centers[:, 2]+corners[:, 2].max()
        if np.any(minimum_z <= 0):
            return None
        # Perspective of a convex cube has extrema at corners when z>0.
        xlow, xhigh = np.full(len(indices), np.inf), np.full(len(indices), -np.inf)
        ylow, yhigh = xlow.copy(), xhigh.copy()
        for corner in corners:
            points = centers+corner
            x, y = points[:, 0]/points[:, 2], points[:, 1]/points[:, 2]
            xlow, xhigh = np.minimum(xlow, x), np.maximum(xhigh, x)
            ylow, yhigh = np.minimum(ylow, y), np.maximum(yhigh, y)
        x, y = (xlow, xhigh), (ylow, yhigh)
        if np.any(self.distortion != 0):
            # Interval bounds include nonlinear distortion extrema along edges
            # and in the footprint interior; corner-only projection is unsound.
            k1, k2, p1, p2, k3 = self.distortion
            x2, y2 = _square(x), _square(y)
            r2 = _add(x2, y2)
            r4, r6 = _square(r2), _mul(_square(r2), r2)
            radial = _add((np.ones(len(indices)), np.ones(len(indices))),
                          _scale(r2, k1), _scale(r4, k2), _scale(r6, k3))
            xy = _mul(x, y)
            distorted_x = _add(_mul(x, radial), _scale(xy, 2*p1), _scale(_add(r2, _scale(x2, 2)), p2))
            distorted_y = _add(_mul(y, radial), _scale(_add(r2, _scale(y2, 2)), p1), _scale(xy, 2*p2))
            x, y = distorted_x, distorted_y
        u = _add(_scale(x, self.intrinsics['fx']), (self.intrinsics['ppx'],)*2)
        v = _add(_scale(y, self.intrinsics['fy']), (self.intrinsics['ppy'],)*2)
        u = (u[0]-1e-9, u[1]+1e-9)
        v = (v[0]-1e-9, v[1]+1e-9)
        height, width = self.depth_m.shape
        if (not np.isfinite(np.concatenate((*u, *v))).all() or np.any(u[0] < 0) or np.any(v[0] < 0)
                or np.any(u[1] > width-1) or np.any(v[1] > height-1)):
            return None
        # Every intersected pixel, including footprint interior, must have valid
        # depth beyond the farthest voxel corner and axial measurement error.
        low = np.column_stack((np.floor(v[0]), np.floor(u[0]))).astype(int)
        high = np.column_stack((np.ceil(v[1]), np.ceil(u[1]))).astype(int)
        return low, high, maximum_z

    def _camera_clear(self, indices):
        # Batches bound temporary arrays and stop on the first unknown volume.
        for start in range(0, len(indices), 16_384):
            if not self._fresh():
                self.last_rejection = 'acquisition or computation budget expired'
                return False
            bounds = self._projected_bounds(indices[start:start+16_384])
            if bounds is None:
                self.last_rejection = 'candidate volume is behind/outside the camera or projection is unresolved'
                return False
            low, high, farthest = bounds
            minimum_depth = self._rectangle_minima(low, high)
            if np.any(minimum_depth <= np.nextafter(farthest+self.depth_uncertainty_m, np.inf)):
                self.last_rejection = 'candidate volume intersects depth, uncertainty, invalid or occluded space'
                return False
        return True

    def _self_clear(self, volumes):
        names = list(volumes)
        keys = {name: np.unique(voxel_keys(volumes[name])) for name in names}
        bounds = {name: (volumes[name].min(axis=0), volumes[name].max(axis=0)) for name in names if len(volumes[name])}
        for i, left in enumerate(names):
            for right in names[i+1:]:
                if tuple(sorted((left, right))) in self.model.allowed_self_contacts:
                    continue
                if left not in bounds or right not in bounds:
                    continue
                if np.any(bounds[left][1] < bounds[right][0]) or np.any(bounds[right][1] < bounds[left][0]):
                    continue
                if np.intersect1d(keys[left], keys[right], assume_unique=True).size:
                    self.last_rejection = f'unallowed self collision/contact: {left}, {right}'
                    return False
        return True

    def _supports_clear(self, volumes, transforms, displacement):
        for name, normal, offset, contacts in self.support_planes:
            support = self.voxel_size_m/2*np.abs(normal).sum()
            for link, indices in volumes.items():
                if displacement == 0 and link in contacts and np.array_equal(transforms[link], self._current_transforms[link]):
                    continue
                if np.any(((indices+.5)*self.voxel_size_m) @ normal + offset <= support):
                    self.last_rejection = f'unallowed support collision/contact: {link}, {name}'
                    return False
        return True

    def _check(self, raw, displacement):
        started = time.monotonic()
        if not self._fresh():
            self.last_rejection = 'acquisition or computation budget expired'
            return None
        try:
            # Copies bind cache keys to validated raw values, never mutable input.
            raw = dict(raw)
            self.model.joint_positions(raw)
            key = (tuple(sorted(raw.items())), displacement)
            if key in self._proofs:
                return self._proofs[key]
            if not self._current_self_valid:
                self.last_rejection = 'acquisition self collision is unresolved: '+str(self._current_self_rejection)
                return None
            volumes, _ = self.model.voxel_volume(raw, voxel_size_m=self.voxel_size_m,
                padding_m=self.clearance_m+self.geometry_uncertainty_m+displacement,
                interior_margin_m=self.geometry_uncertainty_m,
                deadline_monotonic=min(self.stamp_monotonic+.5, self.arm_stamp_monotonic+.5,
                    started+self.max_check_time_s-self._check_elapsed_s))
            # Inflated per-link cells also conservatively cover relative self
            # motion over the whole segment; allowed pairs must be explicit.
            if not self._self_clear(volumes) or not self._supports_clear(volumes, self.model.forward_kinematics(raw), displacement):
                result = None
            else:
                indices = np.unique(np.concatenate(list(volumes.values())), axis=0)
                exterior = ~np.isin(voxel_keys(indices), self._current_interior, assume_unique=False)
                result = self.clearance_m if self._camera_clear(indices[exterior]) else None
            self._proofs[key] = result
            while len(self._proofs) > 64:
                self._proofs.popitem(last=False)
        except (ValueError, KeyError, TypeError, ArithmeticError, np.linalg.LinAlgError) as error:
            self.last_rejection = str(error)
            result = None
        finally:
            self._check_elapsed_s += time.monotonic()-started
        return result if self._fresh() else None

    def minimum_clearance_m(self, raw_pose):
        """Return checked metric clearance or None for collision/unknown space."""
        return self._check(raw_pose, 0.)

    def minimum_segment_clearance_m(self, raw_a, raw_b):
        """Certify the whole continuous raw-linear segment by a motion envelope."""
        try:
            displacement = self.model.maximum_displacement_m(dict(raw_a), dict(raw_b))
            # Outward rounding permits reuse for equal physical step bounds.
            displacement = math.ceil(displacement*1e9)/1e9
        except (ValueError, KeyError, TypeError, ArithmeticError) as error:
            self.last_rejection = str(error)
            return None
        return self._check(raw_a, displacement)
