"""Effective URDF collision volumes for conservative, offline recovery checks.

Only collision geometry participates. Meshes become enclosing convex hulls;
unsupported joints, missing calibration and unresolved mesh URIs are errors.
No adjacent-link or support collision is implicitly ignored.
"""
from __future__ import annotations

from collections import OrderedDict
from collections.abc import Mapping
from types import MappingProxyType
from dataclasses import dataclass
import hashlib
import json
import math
import time
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation

from .camera_recovery import JOINTS


MAX_GRID_INDEX = 2**19
_GRID_BASE = 2**20
_GEOMETRY_EPS = 1e-10
_CORNERS = np.array([[x, y, z] for x in (-.5, .5) for y in (-.5, .5) for z in (-.5, .5)])


def voxel_keys(indices):
    values = np.asarray(indices, dtype=np.int64)
    if values.ndim != 2 or values.shape[1] != 3 or np.any(np.abs(values) >= MAX_GRID_INDEX):
        raise ValueError('voxel indices exceed the bounded collision workspace')
    shifted = values + MAX_GRID_INDEX
    return (shifted[:, 0] * _GRID_BASE + shifted[:, 1]) * _GRID_BASE + shifted[:, 2]


def validate_transform(value):
    value = np.asarray(value, dtype=np.float64)
    if (value.shape != (4, 4) or not np.isfinite(value).all()
            or not np.allclose(value[3], [0, 0, 0, 1], atol=1e-10)
            or not np.allclose(value[:3, :3].T @ value[:3, :3], np.eye(3), atol=1e-8)
            or not np.isclose(np.linalg.det(value[:3, :3]), 1., atol=1e-8)):
        raise ValueError('a finite proper rigid frame transform is required')
    return value.copy()


def _numbers(text, length, default=None):
    values = np.asarray(default if text is None else [float(v) for v in text.split()], dtype=float)
    if values.shape != (length,) or not np.isfinite(values).all():
        raise ValueError(f'URDF field requires {length} finite values')
    return values


def _origin(element):
    result = np.eye(4)
    if element is not None:
        result[:3, 3] = _numbers(element.get('xyz'), 3, [0, 0, 0])
        result[:3, :3] = Rotation.from_euler('xyz', _numbers(element.get('rpy'), 3, [0, 0, 0])).as_matrix()
    return result


def _positive(text):
    value = float(text)
    if not math.isfinite(value) or value <= 0:
        raise ValueError('URDF collision dimensions must be positive and finite')
    return value


@dataclass(frozen=True)
class CollisionShape:
    link_name: str
    kind: str
    link_from_shape: np.ndarray
    extents_m: np.ndarray
    planes: np.ndarray | None = None
    vertices: np.ndarray | None = None
    self_interior_qualified: bool = True

    @property
    def radius_bound_m(self):
        return float(np.linalg.norm(self.link_from_shape[:3, 3]) + np.linalg.norm(self.extents_m)/2)

    def bounds(self, base_from_shape, padding):
        rotation = base_from_shape[:3, :3]
        if self.kind == 'convex':
            points = self.vertices @ rotation.T + base_from_shape[:3, 3]
            return points.min(axis=0)-padding, points.max(axis=0)+padding
        if self.kind == 'sphere':
            extent = np.repeat(self.extents_m[0]/2, 3)
        elif self.kind == 'cylinder':
            radius, length = self.extents_m[0]/2, self.extents_m[2]
            extent = radius*np.linalg.norm(rotation[:, :2], axis=1) + length/2*np.abs(rotation[:, 2])
        else:
            extent = np.abs(rotation) @ (self.extents_m/2)
        return base_from_shape[:3, 3]-extent-padding, base_from_shape[:3, 3]+extent+padding

    def cells(self, indices, base_from_shape, pitch, padding, interior_margin, deadline=None):
        """Overapproximate occupied cubes, underapproximate strict interior.

        Plane dilation and center-distance tests include every intersecting cube.
        A cube is carved as current self only if all corners fit strictly within
        one actual convex shape shrunk by the specified uncertainty margin.
        """
        center = ((indices+.5)*pitch - base_from_shape[:3, 3]) @ base_from_shape[:3, :3]
        corners = (_CORNERS*pitch) @ base_from_shape[:3, :3]
        half_diagonal = math.sqrt(3)*pitch/2
        if self.kind == 'sphere':
            radius = self.extents_m[0]/2
            occupied = np.linalg.norm(center, axis=1) <= radius+padding+half_diagonal+_GEOMETRY_EPS
            strict = np.ones(len(center), dtype=bool)
            for corner in corners:
                strict &= np.linalg.norm(center+corner, axis=1) < radius-interior_margin-_GEOMETRY_EPS
        elif self.kind == 'cylinder':
            radius, half_length = self.extents_m[0]/2, self.extents_m[2]/2
            radial = np.linalg.norm(center[:, :2], axis=1)-radius
            axial = np.abs(center[:, 2])-half_length
            # Exact finite-cylinder signed distance at the cube center.
            outside = np.hypot(np.maximum(radial, 0), np.maximum(axial, 0))
            distance = outside + np.minimum(np.maximum(radial, axial), 0)
            occupied = distance <= padding+half_diagonal+_GEOMETRY_EPS
            strict = np.ones(len(center), dtype=bool)
            for corner in corners:
                point = center+corner
                strict &= ((np.linalg.norm(point[:, :2], axis=1) < radius-interior_margin-_GEOMETRY_EPS)
                           & (np.abs(point[:, 2]) < half_length-interior_margin-_GEOMETRY_EPS))
        else:
            if self.kind == 'box':
                normals = np.concatenate((np.eye(3), -np.eye(3)))
                offsets = -np.tile(self.extents_m/2, 2)
            else:
                normals, offsets = self.planes[:, :3], self.planes[:, 3]
            occupied = np.ones(len(center), dtype=bool)
            strict = np.ones(len(center), dtype=bool)
            # Batch half-spaces in C instead of a Python loop over thousands
            # of curved CAD hull facets. No plane is dropped or approximated.
            for start in range(0, len(normals), 128):
                if deadline is not None and time.monotonic() >= deadline:
                    raise ValueError('voxel proof computation deadline expired')
                batch = normals[start:start+128]
                signed = np.einsum('ij,kj->ik', center, batch) + offsets[start:start+128]
                support = np.max(np.abs(corners @ batch.T), axis=0)
                occupied &= np.all(signed <= support+padding+_GEOMETRY_EPS, axis=1)
                strict &= np.all(signed+support < -interior_margin-_GEOMETRY_EPS, axis=1)
                if not np.any(occupied):
                    break
        if not self.self_interior_qualified:
            # A convex enclosure can fill cavities or gaps in the original CAD.
            # Those spaces are obstacles for planning, not proven robot material
            # that may safely be vacated after this acquisition.
            strict[:] = False
        return indices[occupied], indices[strict]


class URDFCollisionModel:
    """Calibrated complete movable-joint FK and conservative collision solids."""
    def __setattr__(self, name, value):
        if getattr(self, '_sealed', False) and not name.startswith('_'):
            raise AttributeError('effective collision model is immutable')
        object.__setattr__(self, name, value)

    def __init__(self, urdf, *, joint_calibration, calibration_qualified,
                 base_frame, package_paths=None, allowed_self_contacts=(), max_cells=250_000):
        if calibration_qualified is not True:
            raise ValueError('qualified explicit joint calibration is required')
        self.base_frame = str(base_frame)
        if not self.base_frame:
            raise ValueError('base frame is required')
        if isinstance(urdf, Path) or isinstance(urdf, str) and not urdf.lstrip().startswith('<'):
            path = Path(urdf).resolve()
            xml, directory = path.read_bytes(), path.parent
        elif isinstance(urdf, str):
            xml, directory = urdf.encode(), None
        else:
            raise ValueError('URDF must be an effective XML string or path')
        root = ET.fromstring(xml)
        if root.tag != 'robot':
            raise ValueError('effective URDF robot element required')
        link_elements = root.findall('link')
        names = [entry.get('name') for entry in link_elements]
        if not names or any(not name for name in names) or len(names) != len(set(names)):
            raise ValueError('URDF link names must be unique and nonempty')
        self.links = frozenset(names)
        self.joints = []
        mesh_manifest = []
        self.shapes = []
        packages = package_paths or {}
        for link in link_elements:
            for collision in link.findall('collision'):
                geometry = collision.find('geometry')
                if geometry is None or len(geometry) != 1:
                    raise ValueError('exactly one supported solid per collision required')
                element = geometry[0]
                kind, planes, vertices = element.tag, None, None
                if kind == 'box':
                    extents = _numbers(element.get('size'), 3)
                    if np.any(extents <= 0):
                        raise ValueError('box extents must be positive')
                elif kind == 'sphere':
                    extents = np.repeat(2*_positive(element.get('radius')), 3)
                elif kind == 'cylinder':
                    extents = np.array([2*_positive(element.get('radius'))]*2 + [_positive(element.get('length'))])
                elif kind == 'mesh':
                    import trimesh
                    filename = element.get('filename', '')
                    if filename.startswith('package://'):
                        package, relative = filename[len('package://'):].split('/', 1)
                        if package not in packages:
                            raise ValueError(f'unresolved collision mesh package {package}')
                        package_root = Path(packages[package]).resolve()
                        mesh_path = (package_root/relative).resolve()
                        if not mesh_path.is_relative_to(package_root):
                            raise ValueError('collision mesh escapes the explicit package root')
                    elif filename.startswith('file://'):
                        mesh_path = Path(filename[len('file://'):]).resolve()
                    elif '://' in filename:
                        raise ValueError('unsupported collision mesh URI')
                    else:
                        mesh_path = Path(filename)
                        if not mesh_path.is_absolute():
                            if directory is None:
                                raise ValueError('relative collision mesh requires a saved URDF path')
                            mesh_path = directory/mesh_path
                        mesh_path = mesh_path.resolve()
                    mesh_bytes = mesh_path.read_bytes()
                    mesh = trimesh.load(mesh_path, force='mesh', process=False)
                    scale = _numbers(element.get('scale'), 3, [1, 1, 1])
                    if np.any(scale == 0) or not isinstance(mesh, trimesh.Trimesh) or not len(mesh.faces):
                        raise ValueError('nondegenerate collision triangle mesh required')
                    vertices = np.asarray(mesh.vertices, dtype=float)*scale
                    if not np.isfinite(vertices).all():
                        raise ValueError('collision mesh contains nonfinite vertices')
                    try:
                        hull = ConvexHull(vertices)
                    except Exception as error:
                        raise ValueError('collision mesh cannot form a solid convex hull') from error
                    planes = hull.equations.copy()
                    vertices = vertices[hull.vertices]
                    extents = vertices.max(axis=0)-vertices.min(axis=0)
                    # Recenter hull so primitive radius/reach bounds remain valid.
                    center = (vertices.max(axis=0)+vertices.min(axis=0))/2
                    vertices = vertices-center
                    planes[:, 3] += planes[:, :3] @ center
                    shape_origin = _origin(collision.find('origin'))
                    shape_origin[:3, 3] += shape_origin[:3, :3] @ center
                    mesh_manifest.append((filename, hashlib.sha256(mesh_bytes).hexdigest(), scale.tolist()))
                    self.shapes.append(CollisionShape(link.get('name'), 'convex', shape_origin, extents, planes, vertices,
                        bool(mesh.is_watertight and mesh.is_convex)))
                    continue
                else:
                    raise ValueError(f'unsupported URDF collision geometry {kind}')
                self.shapes.append(CollisionShape(link.get('name'), kind, _origin(collision.find('origin')), extents))
        if not self.shapes:
            raise ValueError('effective URDF contains no collision geometry')
        children, joint_names = set(), set()
        for element in root.findall('joint'):
            kind, name = element.get('type'), element.get('name')
            if kind not in {'fixed', 'revolute', 'continuous', 'prismatic'} or not name or name in joint_names:
                raise ValueError('unsupported or duplicate URDF joint')
            parent, child = element.find('parent'), element.find('child')
            if parent is None or child is None or parent.get('link') not in self.links or child.get('link') not in self.links:
                raise ValueError('URDF joint references an unknown link')
            if child.get('link') in children:
                raise ValueError('URDF link has multiple parents')
            children.add(child.get('link'))
            joint_names.add(name)
            axis_element = element.find('axis')
            axis = _numbers(axis_element.get('xyz') if axis_element is not None else None, 3, [1, 0, 0])
            if kind != 'fixed' and np.linalg.norm(axis) < 1e-12:
                raise ValueError('movable joint axis cannot be zero')
            if np.linalg.norm(axis) > 0:
                axis /= np.linalg.norm(axis)
            limit, mimic = element.find('limit'), element.find('mimic')
            bounds = None
            if kind in {'revolute', 'prismatic'}:
                if limit is None:
                    raise ValueError('bounded URDF joints require explicit limits')
                bounds = (float(limit.get('lower')), float(limit.get('upper')))
                if not all(math.isfinite(v) for v in bounds) or bounds[0] >= bounds[1]:
                    raise ValueError('URDF joint limits must be finite and ordered')
            mimics = None
            if mimic is not None:
                mimics = (mimic.get('joint'), float(mimic.get('multiplier', '1')), float(mimic.get('offset', '0')))
                if not mimics[0] or not all(math.isfinite(v) for v in mimics[1:]):
                    raise ValueError('invalid URDF mimic mapping')
            self.joints.append(dict(name=name, type=kind, parent=parent.get('link'), child=child.get('link'),
                axis=axis, origin=_origin(element.find('origin')), limits=bounds, mimic=mimics))
        roots = self.links-children
        if roots != {self.base_frame}:
            raise ValueError('effective URDF root must equal the qualified base frame')
        self.by_joint = {entry['name']: entry for entry in self.joints}
        if not isinstance(joint_calibration, dict) or set(joint_calibration) != set(JOINTS):
            raise ValueError('complete explicit six-joint raw calibration including gripper required')
        self.calibration = {}
        mapped = set()
        for raw_name, source in joint_calibration.items():
            if not isinstance(source, dict) or set(source) != {
                    'urdf_joint', 'raw_reference', 'position_reference', 'units_per_count', 'unit'}:
                raise ValueError('raw calibration requires joint, reference, scale and physical unit')
            entry = dict(source)
            joint = self.by_joint.get(entry['urdf_joint'])
            expected_unit = 'meters' if joint and joint['type'] == 'prismatic' else 'radians'
            if not joint or joint['type'] == 'fixed' or joint['mimic'] or entry['urdf_joint'] in mapped:
                raise ValueError('calibration must uniquely map every independent movable joint')
            if (entry['unit'] != expected_unit or raw_name == 'gripper' and expected_unit != 'radians'
                    or type(entry['raw_reference']) is not int or not 0 <= entry['raw_reference'] <= 4095
                    or any(type(entry[field]) not in (int, float) or not math.isfinite(entry[field])
                           for field in ('position_reference', 'units_per_count')) or entry['units_per_count'] == 0):
                raise ValueError('explicit finite physical unit calibration required, including gripper angle')
            mapped.add(entry['urdf_joint'])
            self.calibration[raw_name] = entry
        independent = {entry['name'] for entry in self.joints if entry['type'] != 'fixed' and not entry['mimic']}
        if independent != mapped:
            raise ValueError('all independent movable URDF joints require calibration')
        self.allowed_self_contacts = set()
        for pair in allowed_self_contacts:
            if len(pair) != 2 or pair[0] == pair[1] or any(name not in self.links for name in pair):
                raise ValueError('self contacts require explicit valid distinct link pairs')
            self.allowed_self_contacts.add(tuple(sorted(pair)))
        manifest = json.dumps({'meshes': mesh_manifest, 'allowed_self_contacts': sorted(self.allowed_self_contacts)}, sort_keys=True, separators=(',', ':')).encode()
        self.model_sha256 = hashlib.sha256(xml+b'\0'+manifest).hexdigest()
        self.calibration_sha256 = hashlib.sha256(json.dumps(self.calibration, sort_keys=True,
            separators=(',', ':'), allow_nan=False).encode()).hexdigest()
        if type(max_cells) is not int or not 1 <= max_cells <= 1_000_000:
            raise ValueError('bounded positive voxel allocation limit required')
        self.max_cells = max_cells
        self._volume_cache = OrderedDict()
        self._volume_cache_cell_count = 0
        # Universal reach encloses every shape and joint origin for every joint
        # angle. Rotation preserves lengths; add maximal calibrated translations.
        self.reach_bound_m = sum(np.linalg.norm(joint['origin'][:3, 3]) for joint in self.joints)
        self.reach_bound_m += max(shape.radius_bound_m for shape in self.shapes)
        for joint in self.joints:
            if joint['type'] == 'prismatic':
                self.reach_bound_m += max(abs(v) for v in joint['limits'])
        self.forward_kinematics({name: entry['raw_reference'] for name, entry in self.calibration.items()})
        for shape in self.shapes:
            for array in (shape.link_from_shape, shape.extents_m, shape.planes, shape.vertices):
                if array is not None:
                    array.setflags(write=False)
        if any(np.any(shape.extents_m > 10) or np.linalg.norm(shape.link_from_shape[:3, 3]) > 10 for shape in self.shapes):
            raise ValueError('collision geometry exceeds the bounded local robot workspace')
        if any(np.linalg.norm(joint['origin'][:3, 3]) > 10 for joint in self.joints):
            raise ValueError('joint origin exceeds the bounded local robot workspace')
        self.shapes = tuple(self.shapes)
        for joint in self.joints:
            joint['axis'].setflags(write=False)
            joint['origin'].setflags(write=False)
        self.joints = tuple(MappingProxyType(joint) for joint in self.joints)
        self.by_joint = MappingProxyType({entry['name']: entry for entry in self.joints})
        self.calibration = MappingProxyType({name: MappingProxyType(value) for name, value in self.calibration.items()})
        self.allowed_self_contacts = frozenset(self.allowed_self_contacts)
        self._sealed = True

    def joint_positions(self, raw):
        if not isinstance(raw, Mapping) or set(raw) != set(JOINTS) or any(
                type(value) is not int or not 0 <= value <= 4095 for value in raw.values()):
            raise ValueError('complete measured integer raw six-joint pose required')
        values = {entry['urdf_joint']: entry['position_reference'] +
                  (raw[name]-entry['raw_reference'])*entry['units_per_count']
                  for name, entry in self.calibration.items()}
        pending = [joint for joint in self.joints if joint['mimic']]
        while pending:
            remaining = []
            for joint in pending:
                parent, multiplier, offset = joint['mimic']
                if parent not in values:
                    remaining.append(joint)
                else:
                    values[joint['name']] = values[parent]*multiplier+offset
            if len(remaining) == len(pending):
                raise ValueError('unresolved or cyclic URDF mimic joints')
            pending = remaining
        for name, value in values.items():
            bounds = self.by_joint[name]['limits']
            if not math.isfinite(value) or bounds and not bounds[0]-1e-12 <= value <= bounds[1]+1e-12:
                raise ValueError('calibrated pose exceeds effective URDF joint limits')
        return values

    def forward_kinematics(self, raw):
        positions = self.joint_positions(raw)
        transforms, pending = {self.base_frame: np.eye(4)}, list(self.joints)
        while pending:
            remaining = []
            for joint in pending:
                if joint['parent'] not in transforms:
                    remaining.append(joint)
                    continue
                motion = np.eye(4)
                if joint['type'] in {'revolute', 'continuous'}:
                    motion[:3, :3] = Rotation.from_rotvec(joint['axis']*positions[joint['name']]).as_matrix()
                elif joint['type'] == 'prismatic':
                    motion[:3, 3] = joint['axis']*positions[joint['name']]
                transforms[joint['child']] = transforms[joint['parent']] @ joint['origin'] @ motion
            if len(remaining) == len(pending):
                raise ValueError('disconnected or cyclic URDF joint graph')
            pending = remaining
        if set(transforms) != self.links:
            raise ValueError('incomplete effective robot forward kinematics')
        return transforms

    def maximum_displacement_m(self, raw_a, raw_b):
        a, b = self.joint_positions(raw_a), self.joint_positions(raw_b)
        # Telescoping transforms bounds motion even with simultaneously varying
        # axes and mimic joints. Every axis and shape lies inside reach_bound.
        bound = 0.
        for joint in self.joints:
            if joint['type'] in {'revolute', 'continuous'}:
                bound += abs(b[joint['name']]-a[joint['name']])*2*self.reach_bound_m
            elif joint['type'] == 'prismatic':
                bound += abs(b[joint['name']]-a[joint['name']])
        return float(bound)

    def voxel_volume(self, raw, *, voxel_size_m, padding_m=0., interior_margin_m=0., deadline_monotonic=None):
        pitch = float(voxel_size_m)
        if (not math.isfinite(pitch) or not 0 < pitch <= .005 or
                not all(math.isfinite(v) and v >= 0 for v in (padding_m, interior_margin_m))):
            raise ValueError('finite conservative voxel size/padding/margin required')
        transforms = self.forward_kinematics(raw)
        occupied_by_link, interior_by_link = {}, {}
        allocation = 0
        for number, shape in enumerate(self.shapes):
            if deadline_monotonic is not None and time.monotonic() >= deadline_monotonic:
                raise ValueError('voxel proof computation deadline expired')
            transform = transforms[shape.link_name] @ shape.link_from_shape
            cache_key = (number, transform.tobytes(), pitch, float(padding_m), float(interior_margin_m))
            if cache_key in self._volume_cache:
                occupied, interior = self._volume_cache[cache_key]
                self._volume_cache.move_to_end(cache_key)
            else:
                low, high = shape.bounds(transform, padding_m)
                # A boundary exactly on a grid plane also occupies the cell on
                # either side; outward-rounded bounds prevent gaps.
                lower = np.floor(np.nextafter(low/pitch, -np.inf)).astype(np.int64)
                upper = np.floor(np.nextafter(high/pitch, np.inf)).astype(np.int64)
                counts = upper-lower+1
                count = math.prod(int(value) for value in counts)
                if count <= 0 or count > self.max_cells:
                    raise ValueError('collision solid exceeds bounded voxel allocation')
                indices = np.stack(np.meshgrid(*(np.arange(lo, hi+1) for lo, hi in zip(lower, upper)), indexing='ij'), axis=-1).reshape(-1, 3)
                occupied, interior = shape.cells(indices, transform, pitch, padding_m, interior_margin_m, deadline_monotonic)
                occupied.setflags(write=False)
                interior.setflags(write=False)
                self._volume_cache[cache_key] = (occupied, interior)
                self._volume_cache_cell_count += len(occupied)+len(interior)
                while len(self._volume_cache) > 96 or self._volume_cache_cell_count > self.max_cells*4:
                    _, removed = self._volume_cache.popitem(last=False)
                    self._volume_cache_cell_count -= len(removed[0])+len(removed[1])
            allocation += len(occupied)
            if allocation > self.max_cells:
                raise ValueError('effective robot exceeds bounded voxel allocation')
            occupied_by_link.setdefault(shape.link_name, []).append(occupied)
            interior_by_link.setdefault(shape.link_name, []).append(interior)
        occupied_by_link = {name: np.unique(np.concatenate(arrays), axis=0) for name, arrays in occupied_by_link.items()}
        interior_by_link = {name: np.unique(np.concatenate(arrays), axis=0) for name, arrays in interior_by_link.items()}
        return occupied_by_link, interior_by_link
