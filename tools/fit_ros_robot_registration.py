#!/usr/bin/env python3
"""Offline, unqualified one-capture URDF-to-depth base registration candidate."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import trimesh
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from scipy.spatial import cKDTree


def _numbers(text, default):
    return np.fromstring(text, sep=" ") if text else np.asarray(default, dtype=float)


def _transform(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    result = np.eye(4)
    result[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                      [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                      [-sp, cp*sr, cp*cr]]
    result[:3, 3] = xyz
    return result


def parse_urdf(xml_or_path):
    """Read visual meshes, joint origins, and joint axes from one URDF."""
    if isinstance(xml_or_path, ET.Element):
        root = xml_or_path
    elif isinstance(xml_or_path, (str, Path)) and Path(str(xml_or_path)).is_file():
        root = ET.parse(xml_or_path).getroot()
    else:
        root = ET.fromstring(xml_or_path)
    links = {link.get("name"): [] for link in root.findall("link")}
    for link in root.findall("link"):
        for visual in link.findall("visual"):
            mesh = visual.find("geometry/mesh")
            if mesh is None or not mesh.get("filename"):
                continue
            origin = visual.find("origin")
            xyz = _numbers(origin.get("xyz") if origin is not None else None, [0, 0, 0])
            rpy = _numbers(origin.get("rpy") if origin is not None else None, [0, 0, 0])
            links[link.get("name")].append((mesh.get("filename"), _transform(xyz, rpy)))
    joints = []
    for item in root.findall("joint"):
        origin, axis = item.find("origin"), item.find("axis")
        joints.append({
            "name": item.get("name"), "type": item.get("type"),
            "parent": item.find("parent").get("link"),
            "child": item.find("child").get("link"),
            "origin": _transform(
                _numbers(origin.get("xyz") if origin is not None else None, [0, 0, 0]),
                _numbers(origin.get("rpy") if origin is not None else None, [0, 0, 0])),
            "axis": _numbers(axis.get("xyz") if axis is not None else None, [1, 0, 0]),
        })
    return links, joints


def _axis_rotation(axis, angle):
    axis = np.asarray(axis, float)
    axis = axis / np.linalg.norm(axis)
    skew = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]],
                     [-axis[1], axis[0], 0]])
    result = np.eye(4)
    result[:3, :3] = np.eye(3) + math.sin(angle)*skew + (1-math.cos(angle))*(skew @ skew)
    return result


def forward_kinematics(links, joints, joint_positions):
    """Compute base_T_link with URDF origin followed by the joint motion."""
    roots = set(links) - {joint["child"] for joint in joints}
    if len(roots) != 1:
        raise ValueError(f"expected one URDF root, found {sorted(roots)}")
    transforms, pending = {next(iter(roots)): np.eye(4)}, list(joints)
    while pending:
        rest, advanced = [], False
        for joint in pending:
            if joint["parent"] not in transforms:
                rest.append(joint)
                continue
            motion = np.eye(4)
            q = float(joint_positions.get(joint["name"], 0.0))
            if joint["type"] in ("revolute", "continuous"):
                motion = _axis_rotation(joint["axis"], q)
            elif joint["type"] == "prismatic":
                motion[:3, 3] = joint["axis"] * q
            transforms[joint["child"]] = transforms[joint["parent"]] @ joint["origin"] @ motion
            advanced = True
        if not advanced:
            raise ValueError("URDF joint graph is disconnected or cyclic")
        pending = rest
    return transforms


def resolve_mesh(filename, package_paths):
    if filename.startswith("package://"):
        package, relative = filename[len("package://"):].split("/", 1)
        if package not in package_paths:
            raise ValueError(f"missing saved path for mesh package {package!r}")
        path = Path(package_paths[package]) / relative
    else:
        path = Path(filename)
    if not path.is_file():
        raise FileNotFoundError(f"URDF mesh is missing: {path}")
    return path


def sample_visual_surfaces(urdf_path, package_paths, joint_positions, samples=4000, seed=17):
    """Area-sample URDF visual triangle surfaces; keep CAD normals and link labels."""
    links, joints = parse_urdf(urdf_path)
    base_from_link = forward_kinematics(links, joints, joint_positions)
    visuals = []
    for name, entries in links.items():
        for filename, link_from_visual in entries:
            mesh = trimesh.load(resolve_mesh(filename, package_paths), force="mesh", process=False)
            if isinstance(mesh, trimesh.Trimesh) and len(mesh.faces) and mesh.area > 0:
                visuals.append((name, mesh, base_from_link[name] @ link_from_visual))
    if not visuals:
        raise ValueError("URDF contains no loadable visual triangle meshes")
    area = sum(mesh.area for _, mesh, _ in visuals)
    counts = [max(1, round(samples * mesh.area / area)) for _, mesh, _ in visuals]
    rng, points, normals, labels = np.random.default_rng(seed), [], [], []
    for (name, mesh, base_from_visual), count in zip(visuals, counts):
        local, face = trimesh.sample.sample_surface(mesh, count, seed=rng)
        rotation = base_from_visual[:3, :3]
        points.append(local @ rotation.T + base_from_visual[:3, 3])
        normals.append(mesh.face_normals[face] @ rotation.T)
        labels.extend([name] * len(local))
    labels = np.asarray(labels)
    counts_by_link = {name: int(np.sum(labels == name)) for name in sorted(set(labels))}
    return np.concatenate(points), np.concatenate(normals), labels, counts_by_link


def load_visual_mesh(urdf_path, package_paths, joint_positions):
    """Assemble exact visual triangles in the URDF base frame with face labels."""
    links, joints = parse_urdf(urdf_path)
    base_from_link = forward_kinematics(links, joints, joint_positions)
    meshes, face_links = [], []
    for name, entries in links.items():
        for filename, link_from_visual in entries:
            mesh = trimesh.load(resolve_mesh(filename, package_paths), force="mesh", process=False)
            if not isinstance(mesh, trimesh.Trimesh) or not len(mesh.faces):
                continue
            mesh = mesh.copy()
            mesh.apply_transform(base_from_link[name] @ link_from_visual)
            meshes.append(mesh)
            face_links.extend([name] * len(mesh.faces))
    if not meshes:
        raise ValueError("URDF contains no loadable visual triangle meshes")
    return trimesh.util.concatenate(meshes), np.asarray(face_links)


def depth_roi_points(points_base, normals_base, initial_world_from_base,
                     tag_world_from_base, world_from_camera, depth_raw,
                     depth_scale, intrinsics, distortion, min_world_z_m=.02,
                     max_depth_gap_m=.06):
    """Build one fixed observed point set from visible CAD z-buffers around both pose seeds."""
    height, width = depth_raw.shape
    camera_from_world = np.linalg.inv(world_from_camera)
    roi = np.zeros((height, width), dtype=bool)
    zbuffer = np.full((height, width), np.inf, dtype=float)
    for pose in (initial_world_from_base, tag_world_from_base):
        world = points_base @ pose[:3, :3].T + pose[:3, 3]
        camera = world @ camera_from_world[:3, :3].T + camera_from_world[:3, 3]
        normal_camera = normals_base @ (camera_from_world[:3, :3] @ pose[:3, :3]).T
        pixels = _project(camera, intrinsics, distortion)
        u, v = np.rint(pixels[:, 0]).astype(int), np.rint(pixels[:, 1]).astype(int)
        valid = ((camera[:, 2] > 0)
                 & (np.einsum("ij,ij->i", normal_camera, -camera) > 0)
                 & (u >= 0) & (u < width) & (v >= 0) & (v < height))
        np.minimum.at(zbuffer, (v[valid], u[valid]), camera[valid, 2])
        roi[v[valid], u[valid]] = True
    roi = cv2.dilate(roi.astype(np.uint8), np.ones((5, 5), np.uint8)).astype(bool)
    valid = roi & np.isfinite(depth_raw) & (depth_raw > 0)
    observed_depth = depth_raw.astype(float) * depth_scale
    valid &= np.abs(observed_depth - zbuffer) <= max_depth_gap_m
    v, u = np.nonzero(valid)
    z = observed_depth[v, u]
    camera_points = _unproject(np.column_stack((u, v)), z, intrinsics, distortion)
    world_points = camera_points @ world_from_camera[:3, :3].T + world_from_camera[:3, 3]
    valid = world_points[:, 2] > min_world_z_m
    pixels = np.column_stack((u[valid], v[valid]))
    world_points = world_points[valid]
    if len(pixels):
        key = pixels[:, 1] * width + pixels[:, 0]
        _, keep = np.unique(key, return_index=True)
        world_points, pixels = world_points[keep], pixels[keep]
    if len(world_points) > 2000:
        keep = np.linspace(0, len(world_points) - 1, 2000, dtype=int)
        world_points, pixels = world_points[keep], pixels[keep]
    if len(world_points) < 40:
        raise ValueError(f"visible CAD z-buffer yielded only {len(world_points)} fixed observed points")
    return world_points, pixels


def closest_surface_metrics(points_base, normals_base, point_links, observed_base,
                            base_delta, labels=None):
    """Nearest sampled CAD surface residuals for one fixed observed population."""
    rotation = Rotation.from_rotvec(base_delta[3:]).as_matrix()
    inverse = (observed_base - base_delta[:3]) @ rotation
    tree = cKDTree(points_base)
    distance, indices = tree.query(inverse, k=1)
    nearest_base = points_base[indices]
    nearest = nearest_base @ rotation.T + base_delta[:3]
    residuals = np.linalg.norm(observed_base - nearest, axis=1)
    signed = np.einsum("ij,ij->i", observed_base - nearest,
                       normals_base[indices] @ rotation.T)
    if labels is None:
        labels = point_links[indices]
    per_link = {}
    for name in sorted(set(point_links)):
        values = residuals[labels == name]
        per_link[name] = {
            "observed_points": int(len(values)),
            "median_surface_residual_m": float(np.median(values)) if len(values) else None,
            "p95_surface_residual_m": float(np.percentile(values, 95)) if len(values) else None,
        }
    return {"observed_points": int(len(residuals)),
            "median_surface_residual_m": float(np.median(residuals)),
            "p95_surface_residual_m": float(np.percentile(residuals, 95)),
            "median_abs_point_to_plane_m": float(np.median(np.abs(signed))),
            "p95_abs_point_to_plane_m": float(np.percentile(np.abs(signed), 95)),
            "per_link": per_link}, labels, nearest_base


def exact_surface_metrics(mesh_base, face_links, observed_base, base_delta, labels=None):
    """Exact triangle closest-point residuals; used for the final fixed-population scores."""
    rotation = Rotation.from_rotvec(base_delta[3:]).as_matrix()
    inverse = (observed_base - base_delta[:3]) @ rotation
    closest, residuals, face_ids = trimesh.proximity.closest_point(mesh_base, inverse)
    signed = np.einsum("ij,ij->i", inverse - closest, mesh_base.face_normals[face_ids])
    if labels is None:
        labels = face_links[face_ids]
    per_link = {}
    for name in sorted(set(face_links)):
        values = residuals[labels == name]
        per_link[name] = {
            "observed_points": int(len(values)),
            "median_surface_residual_m": float(np.median(values)) if len(values) else None,
            "p95_surface_residual_m": float(np.percentile(values, 95)) if len(values) else None,
        }
    absolute_plane = np.abs(signed)
    return {"observed_points": int(len(residuals)),
            "median_surface_residual_m": float(np.median(residuals)),
            "p95_surface_residual_m": float(np.percentile(residuals, 95)),
            "median_abs_point_to_plane_m": float(np.median(absolute_plane)),
            "p95_abs_point_to_plane_m": float(np.percentile(absolute_plane, 95)),
            "per_link": per_link}, labels, closest


def refine_point_to_plane(points_base, normals_base, observed_base, initial_delta=None,
                          translation_limit_m=.03, rotation_limit_deg=6.0,
                          iterations=12):
    """Bounded robust point-to-plane ICP, with exact triangle correspondences."""
    x = np.zeros(6) if initial_delta is None else np.asarray(initial_delta, float).copy()
    angular_bound = math.radians(rotation_limit_deg)
    lower = np.array([-translation_limit_m] * 3 + [-angular_bound] * 3)
    upper = -lower
    x = np.clip(x, lower + 1e-9, upper - 1e-9)
    tree, history = cKDTree(points_base), []
    for iteration in range(iterations):
        rotation = Rotation.from_rotvec(x[3:]).as_matrix()
        inverse = (observed_base - x[:3]) @ rotation
        distance, indices = tree.query(inverse, k=1)
        keep = distance <= .06
        if keep.sum() < 40:
            raise ValueError(f"ICP retained only {keep.sum()} points within 6 cm")
        q = points_base[indices[keep]]
        target = observed_base[keep]
        normals = normals_base[indices[keep]]

        def residual(candidate):
            r = Rotation.from_rotvec(candidate[3:]).as_matrix()
            moved = q @ r.T + candidate[:3]
            moved_normals = normals @ r.T
            return np.einsum("ij,ij->i", moved - target, moved_normals)

        result = least_squares(residual, x, bounds=(lower, upper), loss="soft_l1",
                               f_scale=.002, max_nfev=40)
        bounded = result.x.copy()
        translation_norm = np.linalg.norm(bounded[:3])
        rotation_norm = np.linalg.norm(bounded[3:])
        if translation_norm > translation_limit_m:
            bounded[:3] *= translation_limit_m / translation_norm
        if rotation_norm > angular_bound:
            bounded[3:] *= angular_bound / rotation_norm
        step = float(np.linalg.norm(bounded - x))
        x = bounded
        history.append({"iteration": iteration + 1, "correspondences": int(keep.sum()),
                        "robust_cost": float(result.cost), "parameter_step_norm": step})
        if step < 1e-6:
            break
    if np.linalg.norm(x[3:]) > angular_bound + 1e-6 or np.linalg.norm(x[:3]) > translation_limit_m + 1e-6:
        raise ValueError("ICP correction exceeded translation/rotation norm bounds")
    return x, history


def robust_rigid_fit(source, target, iterations=12, max_residual_m=.06):
    """Kabsch point-pair fit with iterative median/MAD trimming."""
    source, target = np.asarray(source, float), np.asarray(target, float)
    if (source.shape != target.shape or source.ndim != 2 or source.shape[1] != 3
            or len(source) < 6 or not np.isfinite(source).all() or not np.isfinite(target).all()):
        raise ValueError("rigid fit needs at least six finite matched point pairs")
    keep = np.ones(len(source), bool)
    transform = np.eye(4)
    for _ in range(iterations):
        a, b = source[keep], target[keep]
        if len(a) < 6:
            raise ValueError("trimming left fewer than six points")
        ac, bc = a.mean(axis=0), b.mean(axis=0)
        u, _, vt = np.linalg.svd((a-ac).T @ (b-bc))
        rotation = vt.T @ u.T
        if np.linalg.det(rotation) < 0:
            vt[-1] *= -1
            rotation = vt.T @ u.T
        transform[:3, :3] = rotation
        transform[:3, 3] = bc - rotation @ ac
        residual = np.linalg.norm(source @ rotation.T + transform[:3, 3] - target, axis=1)
        median = np.median(residual)
        mad = 1.4826 * np.median(np.abs(residual - median))
        updated = residual <= min(max_residual_m, max(.003, median + 3*mad))
        if updated.sum() < 6:
            raise ValueError("robust fit found too few inliers")
        if np.array_equal(updated, keep):
            break
        keep = updated
    return transform, keep


def _project(points, intrinsics, distortion):
    image, _ = cv2.projectPoints(np.asarray(points, float).reshape(-1, 1, 3),
                                 np.zeros(3), np.zeros(3), intrinsics, distortion)
    return image.reshape(-1, 2)


def _unproject(pixels, depth_m, intrinsics, distortion):
    normalized = cv2.undistortPoints(np.asarray(pixels, float).reshape(-1, 1, 2),
                                     intrinsics, distortion).reshape(-1, 2)
    return np.column_stack((normalized * depth_m[:, None], depth_m))


def _empty_matches(projected_counts=None):
    return {"source": np.empty((0, 3)), "target": np.empty((0, 3)),
            "links": np.empty(0, dtype=str), "pixels": np.empty((0, 2), dtype=int),
            "projected": np.empty((0, 2)), "source_ids": np.empty(0, dtype=int),
            "projected_counts": projected_counts or {}}


def depth_matches(points_base, normals_base, links, world_from_base, world_from_camera,
                  depth_raw, depth_scale, intrinsics, distortion,
                  max_depth_gap_m=.06, min_world_z_m=.02):
    """Match camera-facing CAD samples to close depth returns above the table."""
    height, width = depth_raw.shape
    camera_from_world = np.linalg.inv(world_from_camera)
    world = points_base @ world_from_base[:3, :3].T + world_from_base[:3, 3]
    camera = world @ camera_from_world[:3, :3].T + camera_from_world[:3, 3]
    normal_camera = normals_base @ (camera_from_world[:3, :3] @ world_from_base[:3, :3]).T
    pixels = _project(camera, intrinsics, distortion)
    u, v = np.rint(pixels[:, 0]).astype(int), np.rint(pixels[:, 1]).astype(int)
    valid = ((np.einsum("ij,ij->i", normal_camera, -camera) > 0)
             & np.isfinite(camera).all(axis=1) & (camera[:, 2] > 0))
    valid &= (u >= 1) & (u < width-1) & (v >= 1) & (v < height-1)
    source_ids = np.flatnonzero(valid)
    projected_counts = {name: int(np.sum(links[source_ids] == name))
                        for name in sorted(set(links))}
    if not len(source_ids):
        return _empty_matches(projected_counts)
    u, v = u[source_ids], v[source_ids]
    offsets = np.array([[-1,-1], [0,-1], [1,-1], [-1,0], [0,0], [1,0],
                        [-1,1], [0,1], [1,1]])
    sample_u, sample_v = u[:, None] + offsets[None, :, 0], v[:, None] + offsets[None, :, 1]
    depth_m = depth_raw[sample_v, sample_u].astype(float) * depth_scale
    error = np.abs(depth_m - camera[source_ids, 2, None])
    error[(depth_m <= 0) | ~np.isfinite(depth_m)] = np.inf
    choice = np.argmin(error, axis=1)
    best_error = error[np.arange(len(choice)), choice]
    actual_u, actual_v = sample_u[np.arange(len(choice)), choice], sample_v[np.arange(len(choice)), choice]
    observed_z = depth_m[np.arange(len(choice)), choice]
    keep = (best_error <= max_depth_gap_m) & np.isfinite(observed_z)
    source_ids, actual_u, actual_v, observed_z = (item[keep] for item in
                                                   (source_ids, actual_u, actual_v, observed_z))
    if not len(source_ids):
        return _empty_matches(projected_counts)
    observed_camera = _unproject(np.column_stack((actual_u, actual_v)), observed_z,
                                 intrinsics, distortion)
    observed_world = observed_camera @ world_from_camera[:3, :3].T + world_from_camera[:3, 3]
    keep = observed_world[:, 2] > min_world_z_m
    source_ids, actual_u, actual_v, observed_world = (item[keep] for item in
                                                       (source_ids, actual_u, actual_v, observed_world))
    if not len(source_ids):
        return _empty_matches(projected_counts)
    # Avoid giving one depth sample repeated weight from neighboring mesh faces.
    pixel_key = actual_v * width + actual_u
    order = np.argsort(pixel_key, kind="stable")
    _, first = np.unique(pixel_key[order], return_index=True)
    selected = order[first]
    source_ids, observed_world = source_ids[selected], observed_world[selected]
    return {"source": points_base[source_ids], "target": observed_world,
            "links": links[source_ids], "pixels": np.column_stack((actual_u[selected], actual_v[selected])),
            "projected": pixels[source_ids], "source_ids": source_ids,
            "projected_counts": projected_counts}


def _transform_delta(candidate, initial):
    delta = candidate @ np.linalg.inv(initial)
    angle = math.degrees(math.acos(float(np.clip((np.trace(delta[:3, :3])-1)/2, -1, 1))))
    return float(np.linalg.norm(delta[:3, 3])), angle


def _metrics(matches, transform, all_links, max_gap):
    residual = np.linalg.norm(matches["source"] @ transform[:3, :3].T
                              + transform[:3, 3] - matches["target"], axis=1)
    per_link = {}
    for name in sorted(set(all_links)):
        selected = matches["links"] == name
        values = residual[selected]
        per_link[name] = {
            "matched_depth_pixels": int(selected.sum()),
            "projected_frontface_samples": int(matches["projected_counts"].get(name, 0)),
            "overlap_fraction": float(selected.sum() / max(1, matches["projected_counts"].get(name, 0))),
            "median_residual_m": float(np.median(values)) if len(values) else None,
            "p95_residual_m": float(np.percentile(values, 95)) if len(values) else None,
        }
    return {
        "matched_depth_pixels": int(len(residual)),
        "median_residual_m": float(np.median(residual)) if len(residual) else None,
        "p95_residual_m": float(np.percentile(residual, 95)) if len(residual) else None,
        "inlier_fraction_6cm": float(np.mean(residual <= max_gap)) if len(residual) else 0.0,
        "per_link": per_link,
    }


def _sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def fit_bundle(bundle, *, samples=24000, iterations=3, max_depth_gap_m=.06,
               min_world_z_m=.02, max_translation_m=.03, max_rotation_deg=6.0):
    """Fit and save one unqualified exact-mesh/depth candidate and image overlay."""
    if not 1 <= iterations <= 3:
        raise ValueError("outer ICP rounds must be between one and three")
    bundle = Path(bundle).resolve()
    capture = json.loads((bundle / "capture.json").read_text())
    urdf = bundle / "robot.urdf"
    image = cv2.imread(str(bundle / "color.png"), cv2.IMREAD_COLOR)
    depth = np.load(bundle / "depth.npy", allow_pickle=False)
    camera = capture["camera"]
    if image is None or depth.ndim != 2 or image.shape[:2] != depth.shape:
        raise ValueError("saved color/depth images are missing or have different dimensions")
    if capture.get("depth_encoding") != "16UC1":
        raise ValueError("only saved 16UC1 raw depth is supported")
    intrinsics = np.asarray(camera["k"], float).reshape(3, 3)
    distortion = np.asarray(camera.get("d", [0]*5), float).reshape(-1, 1)
    depth_scale = float(capture["depth_scale_m"])
    world_from_camera = np.asarray(capture["world_from_camera"], float)
    initial = np.asarray(capture["world_from_base"], float)
    if (intrinsics[0, 0] <= 0 or intrinsics[1, 1] <= 0 or depth_scale <= 0
            or not np.isfinite(intrinsics).all() or not np.isfinite(distortion).all()
            or not np.isfinite(world_from_camera).all() or not np.isfinite(initial).all()):
        raise ValueError("saved camera, depth scale, or transform is invalid")
    points, normals, labels, counts = sample_visual_surfaces(
        urdf, capture["mesh_package_paths"], capture["joint_positions"], samples=samples)
    mesh_base, face_links = load_visual_mesh(
        urdf, capture["mesh_package_paths"], capture["joint_positions"])
    tag_path = bundle / "tags" / "gripper-tag-pose-candidate.json"
    tag_pose = (np.asarray(json.loads(tag_path.read_text())["tag_only_world_from_base_candidate"], float)
                if tag_path.is_file() else initial.copy())
    observed_world, observed_pixels = depth_roi_points(
        points, normals, initial, tag_pose, world_from_camera, depth, depth_scale,
        intrinsics, distortion, min_world_z_m, max_depth_gap_m)
    initial_from_world = np.linalg.inv(initial)
    observed_base = observed_world @ initial_from_world[:3, :3].T + initial_from_world[:3, 3]
    tag_delta_matrix = initial_from_world @ tag_pose
    tag_delta = np.r_[tag_delta_matrix[:3, 3], Rotation.from_matrix(tag_delta_matrix[:3, :3]).as_rotvec()]
    initial_delta = np.zeros(6)
    tag_delta[:3] = np.clip(tag_delta[:3], -max_translation_m, max_translation_m)
    tag_delta[3:] = np.clip(tag_delta[3:], -math.radians(max_rotation_deg), math.radians(max_rotation_deg))
    initial_report, initial_labels, _ = exact_surface_metrics(
        mesh_base, face_links, observed_base, initial_delta)
    tag_report, _, _ = exact_surface_metrics(
        mesh_base, face_links, observed_base, tag_delta, initial_labels)
    seeds = {"initial_pose": initial_delta, "tag_pose_candidate": tag_delta}
    runs = {}
    for seed_name, seed in seeds.items():
        fitted_delta, history = refine_point_to_plane(
            points, normals, observed_base, seed, max_translation_m,
            max_rotation_deg, iterations=iterations)
        report, _, _ = exact_surface_metrics(
            mesh_base, face_links, observed_base, fitted_delta, initial_labels)
        runs[seed_name] = {"delta": fitted_delta, "history": history, "metrics": report}
    selected_seed = min(runs, key=lambda name: (
        runs[name]["metrics"]["p95_surface_residual_m"],
        runs[name]["metrics"]["p95_abs_point_to_plane_m"]))
    fitted_delta = runs[selected_seed]["delta"]
    current = initial @ np.block([
        [Rotation.from_rotvec(fitted_delta[3:]).as_matrix(), fitted_delta[:3, None]],
        [np.zeros((1, 3)), np.ones((1, 1))]])
    after_report = runs[selected_seed]["metrics"]
    tag_prior_report = tag_report
    before_report = initial_report
    steps = runs[selected_seed]["history"]
    runs_summary = {name: {"iterations": item["history"], "metrics": item["metrics"]}
                    for name, item in runs.items()}
    translation, rotation = _transform_delta(current, initial)
    candidate = {
        "schema": 1, "status": "single_pose_candidate_unqualified", "qualified": False,
        "hardware_access": False, "ros_started": False,
        "capture_bundle": str(bundle), "capture_stamp": capture["stamp"],
        "urdf_path": str(urdf), "urdf_sha256": _sha256(urdf),
        "world_from_base_initial": initial.tolist(), "world_from_base_candidate": current.tolist(),
        "world_correction_initial_to_candidate": (current @ np.linalg.inv(initial)).tolist(),
        "correction_translation_m": translation, "correction_rotation_deg": rotation,
        "fixed_joint_positions_rad": capture["joint_positions"],
        "visual_surface_samples": len(points), "visual_samples_per_link": counts,
        "fixed_observed_population": int(len(observed_base)),
        "fit_mode": "visible-CAD z-buffer ROI plus bounded sampled-normal point-to-plane ICP; exact-triangle scoring",
        "selected_seed": selected_seed,
        "tag_candidate_path": str(tag_path) if tag_path.is_file() else None,
        "before": before_report, "tag_prior": tag_prior_report, "after": after_report,
        "same_observed_population_after_fit": after_report,
        "fit_runs": runs_summary,
        "limits": {"depth_proximity_m": max_depth_gap_m, "world_table_z_min_m": min_world_z_m,
                   "translation_from_initial_m": max_translation_m,
                   "rotation_from_initial_deg": max_rotation_deg,
                   "population": "fixed for before, tag prior, both ICP seeds, and after"},
        "interpretation": "Single-pose proposal only; requires independent holdout captures for validation.",
    }
    out = bundle / "fit"
    out.mkdir(parents=True, exist_ok=True)
    (out / "registration-candidate.json").write_text(json.dumps(candidate, indent=2, sort_keys=True) + "\n")
    camera_from_world = np.linalg.inv(world_from_camera)
    candidate_world = points @ current[:3, :3].T + current[:3, 3]
    candidate_camera = candidate_world @ camera_from_world[:3, :3].T + camera_from_world[:3, 3]
    projected = _project(candidate_camera, intrinsics, distortion)
    normals_camera = normals @ (camera_from_world[:3, :3] @ current[:3, :3]).T
    front = ((candidate_camera[:, 2] > 0)
             & (np.einsum("ij,ij->i", normals_camera, -candidate_camera) > 0))
    uv = np.rint(projected).astype(int)
    height, width = depth.shape
    front &= ((uv[:, 0] >= 0) & (uv[:, 0] < width)
              & (uv[:, 1] >= 0) & (uv[:, 1] < height))
    front_ids = np.flatnonzero(front)
    pixel_key = uv[front_ids, 1] * width + uv[front_ids, 0]
    order = np.lexsort((candidate_camera[front_ids, 2], pixel_key))
    sorted_ids, sorted_keys = front_ids[order], pixel_key[order]
    visible = np.r_[True, sorted_keys[1:] != sorted_keys[:-1]]
    projected = projected[sorted_ids[visible]]
    _write_overlay(out / "registration-overlay.png", image, observed_pixels, projected)
    return candidate


def _write_overlay(path, image, observed_pixels, cad_pixels):
    overlay = image.copy()
    for u, v in np.rint(cad_pixels).astype(int):
        if 0 <= u < overlay.shape[1] and 0 <= v < overlay.shape[0]:
            cv2.circle(overlay, (u, v), 1, (0, 0, 255), -1)
    for u, v in observed_pixels.astype(int):
        cv2.circle(overlay, (u, v), 2, (0, 255, 0), -1)
    cv2.putText(overlay, "red=projected CAD; green=depth matches", (12, 24),
                cv2.FONT_HERSHEY_SIMPLEX, .55, (255, 255, 255), 2, cv2.LINE_AA)
    if not cv2.imwrite(str(path), overlay):
        raise OSError(f"failed writing overlay {path}")


def evaluate_fixed_candidate(bundle, source_initial, source_candidate, expected_urdf_sha,
                             *, samples=24000, max_depth_gap_m=.06, min_world_z_m=.02):
    """Score a frozen local correction on a separate capture; never optimize it."""
    bundle = Path(bundle).resolve()
    capture = json.loads((bundle / "capture.json").read_text())
    urdf = bundle / "robot.urdf"
    actual_sha = _sha256(urdf)
    if actual_sha != expected_urdf_sha:
        raise ValueError("holdout URDF digest does not match fitted candidate")
    initial = np.asarray(capture["world_from_base"], float)
    local_delta = np.linalg.inv(source_initial) @ source_candidate
    delta = np.r_[local_delta[:3, 3], Rotation.from_matrix(local_delta[:3, :3]).as_rotvec()]
    candidate_pose = initial @ local_delta
    points, normals, _, _ = sample_visual_surfaces(
        urdf, capture["mesh_package_paths"], capture["joint_positions"], samples=samples)
    depth = np.load(bundle / "depth.npy", allow_pickle=False)
    observed_world, _ = depth_roi_points(
        points, normals, initial, candidate_pose,
        np.asarray(capture["world_from_camera"], float), depth,
        float(capture["depth_scale_m"]),
        np.asarray(capture["camera"]["k"], float).reshape(3, 3),
        np.asarray(capture["camera"].get("d", [0] * 5), float).reshape(-1, 1),
        min_world_z_m, max_depth_gap_m)
    world_from_base_inv = np.linalg.inv(initial)
    observed_base = (observed_world @ world_from_base_inv[:3, :3].T
                     + world_from_base_inv[:3, 3])
    mesh, face_links = load_visual_mesh(
        urdf, capture["mesh_package_paths"], capture["joint_positions"])
    baseline, labels, _ = exact_surface_metrics(
        mesh, face_links, observed_base, np.zeros(6))
    frozen, _, _ = exact_surface_metrics(mesh, face_links, observed_base, delta, labels)
    return {
        "status": "frozen_candidate_holdout_scored_no_refit",
        "capture_bundle": str(bundle), "capture_stamp": capture["stamp"],
        "urdf_sha256": actual_sha, "fixed_observed_population": len(observed_base),
        "before": baseline, "frozen_candidate": frozen,
        "candidate_translation_m": float(np.linalg.norm(delta[:3])),
        "candidate_rotation_deg": float(math.degrees(np.linalg.norm(delta[3:]))),
        "interpretation": "Independent image/depth pose evaluation only; candidate was not refit.",
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bundle", type=Path)
    parser.add_argument("--samples", type=int, default=24000)
    parser.add_argument("--iterations", type=int, default=3)
    parser.add_argument("--holdout", type=Path,
                        help="score this separate capture with the frozen fit; do not refit")
    args = parser.parse_args(argv)
    result = fit_bundle(args.bundle, samples=args.samples, iterations=args.iterations)
    if args.holdout:
        result["holdout"] = evaluate_fixed_candidate(
            args.holdout, np.asarray(result["world_from_base_initial"], float),
            np.asarray(result["world_from_base_candidate"], float), result["urdf_sha256"],
            samples=args.samples, max_depth_gap_m=result["limits"]["depth_proximity_m"],
            min_world_z_m=result["limits"]["world_table_z_min_m"])
        output = Path(args.bundle).resolve() / "fit" / "registration-candidate.json"
        output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(json.dumps({key: result[key] for key in (
        "status", "correction_translation_m", "correction_rotation_deg", "before", "after")},
        indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
