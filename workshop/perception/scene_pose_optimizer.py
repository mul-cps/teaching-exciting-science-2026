"""Small offline optimizer for board-anchored station scene poses.

Input observations are already normalized.  A point observation contains a
3-D tag centre in the ChArUco board frame; a pose observation contains the
full board-from-tag transform.  ``entity_from_tag`` is a known mount or the
per-sample robot FK transform to that tag.  No detector, camera, ROS, or GPU
code is used here.
"""
from __future__ import annotations

from typing import Any

import numpy as np


def camera_point_to_world(world_from_camera: Any, camera_point_m: Any) -> np.ndarray:
    """Transform one camera optical XYZ point into the board/world frame."""
    transform = _transform(world_from_camera, "world_from_camera")
    point = np.asarray(camera_point_m, dtype=float)
    if point.shape != (3,) or not np.isfinite(point).all():
        raise ValueError("camera_point_m must contain three finite metres")
    return transform[:3, :3] @ point + transform[:3, 3]


def optimize_scene_poses(problem: dict[str, Any], *, max_nfev: int = 100,
                         loss: str = "soft_l1", f_scale: float = 1.0,
                         initial_solution: dict[str, Any] | None = None) -> dict[str, Any]:
    """Fit candidate entity poses from cached observations.

    Entity poses are initialized by ``initial_world_from_entity``.  Rigid
    entities optimize all six pose components when at least one full pose is
    observed; point-only data optimize translation while retaining the stated
    orientation. ``planar_base`` entities optimize x, y, and yaw only, with z
    fixed to their explicit ``base_origin_height_m``. Each observation carries
    a persistent ``track_id`` (physical association, never the printed code
    number), ``entity_id``, ``entity_from_tag``, and either ``world_point_m``
    or ``world_from_tag``. Optional ``mount_prior_sigma`` adds a finite,
    zero-centred Gaussian prior for a shared per-track mount correction in
    metres/radians. An unbounded or absent prior never creates free mount
    variables, preventing a tool-roll/base-yaw gauge from being silently fit.

    The result is a candidate only: this pure function never publishes TF or
    mutates a running scene.
    """
    try:
        from scipy.optimize import least_squares
        from scipy.spatial.transform import Rotation
    except ImportError as exc:  # scipy remains an optional project extra
        raise RuntimeError("scene pose optimization requires scipy") from exc
    if max_nfev < 1 or f_scale <= 0 or loss not in {"linear", "soft_l1", "huber", "cauchy", "arctan"}:
        raise ValueError("invalid solver settings")
    entities = _entities(problem)
    if initial_solution is not None:
        previous = initial_solution.get("entities", {})
        for entity_id, pose in previous.items():
            if entity_id in entities:
                entities[entity_id]["initial_world_from_entity"] = _transform(
                    pose, f"initial_solution.entities.{entity_id}")
    observations = _observations(problem, entities)
    if not observations:
        raise ValueError("at least one observation is required")

    # Establish free pose variables only when the measurements can inform them.
    pose_slices: dict[str, tuple[int, int, str]] = {}
    mount_slices: dict[tuple[str, str], tuple[int, int, np.ndarray]] = {}
    x0: list[float] = []
    mount_sigmas: dict[tuple[str, str], np.ndarray] = {}
    for observation in observations:
        key = (observation["entity_id"], observation["track_id"])
        if "mount_prior_sigma" in observation:
            sigma = _sigma(observation["mount_prior_sigma"], "mount_prior_sigma")
            old = mount_sigmas.get(key)
            if old is not None and not np.allclose(old, sigma, atol=0, rtol=0):
                raise ValueError(f"inconsistent mount_prior_sigma for {key}")
            mount_sigmas[key] = sigma
    mount_dims: dict[tuple[str, str], np.ndarray] = {}
    for key, sigma in mount_sigmas.items():
        dimensions = np.flatnonzero(sigma > 0)
        if len(dimensions):
            start = len(x0)
            x0.extend([0.0] * len(dimensions))
            mount_slices[key] = (start, len(x0), dimensions)
            mount_dims[key] = sigma

    for entity_id, entity in entities.items():
        kind = entity["kind"]
        kinds = {o["pose_kind"] for o in observations if o["entity_id"] == entity_id}
        if not kinds:
            continue
        if kind == "planar_base":
            start = len(x0)
            x0.extend([0.0, 0.0, 0.0])  # dx, dy, dyaw about supplied initial pose
            pose_slices[entity_id] = (start, len(x0), "planar")
        else:
            full_pose = "pose" in kinds
            dims = 6 if full_pose else 3
            start = len(x0)
            x0.extend([0.0] * dims)  # world translation delta, then left rotvec
            pose_slices[entity_id] = (start, len(x0), "full" if full_pose else "point")
    if not x0:
        raise ValueError("observations contain no optimizable entity")

    def corrected_mount(observation: dict[str, Any], x: np.ndarray) -> np.ndarray:
        transform = observation["entity_from_tag"]
        key = (observation["entity_id"], observation["track_id"])
        if key in mount_slices:
            lo, hi, dimensions = mount_slices[key]
            delta = np.zeros(6)
            delta[dimensions] = x[lo:hi]
            adjustment = np.eye(4)
            adjustment[:3, :3] = Rotation.from_rotvec(delta[3:]).as_matrix()
            adjustment[:3, 3] = delta[:3]
            transform = transform @ adjustment
        return transform

    def world_from_entity(entity_id: str, x: np.ndarray) -> np.ndarray:
        entity = entities[entity_id]
        initial = entity["initial_world_from_entity"]
        lo, hi, mode = pose_slices[entity_id]
        values = x[lo:hi]
        result = initial.copy()
        if mode == "planar":
            result[:3, 3] += [values[0], values[1], 0.0]
            result[2, 3] = entity["base_origin_height_m"]
            result[:3, :3] = Rotation.from_rotvec([0., 0., values[2]]).as_matrix() @ initial[:3, :3]
        elif mode == "full":
            result[:3, 3] += values[:3]
            result[:3, :3] = Rotation.from_rotvec(values[3:]).as_matrix() @ initial[:3, :3]
        else:
            result[:3, 3] += values[:3]
        return result

    def residuals(x: np.ndarray) -> np.ndarray:
        values: list[float] = []
        for observation in observations:
            entity_id = observation["entity_id"]
            predicted = world_from_entity(entity_id, x) @ corrected_mount(observation, x)
            sigma_t = observation["sigma_translation_m"]
            if observation["pose_kind"] == "point_only":
                delta = predicted[:3, 3] - observation["world_point_m"]
                values.extend((delta / sigma_t).tolist())
            else:
                measured = observation["world_from_tag"]
                delta = predicted[:3, 3] - measured[:3, 3]
                rotation_delta = Rotation.from_matrix(measured[:3, :3] @ predicted[:3, :3].T).as_rotvec()
                values.extend((delta / sigma_t).tolist())
                values.extend((rotation_delta / observation["sigma_rotation_rad"]).tolist())
        for key, (lo, hi, dimensions) in mount_slices.items():
            values.extend((x[lo:hi] / mount_dims[key][dimensions]).tolist())
        return np.asarray(values, dtype=float)

    result = least_squares(residuals, np.asarray(x0), loss=loss, f_scale=f_scale,
                           max_nfev=max_nfev, method="trf", x_scale="jac")
    if not result.success:
        raise ValueError(f"scene pose optimization did not converge: {result.message}")
    jac = np.asarray(result.jac, dtype=float)
    singular_values = np.linalg.svd(jac, compute_uv=False)
    rank_tolerance = (singular_values[0] if len(singular_values) else 0.0) * 1e-10
    rank = int(np.count_nonzero(singular_values > rank_tolerance))
    if rank < len(x0):
        raise ValueError(f"scene pose problem is unobservable: Jacobian rank {rank}/{len(x0)}")
    fitted = {entity_id: world_from_entity(entity_id, result.x).tolist()
              for entity_id in pose_slices}
    raw = residuals(result.x)
    data_count = sum(3 if o["pose_kind"] == "point_only" else 6 for o in observations)
    data_residuals = raw[:data_count]
    per_obs = []
    offset = 0
    for observation in observations:
        count = 3 if observation["pose_kind"] == "point_only" else 6
        per_obs.append(float(np.linalg.norm(data_residuals[offset:offset + count])))
        offset += count
    return {
        "status": "candidate",
        "world_frame": str(problem.get("world_frame", "charuco_board")),
        "entities": fitted,
        "mount_corrections": {f"{key[0]}::{key[1]}": _mount_result(result.x, mount_slices.get(key), mount_dims.get(key))
                              for key in mount_dims},
        "metrics": {
            "observation_count": len(observations),
            "track_count": len({o["track_id"] for o in observations}),
            "weighted_rms": float(np.sqrt(np.mean(data_residuals ** 2))),
            "median_observation_norm": float(np.median(per_obs)),
            "jacobian_rank": rank,
            "parameter_count": len(x0),
            "evaluations": int(result.nfev),
            "converged": bool(result.success),
            "message": str(result.message),
        },
    }


def _mount_result(x: np.ndarray, section: Any, sigma: Any) -> dict[str, list[float]]:
    delta = np.zeros(6)
    if section is not None:
        start, end, dimensions = section
        delta[dimensions] = x[start:end]
    return {"translation_m": delta[:3].tolist(), "rotation_rotvec_rad": delta[3:].tolist(),
            "prior_sigma_m_rad": None if sigma is None else sigma.tolist()}


def _transform(value: Any, name: str) -> np.ndarray:
    matrix = np.asarray(value, dtype=float)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError(f"{name} must be a finite 4x4 transform")
    if not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-8):
        raise ValueError(f"{name} must have homogeneous bottom row [0,0,0,1]")
    if not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-4) or np.linalg.det(matrix[:3, :3]) < .999:
        raise ValueError(f"{name} rotation must be right-handed orthonormal")
    return matrix.copy()


def _sigma(value: Any, name: str) -> np.ndarray:
    result = np.asarray(value, dtype=float)
    if result.shape == ():
        result = np.repeat(result, 6)
    if result.shape != (6,) or not np.isfinite(result).all() or np.any(result < 0):
        raise ValueError(f"{name} must contain six finite nonnegative values")
    return result


def _entities(problem: dict[str, Any]) -> dict[str, dict[str, Any]]:
    if not isinstance(problem, dict) or not isinstance(problem.get("entities"), list):
        raise ValueError("problem.entities must be a list")
    entities: dict[str, dict[str, Any]] = {}
    for item in problem["entities"]:
        if not isinstance(item, dict) or not item.get("id"):
            raise ValueError("each entity needs a nonempty id")
        entity_id = str(item["id"])
        if entity_id in entities:
            raise ValueError(f"duplicate entity id {entity_id}")
        kind = item.get("kind", "rigid")
        if kind not in {"rigid", "planar_base"}:
            raise ValueError(f"unsupported entity kind {kind}")
        entity = dict(item)
        entity["id"] = entity_id
        entity["kind"] = kind
        entity["initial_world_from_entity"] = _transform(item.get("initial_world_from_entity"),
                                                          f"{entity_id}.initial_world_from_entity")
        if kind == "planar_base":
            height = item.get("base_origin_height_m")
            if height is None or not np.isfinite(height):
                raise ValueError(f"planar base {entity_id} needs explicit base_origin_height_m")
            entity["base_origin_height_m"] = float(height)
        entities[entity_id] = entity
    return entities


def _observations(problem: dict[str, Any], entities: dict[str, dict[str, Any]]) -> list[dict[str, Any]]:
    if not isinstance(problem.get("observations"), list):
        raise ValueError("problem.observations must be a list")
    result = []
    for index, item in enumerate(problem["observations"]):
        if not isinstance(item, dict):
            raise ValueError(f"observation {index} must be an object")
        track_id = item.get("track_id")
        if not isinstance(track_id, str) or not track_id.strip():
            raise ValueError(f"observation {index} needs persistent physical track_id")
        entity_id = str(item.get("entity_id", ""))
        if entity_id not in entities:
            raise ValueError(f"observation {index} references unknown entity {entity_id}")
        # Explicitly reject the known failure mode where code ID zero is reused
        # as physical identity for every printed tag.
        if track_id.strip() == "0":
            raise ValueError("track_id '0' is ambiguous; use persistent physical association, not code_id")
        pose_kind = item.get("pose_kind", "pose" if "world_from_tag" in item else "point_only")
        if pose_kind not in {"pose", "point_only"}:
            raise ValueError(f"unsupported pose_kind {pose_kind}")
        if pose_kind == "pose":
            world_from_tag = _transform(item.get("world_from_tag"), f"observation {index}.world_from_tag")
        else:
            world_point = np.asarray(item.get("world_point_m"), dtype=float)
            if world_point.shape != (3,) or not np.isfinite(world_point).all():
                raise ValueError(f"observation {index}.world_point_m must contain finite XYZ metres")
            world_from_tag = None
        if "entity_from_tag" not in item:
            raise ValueError(f"observation {index} needs known entity_from_tag (static mount or FK pose)")
        sigma_t = float(item.get("sigma_translation_m", .01))
        sigma_r = float(item.get("sigma_rotation_rad", .1))
        if not np.isfinite(sigma_t) or sigma_t <= 0 or not np.isfinite(sigma_r) or sigma_r <= 0:
            raise ValueError(f"observation {index} sigmas must be positive and finite")
        observation = {"track_id": track_id.strip(), "entity_id": entity_id,
                       "entity_from_tag": _transform(item["entity_from_tag"], f"observation {index}.entity_from_tag"),
                       "pose_kind": pose_kind, "world_from_tag": world_from_tag,
                       "world_point_m": world_point if pose_kind == "point_only" else None,
                       "sigma_translation_m": sigma_t, "sigma_rotation_rad": sigma_r}
        if "mount_prior_sigma" in item:
            observation["mount_prior_sigma"] = _sigma(item["mount_prior_sigma"], "mount_prior_sigma")
        elif item.get("optimize_mount", False):
            raise ValueError("optimize_mount requires finite mount_prior_sigma; free tag mounts are gauge-coupled")
        result.append(observation)
    return result
