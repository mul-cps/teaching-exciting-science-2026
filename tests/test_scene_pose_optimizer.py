import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from workshop.perception.scene_pose_optimizer import (
    camera_point_to_world,
    optimize_scene_poses,
)


def _transform(xyz=(0., 0., 0.), rpy=(0., 0., 0.)):
    result = np.eye(4)
    result[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
    result[:3, 3] = xyz
    return result


def _pose_observation(track, mount, world_tag, *, sigma_t=.002, sigma_r=.02):
    return {"track_id": track, "entity_id": "fixture", "pose_kind": "pose",
            "entity_from_tag": mount.tolist(), "world_from_tag": world_tag.tolist(),
            "sigma_translation_m": sigma_t, "sigma_rotation_rad": sigma_r}


def test_full_pose_batch_recovers_rigid_scene_entity_and_rejects_outlier():
    truth = _transform((.31, -.17, .48), (.04, -.03, .22))
    problem = {"entities": [{"id": "fixture", "initial_world_from_entity":
                             _transform((.28, -.14, .51), (.07, -.01, .17)).tolist()}],
               "observations": []}
    for i in range(8):
        mount = _transform((.04 * i, .03 * (-1) ** i, .02 * (i % 3)),
                           (.1 * i, -.06 * i, .19 * i))
        measured = truth @ mount
        if i == 7:
            measured = measured.copy()
            measured[:3, 3] += [.11, -.08, .07]
            measured[:3, :3] = measured[:3, :3] @ Rotation.from_euler("z", .8).as_matrix()
        problem["observations"].append(_pose_observation(f"ev3-marker-{i}", mount, measured))

    solution = optimize_scene_poses(problem, loss="soft_l1", f_scale=1.)
    fitted = np.asarray(solution["entities"]["fixture"])
    assert np.linalg.norm(fitted[:3, 3] - truth[:3, 3]) < .004
    assert Rotation.from_matrix(fitted[:3, :3] @ truth[:3, :3].T).magnitude() < .025
    assert solution["status"] == "candidate"
    assert solution["metrics"]["jacobian_rank"] == solution["metrics"]["parameter_count"]


def test_planar_base_recovers_xy_yaw_with_explicit_fixed_height():
    truth = _transform((.23, -.19, .115), (0., 0., .28))
    problem = {"entities": [{"id": "so101_base", "kind": "planar_base",
                             "base_origin_height_m": .115,
                             "initial_world_from_entity": _transform((.2, -.22, .9), (0., 0., .18)).tolist()}],
               "observations": []}
    for i in range(9):
        # Per-sample base-from-tag is the cached robot FK result for that joint pose.
        angle = -.6 + .17 * i
        fk = _transform((.25 * np.cos(angle), .18 * np.sin(1.3 * angle), .31 + .025 * i),
                        (.1 * i, .2 * np.sin(angle), .4 * angle))
        point = (truth @ fk)[:3, 3]
        if i == 8:
            point = point + np.array([.09, -.07, .06])
        problem["observations"].append({"track_id": "tool-tag-physical-A", "entity_id": "so101_base",
                                        "pose_kind": "point_only", "entity_from_tag": fk.tolist(),
                                        "world_point_m": point.tolist(), "sigma_translation_m": .003})

    fitted = np.asarray(optimize_scene_poses(problem)["entities"]["so101_base"])
    assert np.linalg.norm(fitted[:2, 3] - truth[:2, 3]) < .005
    assert abs(fitted[2, 3] - .115) < 1e-10
    yaw_error = Rotation.from_matrix(fitted[:3, :3] @ truth[:3, :3].T).as_rotvec()[2]
    assert abs(yaw_error) < .025


def test_point_only_rigid_estimate_keeps_unobservable_orientation_fixed():
    initial = _transform((.02, -.01, .2), (.15, -.1, .37))
    truth = _transform((.12, .08, .31), (.15, -.1, .37))
    problem = {"entities": [{"id": "table", "initial_world_from_entity": initial.tolist()}],
               "observations": []}
    for i, mount in enumerate((_transform((0, 0, 0)), _transform((.2, 0, 0)),
                               _transform((0, .17, .04)), _transform((.05, .03, .2)))):
        problem["observations"].append({"track_id": f"table-tag-{i}", "entity_id": "table",
                                        "pose_kind": "point_only", "entity_from_tag": mount.tolist(),
                                        "world_point_m": (truth @ mount)[:3, 3].tolist()})
    fitted = np.asarray(optimize_scene_poses(problem)["entities"]["table"])
    assert np.allclose(fitted[:3, 3], truth[:3, 3], atol=1e-8)
    assert np.allclose(fitted[:3, :3], initial[:3, :3], atol=1e-10)


def test_unknown_mount_needs_finite_prior_and_identity_is_track_not_code_zero():
    problem = {"entities": [{"id": "fixture", "initial_world_from_entity": np.eye(4).tolist()}],
               "observations": [{"track_id": "tag-a", "entity_id": "fixture", "pose_kind": "pose",
                                 "entity_from_tag": np.eye(4).tolist(), "world_from_tag": np.eye(4).tolist(),
                                 "optimize_mount": True}]}
    with pytest.raises(ValueError, match="gauge-coupled"):
        optimize_scene_poses(problem)
    problem["observations"][0]["mount_prior_sigma"] = [0, 0, 0, 0, 0, float("inf")]
    with pytest.raises(ValueError, match="mount_prior_sigma"):
        optimize_scene_poses(problem)
    problem["observations"][0]["mount_prior_sigma"] = [.01, .01, .01, .1, .1, .1]
    solution = optimize_scene_poses(problem)
    assert solution["metrics"]["parameter_count"] == 12
    problem["observations"][0]["track_id"] = "0"
    problem["observations"][0].pop("mount_prior_sigma")
    with pytest.raises(ValueError, match="physical association"):
        optimize_scene_poses(problem)


def test_board_plane_camera_transform_maps_depth_point_into_world():
    world_from_camera = _transform((.4, -.2, .7), (0., 0., np.pi / 2))
    got = camera_point_to_world(world_from_camera, [.1, .2, .3])
    assert np.allclose(got, [.2, -.1, 1.])
    with pytest.raises(ValueError, match="finite"):
        camera_point_to_world(world_from_camera, [1, np.nan, 2])


def test_unvaried_base_fk_is_reported_as_unobservable():
    problem = {"entities": [{"id": "base", "kind": "planar_base", "base_origin_height_m": .1,
                             "initial_world_from_entity": _transform((0, 0, .1)).tolist()}],
               "observations": []}
    fk = _transform((.2, 0, .3))
    for i in range(4):
        problem["observations"].append({"track_id": f"fixed-tag-{i}", "entity_id": "base",
                                        "pose_kind": "point_only", "entity_from_tag": fk.tolist(),
                                        "world_point_m": (fk[:3, 3] + [.1, .2, 0]).tolist()})
    with pytest.raises(ValueError, match="unobservable"):
        optimize_scene_poses(problem)


def test_nonconverged_solver_result_is_not_returned_as_candidate(monkeypatch):
    from types import SimpleNamespace
    import scipy.optimize

    problem = {"entities": [{"id": "fixture", "initial_world_from_entity": np.eye(4).tolist()}],
               "observations": []}
    for index, offset in enumerate(([0, 0, 0], [1, 0, 0], [0, 1, 0])):
        mount = _transform(offset)
        problem["observations"].append(_pose_observation(
            f"fixture-tag-{index}", mount, mount))

    monkeypatch.setattr(scipy.optimize, "least_squares", lambda *args, **kwargs: SimpleNamespace(
        x=np.zeros(6), jac=np.eye(6), success=False, nfev=1,
        message="maximum evaluations exceeded"))
    with pytest.raises(ValueError, match="did not converge"):
        optimize_scene_poses(problem)
