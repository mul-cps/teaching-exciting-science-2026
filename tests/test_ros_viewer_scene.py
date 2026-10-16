import numpy as np
from scipy.spatial.transform import Rotation

from workshop.digital_twin.ros_state import transform_matrix
from workshop.digital_twin import ros_viewer
from workshop.digital_twin.placement_region import intersect_ray_with_image_plane


def test_ros_object_pose_is_composed_into_root_frame():
    world_from_camera = transform_matrix((.31, -.22, .13),
        Rotation.from_euler("z", 27, degrees=True).as_quat())
    camera_from_marker = transform_matrix((.08, .02, .27),
        Rotation.from_euler("y", 33, degrees=True).as_quat())
    snapshot = {
        "root_frame": "charuco_board",
        "transforms": {"camera": {"parent": "charuco_board", "child": "camera",
            "matrix": world_from_camera.tolist()}},
        "transform_fresh": {"camera": True},
    }
    pose = ros_viewer._pose_matrix_in_root(snapshot, "camera", camera_from_marker)
    assert pose == pytest.approx(world_from_camera @ camera_from_marker)


def test_object_visuals_follow_marker_kind_and_pose_array_role():
    cube = ros_viewer._scene_object_spec("colored_objects/2", {
        "source": "/so101/object_markers", "kind": 1,
        "scale_m": [.025, .0208, .0047], "color_rgba": [1., .1, .1, .35],
    })
    label = ros_viewer._scene_object_spec("colored_object_labels/2", {
        "source": "/so101/object_markers", "kind": 9, "text": "red-0002",
        "color_rgba": [1., 1., 1., 1.],
    })
    contact = ros_viewer._scene_object_spec("/so101/grasp_contacts/0", {
        "source": "/so101/grasp_contacts",
    })
    candidate = ros_viewer._scene_object_spec("/so101/grasp_candidates/0", {
        "source": "/so101/grasp_candidates",
    })

    assert cube == {"kind": "box", "dimensions": (.025, .0208, .0047),
                    "color": (255, 26, 26), "opacity": .35}
    assert label == {"kind": "label", "text": "red-0002", "font_scene_height": .015}
    assert contact["kind"] == "point" and contact["color"] != candidate["color"]
    assert candidate["kind"] == "point"
    assert ros_viewer._scene_object_spec("unknown", {"source": "other"}) is None


import pytest


class _Handle:
    def __init__(self, **props):
        self.__dict__.update(props)
        self.visible = True
        self.removed = False

    def remove(self):
        self.removed = True


class _Scene:
    def __init__(self):
        self.created = []

    def add_box(self, name, **props):
        handle = _Handle(name=name, **props)
        self.created.append(("box", name, props, handle))
        return handle

    def add_label(self, name, text, **props):
        handle = _Handle(name=name, text=text, **props)
        self.created.append(("label", name, props, handle))
        return handle

    def add_icosphere(self, name, **props):
        handle = _Handle(name=name, **props)
        self.created.append(("point", name, props, handle))
        return handle


def test_scene_upsert_uses_box_dimensions_and_root_composed_pose():
    world_from_camera = transform_matrix((.31, -.22, .13),
        Rotation.from_euler("z", 27, degrees=True).as_quat())
    camera_from_object = transform_matrix((.08, .02, .27),
        Rotation.from_euler("y", 33, degrees=True).as_quat())
    snapshot = {
        "root_frame": "charuco_board",
        "transforms": {"camera": {"parent": "charuco_board", "child": "camera",
            "matrix": world_from_camera.tolist()}},
        "transform_fresh": {"camera": True},
    }
    item = {"source": "/so101/object_markers", "kind": 1,
            "frame_id": "camera", "matrix": camera_from_object.tolist(),
            "scale_m": [.025, .02, .005], "color_rgba": [1., .1, .1, .35]}
    scene, handles = _Scene(), {}
    ros_viewer._upsert_scene_object(scene, handles, "colored_objects/2", item, snapshot)
    assert [created[0] for created in scene.created] == ["box"]
    _, name, props, _ = scene.created[0]
    assert name == "/scene/colored_objects/2"
    assert props["dimensions"] == (.025, .02, .005)
    assert props["position"] == pytest.approx((world_from_camera @ camera_from_object)[:3, 3])
    assert props["opacity"] == pytest.approx(.35)


def test_live_camera_plane_uses_exact_intrinsics_and_optional_acquisition_tf():
    root_from_camera = transform_matrix((.2, -.1, .4),
        Rotation.from_euler("xyz", [12, -8, 31], degrees=True).as_quat())
    frame = {"image_shape": [480, 848], "camera_matrix":
             [[604., 0., 431.36], [0., 604., 247.2], [0., 0., 1.]],
             "root_from_camera": root_from_camera.tolist()}
    spec = ros_viewer._camera_image_plane_spec(frame, .35)
    assert spec["render_width"] == pytest.approx(848 / 604 * .35)
    assert spec["render_height"] == pytest.approx(480 / 604 * .35)
    assert spec["camera_transform_qualified"] is True
    # The scene node follows the exact optical camera axes. Viser's ImageHandle
    # applies its own pi-X mesh orientation; applying another roll flips the
    # image vertically and breaks clicked-pixel correspondence.
    displayed_rotation = Rotation.from_quat(spec["wxyz"][[1, 2, 3, 0]])
    assert displayed_rotation.as_matrix() == pytest.approx(root_from_camera[:3, :3])
    center = spec["position"]
    origin = root_from_camera[:3, 3]
    pixel = intersect_ray_with_image_plane(origin, center - origin,
        root_from_camera, frame["camera_matrix"], frame["image_shape"], distance_m=.35)
    assert pixel == pytest.approx((424., 240.))
    for image_pixel in ((0., 0.), (847., 479.)):
        local = np.array([((image_pixel[0] - 431.36) / 604.) * .35,
                          ((image_pixel[1] - 247.2) / 604.) * .35, .35])
        camera_ray = root_from_camera[:3, :3] @ local
        corner = intersect_ray_with_image_plane(origin, camera_ray,
            root_from_camera, frame["camera_matrix"], frame["image_shape"], distance_m=.35)
        assert corner == pytest.approx(image_pixel)

    frame["root_from_camera"] = None
    unqualified = ros_viewer._camera_image_plane_spec(frame)
    assert unqualified["camera_transform_qualified"] is False


@pytest.mark.parametrize("clicks,shape,expected", [
    ([(10.2, 20.9), (30.1, 50.1)], (100, 200), [10, 20, 31, 51]),
    ([(-4., 2.), (5., 12.)], (40, 40), [0, 2, 5, 12]),
])
def test_two_camera_image_clicks_make_bounded_half_open_rectangle(clicks, shape, expected):
    assert ros_viewer._placement_rect_from_pixels(clicks, shape) == expected


def test_camera_area_selection_rejects_tiny_or_nonfinite_rectangles():
    with pytest.raises(ValueError, match="at least"):
        ros_viewer._placement_rect_from_pixels([(12., 12.), (14., 14.)], (80, 100))
    with pytest.raises(ValueError, match="finite"):
        ros_viewer._placement_rect_from_pixels([(0., 0.), (np.nan, 4.)], (80, 100))


def test_tracked_goal_line_colors_match_viser_segment_endpoint_shape():
    colors = ros_viewer._line_segment_colors((40, 235, 135), 4)
    assert colors.shape == (4, 2, 3)
    assert np.all(colors == np.array([40, 235, 135], dtype=np.uint8))


def test_tracked_goal_outline_is_projected_in_front_of_camera_image_plane():
    root_from_camera = transform_matrix((.3, -.2, .4),
        Rotation.from_euler("xyz", [12, -8, 31], degrees=True).as_quat())
    intrinsics = np.array([[604., 0., 327.36], [0., 604., 247.2], [0., 0., 1.]])
    points_camera = np.array([[-.04, -.03, .55], [.04, -.03, .62],
                              [.04, .03, .60], [-.04, .03, .53]])
    points_root = points_camera @ root_from_camera[:3, :3].T + root_from_camera[:3, 3]
    frame = {"root_from_camera": root_from_camera.tolist(),
             "camera_matrix": intrinsics.tolist()}

    projected = ros_viewer._root_polygon_on_camera_plane(points_root, frame, .348)

    projected_camera = (projected - root_from_camera[:3, 3]) @ root_from_camera[:3, :3]
    original_pixels = np.column_stack((
        intrinsics[0, 0] * points_camera[:, 0] / points_camera[:, 2] + intrinsics[0, 2],
        intrinsics[1, 1] * points_camera[:, 1] / points_camera[:, 2] + intrinsics[1, 2]))
    projected_pixels = np.column_stack((
        intrinsics[0, 0] * projected_camera[:, 0] / projected_camera[:, 2] + intrinsics[0, 2],
        intrinsics[1, 1] * projected_camera[:, 1] / projected_camera[:, 2] + intrinsics[1, 2]))
    assert projected_camera[:, 2] == pytest.approx(np.full(4, .348))
    assert projected_pixels == pytest.approx(original_pixels)
    assert ros_viewer._root_polygon_on_camera_plane(points_root, {
        "root_from_camera": None, "camera_matrix": intrinsics.tolist()}, .348) is None


def test_placement_request_is_atomic_and_carries_exact_frozen_generation(tmp_path):
    state = tmp_path / "ros-twin-state.json"
    request = {"request_id": "request-1", "action": "select_feeder_anchor",
               "generation": "camera-stamp-17", "rect_xyxy": [20, 30, 100, 140]}
    ros_viewer._write_placement_selection_request(state, request)
    path = tmp_path / "placement-region-request.json"
    assert __import__("json").loads(path.read_text()) == request
    assert list(tmp_path.glob(".placement-request-*")) == []
