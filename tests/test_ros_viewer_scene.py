import numpy as np
from scipy.spatial.transform import Rotation

from workshop.digital_twin.ros_state import transform_matrix
from workshop.digital_twin import ros_viewer


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
