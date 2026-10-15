"""Independent regressions for live mesh ownership and bounded saved evidence."""
import ast
import inspect
import json
import os
import threading
from datetime import datetime, timezone
from pathlib import Path
from types import SimpleNamespace

import numpy as np


def test_actual_pose_refresh_suppresses_synchronous_slider_callback_without_rlock():
    from types import SimpleNamespace
    from workshop.digital_twin.viewer import _reflect_actual_pose

    lock = threading.Lock()
    virtual_override = [False]
    reflecting_actual = [False]
    degrees = {'shoulder_pan': 0.}
    callbacks = []

    class Slider:
        def __init__(self, value):
            self._value = value
            self.callback = None

        @property
        def value(self):
            return self._value

        @value.setter
        def value(self, value):
            self._value = value
            if self.callback:
                self.callback(SimpleNamespace(target=self))

        def on_update(self, callback):
            self.callback = callback

    slider = Slider(0.)

    def on_change(event):
        if reflecting_actual[0]:
            return
        with lock:
            virtual_override[0] = True
            callbacks.append(event.target.value)

    slider.on_update(on_change)
    finished = threading.Event()

    def refresh():
        _reflect_actual_pose(lock, {'shoulder_pan': slider}, degrees,
            {'shoulder_pan': 12.5}, lambda: None, reflecting_actual)
        finished.set()

    worker = threading.Thread(target=refresh, daemon=True)
    worker.start()
    assert finished.wait(.5), 'refresh deadlocked on synchronous slider callback'
    worker.join(.1)
    assert not worker.is_alive()
    assert degrees['shoulder_pan'] == 12.5
    assert slider.value == 12.5
    assert virtual_override[0] is False
    assert callbacks == []

    # A real user change after the reflection still enters the normal callback.
    slider.value = 15.
    assert virtual_override[0] is True
    assert callbacks == [15.]


def test_orientation_uncertain_overlay_keeps_expected_axes_but_hides_observed_axes():
    from workshop.digital_twin.viewer import _set_fiducial_axes

    class Axes:
        visible = True
        position = None
        wxyz = None

    expected_axes, observed_axes = Axes(), Axes()
    expected = np.eye(4)
    observed = np.eye(4)
    observed[:3, :3] = np.diag([1., -1., -1.])
    observed[:3, 3] = [.1, .2, .3]
    result = {'orientation_status': 'orientation_uncertain',
              'expected_camera_from_tag': expected,
              'observed_camera_from_tag': observed,
              'depth_anchored_camera_from_tag': observed}

    _set_fiducial_axes(result, np.eye(4), expected_axes, observed_axes)

    assert expected_axes.visible is True
    assert observed_axes.visible is False
    np.testing.assert_allclose(expected_axes.position, expected[:3, 3])


def test_resolved_overlay_updates_both_fiducial_axes():
    from workshop.digital_twin.viewer import _set_fiducial_axes

    expected_axes = SimpleNamespace(visible=False, position=None, wxyz=None)
    observed_axes = SimpleNamespace(visible=False, position=None, wxyz=None)
    expected = np.eye(4)
    expected[:3, 3] = [.1, 0., .5]
    observed = np.eye(4)
    observed[:3, 3] = [.2, 0., .5]
    result = {'orientation_status': 'depth_disambiguated',
              'expected_camera_from_tag': expected,
              'observed_camera_from_tag': observed,
              'depth_anchored_camera_from_tag': observed}

    _set_fiducial_axes(result, np.eye(4), expected_axes, observed_axes)

    assert expected_axes.visible is True
    assert observed_axes.visible is True
    np.testing.assert_allclose(observed_axes.position, observed[:3, 3])


def test_surface_mutation_uses_the_same_lock_as_render_callbacks():
    from workshop.digital_twin import viewer
    tree = ast.parse(inspect.getsource(viewer.serve))
    refresh = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)
                   and n.name == 'refresh_evidence')
    locked_calls = set()
    for node in ast.walk(refresh):
        if isinstance(node, ast.With) and any(
                isinstance(item.context_expr, ast.Name) and item.context_expr.id == 'lock'
                for item in node.items):
            locked_calls.update(n.func.id for n in ast.walk(node)
                                if isinstance(n, ast.Call) and isinstance(n.func, ast.Name))
    assert '_update_compiled_surface' in locked_calls


def test_surface_face_refresh_updates_normal_and_uv_indices(monkeypatch):
    from workshop.digital_twin import viewer
    model = SimpleNamespace(
        geom_dataid=np.array([0]), mesh_vertadr=np.array([0]), mesh_vertnum=np.array([4]),
        mesh_normaladr=np.array([0]), mesh_normalnum=np.array([4]),
        mesh_faceadr=np.array([0]), mesh_facenum=np.array([4]),
        mesh_quat=np.array([[1., 0., 0., 0.]]), mesh_pos=np.zeros((1, 3)),
        mesh_vert=np.zeros((4, 3), np.float32), mesh_normal=np.zeros((4, 3), np.float32),
        mesh_face=np.zeros((4, 3), np.int32), mesh_facenormal=np.zeros((4, 3), np.int32),
        mesh_facetexcoord=np.zeros((4, 3), np.int32),
        mesh_texcoordadr=np.array([0]), mesh_texcoordnum=np.array([4]),
        tex_nchannel=np.array([3]), tex_width=np.array([2]), tex_height=np.array([2]),
        tex_adr=np.array([0]), tex_data=np.zeros(12, np.uint8),
        geom_aabb=np.zeros((1, 6)), geom_rbound=np.zeros(1))
    monkeypatch.setattr(viewer.mujoco, 'mj_name2id', lambda *args: 0)
    surface = {'vertices_scene': np.array([[0,0,0],[1,0,0],[0,1,0],[0,0,1.]]),
               'faces': np.array([[0,1,2],[0,2,3]], np.int32),
               'used_vertices': np.array([0,1,2,3]), 'rgb': np.full((2,2,3), 21, np.uint8)}
    viewer._update_compiled_surface(model, 0, surface)
    np.testing.assert_array_equal(model.mesh_facenormal[:2], surface['faces'])
    np.testing.assert_array_equal(model.mesh_facetexcoord[:2], surface['faces'])


def test_negative_camera_to_arm_delta_is_valid_within_pairing_window(tmp_path):
    from workshop.digital_twin.evidence_bridge import select_live_rgbd_pair
    image, depth = tmp_path / 'latest.png', tmp_path / 'latest-depth.npy'
    image.write_bytes(b'rgb'); depth.write_bytes(b'depth')
    wall = 999.8
    record = {'monotonic': 199.8, 'utc': datetime.fromtimestamp(wall, timezone.utc).isoformat(),
              'paths': {'latest_color': image.name, 'latest_depth': depth.name},
              'arm': {'monotonic': 199.81, 'delta_s': -.01, 'qualified_sync': True,
                      'raw': dict.fromkeys(('shoulder_pan','shoulder_lift','elbow_flex',
                                            'wrist_flex','wrist_roll','gripper'), 2000)}}
    (tmp_path / 'latest.json').write_text(json.dumps(record))
    os.utime(image, (wall, wall)); os.utime(depth, (wall, wall))
    assert select_live_rgbd_pair(tmp_path, now_wall=1000., now_monotonic=200.) is not None


def test_invalid_live_metadata_never_scans_historical_frames(tmp_path, monkeypatch):
    from workshop.digital_twin.evidence_bridge import select_latest_rgbd_pair
    (tmp_path / 'latest.png').write_bytes(b'rgb')
    (tmp_path / 'latest-depth.npy').write_bytes(b'depth')
    (tmp_path / 'latest.json').write_text(json.dumps({'arm': {'qualified_sync': False}}))
    (tmp_path / 'initial-align-retry' / 'frames').mkdir(parents=True)
    def forbidden_scan(*args, **kwargs):
        raise AssertionError('invalid live metadata must fail closed without a historical scan')
    monkeypatch.setattr(Path, 'glob', forbidden_scan)
    assert select_latest_rgbd_pair(tmp_path, now_wall=1000., now_monotonic=200.) is None
