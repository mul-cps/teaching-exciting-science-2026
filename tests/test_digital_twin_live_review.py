"""Independent regressions for live mesh ownership and bounded saved evidence."""
import ast
import inspect
import json
import os
from datetime import datetime, timezone
from pathlib import Path
from types import SimpleNamespace

import numpy as np


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
