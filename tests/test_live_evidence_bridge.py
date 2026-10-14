import json
import os
import time
from datetime import datetime, timezone
from pathlib import Path

import pytest


def test_resident_state_schema_normalizes_raw_pose_and_applies_half_second_limit(tmp_path):
    from workshop.digital_twin.evidence_bridge import read_phase_evidence

    state = tmp_path / 'arm-resident-state.json'
    state.write_text(json.dumps({
        'schema': 1, 'event': 'sample',
        'timestamp_utc': '2026-10-14T16:10:14.842031+00:00',
        'monotonic': 1818397.552279619,
        'raw': {'shoulder_pan': 2693, 'gripper': 2167},
    }))

    fresh = read_phase_evidence(state, now_monotonic=1818398.052279619)
    stale = read_phase_evidence(state, now_monotonic=1818398.052279620)

    assert fresh['freshness'] == 'fresh'
    assert fresh['pose_raw'] == {'shoulder_pan': 2693, 'gripper': 2167}
    assert fresh['timestamp_utc'] == '2026-10-14T16:10:14.842031+00:00'
    assert stale['freshness'] == 'stale'
    assert stale['pose_raw'] == fresh['pose_raw']


def test_live_camera_pair_requires_fresh_matching_timestamp_metadata(tmp_path):
    from datetime import datetime, timezone

    from workshop.digital_twin.evidence_bridge import select_latest_rgbd_pair

    image = tmp_path / 'latest.png'
    depth = tmp_path / 'latest-depth.npy'
    image.write_bytes(b'rgb')
    depth.write_bytes(b'depth')
    capture_wall = 999.8
    capture_utc = datetime.fromtimestamp(capture_wall, timezone.utc).isoformat()
    (tmp_path / 'timestamps.jsonl').write_text(json.dumps({
        'index': 12, 'monotonic': 199.8, 'utc': capture_utc,
    }) + '\n')
    os.utime(image, (capture_wall, capture_wall))
    os.utime(depth, (capture_wall, capture_wall))

    pair = select_latest_rgbd_pair(tmp_path, now_wall=1000., now_monotonic=200.)
    assert pair['image'] == image
    assert pair['depth'] == depth
    assert pair['timestamp_monotonic'] == pytest.approx(199.8)
    assert pair['camera_frame_index'] == 12
    assert pair['freshness'] == 'fresh'
    assert select_latest_rgbd_pair(tmp_path, now_wall=1000.301,
                                   now_monotonic=200.301) is None


def test_live_camera_pair_rejects_rgb_depth_mtime_mismatch(tmp_path):
    from workshop.digital_twin.evidence_bridge import select_live_rgbd_pair

    image = tmp_path / 'latest.png'
    depth = tmp_path / 'latest-depth.npy'
    image.write_bytes(b'rgb')
    depth.write_bytes(b'depth')
    capture_wall = 999.8
    (tmp_path / 'timestamps.jsonl').write_text(json.dumps({
        'index': 12, 'monotonic': 199.8,
        'utc': datetime.fromtimestamp(capture_wall, timezone.utc).isoformat(),
    }) + '\n')
    os.utime(image, (capture_wall, capture_wall))
    os.utime(depth, (capture_wall + 0.6, capture_wall + 0.6))

    assert select_live_rgbd_pair(tmp_path, now_wall=1000., now_monotonic=200.) is None


def test_live_camera_pair_requires_qualified_camera_arm_sample(tmp_path):
    from workshop.digital_twin.evidence_bridge import select_live_rgbd_pair

    image, depth = tmp_path / 'latest.png', tmp_path / 'latest-depth.npy'
    image.write_bytes(b'rgb')
    depth.write_bytes(b'depth')
    capture_wall = 999.8
    record = {
        'index': 12, 'monotonic': 199.8,
        'utc': datetime.fromtimestamp(capture_wall, timezone.utc).isoformat(),
        'paths': {'latest_color': image.name, 'latest_depth': depth.name},
        'arm': {'monotonic': 199.7, 'delta_s': .1, 'qualified_sync': True,
                'raw': {'shoulder_pan': 2000}},
    }
    latest = tmp_path / 'latest.json'
    latest.write_text(json.dumps(record))
    os.utime(image, (capture_wall, capture_wall))
    os.utime(depth, (capture_wall, capture_wall))

    pair = select_live_rgbd_pair(tmp_path, now_wall=1000., now_monotonic=200.)
    assert pair['pose_qualified_sync'] is True
    assert pair['pose_raw'] == {'shoulder_pan': 2000}
    assert pair['pose_sample_delta_seconds'] == pytest.approx(.1)

    record['arm']['qualified_sync'] = False
    latest.write_text(json.dumps(record))
    assert select_live_rgbd_pair(tmp_path, now_wall=1000., now_monotonic=200.) is None


@pytest.mark.parametrize(('phase_freshness', 'current_raw', 'expected_pairing'), [
    ('fresh', {'shoulder_pan': 2000}, 'paired'),
    ('stale', {'shoulder_pan': 2000}, 'unpaired'),
    ('fresh', {'shoulder_pan': 2001}, 'unpaired'),
])
def test_live_camera_status_requires_fresh_matching_resident_pose(
        phase_freshness, current_raw, expected_pairing):
    from workshop.digital_twin.evidence_bridge import camera_sample_status

    report = {
        'camera_sample_timestamp_wall': 999.95,
        'camera_sample_timestamp_monotonic': 199.95,
        'camera_sample_timestamp_source': 'camera timestamps.jsonl',
        'camera_sample_pose_raw': {'shoulder_pan': 2000},
        'camera_sample_pose_monotonic': 199.9,
        'camera_sample_pose_delta_seconds': .05,
        'camera_sample_pose_qualified_sync': True,
    }
    phase = {'freshness': phase_freshness, 'event_monotonic': 200.1,
             'pose_raw': current_raw}
    status = camera_sample_status(report, phase, now_wall=1000., now_monotonic=200.)
    assert status['pairing_status'] == expected_pairing
    assert (status['freshness'] == 'fresh') == (expected_pairing == 'paired')
    if expected_pairing == 'unpaired':
        assert 'unpaired' in status['reason']


def test_jsonl_tail_reader_reads_at_most_8k_from_large_file(tmp_path, monkeypatch):
    from io import BytesIO

    from workshop.digital_twin import evidence_bridge

    path = tmp_path / 'timestamps.jsonl'
    payload = b'{"padding":"' + b'x' * (1024 * 1024) + b'"}\n'
    record = {'index': 7, 'monotonic': 12.5,
              'utc': '2026-10-14T16:00:00+00:00'}
    path.write_bytes(payload + json.dumps(record).encode() + b'\n')
    original_open = Path.open
    reads = []

    class TrackedStream(BytesIO):
        def read(self, size=-1):
            result = super().read(size)
            reads.append(len(result))
            return result

    def tracked_open(self, *args, **kwargs):
        if self == path:
            with original_open(path, 'rb') as source:
                return TrackedStream(source.read())
        return original_open(self, *args, **kwargs)

    monkeypatch.setattr(Path, 'open', tracked_open)
    assert evidence_bridge._last_jsonl_record(path) == record
    assert sum(reads) <= 8192


def test_warmed_live_camera_selector_benchmark_stays_bounded(tmp_path):
    from workshop.digital_twin.evidence_bridge import select_live_rgbd_pair

    image, depth = tmp_path / 'latest.png', tmp_path / 'latest-depth.npy'
    image.write_bytes(b'rgb')
    depth.write_bytes(b'depth')
    capture_wall, capture_monotonic = 999.95, 199.95
    timestamps = tmp_path / 'timestamps.jsonl'
    timestamps.write_bytes(b'{"padding":"' + b'x' * (1024 * 1024) + b'"}\n'
        + json.dumps({'index': 1, 'monotonic': capture_monotonic,
                      'utc': datetime.fromtimestamp(capture_wall, timezone.utc).isoformat()}).encode() + b'\n')
    os.utime(image, (capture_wall, capture_wall))
    os.utime(depth, (capture_wall, capture_wall))

    for _ in range(10):
        assert select_live_rgbd_pair(tmp_path, now_wall=1000., now_monotonic=200.)
    samples = []
    for _ in range(100):
        start = time.perf_counter_ns()
        assert select_live_rgbd_pair(tmp_path, now_wall=1000., now_monotonic=200.)
        samples.append((time.perf_counter_ns() - start) / 1e6)
    samples.sort()
    p95 = samples[int(.95 * (len(samples) - 1))]
    assert p95 < 20., f'warmed live selector p95 was {p95:.3f} ms'


def test_live_surface_frame_uses_full_registration_and_compacts_bounds(tmp_path, monkeypatch):
    from scipy.spatial.transform import Rotation
    from workshop.digital_twin import registration, viewer

    image, depth = tmp_path / 'rgb.png', tmp_path / 'depth.npy'
    from PIL import Image
    import numpy as np
    Image.fromarray(np.full((480, 480, 3), [40, 120, 190], dtype=np.uint8)).save(image)
    np.save(depth, np.full((480, 480), 400., dtype=np.float32))
    monkeypatch.setattr(registration, 'white_arm_mask', lambda rgb, points, valid: np.zeros(valid.shape, bool))
    scene_from_camera = np.eye(4)
    scene_from_camera[:3, 3] = [.1, -.05, 0.]
    world_from_scene = np.eye(4)
    world_from_scene[:3, :3] = Rotation.from_euler('z', 90, degrees=True).as_matrix()
    world_from_scene[:3, 3] = [1., 2., -.3]
    report = {'camera': {'intrinsics': {'fx': 100., 'fy': 100., 'ppx': 12., 'ppy': 12., 'coeffs': []},
                         'depth_scale_m': .001, 'timestamp': 1.},
              'scene_world_from_camera': scene_from_camera.tolist(),
              'world_from_camera': scene_from_camera.tolist()}
    _, surface = viewer._rgbd_frame(image, depth, report, world_from_scene)
    expected = surface['vertices_scene'][surface['used_vertices']] @ world_from_scene[:3, :3].T + world_from_scene[:3, 3]
    assert np.allclose(surface['mesh'].vertices, expected)
    assert len(surface['mesh'].faces) > 0
    center, position, distance = viewer._view_frame(surface['mesh'].vertices, [[1., 2., 0.]])
    assert np.isfinite(center).all() and np.isfinite(position).all() and distance > 0
    assert np.all(center >= np.minimum(surface['mesh'].vertices.min(0), [1., 2., 0.]) - 1e-9)


def test_compiled_surface_update_is_in_place_and_validates_before_mutating(monkeypatch):
    from types import SimpleNamespace
    import numpy as np
    from workshop.digital_twin import viewer

    model = SimpleNamespace(
        geom_dataid=np.array([0]), mesh_vertadr=np.array([0]), mesh_vertnum=np.array([4]),
        mesh_normaladr=np.array([0]), mesh_normalnum=np.array([4]),
        mesh_faceadr=np.array([0]), mesh_facenum=np.array([4]),
        mesh_texcoordadr=np.array([0]), mesh_texcoordnum=np.array([4]),
        mesh_quat=np.array([[1., 0., 0., 0.]]), mesh_pos=np.zeros((1, 3)),
        mesh_vert=np.full((4, 3), 9., dtype=np.float32), mesh_face=np.full((4, 3), 7, dtype=np.int32),
        mesh_normal=np.full((4, 3), 8., dtype=np.float32), mesh_facenormal=np.zeros((4,3),np.int32),
        mesh_facetexcoord=np.zeros((4,3),np.int32), geom_aabb=np.zeros((1,6)), geom_rbound=np.zeros(1),
        tex_nchannel=np.array([3]),
        tex_width=np.array([2]), tex_height=np.array([2]), tex_adr=np.array([0]),
        tex_data=np.zeros(12, dtype=np.uint8))
    monkeypatch.setattr(viewer.mujoco, 'mj_name2id', lambda *args: 0)
    surface = {'vertices_scene': np.array([[0,0,0],[1,0,0],[0,1,0],[0,0,1]], dtype=float),
               'faces': np.array([[0,1,2],[0,2,3]], dtype=np.int32),
               'used_vertices': np.array([0,1,2,3]),
               'rgb': np.full((2,2,3), 21, dtype=np.uint8)}
    viewer._update_compiled_surface(model, 0, surface)
    assert np.allclose(model.mesh_vert, surface['vertices_scene'])
    assert np.array_equal(model.mesh_face[:2], surface['faces'])
    assert np.array_equal(model.mesh_facenormal[:2], surface['faces'])
    assert np.array_equal(model.mesh_facetexcoord[:2], surface['faces'])
    assert np.array_equal(model.tex_data, np.full(12, 21, dtype=np.uint8))
    np.testing.assert_allclose(model.geom_aabb[0], [.5,.5,.5,.5,.5,.5])
    assert model.geom_rbound[0] == pytest.approx(1.)
    before = model.mesh_vert.copy()
    surface['faces'] = np.array([[0,1,8]], dtype=np.int32)
    with pytest.raises(ValueError, match='face index'):
        viewer._update_compiled_surface(model, 0, surface)
    assert np.array_equal(model.mesh_vert, before)


def test_live_surface_refresh_has_bounded_cadence_and_no_recompile_or_history_scan():
    import ast
    import inspect
    from workshop.digital_twin import viewer

    tree = ast.parse(inspect.getsource(viewer.serve))
    refresh = next(node for node in ast.walk(tree)
                   if isinstance(node, ast.FunctionDef) and node.name == 'refresh_evidence')
    calls = [node.func.id for node in ast.walk(refresh) if isinstance(node, ast.Call)
             and isinstance(node.func, ast.Name)]
    assert viewer.LIVE_SURFACE_REFRESH_INTERVAL_S >= .2
    assert '_update_compiled_surface' in calls and '_rgbd_frame' in calls
    assert not {'MjModel', 'Renderer', 'glob', 'rglob', 'listdir', 'scandir'} & set(calls)
    assert 'last_surface_refresh' in {node.id for node in ast.walk(refresh) if isinstance(node, ast.Name)}


def test_keyframe_poll_reads_fixed_tail_and_selects_late_valid_row(tmp_path, monkeypatch):
    from workshop.digital_twin.evidence_bridge import select_latest_keyframe_pair_tail

    for index in (4, 5):
        (tmp_path / f'{index}.png').write_bytes(b'rgb')
        (tmp_path / f'{index}.npy').write_bytes(b'depth')
    base_wall, base_mono = 1000., 200.
    records = [
        {'index': 4, 'color': '4.png', 'depth': '4.npy', 'utc': datetime.fromtimestamp(base_wall - .1, timezone.utc).isoformat(),
         'monotonic': base_mono - .1, 'phase_event_monotonic': base_mono - .11,
         'phase_event_delta_s': .01, 'pose_raw': {'shoulder_pan': 4}, 'pose_synchronized': True},
        {'index': 5, 'color': '5.png', 'depth': '5.npy', 'utc': datetime.fromtimestamp(base_wall - .02, timezone.utc).isoformat(),
         'monotonic': base_mono - .02, 'phase_event_monotonic': base_mono - .03,
         'phase_event_delta_s': .01, 'pose_raw': {'shoulder_pan': 5}, 'pose_synchronized': True},
    ]
    manifest = tmp_path / 'manifest.jsonl'
    manifest.write_bytes(b'{"padding":"' + b'x' * (1024 * 1024) + b'"}\n'
                         + b'\n'.join(json.dumps(row).encode() for row in records) + b'\n')
    original_open = Path.open
    reads = []

    class TrackedStream:
        def __init__(self, stream):
            self.stream = stream

        def __enter__(self):
            return self

        def __exit__(self, *args):
            self.stream.close()

        def seek(self, *args):
            return self.stream.seek(*args)

        def tell(self):
            return self.stream.tell()

        def read(self, size=-1):
            assert size >= 0, 'keyframe poll must never make an unbounded read'
            chunk = self.stream.read(size)
            reads.append(len(chunk))
            return chunk

    def tracked_open(path, *args, **kwargs):
        if path == manifest:
            return TrackedStream(original_open(manifest, 'rb'))
        return original_open(path, *args, **kwargs)

    monkeypatch.setattr(Path, 'open', tracked_open)
    selected = select_latest_keyframe_pair_tail(tmp_path, now_wall=base_wall,
                                                now_monotonic=base_mono)
    assert selected['keyframe_index'] == 5
    assert selected['pose_raw'] == {'shoulder_pan': 5}
    assert sum(reads) <= 8192


def test_keyframe_poll_rejects_stale_tail_without_historical_scan(tmp_path, monkeypatch):
    from workshop.digital_twin.evidence_bridge import select_latest_keyframe_pair_tail

    old_wall, old_mono = 900., 100.
    (tmp_path / 'old.png').write_bytes(b'rgb')
    (tmp_path / 'old.npy').write_bytes(b'depth')
    record = {'index': 1, 'color': 'old.png', 'depth': 'old.npy',
              'utc': datetime.fromtimestamp(old_wall, timezone.utc).isoformat(),
              'monotonic': old_mono, 'phase_event_monotonic': old_mono,
              'phase_event_delta_s': 0., 'pose_raw': {'shoulder_pan': 1},
              'pose_synchronized': True}
    (tmp_path / 'manifest.jsonl').write_text(json.dumps(record) + '\n')
    monkeypatch.setattr(Path, 'glob', lambda *args, **kwargs: (_ for _ in ()).throw(
        AssertionError('poll selector must not scan history')))
    assert select_latest_keyframe_pair_tail(tmp_path, now_wall=1000.,
                                           now_monotonic=200.) is None


def test_keyframe_poll_reopens_manifest_after_truncate_or_atomic_replace(tmp_path):
    from workshop.digital_twin.evidence_bridge import select_latest_keyframe_pair_tail

    manifest = tmp_path / 'manifest.jsonl'
    for index in (6, 7):
        (tmp_path / f'{index}.png').write_bytes(b'rgb')
        (tmp_path / f'{index}.npy').write_bytes(b'depth')
    def record(index, wall, mono):
        return {'index': index, 'color': f'{index}.png', 'depth': f'{index}.npy',
                'utc': datetime.fromtimestamp(wall, timezone.utc).isoformat(),
                'monotonic': mono, 'phase_event_monotonic': mono,
                'phase_event_delta_s': 0., 'pose_raw': {'shoulder_pan': index},
                'pose_synchronized': True}
    manifest.write_text(json.dumps(record(6, 999.9, 199.9)) + '\n')
    assert select_latest_keyframe_pair_tail(tmp_path, now_wall=1000., now_monotonic=200.)['keyframe_index'] == 6
    replacement = tmp_path / 'new-manifest.jsonl'
    replacement.write_text(json.dumps(record(7, 999.98, 199.98)) + '\n')
    os.replace(replacement, manifest)
    assert select_latest_keyframe_pair_tail(tmp_path, now_wall=1000., now_monotonic=200.)['keyframe_index'] == 7
