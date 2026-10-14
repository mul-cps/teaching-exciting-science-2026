"""Pixel geometry and optional trusted Cornell checkpoint inference."""
from pathlib import Path
import numpy as np
import pytest

from workshop.digital_twin.grasp_ggcnn import prepare_depth, candidates_from_maps, load_model, predict


def test_preparation_preserves_holes_and_centres_valid_depth():
    depth = np.full((480, 640), 550, np.uint16)
    depth[200:220, 310:330] = 0
    depth[230, 400] = 65535
    prep = prepare_depth(depth, (310, 200, 510, 400))
    assert prep.image.shape == (300, 300)
    assert not prep.valid[2, 2]
    assert np.isfinite(prep.image).all()
    assert abs(float(np.median(prep.image[prep.valid]))) < .002
    assert prep.image.max() <= 1
    assert prep.image.min() >= -1


def test_prepare_rejects_empty_nonsquare_outside_roi():
    depth = np.ones((480, 640))
    for roi in [(0, 0, 200, 100), (-1, 0, 199, 200), (0, 0, 700, 700)]:
        with pytest.raises(ValueError):
            prepare_depth(depth, roi)
    with pytest.raises(ValueError, match='valid depth'):
        prepare_depth(np.zeros((300, 300)), (0, 0, 300, 300))


@pytest.mark.parametrize('angle,axis', [(0, 'horizontal'), (np.pi / 2, 'vertical')])
def test_inverse_crop_center_and_contact_angle(angle, axis):
    prep = prepare_depth(np.ones((480, 640)), (310, 200, 510, 400), depth_scale=1)
    quality = np.zeros((300, 300)); quality[150, 120] = .9
    angles = np.full_like(quality, angle)
    width = np.full_like(quality, 60.)
    candidate = candidates_from_maps(quality, angles, width, prep, top_n=1)[0]
    assert candidate['center_xy'] == pytest.approx([310 + 120.5 * 2 / 3 - .5, 200 + 150.5 * 2 / 3 - .5])
    assert candidate['opening_pixels'] == pytest.approx(40)
    p1, p2 = candidate['contacts_xy']
    if axis == 'horizontal':
        assert p2[0] - p1[0] == pytest.approx(40)
        assert p2[1] == pytest.approx(p1[1])
    else:
        assert p2[1] - p1[1] == pytest.approx(-40)
        assert p2[0] == pytest.approx(p1[0])


def test_holes_and_object_mask_filter_quality_peaks_before_selection():
    depth = np.ones((300, 300)); depth[50, 50] = 0
    mask = np.zeros_like(depth, dtype=bool); mask[150:250, 150:250] = True
    prep = prepare_depth(depth, (0, 0, 300, 300), object_mask=mask, depth_scale=1)
    q = np.zeros_like(depth); q[50, 50] = 1; q[100, 100] = .95; q[200, 200] = .8
    found = candidates_from_maps(q, np.zeros_like(q), np.full_like(q, 20), prep, top_n=5)
    assert len(found) == 1
    assert found[0]['center_xy'] == [200., 200.]
    assert 'confidence' not in found[0]


def test_optional_actual_cornell_state_checkpoint():
    pytest.importorskip('torch')
    state = Path.home() / '.cache/cps-grasp-models/cornell-state.pt'
    if not state.exists():
        pytest.skip('trusted Cornell state checkpoint not cached')
    model = load_model(state)
    assert not model.training
    assert next(model.parameters()).device.type == 'cpu'
    yy, xx = np.mgrid[:300, :300]
    depth = np.full((300, 300), .55, np.float32)
    depth[(xx > 110) & (xx < 190) & (yy > 130) & (yy < 170)] -= .02
    result = predict(model, prepare_depth(depth, (0, 0, 300, 300), depth_scale=1), top_n=3)
    assert len(result) <= 3
    assert all(np.isfinite(item['quality_score']) for item in result)


def test_download_hash_mismatch_never_publishes_state(monkeypatch, tmp_path):
    import io
    from workshop.digital_twin import grasp_ggcnn
    monkeypatch.setattr(grasp_ggcnn.urllib.request, 'urlopen', lambda *a, **kw: io.BytesIO(b'untrusted'))
    with pytest.raises(ValueError, match='SHA256'):
        grasp_ggcnn.download_weights(tmp_path)
    assert not list(tmp_path.iterdir())


def test_download_archive_rejects_traversal(monkeypatch, tmp_path):
    import hashlib
    import io
    import zipfile
    from workshop.digital_twin import grasp_ggcnn
    stream = io.BytesIO()
    with zipfile.ZipFile(stream, 'w') as archive:
        archive.writestr('../escape', b'bad')
    content = stream.getvalue()
    monkeypatch.setattr(grasp_ggcnn, 'ARCHIVE_SHA256', hashlib.sha256(content).hexdigest())
    monkeypatch.setattr(grasp_ggcnn.urllib.request, 'urlopen', lambda *a, **kw: io.BytesIO(content))
    with pytest.raises(ValueError, match='Unsafe'):
        grasp_ggcnn.download_weights(tmp_path)
    assert not list(tmp_path.iterdir())


def test_cpu_only_lazy_import_static_guard():
    import ast
    from workshop.digital_twin import grasp_ggcnn
    tree = ast.parse(Path(grasp_ggcnn.__file__).read_text())
    for item in tree.body:
        if isinstance(item, ast.Import):
            assert all(not alias.name.startswith('torch') for alias in item.names)
        if isinstance(item, ast.ImportFrom):
            assert not (item.module or '').startswith('torch')
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
            assert node.func.attr not in {'cuda', 'synchronize', 'item'}
            if node.func.attr == 'load' and isinstance(node.func.value, ast.Name) and node.func.value.id == 'torch':
                values = {kw.arg: ast.literal_eval(kw.value) for kw in node.keywords}
                assert values['weights_only'] is True
                assert values['map_location'] == 'cpu'


def test_verified_cached_archive_download_writes_only_tensor_state(monkeypatch, tmp_path):
    import io
    from workshop.digital_twin import grasp_ggcnn
    pytest.importorskip('torch')
    archive_path = Path.home() / '.cache/cps-grasp-models/ggcnn_weights_cornell.zip'
    if not archive_path.exists():
        pytest.skip('SHA-pinned archive not cached')
    monkeypatch.setattr(grasp_ggcnn.urllib.request, 'urlopen', lambda *a, **kw: io.BytesIO(archive_path.read_bytes()))
    target = grasp_ggcnn.download_weights(tmp_path)
    assert target.name == 'cornell-state.pt'
    assert list(tmp_path.iterdir()) == [target]
    assert not grasp_ggcnn.load_model(target).training


def test_impossible_contact_plateau_never_enters_peak_scan(monkeypatch):
    from workshop.digital_twin import grasp_ggcnn
    prep = prepare_depth(np.ones((300, 300)), (0, 0, 300, 300), depth_scale=1)
    q = np.full((300, 300), .9)
    def unexpected(*args, **kwargs):
        raise AssertionError('No peak ranking or selection should run for invalid contacts')
    monkeypatch.setattr(grasp_ggcnn.np, 'argmax', unexpected)
    monkeypatch.setattr(grasp_ggcnn.np, 'argsort', unexpected)
    assert candidates_from_maps(q, np.zeros_like(q), np.full_like(q, 1000), prep) == []


def test_feasible_plateau_has_bounded_vectorized_peak_scans(monkeypatch):
    from workshop.digital_twin import grasp_ggcnn
    prep = prepare_depth(np.ones((300, 300)), (0, 0, 300, 300), depth_scale=1)
    q = np.full((300, 300), .9)
    actual_argmax = np.argmax
    calls = []
    def counted(*args, **kwargs):
        calls.append(1)
        return actual_argmax(*args, **kwargs)
    monkeypatch.setattr(grasp_ggcnn.np, 'argmax', counted)
    result = candidates_from_maps(q, np.zeros_like(q), np.full_like(q, 20), prep, top_n=5)
    assert len(result) == 5
    assert len(calls) == 5
    assert all(0 <= x <= 299 and 0 <= y <= 299 for candidate in result for x, y in candidate['contacts_xy'])


def test_contact_boundary_filter_precedes_nms_loop_static_guard():
    import ast
    import inspect
    from workshop.digital_twin import grasp_ggcnn
    function = ast.parse(inspect.getsource(grasp_ggcnn.candidates_from_maps)).body[0]
    assignments = {node.targets[0].id: node.lineno for node in function.body
                   if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name)}
    loops = [node for node in function.body if isinstance(node, ast.For)]
    assert len(loops) == 1
    assert assignments['contact_inside'] < assignments['allowed'] < assignments['peak_scores'] < loops[0].lineno
    assert any(isinstance(node, ast.Name) and node.id == 'contact_inside'
               for node in ast.walk(next(node for node in function.body if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name) and node.targets[0].id == 'allowed')))


def test_predict_smooths_periodic_grasp_axis_across_antipodal_wrap(monkeypatch):
    torch = pytest.importorskip('torch')
    from workshop.digital_twin import grasp_ggcnn
    angles = np.full((300, 300), np.deg2rad(89), dtype=np.float32)
    angles[:, 150:] = np.deg2rad(-89)
    maps = [np.ones_like(angles), np.cos(2*angles), np.sin(2*angles), np.full_like(angles, .2)]
    class FixedModel:
        def __call__(self, tensor):
            return tuple(torch.from_numpy(value).reshape(1, 1, 300, 300) for value in maps)
    captured = {}
    def capture(quality, angle, width, prepared, **kwargs):
        captured['angle'] = angle
        return []
    monkeypatch.setattr(grasp_ggcnn, 'candidates_from_maps', capture)
    prepared = prepare_depth(np.ones((300, 300)), (0, 0, 300, 300), depth_scale=1)
    grasp_ggcnn.predict(FixedModel(), prepared)
    assert np.all(np.abs(captured['angle'][:, 148:152]) > np.deg2rad(88))
