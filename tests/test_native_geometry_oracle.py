"""Frozen complete geometry evidence, independent of any Vulkan implementation."""
import importlib.util
from pathlib import Path
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('native_geometry_oracle', ROOT/'tools/native_geometry_oracle.py')
oracle = importlib.util.module_from_spec(spec)
spec.loader.exec_module(oracle)


def test_snapshot_covers_full_report_original_memberships_and_ordered_contours(tmp_path):
    snapshot = oracle.freeze(oracle.procedural_scene(), tmp_path/'case')
    report, arrays = oracle.load_verified(tmp_path/'case')
    assert report['schema'] == oracle.ORACLE_SCHEMA
    assert report['provenance']['principal_axes_policy'] == oracle.principal_axes_policy()
    assert set(report['provenance']['sources']) == set(oracle.SOURCES)
    assert len(report['report']['objects']) == 1
    assert report['report']['qualified'] is False
    assert report['provenance']['numeric_absolute_tolerance'] == 1e-9
    assert report['provenance']['local_ray_absolute_tolerance'] == 1e-12
    assert len(report['trace']['pieces']) == 1
    piece = report['trace']['pieces'][0]
    assert arrays[piece['mask']]['dtype'] == 'bool'
    assert piece['contours']
    contour = arrays[piece['contours'][0]['vertices']]['value']
    assert contour.shape[1:] == (1, 2)
    assert snapshot['arrays'] == report['arrays']


def test_snapshot_is_reproducible_and_any_array_corruption_is_rejected(tmp_path):
    oracle.freeze(oracle.procedural_scene(), tmp_path/'a')
    oracle.freeze(oracle.procedural_scene(), tmp_path/'b')
    files_a = sorted(x.relative_to(tmp_path/'a') for x in (tmp_path/'a').rglob('*') if x.is_file())
    assert files_a == sorted(x.relative_to(tmp_path/'b') for x in (tmp_path/'b').rglob('*') if x.is_file())
    for name in files_a:
        assert (tmp_path/'a'/name).read_bytes() == (tmp_path/'b'/name).read_bytes()
    victim = tmp_path/'a'/next(x for x in files_a if x.suffix == '.npy')
    data = bytearray(victim.read_bytes()); data[-1] ^= 1; victim.write_bytes(data)
    with pytest.raises(ValueError, match='digest'):
        oracle.load_verified(tmp_path/'a')


def test_exact_discrete_comparator_rejects_bool_integer_and_reason_order():
    with pytest.raises(AssertionError):
        oracle.assert_report_equal({'accepted': True}, {'accepted': 1})
    with pytest.raises(AssertionError):
        oracle.assert_report_equal({'reason': ['a', 'b']}, {'reason': ['b', 'a']})
    oracle.assert_report_equal({'x': 1.0}, {'x': 1.0+5e-10})
    with pytest.raises(AssertionError):
        oracle.assert_report_equal({'x': 1.0}, {'x': 1.0+2e-9})


def test_distorted_local_rays_preserve_full_image_convention():
    args = oracle.procedural_scene()
    args['distortion'] = np.array([.04, -.02, .003, -.005, .002])
    trace = oracle.trace_intermediates(args)
    assert trace['pieces']
    for piece in trace['pieces']:
        np.testing.assert_allclose(piece['camera_global'], piece['camera_local'], rtol=0, atol=1e-12)


def test_support_selection_matches_retained_seed17_maximum():
    args = oracle.procedural_scene(width=640, height=480)
    args['support_roi'] = [-10, 10, -10, 10, -10, 10]
    trace = oracle.trace_intermediates(args)
    assert len(trace['support']['inside_board']) > 2000
    indices = np.random.default_rng(17).choice(len(trace['support']['inside_board']), 2000, replace=False)
    np.testing.assert_array_equal(trace['support']['selected_indices'], indices)
    np.testing.assert_array_equal(trace['support']['selected_board'], trace['support']['inside_board'][indices])


def test_restore_arrays_keeps_full_published_report_comparable(tmp_path):
    args = oracle.procedural_scene()
    oracle.freeze(args, tmp_path/'case')
    manifest, arrays = oracle.load_verified(tmp_path/'case')
    restored = oracle.restore_arrays(manifest['report'], arrays)
    oracle.assert_report_equal(restored, oracle.analyze_reference(args))
    restored['objects'][0]['acquisition_eligible'] = 1
    with pytest.raises(AssertionError):
        oracle.assert_report_equal(restored, oracle.analyze_reference(args))


def test_dense_integer_memberships_never_use_float_tolerance():
    with pytest.raises(AssertionError):
        oracle.assert_report_equal(np.array([1], dtype=np.int64), np.array([1.0000000001]))


def test_preparation_reference_equals_retained_support_population_before_sampling():
    a = oracle.procedural_scene()
    a['distortion'] = np.array([.02, -.01, .002, -.003, .001])
    prepared, retained = oracle.preparation_reference(a), oracle.trace_intermediates(a)
    np.testing.assert_allclose(prepared['selected_board'], retained['support']['inside_board'], rtol=0, atol=1e-12)
    assert prepared['valid_count'] == np.count_nonzero(retained['valid_depth'])
    assert prepared['selected_pixels'].max() < a['depth'].size
    assert np.all(np.diff(prepared['selected_pixels'].astype(np.int64)) > 0)


def test_preparation_reference_vivid_exclusion_uses_radius3_at_sensor_edge():
    a = oracle.procedural_scene(width=17, height=9)
    a['rgb'][:] = 128
    a['rgb'][0, 0] = [255, 0, 0]
    a['support_roi'] = [-10, 10, -10, 10, -10, 10]
    p = oracle.preparation_reference(a)
    assert np.all((p['flags'][:4, :4]&2) != 0)
    assert not np.any(p['flags'][4:, :]&2)
    assert p['selected_pixels'].tolist() == [8, 16, 136, 144, 152]


def test_procedural_full_report_and_ordered_child_evidence_match_frozen_reference():
    import json
    expected = json.loads((ROOT/'tests/fixtures/native_geometry/procedural-v2.json').read_text())
    actual = oracle.reference_case(oracle.procedural_scene())
    oracle.assert_report_equal(expected['report'], actual['report'])
    oracle.assert_report_equal(expected['pieces'], actual['pieces'], tolerance=1e-12)


@pytest.mark.parametrize('coefficients', [
    [.04, -.02, .003, -.005],
    [.04, -.02, .003, -.005, .002],
    [.04, -.02, .003, -.005, .002, .001, -.001, .0005],
])
def test_preparation_lut_preserves_4_5_8_coefficient_distorted_rays(coefficients):
    a = oracle.procedural_scene()
    a['distortion'] = np.array(coefficients)
    angle = .12
    rotation = np.array([[np.cos(angle), 0, np.sin(angle)], [0, 1, 0], [-np.sin(angle), 0, np.cos(angle)]])
    a['board_from_camera'][:3, :] = rotation@a['board_from_camera'][:3, :]
    prepared, trace = oracle.preparation_reference(a), oracle.trace_intermediates(a)
    np.testing.assert_allclose(prepared['selected_board'], trace['support']['inside_board'], rtol=0, atol=1e-12)
    for piece in trace['pieces']:
        np.testing.assert_allclose(piece['camera_global'], piece['camera_local'], rtol=0, atol=1e-12)


def test_preparation_float32_validity_preserves_nextafter_edges_and_nonfinite():
    a = oracle.procedural_scene(width=8, height=8)
    a['depth_scale_m'] = 1.
    low, high = np.float32(.1), np.float32(1.2)
    samples = [np.nextafter(low, -np.inf, dtype=np.float32), low,
               np.nextafter(low, np.inf, dtype=np.float32),
               np.nextafter(high, -np.inf, dtype=np.float32), high,
               np.nextafter(high, np.inf, dtype=np.float32), np.nan, np.inf]
    a['depth'] = np.tile(np.array(samples, np.float32), (8, 1))
    with np.errstate(invalid='ignore'):
        prepared = oracle.preparation_reference(a)
    np.testing.assert_array_equal(prepared['flags'][0]&1, [0, 1, 1, 1, 1, 0, 0, 0])
    assert prepared['valid_count'] == 32


def test_versioned_metadata_declares_exact_policy_without_qualification():
    policy = oracle.principal_axes_policy()
    assert oracle.ORACLE_SCHEMA == 'so101-native-full-geometry-oracle-v2'
    assert oracle.REFERENCE_SCHEMA == 'so101-native-geometry-procedural-reference-v2'
    assert len(oracle.SOURCES) == 8
    assert 'workshop/digital_twin/principal_axes.py' in oracle.SOURCES
    assert policy['name'] == 'canonical-principal-axes-v1'
    assert policy['spectrum_cluster']['eps64_multiplier'] == 8
    assert policy['spectrum_cluster']['units'] == {'svd': 'singular values', 'covariance_eigh': 'eigenvalues'}
    assert policy['projected_anchor_tie']['eps64_multiplier'] == 64
    assert policy['grasp_width_ranking_tie']['eps64_multiplier'] == 32
    assert policy['universal_cross_platform_bit_identity'] is False


def test_read_only_legacy_schema_loading_does_not_relabel_sources(tmp_path):
    import json
    historical = {'schema': oracle.LEGACY_ORACLE_SCHEMA, 'arrays': {},
                  'provenance': {'sources': {'historical-only.py': 'old-source'}}}
    manifest = tmp_path/'manifest.json'
    original = json.dumps(historical)+'\n'
    manifest.write_text(original)
    loaded, arrays = oracle.load_verified(tmp_path)
    assert loaded == historical and arrays == {}
    assert manifest.read_text() == original
    assert 'principal_axes_policy' not in loaded['provenance']


@pytest.mark.parametrize('mutation', ['missing-policy', 'wrong-policy', 'floating-multiplier', 'missing-source', 'wrong-schema'])
def test_v2_loader_rejects_wrong_policy_or_missing_helper_binding(tmp_path, mutation):
    import json
    value = dict(schema=oracle.ORACLE_SCHEMA, arrays={}, provenance=dict(
        principal_axes_policy=oracle.principal_axes_policy(),
        sources={'workshop/digital_twin/principal_axes.py': '0'*64}))
    if mutation == 'missing-policy': del value['provenance']['principal_axes_policy']
    elif mutation == 'wrong-policy': value['provenance']['principal_axes_policy']['name'] = 'unreviewed-policy'
    elif mutation == 'floating-multiplier': value['provenance']['principal_axes_policy']['spectrum_cluster']['eps64_multiplier'] = 8.0
    elif mutation == 'missing-source': value['provenance']['sources'].clear()
    else: value['schema'] = 'so101-native-full-geometry-oracle-v3'
    (tmp_path/'manifest.json').write_text(json.dumps(value))
    with pytest.raises(ValueError): oracle.load_verified(tmp_path)


@pytest.mark.parametrize('name,sha256', [
    ('procedural-v1.json', 'efebf08df0141b87f3d335a25edf22ed30b9d446fcb59ee704ff8d90f63d6ec6'),
    ('matrix-v1.json', '7f9feb3f53353a301f575805762391a3bd7f9fb9c55571f0d098c7d711028625'),
    ('observed-children-v1.json', '4e6d34310c69c29c11db6d8f5d743867d6622d9f982e967cb12e74f1c38b3fea'),
])
def test_historical_v1_payload_bytes_remain_immutable(name, sha256):
    import hashlib
    # Historical reports were generated by immutable 1d088c7 sources, not live imports.
    assert hashlib.sha256((ROOT/'tests/fixtures/native_geometry'/name).read_bytes()).hexdigest() == sha256
