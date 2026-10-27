"""The offline matrix must freeze retained evidence before a native port exists."""
import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('native_geometry_matrix', ROOT/'tools/native_geometry_matrix.py')
matrix = importlib.util.module_from_spec(spec)
spec.loader.exec_module(matrix)

NAMES = [
    'base', 'invalid-hole', 'touching-depth-children', 'nonplanar-two-face',
    'sensor-edge', 'sparse-concave', 'partial-observation-roi',
    'partial-pickup-roi', 'distortion-4', 'distortion-5', 'distortion-8',
    'large-gray-support',
]


def test_matrix_preserves_distinct_holes_children_edges_and_independent_inputs():
    entries = matrix.cases()
    assert [name for name, _ in entries] == NAMES
    a = dict(entries)
    assert np.all(a['invalid-hole']['depth'][45:50, 61:66] == 0)
    assert np.all(a['base']['depth'][45:50, 61:66] == 780)
    assert np.all(a['touching-depth-children']['depth'][42:54, 52:64] == 780)
    assert np.all(a['touching-depth-children']['depth'][42:54, 64:76] == 740)
    assert np.all(a['sensor-edge']['rgb'][42:54, :12] == [255, 0, 0])
    assert np.all(a['sensor-edge']['rgb'][42:54, 58:70] == 128)
    assert np.all(a['sensor-edge']['depth'][42:54, 58:70] == 800)
    assert np.all(a['sparse-concave']['rgb'][42:51, 61:70] == 128)
    assert a['partial-observation-roi']['observation_roi'] == [0, .03, -.03, .03, -.1, .1]
    assert a['partial-pickup-roi']['pickup_roi'] == [0, .03, -.03, .03, -.1, .1]
    assert [a[f'distortion-{n}']['distortion'].size for n in [4, 5, 8]] == [4, 5, 8]
    for i, (_, left) in enumerate(entries):
        for _, right in entries[i+1:]:
            assert not np.shares_memory(left['rgb'], right['rgb'])
            assert not np.shares_memory(left['depth'], right['depth'])
    assert a['large-gray-support']['depth'].shape == (480, 640)
    assert np.all(a['large-gray-support']['rgb'] == 128)
    assert np.all(a['large-gray-support']['depth'] == 800)


def test_base_report_and_ordered_pieces_match_current_versioned_reference():
    expected = json.loads((ROOT/'tests/fixtures/native_geometry/procedural-v2.json').read_text())
    actual = matrix.oracle.reference_case(dict(matrix.cases())['base'])
    matrix.oracle.assert_report_equal(expected['report'], actual['report'])
    matrix.oracle.assert_report_equal(expected['pieces'], actual['pieces'], tolerance=1e-12)


def test_large_support_retains_order_and_seed17_population_selection():
    a = dict(matrix.cases())['large-gray-support']
    trace = matrix.oracle.trace_intermediates(a)
    assert len(trace['support']['inside_board']) == 4800
    indices = np.random.default_rng(17).choice(4800, 2000, replace=False)
    np.testing.assert_array_equal(trace['support']['selected_indices'], indices)
    np.testing.assert_array_equal(trace['support']['selected_board'], trace['support']['inside_board'][indices])


def test_invalid_parent_hole_and_depth_children_are_retained_as_different_evidence():
    a = dict(matrix.cases())
    hole = matrix.oracle.trace_intermediates(a['invalid-hole'])
    assert len(hole['parents']) == 1
    assert int(hole['parents'][0]['mask'].sum()) == 144
    assert int(hole['parents'][0]['valid'].sum()) == 119
    assert len(hole['pieces']) == 1
    assert hole['pieces'][0]['support_mask'].sum() > hole['pieces'][0]['mask'].sum()
    children = matrix.oracle.trace_intermediates(a['touching-depth-children'])
    assert len(children['parents']) == 1
    assert len(children['pieces']) == 2
    assert all(p['parent_pixels'] == 288 for p in children['pieces'])


def test_existing_output_is_never_rewritten(tmp_path):
    target = tmp_path/'existing'
    target.mkdir()
    sentinel = target/'capture.json'
    sentinel.write_text('{"keep": true}\n')
    with pytest.raises(FileExistsError):
        matrix.freeze_matrix(target)
    assert sentinel.read_text() == '{"keep": true}\n'
    assert sorted(x.name for x in target.iterdir()) == ['capture.json']


@pytest.fixture(scope='module')
def frozen_matrix(tmp_path_factory):
    target = tmp_path_factory.mktemp('geometry-matrix-parent')/'matrix'
    manifest = matrix.freeze_matrix(target)
    return target, manifest


def test_freeze_is_complete_verified_cpu_evidence_and_matches_committed_golden(frozen_matrix):
    target, manifest = frozen_matrix
    assert manifest['state'] == 'complete'
    assert manifest['schema'] == 'so101-native-full-geometry-matrix-v2'
    assert manifest['principal_axes_policy'] == matrix.oracle.principal_axes_policy()
    assert [c['name'] for c in manifest['cases']] == NAMES
    assert manifest['numeric_absolute_tolerance'] == 1e-9
    assert manifest['local_ray_absolute_tolerance'] == 1e-12
    expected = json.loads((ROOT/'tests/fixtures/native_geometry/matrix-v2.json').read_text())
    matrix.assert_matrix_payload_equal(expected, manifest)
    for case in manifest['cases']:
        stored, arrays = matrix.oracle.load_verified(target/case['name'])
        assert case['arrays'] == stored['arrays']
        actual = matrix.oracle.plain_values(matrix.oracle.restore_arrays(stored['report'], arrays))
        matrix.oracle.assert_report_equal(case['reference']['report'], actual)
        assert actual['qualified'] is False
        assert 'offline retained oracle' in stored['provenance']['scope']


def test_repeat_freeze_is_byte_reproducible_and_array_corruption_is_rejected(tmp_path, frozen_matrix):
    first, old = frozen_matrix
    second = tmp_path/'repeat'
    new = matrix.freeze_matrix(second)
    assert new == old
    paths = sorted(p.relative_to(first) for p in first.rglob('*') if p.is_file())
    assert paths == sorted(p.relative_to(second) for p in second.rglob('*') if p.is_file())
    for path in paths:
        assert (first/path).read_bytes() == (second/path).read_bytes()
    victim = second/'invalid-hole'/next(iter(new['cases'][1]['arrays']))
    data = bytearray(victim.read_bytes()); data[-1] ^= 1; victim.write_bytes(data)
    with pytest.raises(ValueError, match='digest'):
        matrix.oracle.load_verified(second/'invalid-hole')


def test_failed_real_cpu_case_cannot_publish_complete_matrix(tmp_path, monkeypatch):
    invalid = matrix.oracle.procedural_scene()
    invalid['depth'][:] = 0
    monkeypatch.setattr(matrix, 'cases', lambda: [('invalid-test-input', invalid)])
    target = tmp_path/'failed'
    with pytest.raises(ValueError, match='insufficient valid depth'):
        matrix.freeze_matrix(target)
    assert not (target/'manifest.json').exists()
    status = json.loads((target/'status.json').read_text())
    assert status['state'] == 'failed'
    assert status['case'] == 'invalid-test-input'


def test_matrix_comparator_does_not_relax_discrete_or_numeric_tolerances(frozen_matrix):
    import copy
    _, frozen = frozen_matrix
    wrong = copy.deepcopy(frozen)
    wrong['cases'][0]['reference']['report']['qualified'] = 0
    with pytest.raises(AssertionError):
        matrix.assert_matrix_equal(frozen, wrong)
    wrong = copy.deepcopy(frozen)
    wrong['cases'][0]['reference']['report']['objects'][0]['height_m'] += 2e-9
    with pytest.raises(AssertionError):
        matrix.assert_matrix_equal(frozen, wrong)


def test_frozen_array_metadata_keeps_integer_types_exact(frozen_matrix):
    import copy
    _, frozen = frozen_matrix
    wrong = copy.deepcopy(frozen)
    shape = wrong['cases'][0]['arrays']['array-00002.npy']['shape']
    shape[0] = float(shape[0])
    with pytest.raises(AssertionError):
        matrix.assert_matrix_equal(frozen, wrong)


def test_ordered_ray_evidence_keeps_existing_1e12_tolerance(frozen_matrix):
    import copy
    _, frozen = frozen_matrix
    wrong = copy.deepcopy(frozen)
    wrong['cases'][0]['reference']['pieces'][0]['camera_global'][0][0] += 2e-12
    with pytest.raises(AssertionError):
        matrix.assert_matrix_equal(frozen, wrong)


def test_version_only_change_preserves_payload_but_not_qualification_identity(frozen_matrix):
    import copy
    _, frozen = frozen_matrix
    changed = copy.deepcopy(frozen)
    for case in changed['cases']:
        case['provenance']['python'] = 'other-python-version'
        case['oracle_manifest_sha256'] = '0'*64
    matrix.assert_matrix_payload_equal(frozen, changed)
    with pytest.raises(AssertionError):
        matrix.assert_matrix_equal(frozen, changed)


def test_payload_comparison_keeps_envelopes_and_same_runtime_identity(frozen_matrix):
    import copy
    _, frozen = frozen_matrix
    preserved = copy.deepcopy(frozen)
    matrix.assert_matrix_payload_equal(frozen, preserved)
    assert preserved == frozen
    changed = copy.deepcopy(frozen)
    changed['cases'][0]['oracle_manifest_sha256'] = '0'*64
    with pytest.raises(AssertionError):
        matrix.assert_matrix_payload_equal(frozen, changed)
    changed = copy.deepcopy(frozen)
    changed['cases'][0]['provenance']['python'] = False
    with pytest.raises(AssertionError):
        matrix.assert_matrix_payload_equal(frozen, changed)


@pytest.mark.parametrize('runtime_field', ['python', 'numpy', 'opencv', 'scipy'])
def test_only_each_explicit_runtime_version_can_differ(runtime_field, frozen_matrix):
    import copy
    _, frozen = frozen_matrix
    changed = copy.deepcopy(frozen)
    changed['cases'][0]['provenance'][runtime_field] = 'other-version'
    changed['cases'][0]['oracle_manifest_sha256'] = '0'*64
    original = copy.deepcopy(changed)
    matrix.assert_matrix_payload_equal(frozen, changed)
    assert changed == original
    with pytest.raises(AssertionError):
        matrix.assert_matrix_equal(frozen, changed)


@pytest.mark.parametrize('mutation', [
    'report', 'ray', 'array-sha', 'array-dtype', 'array-shape',
    'array-shape-type', 'input', 'source', 'trace-payload-hash', 'order',
])
def test_cross_runtime_payload_does_not_hide_semantic_mutations(mutation, frozen_matrix):
    import copy
    _, frozen = frozen_matrix
    changed = copy.deepcopy(frozen)
    for case in changed['cases']:
        case['provenance']['python'] = 'other-version'
        case['oracle_manifest_sha256'] = '0'*64
    case = changed['cases'][0]
    array = case['arrays']['array-00002.npy']
    if mutation == 'report':
        case['reference']['report']['objects'][0]['height_m'] += 2e-9
    elif mutation == 'ray':
        case['reference']['pieces'][0]['camera_global'][0][0] += 2e-12
    elif mutation == 'array-sha':
        array['sha256'] = '0'*64
    elif mutation == 'array-dtype':
        array['dtype'] = 'uint8'
    elif mutation == 'array-shape':
        array['shape'][0] += 1
    elif mutation == 'array-shape-type':
        array['shape'][0] = float(array['shape'][0])
    elif mutation == 'input':
        case['inputs']['intrinsics']['fx'] += 1
    elif mutation == 'source':
        case['provenance']['sources']['tools/native_geometry_oracle.py'] = '0'*64
    elif mutation == 'trace-payload-hash':
        case['oracle_payload_manifest_sha256'] = '0'*64
    elif mutation == 'order':
        changed['cases'][:2] = reversed(changed['cases'][:2])
    with pytest.raises(AssertionError):
        matrix.assert_matrix_payload_equal(frozen, changed)


def test_full_payload_digest_binds_trace_schema_inputs_sources_and_array_mapping(frozen_matrix):
    import copy
    target, frozen = frozen_matrix
    snapshot = json.loads((target/'base'/'manifest.json').read_text())
    expected = frozen['cases'][0]['oracle_payload_manifest_sha256']
    assert matrix._oracle_payload_sha256(snapshot) == expected
    for field in ['python', 'numpy', 'opencv', 'scipy']:
        changed = copy.deepcopy(snapshot)
        changed['provenance'][field] = 'other-version'
        assert matrix._oracle_payload_sha256(changed) == expected
    mutations = []
    changed = copy.deepcopy(snapshot); changed['trace']['parents'][0]['origin'][0] += 1; mutations.append(changed)
    changed = copy.deepcopy(snapshot); changed['schema'] += '-wrong'; mutations.append(changed)
    changed = copy.deepcopy(snapshot); changed['inputs']['intrinsics']['fx'] += 1; mutations.append(changed)
    changed = copy.deepcopy(snapshot); changed['provenance']['sources']['tools/native_geometry_oracle.py'] = '0'*64; mutations.append(changed)
    changed = copy.deepcopy(snapshot); changed['preparation']['rays'] = 'wrong-array.npy'; mutations.append(changed)
    changed = copy.deepcopy(snapshot); changed['arrays']['array-00002.npy']['dtype'] = 'uint8'; mutations.append(changed)
    for changed in mutations:
        assert matrix._oracle_payload_sha256(changed) != expected
