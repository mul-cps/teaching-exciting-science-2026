"""Freeze public procedural geometry evidence using the retained CPU oracle.

The complete arrays remain in the fresh output directory. The lightweight
matrix manifest pins their lossless hashes and contains complete report and
ordered child evidence. This tool never accepts or executes a native result.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tools import native_geometry_oracle as oracle


def cases():
    """Return independent inputs, in the canonical version-one matrix order."""
    result = [('base', oracle.procedural_scene())]

    a = oracle.procedural_scene()
    a['depth'][45:50, 61:66] = 0
    result.append(('invalid-hole', a))

    a = oracle.procedural_scene()
    a['rgb'][42:54, 58:70] = 128
    a['depth'][42:54, 58:70] = 800
    a['rgb'][42:54, 52:76] = [255, 0, 0]
    a['depth'][42:54, 52:64] = 780
    a['depth'][42:54, 64:76] = 740
    result.append(('touching-depth-children', a))

    a = oracle.procedural_scene()
    a['depth'][42:54, 64:70] = 777
    result.append(('nonplanar-two-face', a))

    a = oracle.procedural_scene()
    a['rgb'][42:54, 58:70] = 128
    a['depth'][42:54, 58:70] = 800
    a['rgb'][42:54, :12] = [255, 0, 0]
    a['depth'][42:54, :12] = 780
    result.append(('sensor-edge', a))

    a = oracle.procedural_scene()
    a['rgb'][42:54, 58:70] = 128
    a['depth'][42:54, 58:70] = 800
    for rows, columns in [(slice(42, 54), slice(58, 61)),
                          (slice(51, 54), slice(58, 70))]:
        a['rgb'][rows, columns] = [255, 0, 0]
        a['depth'][rows, columns] = 780
    result.append(('sparse-concave', a))

    a = oracle.procedural_scene()
    a['observation_roi'] = [0, .03, -.03, .03, -.1, .1]
    result.append(('partial-observation-roi', a))
    a = oracle.procedural_scene()
    a['pickup_roi'] = [0, .03, -.03, .03, -.1, .1]
    result.append(('partial-pickup-roi', a))

    coefficients = [.04, -.02, .003, -.005, .002, .001, -.001, .0005]
    for length in [4, 5, 8]:
        a = oracle.procedural_scene()
        a['distortion'] = np.array(coefficients[:length])
        result.append((f'distortion-{length}', a))

    a = oracle.procedural_scene(width=640, height=480)
    a['rgb'][:] = 128
    a['depth'][:] = 800
    a['support_roi'] = [-10, 10, -10, 10, -10, 10]
    result.append(('large-gray-support', a))
    return result


def _write_json(path, value):
    # A complete JSON file becomes visible in one rename. The root was claimed
    # with mkdir(exist_ok=False), so existing captures can never be rewritten.
    pending = path.with_name(path.name+'.pending')
    with pending.open('x') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write('\n')
    os.replace(pending, path)


_RUNTIME_VERSION_FIELDS = ('python', 'numpy', 'opencv', 'scipy')


def _payload_provenance(provenance):
    for field in _RUNTIME_VERSION_FIELDS:
        assert type(provenance[field]) is str and provenance[field], field
    return {key: value for key, value in provenance.items()
            if key not in _RUNTIME_VERSION_FIELDS}


def _oracle_payload_sha256(snapshot):
    """Bind the entire frozen snapshot except its four runtime version strings."""
    payload = dict(snapshot, provenance=_payload_provenance(snapshot['provenance']))
    encoded = json.dumps(payload, sort_keys=True, separators=(',', ':'), allow_nan=False)
    return hashlib.sha256(encoded.encode()).hexdigest()


def freeze_matrix(directory):
    """Claim a new directory and publish a complete manifest only on success.

    Even an existing empty directory is refused. Failed-case evidence remains
    in its claimed directory, with failed status and no complete manifest.
    """
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=False)
    _write_json(directory/'status.json', {'state': 'building'})
    manifest = dict(
        schema='so101-native-full-geometry-matrix-v2', state='complete',
        principal_axes_policy=oracle.principal_axes_policy(),
        scope='offline retained CPU oracle; no native or physical qualification',
        numeric_absolute_tolerance=1e-9, local_ray_absolute_tolerance=1e-12,
        matrix_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        cases=[])
    current_name = None
    try:
        for name, arguments in cases():
            current_name = name
            snapshot = oracle.freeze(arguments, directory/name)
            # Both hash verification and full report comparison remain the
            # retained oracle's contract, rather than new approximate reducers.
            verified, arrays = oracle.load_verified(directory/name)
            reference = oracle.reference_case(arguments)
            oracle.assert_report_equal(
                reference['report'],
                oracle.plain_values(oracle.restore_arrays(verified['report'], arrays)))
            manifest['cases'].append(dict(
                name=name, inputs=snapshot['inputs'],
                provenance=snapshot['provenance'], arrays=snapshot['arrays'],
                oracle_manifest_sha256=hashlib.sha256(
                    (directory/name/'manifest.json').read_bytes()).hexdigest(),
                oracle_payload_manifest_sha256=_oracle_payload_sha256(verified),
                reference=reference))
        _write_json(directory/'manifest.json', manifest)
        _write_json(directory/'status.json', {'state': 'complete'})
        return manifest
    except Exception as error:
        _write_json(directory/'status.json', dict(
            state='failed', case=current_name,
            error_type=type(error).__name__, error=str(error)))
        raise


def assert_matrix_equal(expected, actual):
    """Compare complete reports at 1e-9 and ordered child/ray evidence at 1e-12.

    Every other field, including array hashes, schema, input/provenance identity
    and case order, is exact. Native reports should use assert_report_equal
    directly; this comparison verifies regenerated frozen CPU evidence.
    """
    assert expected.keys() == actual.keys(), 'matrix schema'
    for key in expected.keys()-{'cases'}:
        oracle.assert_report_equal(expected[key], actual[key], path=key, tolerance=0)
    assert len(expected['cases']) == len(actual['cases']), 'case count'
    for left, right in zip(expected['cases'], actual['cases']):
        assert left.keys() == right.keys(), left['name']
        for key in left.keys()-{'reference'}:
            oracle.assert_report_equal(left[key], right[key],
                                       path=left['name']+'.'+key, tolerance=0)
        assert left['reference'].keys() == right['reference'].keys(), left['name']
        for key in left['reference'].keys()-{'report', 'pieces'}:
            oracle.assert_report_equal(left['reference'][key], right['reference'][key],
                                       path=left['name']+'.reference.'+key, tolerance=0)
        oracle.assert_report_equal(left['reference']['report'], right['reference']['report'])
        oracle.assert_report_equal(left['reference']['pieces'], right['reference']['pieces'],
                                   tolerance=1e-12)


def assert_matrix_payload_equal(expected, actual):
    """Validate frozen payload compatibility without asserting runtime identity.

    Runtime envelopes remain unchanged in the caller's manifests. Comparison
    views omit only the four version strings and, when those differ, the full
    manifest digest derived from them. The full runtime-free snapshot digest is
    mandatory and exact; every other check delegates to strict qualification.
    """
    assert len(expected['cases']) == len(actual['cases']), 'case count'
    left_cases, right_cases = [], []
    for left, right in zip(expected['cases'], actual['cases']):
        for case in (left, right):
            digest = case['oracle_payload_manifest_sha256']
            assert type(digest) is str and len(digest) == 64, 'payload digest'
            assert all(character in '0123456789abcdef' for character in digest), 'payload digest'
        left_provenance = _payload_provenance(left['provenance'])
        right_provenance = _payload_provenance(right['provenance'])
        changed_runtime = any(left['provenance'][field] != right['provenance'][field]
                              for field in _RUNTIME_VERSION_FIELDS)
        left_view, right_view = dict(left), dict(right)
        left_view['provenance'], right_view['provenance'] = left_provenance, right_provenance
        if changed_runtime:
            del left_view['oracle_manifest_sha256']
            del right_view['oracle_manifest_sha256']
        left_cases.append(left_view)
        right_cases.append(right_view)
    assert_matrix_equal(dict(expected, cases=left_cases), dict(actual, cases=right_cases))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True,
                        help='New directory; existing paths are never overwritten')
    args = parser.parse_args()
    manifest = freeze_matrix(args.output)
    print(json.dumps(dict(schema=manifest['schema'], state=manifest['state'],
                          cases=len(manifest['cases']), output=str(args.output))))


if __name__ == '__main__':
    main()
