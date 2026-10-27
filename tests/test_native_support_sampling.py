"""Cold sampling ABI and retained CPU stream equivalence."""
import hashlib
import json
import struct
from pathlib import Path

import numpy as np
import pytest


def api():
    from tools import prepare_support_sampling
    return prepare_support_sampling


@pytest.fixture(scope='module')
def full(tmp_path_factory):
    p = tmp_path_factory.mktemp('sampling')/'controls.bin'
    manifest = api().build_sampling_artifact(p, 4800)
    return p, manifest, api().read_sampling_artifact(p)


def test_exhaustive_independent_streams(full):
    _, _, table = full
    for p in range(4801):
        expected = np.arange(p) if p <= 2000 else np.random.default_rng(17).choice(p, 2000, replace=False)
        np.testing.assert_array_equal(api().selection_indices(table, p), expected)
    for m in range(2001):
        rng = np.random.default_rng(17)
        valid = np.flatnonzero(np.ones(m, dtype=bool))
        expected = np.array([rng.choice(valid, 3, replace=False) for _ in range(120)], dtype=np.uint32) if m >= 30 else np.empty((0, 3), dtype=np.uint32)
        np.testing.assert_array_equal(api().proposal_indices(table, m), expected)


def test_actual_retained_cpu_choices(full):
    from tools.native_support_oracle import support_cases, trace_support_case
    table = full[2]
    cases = {c['name']: c for c in support_cases()}
    for name in ('population-2001', 'population-4800', 'population-30', 'flat-equal-score'):
        trace = trace_support_case(cases[name])
        p = len(cases[name]['points'])
        np.testing.assert_array_equal(api().selection_indices(table, p), trace['selected_indices'])
        np.testing.assert_array_equal(api().proposal_indices(table, min(p, 2000)), trace['rng'][-1]['choices'])


def test_abi_stability_and_readonly_ownership(full, tmp_path):
    p, manifest, table = full
    other = tmp_path/'other.bin'
    second = api().build_sampling_artifact(other, 4800)
    assert p.read_bytes() == other.read_bytes()
    assert manifest == second
    h = struct.unpack('<16I', p.read_bytes()[:64])
    assert h == (0x31545053, 1, 4800, 2000, 120, 30, 4802, 2002, 5600000, 709560,
                 64, 19272, 27280, 22427280, 25265520, 0)
    assert manifest['artifact_sha256'] == hashlib.sha256(p.read_bytes()).hexdigest()
    assert manifest['payload_bytes'] == 25238240
    for key in ('selection_offsets', 'proposal_offsets', 'selection', 'proposals'):
        a = table[key]
        assert not a.flags.writeable
        with pytest.raises(ValueError):
            a.setflags(write=True)
    owned = api().read_sampling_artifact(other)
    other.unlink()
    assert api().proposal_indices(owned, 30).shape == (120, 3)


@pytest.mark.parametrize('value', [True, False, 0, -1, 4801, 2**40, 1.5, '30'])
def test_invalid_configuration(tmp_path, value):
    with pytest.raises((ValueError, TypeError)):
        api().build_sampling_artifact(tmp_path/'bad.bin', value)
    assert not list(tmp_path.iterdir())


@pytest.mark.parametrize('g', [1, 29, 30, 2000, 2001])
def test_small_ranges(tmp_path, g):
    p = tmp_path/'small.bin'
    api().build_sampling_artifact(p, g)
    t = api().read_sampling_artifact(p)
    assert len(api().selection_indices(t, g)) == min(g, 2000)
    for bad in (-1, g+1, True):
        with pytest.raises((ValueError, TypeError)):
            api().selection_indices(t, bad)
    with pytest.raises(ValueError):
        api().proposal_indices(t, min(g, 2000)+1)


@pytest.mark.parametrize('kind', ['header', 'offset', 'selection-range', 'selection-duplicate', 'proposal-range', 'triplet', 'truncated', 'trailing', 'digest', 'missing-manifest'])
def test_corruption(full, tmp_path, kind):
    original = full[0]
    p = tmp_path/'corrupt.bin'
    data = bytearray(original.read_bytes())
    h = struct.unpack('<16I', data[:64])
    if kind == 'header':
        struct.pack_into('<I', data, 12, 1999)
    elif kind == 'offset':
        struct.pack_into('<I', data, 64+4*2001, 1)
    elif kind.startswith('selection'):
        offset = h[12]
        value = 5000 if kind.endswith('range') else struct.unpack_from('<I', data, offset+4)[0]
        struct.pack_into('<I', data, offset, value)
    elif kind in ('proposal-range', 'triplet'):
        offset = h[13]
        value = 30 if kind.endswith('range') else struct.unpack_from('<I', data, offset+4)[0]
        struct.pack_into('<I', data, offset, value)
    elif kind == 'truncated':
        data.pop()
    elif kind == 'trailing':
        data.extend(b'x')
    p.write_bytes(data)
    manifest = dict(full[1])
    manifest['artifact_sha256'] = '0'*64 if kind == 'digest' else hashlib.sha256(data).hexdigest()
    if kind != 'missing-manifest':
        p.with_suffix(p.suffix+'.json').write_text(json.dumps(manifest))
    with pytest.raises((ValueError, FileNotFoundError)):
        api().read_sampling_artifact(p)


def test_refuse_overwrite_and_atomic_failure(tmp_path, monkeypatch):
    p = tmp_path/'controls.bin'
    api().build_sampling_artifact(p, 30)
    before = p.read_bytes()
    with pytest.raises(FileExistsError):
        api().build_sampling_artifact(p, 30)
    assert p.read_bytes() == before
    q = tmp_path/'failed.bin'
    import os
    original = os.link
    def fail_manifest(source, destination):
        if str(destination).endswith('.json'):
            raise OSError('injected publish failure')
        return original(source, destination)
    monkeypatch.setattr(os, 'link', fail_manifest)
    with pytest.raises(OSError):
        api().build_sampling_artifact(q, 30)
    assert not q.exists()
    assert not list(tmp_path.glob('*.pending'))


@pytest.mark.parametrize('word', range(16))
def test_every_header_word_is_checked(tmp_path, word):
    p = tmp_path/'header.bin'
    api().build_sampling_artifact(p, 30)
    data = bytearray(p.read_bytes())
    value = struct.unpack_from('<I', data, word*4)[0]
    struct.pack_into('<I', data, word*4, value+1)
    p.write_bytes(data)
    metadata = json.loads(p.with_suffix('.bin.json').read_text())
    metadata['artifact_sha256'] = hashlib.sha256(data).hexdigest()
    p.with_suffix('.bin.json').write_text(json.dumps(metadata))
    with pytest.raises(ValueError):
        api().read_sampling_artifact(p)


def test_missing_artifact(tmp_path):
    with pytest.raises(FileNotFoundError):
        api().read_sampling_artifact(tmp_path/'missing.bin')


@pytest.mark.parametrize('peer_part', ['binary', 'manifest', 'both'])
def test_peer_arrives_after_precheck_is_preserved(tmp_path, monkeypatch, peer_part):
    p = tmp_path/'race.bin'
    manifest = p.with_suffix('.bin.json')
    original = api()._sources

    def peer_during_construction():
        if peer_part in ('binary', 'both'):
            p.write_bytes(b'peer binary')
        if peer_part in ('manifest', 'both'):
            manifest.write_bytes(b'peer manifest')
        return original()

    monkeypatch.setattr(api(), '_sources', peer_during_construction)
    with pytest.raises(FileExistsError):
        api().build_sampling_artifact(p, 30)
    if peer_part in ('binary', 'both'):
        assert p.read_bytes() == b'peer binary'
    else:
        assert not p.exists()
    if peer_part in ('manifest', 'both'):
        assert manifest.read_bytes() == b'peer manifest'
    assert not list(tmp_path.glob('*.pending'))


def test_failed_publication_preserves_replaced_peer_artifact(tmp_path, monkeypatch):
    import os
    p = tmp_path/'race.bin'
    manifest = p.with_suffix('.bin.json')
    # Exercise both the old replace publisher (RED) and no-clobber publisher.
    for operation in ('replace', 'link'):
        original = getattr(os, operation)
        def interleave(source, destination, _original=original):
            if Path(destination) == manifest:
                p.unlink()
                p.write_bytes(b'peer binary')
                manifest.write_bytes(b'peer manifest')
                raise OSError('peer arrived before manifest failure')
            return _original(source, destination)
        monkeypatch.setattr(os, operation, interleave)
    with pytest.raises(OSError):
        api().build_sampling_artifact(p, 30)
    assert p.read_bytes() == b'peer binary'
    assert manifest.read_bytes() == b'peer manifest'
    assert not list(tmp_path.glob('*.pending'))
