"""CPU-only contract for a disposable child_hook fault sampler; no GPU execution."""
import hashlib
import importlib.util
import json
from pathlib import Path
import re

import pytest

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'ros2/src/so101_station_vulkan/shaders/children.slang'
SOURCE_SHA256 = '8412aec1442aed563f99581a11bd3105db60b60947b5909ccf4e2e57ec0fe0ec'


@pytest.fixture
def probe():
    path = ROOT / 'tools/performance/child_hook_probe.py'
    assert path.is_file(), f'missing CPU-only probe utility: {path}'
    spec = importlib.util.spec_from_file_location('child_hook_probe', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def digest(data):
    return hashlib.sha256(data).hexdigest()


def test_prepare_exact_reversible_patch_and_manifest(probe, tmp_path):
    before = SOURCE.read_bytes()
    assert digest(before) == SOURCE_SHA256
    destination = tmp_path / 'probe'
    manifest = probe.prepare(SOURCE, destination)
    assert SOURCE.read_bytes() == before
    derived = (destination / 'children.slang').read_bytes()
    assert json.loads((destination / 'manifest.json').read_text()) == manifest
    assert manifest['scope'] == 'child_hook_fault_sample_only'
    assert manifest['qualifying'] is False
    assert manifest['original_source_sha256'] == SOURCE_SHA256
    assert manifest['derived_source_sha256'] == digest(derived)

    # Everything outside this helper insertion and one call replacement is exact.
    insertion = before.index(b'[numthreads(256,1,1)]\nvoid child_hook(')
    hook_end = before.index(b'\n', insertion + len(b'[numthreads(256,1,1)]\n'))
    original_hook = before[insertion:hook_end]
    patched_hook = original_hook.replace(b'fault(4)', b'hookFaultSample(i,root,par)')
    assert original_hook.count(b'fault(4)') == 1
    assert derived.count(b'hookFaultSample(i,root,par)') == 1
    assert derived.startswith(before[:insertion])
    helper_end = derived.index(patched_hook, insertion)
    helper = derived[insertion:helper_end]
    assert derived[:insertion] + derived[helper_end:].replace(
        patched_hook, original_hook, 1
    ) == before
    assert before.count(b'fault(4)') == 8
    assert (derived[:insertion] + derived[helper_end:]).count(b'fault(4)') == 7

    compact = re.sub(rb'\s+', b'', helper)
    assert b'voidhookFaultSample(uinti,uintroot,uintpar)' in compact
    assert b'0x80000004u|((i&255u)<<7)|((root&255u)<<15)|((par&63u)<<23)|' in compact
    assert b'(uint(par>63u)<<29)|(uint(root>255u)<<30)' in compact
    assert compact.count(b'InterlockedCompareExchange(') == 1
    assert b'compact.InterlockedCompareExchange(28,0u,payload,ignored);' in compact
    assert compact.count(b'fault(4);') == 1
    assert compact.index(b'InterlockedCompareExchange(') < compact.index(b'fault(4);')
    assert b'Load' not in helper
    assert b'selectedParent' not in helper
    assert re.search(rb'\b(?:wl|pl|hl|candidate)\s*\(', helper) is None


def test_prepare_refuses_existing_destination_without_changes(probe, tmp_path):
    destination = tmp_path / 'existing'
    destination.mkdir()
    sentinel = destination / 'sentinel'
    sentinel.write_bytes(b'keep exactly')
    before = SOURCE.read_bytes()
    with pytest.raises(FileExistsError):
        probe.prepare(SOURCE, destination)
    assert SOURCE.read_bytes() == before
    assert list(destination.iterdir()) == [sentinel]
    assert sentinel.read_bytes() == b'keep exactly'


def test_prepare_rejects_modified_source(probe, tmp_path):
    source = tmp_path / 'children.slang'
    modified = SOURCE.read_bytes() + b'\n// modified\n'
    source.write_bytes(modified)
    destination = tmp_path / 'rejected'
    with pytest.raises(ValueError):
        probe.prepare(source, destination)
    assert source.read_bytes() == modified
    assert not destination.exists()


@pytest.mark.parametrize('word,expected', [
    (0x80000004, (0, 0, 0, False, False)),
    (0x80008104, (2, 1, 0, False, False)),
    (0x81839284, (37, 7, 3, False, False)),
    (0x9FFFFF84, (255, 255, 63, False, False)),
    (0xA0000004, (0, 0, 0, False, True)),
    (0xC0000004, (0, 0, 0, True, False)),
    (0xE0001184, (35, 0, 0, True, True)),
    (0x818392C5, (37, 7, 3, False, False)),
])
def test_decode_independent_encodings_and_observed_anomalies(probe, word, expected):
    decoded = probe.decode(word)
    assert decoded['raw_word'] == word
    assert decoded['low_fault_bits'] == (word & 127)
    assert decoded['captured'] is True
    assert tuple(decoded[key] for key in (
        'i', 'root_mod_256', 'parent_mod_64', 'root_overflow', 'parent_overflow'
    )) == expected


@pytest.mark.parametrize('word', [0, 4, 5, 68, 127, 0x7FFFFFFF])
def test_decode_absent_marker_has_no_sample_values(probe, word):
    decoded = probe.decode(word)
    assert decoded['raw_word'] == word
    assert decoded['low_fault_bits'] == (word & 127)
    assert decoded['captured'] is False
    assert not ({'i', 'lane', 'root', 'parent', 'root_mod_256', 'parent_mod_64',
                 'root_overflow', 'parent_overflow'} & decoded.keys())


@pytest.mark.parametrize('word', [-1, 2**32, True, False, 4.0, '4', None])
def test_decode_rejects_non_uint32(probe, word):
    with pytest.raises((TypeError, ValueError)):
        probe.decode(word)
