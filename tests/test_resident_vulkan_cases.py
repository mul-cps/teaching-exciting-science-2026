"""Procedural fixtures retain the measured failed baseline's exact byte hashes."""
import importlib.util
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
spec = importlib.util.spec_from_file_location('resident_cases', ROOT/'tools/benchmark_resident_rgbd_matrix.py')
cases = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cases)


def test_procedural_bytes_match_failed_seven_stage_baseline():
    qualification = json.loads((ROOT/'docs/resident-vulkan-qualification-20261022.json').read_text())
    frozen = {row['case']: row['input_sha256'] for row in qualification['matrix']['amd']}
    generated = {}
    for name, rgb, raw, _ in cases.procedural_cases():
        generated[name] = cases.source_hashes(rgb, raw)
        if name in frozen:
            assert generated[name] == frozen[name], name
    assert set(frozen)-{'recorded'} <= generated.keys()
    assert generated['overflow_large'] == {
        'rgb.bin': '21d53e92296aa76ad5507bb2d7a89eb51d99436994d5f60d9b6ae2435abf4c03',
        'depth.bin': '49496d431989a7eb16bd26a119dd4b5ff21e3c61c79754a626cc1bce2d4e4459'}


def test_prepare_is_deterministic_and_omits_private_recording(tmp_path):
    names = cases.prepare(tmp_path)
    assert 'recorded' not in names
    assert len(names) == 8
    first = {name: (tmp_path/name/'case.json').read_text() for name in names}
    assert cases.prepare(tmp_path) == names
    assert first == {name: (tmp_path/name/'case.json').read_text() for name in names}
