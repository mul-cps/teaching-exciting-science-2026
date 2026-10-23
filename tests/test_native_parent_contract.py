"""CPU graph and literal GRANA witnesses; never loads Vulkan or rewrites goldens."""
import json
from pathlib import Path
import subprocess
import pytest

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT/'ros2/src/so101_station_vulkan'
FIXTURE = ROOT/'tests/fixtures/native_geometry/parent-order-v1.json'

@pytest.fixture(scope='module')
def graph_probe(tmp_path_factory):
    output = tmp_path_factory.mktemp('parent-graph')/'probe'
    subprocess.run(['g++', '-std=c++17', '-O2', '-UNDEBUG', '-DPARENT_GRAPH_ONLY',
                    '-I'+str(PACKAGE/'include'), str(PACKAGE/'test/parent_preparation_contract.cpp'),
                    '-o', str(output)], check=True, capture_output=True, text=True)
    return output


def run_graph(probe, rows, minimum=1, capacity=4096, rounds=32, reverse=False):
    # Source is literal original color bit, independent of any depth field.
    width, height = len(rows[0]), len(rows)
    payload = f'{width} {height} {minimum} {capacity} {rounds}\n'
    payload += ' '.join(str(int(x, 16)) for row in rows for x in row)+'\n'
    return json.loads(subprocess.run([str(probe), '--graph-reverse' if reverse else '--graph'], input=payload,
                                    check=True, capture_output=True, text=True).stdout)

@pytest.mark.parametrize('case', json.loads(FIXTURE.read_text())['cases'], ids=lambda c:c['id'])
def test_literal_full_maps_and_ordered_statistics(graph_probe, case):
    actual = run_graph(graph_probe, case['mask_rows'], case['min_component_pixels'])
    expected = case['expected']
    assert actual['fault'] == 0
    assert actual['labels'] == expected['labels']
    assert run_graph(graph_probe, case['mask_rows'], case['min_component_pixels'], reverse=True) == actual
    assert [{k:c[k] for k in ('label','area','bounds_xywh','min_flat_pixel','min_block_key')}
            for c in actual['components']] == expected['components']
    assert [c['label'] for c in actual['records']] == expected['kept_label_ids']


def test_segmented_colors_key_zero_and_odd_dimensions(graph_probe):
    result = run_graph(graph_probe, ['12480','00000','10002'])
    assert result['fault'] == 0
    assert result['labels'] == [[1,1,1,1,0],[0,0,0,0,0],[2,0,0,0,2]]
    assert [(c['color'],c['label'],c['min_block_key']) for c in result['records']] == [
        (0,1,0),(0,2,3),(1,1,0),(1,2,5),(2,1,1),(3,1,1)]


def test_occupied_blocks_are_not_automatically_connected(graph_probe):
    assert run_graph(graph_probe, ['1001'])['labels'] == [[1,0,0,2]]
    assert run_graph(graph_probe, ['0100','0010'])['labels'] == [[0,1,0,0],[0,0,1,0]]
    assert run_graph(graph_probe, ['1000','0000','0000','1000'])['labels'] == [[1,0,0,0],[0,0,0,0],[0,0,0,0],[2,0,0,0]]


def test_overflow_and_nonconvergence_are_explicit(graph_probe):
    result = run_graph(graph_probe, ['3'])
    assert result['fault'] & 1 and result['records'] == []
    result = run_graph(graph_probe, ['10101'], capacity=2)
    assert result['fault'] & 2
    assert result['records'] == []
    result = run_graph(graph_probe, ['11111'], rounds=0)
    assert result['fault'] & 4
    assert result['records'] == []


def test_literal_mutation_is_rejected(graph_probe):
    case=json.loads(FIXTURE.read_text())['cases'][0]
    result=run_graph(graph_probe, case['mask_rows'])
    changed=json.loads(json.dumps(case['expected']))
    changed['labels'][0][2]=1
    assert result['labels'] != changed['labels']
    result=run_graph(graph_probe, ['1001'])
    assert result['labels'] != [[1,0,0,1]]  # rejects occupied-neighbor mutation


def test_shader_and_host_contract_are_bounded():
    shader=(PACKAGE/'shaders/parents.slang').read_text()
    source=(PACKAGE/'src/parents.cpp').read_text()
    assert 'source.Load(4*i)' in shader
    for forbidden in ('double', 'uint64_t', 'asfloat', 'source.Load(4*(p.count'):
        assert forbidden not in shader
    submit=source.split('ParentLease ParentPreparation::submit(',1)[1].split('bool ParentPreparation::ready(',1)[0]
    for forbidden in ('vkWait', 'vkAllocate', '.create(', 'vkMapMemory', 'new ', 'std::vector', 'make_shared'):
        assert forbidden not in submit
    assert 'belongs_to(s.source_token)' in submit
    assert 'source.calibration.generation!=s.config.calibration_generation' in submit
    assert 'source.metadata.calibration_generation!=s.config.calibration_generation' in submit
    assert '.arena.mapped' not in source
    assert 'for(unsigned round=0;round<32;round++)' in submit
    assert 'parents_validate' in source


def test_all_1024_block_edges_against_pixel_distances(graph_probe):
    actual=subprocess.run([str(graph_probe), '--edges'], check=True, text=True, capture_output=True).stdout.strip()
    expected=[]
    for dx,dy in ((-2,0),(0,-2),(-2,-2),(2,-2)):
        for a in range(16):
            for b in range(16):
                connected=any(max(abs((i%2)-(j%2+dx)),abs((i//2)-(j//2+dy)))<=1
                              for i in range(4) for j in range(4) if a&(1<<i) and b&(1<<j))
                expected.append(str(int(connected)))
    assert actual == ''.join(expected)


def test_wrong_occupied_edge_implementation_fails_literal_contract(tmp_path):
    include=tmp_path/'so101_station_vulkan'
    include.mkdir()
    header=(PACKAGE/'include/so101_station_vulkan/parent_graph.hpp').read_text()
    assert 'return (current&5)&&(predecessor&10);' in header
    (include/'parent_graph.hpp').write_text(header.replace(
        'return (current&5)&&(predecessor&10);','return current&&predecessor;'))
    probe=tmp_path/'mutant'
    subprocess.run(['g++','-std=c++17','-O2','-DPARENT_GRAPH_ONLY','-I'+str(tmp_path),
                    str(PACKAGE/'test/parent_preparation_contract.cpp'),'-o',str(probe)],
                   check=True,capture_output=True,text=True)
    assert run_graph(probe, ['1001'])['labels'] != [[1,0,0,2]]


def test_fused_atomic_compression_reduces_chain_reads_and_rejects_plain_store(graph_probe):
    proof=json.loads(subprocess.run([str(graph_probe),'--walks'],check=True,text=True,capture_output=True).stdout)
    assert proof['fused_A_reads'] < proof['baseline_A_reads']/4
    assert proof['reverse_fused_A_reads'] == proof['fused_A_reads']
    assert proof['plain_store_mutant_rejected']
    shader=(PACKAGE/'shaders/parents.slang').read_text()
    hook=shader.split('void parents_hook(',1)[1].split('void parents_flatten(',1)[0]
    assert 'arena.InterlockedMin(4*(p.nodes+j),loadWord(2*p.nodes+j),ignored)' in hook
    assert 'storeWord(' not in hook and '.Store(' not in hook


@pytest.mark.parametrize('minimum', [30, 0xffffffff])
def test_above_image_area_threshold_keeps_full_parent_ids(graph_probe, minimum):
    import importlib.util
    import numpy as np
    spec=importlib.util.spec_from_file_location('parent_order_threshold_reference', ROOT/'tools/native_geometry_parent_order.py')
    oracle=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(oracle)
    expected=oracle.trace_parent_mask(np.ones((2,2),np.uint8), minimum)
    actual=run_graph(graph_probe, ['11','11'], minimum=minimum)
    assert expected['labels'] == actual['labels'] == [[1,1],[1,1]]
    assert len(actual['components']) == len(expected['components']) == 1
    assert actual['fault'] == 0
    assert expected['kept_label_ids'] == actual['records'] == []
