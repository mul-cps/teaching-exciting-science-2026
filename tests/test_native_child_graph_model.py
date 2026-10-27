"""Offline CHILD graph witness against frozen retained CPU literals.

No splitter, CCL, oracle trace, GPU or fixture writer supplies expectations here.
The only normalization of frozen CCL IDs is per-component minimum pixel + 1.
"""
import json
from pathlib import Path
import re
import shutil
import subprocess

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
FIXTURE = ROOT / 'tests/fixtures/native_geometry/child-evidence-v1.json'
SOURCE = ROOT / 'ros2/src/so101_station_vulkan/test/child_graph_model.cpp'
SHADER = ROOT / 'ros2/src/so101_station_vulkan/shaders/children.slang'
HEADER = ROOT / 'ros2/src/so101_station_vulkan/include/so101_station_vulkan/child_graph.hpp'


def unpack(value):
    dtype = {'bool': np.bool_, 'int32': np.dtype('<i4'), 'uint32': np.dtype('<u4')}[value['dtype']]
    rows = [row for count, row in value['row_runs'] for _ in range(count)]
    result = np.array(rows, dtype=dtype)
    assert list(result.shape) == value['shape']
    return result


def cases():
    return json.loads(FIXTURE.read_text())['cases']


def exact(left, right):
    assert type(left) is type(right)
    if isinstance(left, dict):
        assert left.keys() == right.keys()
        for key in left:
            exact(left[key], right[key])
    elif isinstance(left, list):
        assert len(left) == len(right)
        for a, b in zip(left, right):
            exact(a, b)
    else:
        assert left == right


@pytest.fixture(scope='session')
def model(tmp_path_factory):
    binary = tmp_path_factory.mktemp('child-model') / 'model'
    subprocess.run([shutil.which('g++'), '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror',
                    '-I', str(ROOT/'ros2/src/so101_station_vulkan/include'), str(SOURCE), '-o', str(binary)], check=True)
    return binary


def fixture_input(case, translation=(0, 0)):
    e, c = case['expected'], case['input']
    x, y, w, h = e['tight_global_bbox_xywh']
    x += translation[0]; y += translation[1]
    fw, fh = x+w+1, y+h+1
    parent = np.zeros((fh, fw), dtype=np.uint32)
    depth = np.zeros((fh, fw), dtype='<u4')
    parent[y:y+h, x:x+w] = unpack(e['parent'])
    depth[y:y+h, x:x+w] = unpack(e['depth_bits'])
    return (fw, fh, x, y, w, h, c['min_pixels'], c['max_observation_depth_m'], c['max_neighbor_jump_m']), parent, depth


def execute(model, case, *, translation=(0, 0), rounds=32, records=4096, payload=None, jobs=100000, reverse=0, maximum=None, jump=None, outside_depth=None):
    config, parent, depth = fixture_input(case, translation)
    config = list(config)
    if maximum is not None:
        config[7] = maximum
    if jump is not None:
        config[8] = jump
    if outside_depth is not None:
        depth[parent == 0] = np.array(outside_depth, dtype=np.float32).view(np.uint32)
    payload = parent.size*4 if payload is None else payload
    header = [*config[:7], repr(config[7]), repr(config[8]), rounds, records, payload, jobs, reverse]
    data = ' '.join(map(str, header))+'\n'+' '.join(map(str, parent.ravel()))+'\n'+' '.join(map(str, depth.ravel()))+'\n'
    return json.loads(subprocess.run([str(model)], input=data, text=True, capture_output=True, check=True).stdout)


def expected(case, translation=(0, 0)):
    e = case['expected']
    config, parent, _ = fixture_input(case, translation)
    fw, fh, x, y, w, h = config[:6]
    labels = unpack(e['labels'])
    roots = np.zeros((fh, fw), dtype=np.uint32)
    for component in e['components']:
        member = labels == component['label_id']
        yy, xx = np.where(member)
        canonical = int(((yy+y)*fw+xx+x).min())+1
        roots[y:y+h, x:x+w][member] = canonical
    flags = np.zeros_like(roots)
    flags[y:y+h, x:x+w] = unpack(e['valid']).astype(np.uint32) | unpack(e['cut']).astype(np.uint32)*2
    kept = []
    for piece in e['accepted']:
        component = next(v for v in e['components'] if v['label_id'] == piece['label_id'])
        left, top, rw, rh = piece['local_bounds_xywh']
        region = unpack(piece['region'])
        local_parent = unpack(e['parent'])[top:top+rh, left:left+rw]
        local_cut = unpack(e['cut'])[top:top+rh, left:left+rw]
        nearby = np.zeros_like(region)
        for dy in range(-2, 3):
            for dx in range(-2, 3):
                sy, sx = max(0, -dy), max(0, -dx)
                ey, ex = min(rh, rh-dy), min(rw, rw-dx)
                nearby[sy+dy:ey+dy, sx+dx:ex+dx] |= region[sy:ey, sx:ex]
        packed = region.astype(np.uint32) | local_parent.astype(np.uint32)*2 | nearby.astype(np.uint32)*4 | local_cut.astype(np.uint32)*8 | unpack(piece['support']).astype(np.uint32)*16
        kept.append({'root': int(roots[y:y+h, x:x+w][labels == piece['label_id']][0]),
                     'parent': 0, 'key': component['min_crop_block_key'], 'area': component['area'],
                     'bounds': component['bounds_xywh'], 'halo': [left, top, rw, rh],
                     'origin': [x+left, y+top], 'cut': piece['depth_cut_pixels'],
                     'support': int(unpack(piece['support']).sum()), 'packed': packed.ravel().tolist()})
    return {'usable': True, 'roots': roots.ravel().tolist(), 'flags': flags.ravel().tolist(),
            'stats': [int((unpack(e['valid']) & unpack(e['parent'])).sum()), int(unpack(e['candidate']).sum()), len(e['components']), len(kept)], 'records': kept}


@pytest.mark.parametrize('case', cases(), ids=lambda c: c['id'])
def test_literal_partition_flags_order_halo_support(model, case):
    actual = execute(model, case)
    exact(actual, expected(case))
    exact(execute(model, case, reverse=1), actual)


@pytest.mark.parametrize('translation', [(0, 0), (0, 1), (1, 0), (1, 1)])
def test_accepted_u_true_member_key_at_every_origin(model, translation):
    case = next(c for c in cases() if c['id'] == 'u-membership')
    assert case['expected']['components'][0]['area'] == 12
    assert case['expected']['components'][0]['min_crop_block_key'] != case['expected']['components'][0]['minpixel_transformed_key']
    exact(execute(model, case, translation=translation), expected(case, translation))


@pytest.mark.parametrize('budget', [{'records': 0}, {'payload': 1}, {'jobs': 0}, {'rounds': 1}])
def test_overflow_and_nonconvergence_fail_closed(model, budget):
    case = next(c for c in cases() if c['id'] == 'invalid-interior-hole')
    result = execute(model, case, **budget)
    assert result == {'usable': False}


def test_actual_cold_rne_matches_independent_numpy(model):
    rng = np.random.default_rng(20261023)
    raw = rng.integers(1, 0x7ff0000000000000, size=10000, dtype=np.uint64)
    tie = [2.0**-150, np.nextafter(2.0**-150, np.inf), 2.0**-149,
           1+2.0**-24, 1+3*2.0**-24, 2.0**128-2.0**103,
           .1, np.nextafter(.1, np.inf), 1.2, .012,
           np.finfo(np.float64).max, np.finfo(np.float64).tiny]
    raw = np.concatenate([raw, np.array(tie, dtype=np.float64).view(np.uint64)])
    actual = json.loads(subprocess.run([str(model), '--round'], input=' '.join(map(str, raw)),
                                      text=True, capture_output=True, check=True).stdout)
    with np.errstate(over='ignore', under='ignore'):
        wanted = raw.view(np.float64).astype(np.float32).view(np.uint32).tolist()
    exact(actual, wanted)


def test_exact_comparison_rejects_numeric_and_order_coercion():
    for a, b in [(True, 1), (1, 1.0), ([1, 2], [2, 1])]:
        with pytest.raises(AssertionError):
            exact(a, b)


def test_model_is_linked_to_actual_production_predicates():
    source = SOURCE.read_text()
    for predicate in ['child_f32_rne_bits', 'child_depth_valid', 'child_bad_edge',
                      'child_same_parent', 'child_local_key', 'ChildGraphLayout']:
        assert predicate in source
    assert '#include "so101_station_vulkan/child_graph.hpp"' in source


@pytest.mark.parametrize('maximum,jump', [(1e300, 1e-300), (1e300, 1e300), (.5, .012)])
def test_original_finite_double_config_allows_rounded_zero_infinity_and_equality(model, maximum, jump):
    case = next(c for c in cases() if c['id'] == 'u-membership')
    exact(execute(model, case, maximum=maximum, jump=jump), expected(case))


def test_valid_flags_are_global_but_cut_and_support_use_selected_parent(model):
    case = next(c for c in cases() if c['id'] == 'u-membership')
    wanted = expected(case)
    _, parent, _ = fixture_input(case)
    flags = np.array(wanted['flags'], dtype=np.uint32).reshape(parent.shape)
    flags[parent == 0] = 1
    wanted['flags'] = flags.ravel().tolist()
    exact(execute(model, case, outside_depth=.5), wanted)


def function_body(source, name):
    """Extract actual balanced source body, discarding only whitespace/comments."""
    source = re.sub(r'//[^\n]*|/\*.*?\*/', '', source, flags=re.S)
    match = re.search(r'\b'+re.escape(name)+r'\([^)]*\)\s*\{', source)
    assert match, name
    start, depth, end = match.end(), 1, match.end()
    while depth and end < len(source):
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    assert depth == 0
    return re.sub(r'\s+', '', source[start:end-1])


def shader_predicate_contract(shader, header):
    """Fail on semantic source drift; this is not compilation/execution of Slang.

    Guards cover the exact production predicates used by the compiled shared
    helpers and the bounded graph transcription. Changes require reviewing this
    finite witness, rather than silently keeping a disconnected test algorithm.
    """
    body = lambda name: function_body(shader, name)
    host = lambda name: function_body(header, name)
    assert body('validDepth') == 'floatz=asfloat(source.Load(4*(p.count+i)));returnisfinite(z)&&z>=asfloat(0x3dcccccdu)&&z<=asfloat(p.maxBits);'
    assert host('child_depth_valid') == 'floatz=child_float(depth_bits);returnstd::isfinite(z)&&z>=child_float(0x3dcccccdu)&&z<=child_float(maximum_bits);'
    assert body('badEdge') == 'returnabs(asfloat(source.Load(4*(p.count+a)))-asfloat(source.Load(4*(p.count+b))))>asfloat(p.jumpBits);'
    assert host('child_bad_edge') == 'returnstd::fabs(child_float(left_bits)-child_float(right_bits))>child_float(jump_bits);'
    assert body('localKey') == 'return((y-py)/2)*((width+1)/2)+(x-px)/2;'
    assert host('child_local_key') == 'return((y-parent_y)/2)*((parent_width+1)/2)+(x-parent_x)/2;'
    assert host('child_same_parent') == 'returna!=0&&a==b;'
    init = body('child_init')
    assert 'if(par!=0&&valid){' in init
    for neighbor, boundary in [('i-1', 'x>0'), ('i+1', 'x+1<p.width'),
                               ('i-p.width', 'y>0'), ('i+p.width', 'y+1<p.height')]:
        assert f'if({boundary}&&selectedParent({neighbor})==par&&validDepth({neighbor})&&badEdge(i,{neighbor}))cut=true;' in init
    assert 'ps(p.count+i,uint(valid)|(uint(cut)<<1));uintroot=par!=0&&valid&&!cut?i+1:0;' in init
    assert 'if(par!=0&&valid){uintslot=hashSlot(par);' in init
    assert 'if(!cut)InterlockedAdd(sums2[slot],1,ignored);' in init
    edge = body('childEdge')
    assert 'uintother=wl(p.count+j);if(other==0||selectedParent(j)!=par)return;' in edge
    assert 'work.InterlockedMin(4*(max(root,other)-1),min(root,other),ignored);' in edge
    hook, jump = body('child_hook'), body('child_jump')
    assert 'uintroot=wl(p.count+i);' in hook
    assert 'work.InterlockedMin(4*i,root,ignored);' in hook
    for neighbor in ['i-1', 'i-p.width', 'i-p.width-1', 'i-p.width+1']:
        assert f'childEdge(i,{neighbor},par,root);' in hook
    assert 'uinta=wl(i);' in jump and 'uintnext=wl(a-1);' in jump
    assert 'ws(p.count+i,next);' in jump
    assert 'a>i+1||selectedParent(a-1)!=selectedParent(i)' in jump
    assert 'next==0||next>a||selectedParent(next-1)!=selectedParent(i)' in jump
    for name in ['child_hook', 'childEdge', 'child_jump', 'child_fill_hook', 'fillEdge', 'child_fill_jump']:
        assert not re.search(r'\b(for|while|do)\b', function_body(shader, name)), name
    reduction = body('child_reduce')
    assert 'uintroot=pl(i);if(root!=0){uintpar=selectedParent(i),x=i%p.width,y=i/p.width;' in reduction
    assert 'uintkey=localKey(x,y,px,py,pw),slot=hashSlot(root);' in reduction
    assert 'InterlockedMin(keys[slot],key,ignored);' in reduction
    assert 'work.InterlockedMin(4*(7*p.count+j),keys[k],ignored);' in reduction
    assert reduction.index('uintkey=localKey') < reduction.index('if(root==i+1)')
    validate = body('child_validate')
    assert 'wl(p.count+root-1)!=root' in validate
    for neighbor in ['i-1', 'i-p.width', 'i-p.width-1', 'i-p.width+1']:
        assert f'validateChildEdge(i,{neighbor},par,root);' in validate
    assert 'root!=other)fault(4);' in body('validateChildEdge')
    fill = body('child_packed_init')
    assert 'uinta=region?0:i+1;' in fill  # complement includes other parent/child/hole pixels
    assert 'boolcut=ownParent&&(pl(p.count+global)&2)!=0;' in fill
    assert 'if(local%width>0)fillEdge(i,i-1,root,begin,end);if(local>=width)fillEdge(i,i-width,root,begin,end);' in body('child_fill_hook')
    assert 'uintbegin=recordWord(r,12),end=begin+recordWord(r,10)*recordWord(r,11);' in body('child_fill_jump')
    assert 'a<=begin||a>end||a>i+1' in body('child_fill_jump')


def test_actual_shader_and_shared_helper_predicates_stay_linked():
    shader_predicate_contract(SHADER.read_text(), HEADER.read_text())


@pytest.mark.parametrize('old,new', [
    ('isfinite(z)&&', ''),
    ('z<=asfloat(p.maxBits)', 'z<asfloat(p.maxBits)'),
    ('>asfloat(p.jumpBits)', '>=asfloat(p.jumpBits)'),
    ('selectedParent(j)!=par', 'false'),
    ('if(x>0&&selectedParent(i-1)==par', 'if(false&&selectedParent(i-1)==par'),
    ('((y-py)/2)*((width+1)/2)+(x-px)/2', '(y/2)*((p.width+1)/2)+x/2'),
    ('uint key=localKey(x,y,px,py,pw)', 'uint key=localKey((root-1)%p.width,(root-1)/p.width,px,py,pw)'),
    ('uint next=wl(a-1);', 'uint next=a;'),
    ('work.InterlockedMin(4*i,root,ignored);', 'work.Store(4*i,root);'),
    ('uint a=region?0:i+1;', 'uint a=region||!ownParent?0:i+1;'),
    ('bool cut=ownParent&&', 'bool cut='),
    ('a<=begin||a>end||a>i+1', 'a>i+1'),
])
def test_shader_predicate_mutants_are_rejected(old, new):
    shader = SHADER.read_text()
    assert old in shader
    with pytest.raises(AssertionError):
        shader_predicate_contract(shader.replace(old, new, 1), HEADER.read_text())


@pytest.mark.parametrize('old,new', [
    ('std::isfinite(z)&&', ''),
    ('>child_float(jump_bits)', '>=child_float(jump_bits)'),
    ('((y-parent_y)/2)*((parent_width+1)/2)+(x-parent_x)/2', '(y/2)*((parent_width+1)/2)+x/2'),
])
def test_shared_helper_predicate_mutants_are_rejected(old, new):
    header = HEADER.read_text()
    assert old in header
    with pytest.raises(AssertionError):
        shader_predicate_contract(SHADER.read_text(), header.replace(old, new, 1))
