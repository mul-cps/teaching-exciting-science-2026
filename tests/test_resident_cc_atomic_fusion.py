"""Exact CPU-only contracts for active-pixel atomic flattening plus boundary hooks."""
import itertools
from pathlib import Path

import numpy as np
import pytest

from resident_cc_copyback_model import graph, reference, solve
from test_resident_cc_copyback_model import scene

PACKAGE=Path(__file__).resolve().parents[1]/'ros2/src/so101_station_vulkan'


def function(text, symbol):
    begin=text.index('{',text.index(symbol));end=begin+1;level=1
    while level:
        level+=(text[end]=='{')-(text[end]=='}');end+=1
    return text[begin:end]


def test_active_flatten_is_atomic_before_boundary_lane_early_return():
    code=function((PACKAGE/'shaders/resident.slang').read_text(),'void connect(')
    assert 'scratch.InterlockedMin(parentOffset(i),a,ignored)' in code
    assert code.index('canonicalParent(i)')<code.index('if(a==0)return')
    assert code.index('if(a==0)return')<code.index('scratch.InterlockedMin(parentOffset(i),a,ignored)')
    assert code.index('scratch.InterlockedMin(parentOffset(i),a,ignored)')<code.index('if((x&15u)')
    assert code.count('scratch.InterlockedMin(')==2
    for forbidden in ('forestParent(', 'root(', 'scratch.Store(', 'scratch.Load(parentOffset('):
        assert forbidden not in code
    # The fused write must not weaken which original edges may be joined.
    for gate in ('if(dy==0&&dx!=1)continue',
                 'nx<0||ny<0||nx>=int(p.width)||ny>=int(p.height)',
                 'uint(nx)/16==x/16&&uint(ny)/16==y/16',
                 'if(mask(j)!=m)continue', 'if(b!=0&&a!=b)'):
        assert gate in code
    assert 'scratch.InterlockedMin(parentOffset(max(a,b)-1),min(a,b),ignored)' in code


def test_submission_has_no_parent_copy_or_extra_copy_barriers():
    source=(PACKAGE/'src/gpu_pipeline.cpp').read_text()
    submit=function(source,'void Pipeline::submit(')
    assert 'copy_canonical_parents' not in submit and 'parent_copy' not in submit
    assert 'for(unsigned i=0;i<32;i++){dispatch(3);dispatch(2);}dispatch(3)' in submit
    assert submit.count('vkCmdCopyBuffer(')==3  # upload, compact, explicit diagnostics only
    assert 'vkCmdCopyBuffer(s.command,s.scratch.buffer,s.scratch.buffer' not in submit
    assert submit.count('VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,VK_PIPELINE_STAGE_TRANSFER_BIT,')==0
    assert 'VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT|VK_PIPELINE_STAGE_TRANSFER_BIT,VK_PIPELINE_STAGE_TRANSFER_BIT' in submit


def snapshot(forest):
    """Independent bounded monotone-forest oracle; invalid walks stay rejected."""
    roots=[];fault=0
    for first in forest:
        value=first
        for _ in range(len(forest)):
            if value==0:break
            if value>len(forest):fault|=4;value=0;break
            following=forest[value-1]
            if following==value:break
            if following==0 or following>=value:fault|=4;value=0;break
            value=following
        else:fault|=4;value=0
        roots.append(value)
    return roots,fault


def operations(roots,boundary):
    result=[(index,value) for index,value in enumerate(roots) if value]
    for left,right in boundary:
        a,b=roots[left],roots[right]
        if a and b and a!=b:result.append((max(a,b)-1,min(a,b)))
    return result


def bulk_round(roots,boundary):
    result=list(roots)
    for left,right in boundary:
        a,b=roots[left],roots[right]
        if a and b and a!=b:result[max(a,b)-1]=min(result[max(a,b)-1],min(a,b))
    return result


def fused_solve(initial,edges,boundary,order='forward',rounds=32):
    forest=list(initial);fault=0;rng=np.random.default_rng(4917)
    for _ in range(rounds):
        roots,error=snapshot(forest);fault|=error;ops=operations(roots,boundary)
        if order=='reverse':ops.reverse()
        elif order=='shuffled':rng.shuffle(ops)
        for index,value in ops:forest[index]=min(forest[index],value)
    roots,error=snapshot(forest);fault|=error
    if any(roots[a] and roots[b] and roots[a]!=roots[b] for a,b in edges):fault|=1
    return {'roots':roots,'fault':fault}


@pytest.mark.parametrize('forest,boundary',[
    ([1,2,3],[(0,1),(1,2)]),
    ([1,1,3,3],[(1,2)]),
    ([0,2,2],[]),
    ([1,2,3],[(0,1),(0,1),(1,2)]),
])
def test_every_atomic_interleaving_matches_serial_copy_then_hooks(forest,boundary):
    roots,fault=snapshot(forest);assert fault==0
    expected=bulk_round(roots,boundary);ops=operations(roots,boundary)
    for sequence in itertools.permutations(ops):
        actual=list(forest)
        for index,value in sequence:actual[index]=min(actual[index],value)
        assert actual==expected
    # General per-address proof: all proposals are fixed B-derived values;
    # min is associative/commutative/idempotent and B[i] <= initial A[i].
    for i,value in enumerate(forest):
        proposals=[candidate for index,candidate in ops if index==i]
        assert min([value]+proposals)==expected[i]


@pytest.mark.parametrize('name',['flat','holes','rings','checker','split','snake','diagonal'])
@pytest.mark.parametrize('order',['forward','reverse','shuffled'])
def test_full_minimum_roots_equal_old_copyback_and_independent_floodfill(name,order):
    colors,active=scene(name)
    forest,edges,boundary=graph(colors,active)
    old=solve(forest,edges,boundary,copyback=True)
    fused=fused_solve(forest,edges,boundary,order)
    assert old['fault']==fused['fault']==0
    assert old['roots']==fused['roots']==reference(colors,active)


@pytest.mark.parametrize('width,height',[(1,1),(1,33),(17,19),(33,31)])
def test_partial_tiles_random_colors_repeated_edges(width,height):
    rng=np.random.default_rng(width*100+height)
    for _ in range(4):
        colors=rng.integers(0,5,(height,width));active=(colors!=0)&(rng.random(colors.shape)>.15)
        forest,edges,boundary=graph(colors,active)
        old=solve(forest,edges,boundary*3,copyback=True)
        for order in ('forward','reverse','shuffled'):
            new=fused_solve(forest,edges,boundary*3,order)
            assert new['fault']==old['fault']==0
            assert new['roots']==old['roots']==reference(colors,active)


def test_long_chain_and_invalid_inactive_walks_preserve_roots_and_sticky_faults():
    for forest in ([1]+list(range(1,1024)),[2,1],[1,3],[1,0,2],[1,3,2]):
        old=solve(forest,[],[],copyback=True)
        for order in ('forward','reverse','shuffled'):
            new=fused_solve(forest,[],[],order)
            assert new['roots']==old['roots'] and new['fault']==old['fault']


def test_missing_and_false_hooks_remain_detectable_by_existing_independent_oracles():
    colors,active=scene('flat',17,17);forest,edges,boundary=graph(colors,active)
    assert fused_solve(forest,edges,[])['fault']==1
    assert fused_solve(forest,edges,boundary,rounds=0)['fault']==1
    colors=np.array([[1,2]]);active=np.ones(colors.shape,bool)
    forest,edges,boundary=graph(colors,active)
    false_union=fused_solve(forest,edges,[(0,1)])
    assert false_union['fault']==0
    assert false_union['roots']!=reference(colors,active)
