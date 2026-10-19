"""Bounded cached NOMINAL CAD visuals use exact URDF transforms and all meshes."""
import ast
import inspect
from pathlib import Path
import struct
import numpy as np
import pytest
from workshop.digital_twin.gripper_mesh import load_gripper_visuals


def stl(path):
    records=np.zeros(1,dtype=[('normal','<f4',3),('vertices','<f4',(3,3)),('attribute','<u2')])
    records['vertices'][0]=[[0,0,0],[.01,0,0],[0,.01,0]]
    path.write_bytes(bytes(80)+struct.pack('<I',1)+records.tobytes())
    return path


def description(tmp_path):
    p=stl(tmp_path/'one.stl')
    visual=f'<visual><geometry><mesh filename="{p}"/></geometry></visual>'
    transformed=f'<visual><origin xyz=".1 0 0" rpy="0 0 1.5707963267948966"/><geometry><mesh filename="{p}" scale="2 2 2"/></geometry></visual>'
    return f'<robot><link name="gripper_link">{visual}{transformed}</link><link name="moving_jaw_so101_v1_link">{visual}</link></robot>'


def test_every_gripper_visual_loaded_with_declared_origin_scale_and_hash(tmp_path):
    meshes,meta=load_gripper_visuals(description(tmp_path))
    assert len(meshes)==3 and meta['triangles']==3 and meta['qualified'] is False
    assert meshes[1]['points']==pytest.approx(np.array([[.1,0,0],[.1,.02,0],[.08,0,0]]),abs=1e-8)
    assert meshes[1]['colors'].shape==(3,3) and meshes[1]['colors'].dtype==np.uint8
    assert all(len(v['sha256'])==64 for v in meta['visuals'])
    assert meta['visuals'][1]['scale']==[2.,2.,2.]


def test_missing_or_malformed_visuals_never_silently_omit_requested_parts(tmp_path):
    d=description(tmp_path)
    with pytest.raises(ValueError):load_gripper_visuals(d.replace('moving_jaw_so101_v1_link','other_link'))
    with pytest.raises(ValueError):load_gripper_visuals(d.replace('<mesh filename=', '<box filename='))
    with pytest.raises(ValueError):load_gripper_visuals('<invalid')
    p=tmp_path/'one.stl';p.write_bytes(b'not a mesh')
    with pytest.raises(ValueError):load_gripper_visuals(d)


def test_mesh_loader_uses_metadata_loops_and_vectorized_vertices():
    source=inspect.getsource(load_gripper_visuals);tree=ast.parse(source)
    loops=[ast.unparse(n.iter) for n in ast.walk(tree) if isinstance(n,ast.For)]
    assert loops==['LINKS',"enumerate(link.findall('visual'))"]
    assert not any(isinstance(n,ast.While) for n in ast.walk(tree))
    assert 'np.frombuffer' in source and 'np.cross' in source
