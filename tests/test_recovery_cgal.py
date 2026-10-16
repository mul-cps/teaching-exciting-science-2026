"""Offline exact backend contract; never touches devices or ROS."""
import importlib.util
import os
from pathlib import Path
import sys

import numpy as np
import pytest
import trimesh

ROOT = Path(__file__).resolve().parents[1]
MODULE = ROOT / 'tools/recovery_cgal/certify.py'
BINARY = Path(os.environ.get('RECOVERY_CGAL_BINARY', Path.home()/'.local/share/exciting-science/recovery-cgal/bin/recovery-cgal'))
BUILD_PIN = os.environ.get('RECOVERY_CGAL_BUILD_SHA256')


def api():
    assert MODULE.is_file(), 'bounded offline CGAL wrapper has not been implemented'
    spec = importlib.util.spec_from_file_location('recovery_cgal_certify', MODULE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def cube(center=(0.,0.,0.), extent=2.):
    mesh = trimesh.creation.box(extents=[extent]*3)
    mesh.apply_translation(center)
    return mesh.triangles.copy()


def proof(triangles, cells=(), **kwargs):
    require_binary()
    return api().certify_triangles(triangles, cells, binary=BINARY, trusted_build_sha256=BUILD_PIN, **kwargs)


def require_binary():
    if not BUILD_PIN:
        pytest.skip('explicit trusted RECOVERY_CGAL_BUILD_SHA256 required')
    if not BINARY.is_file():
        if 'RECOVERY_CGAL_BINARY' in os.environ:
            pytest.fail('explicit RECOVERY_CGAL_BINARY does not exist')
        pytest.skip('offline CGAL executable not built; see tools/recovery_cgal/README.md')


def cell(center=(0.,0.,0.), pitch=.1, uncertainty=0.):
    return {'center':list(center), 'pitch':pitch, 'boundary_uncertainty':uncertainty}


def test_closed_cube_certifies_only_strict_full_uncertain_cells():
    result = proof(cube(), [cell(), cell((.95,0,0)), cell((1,0,0)),
                             cell((.90,0,0),uncertainty=.02), cell((2,0,0))])
    assert result['status'] == 'proved'
    assert [item['certified'] for item in result['cells']] == [True,False,False,False,False]
    assert result['production_recovery_qualified'] is False


def test_hollow_nested_shell_uses_parity_and_never_fills_cavity():
    inner = cube(extent=1.)[:,::-1,:]
    result = proof(np.concatenate([cube(),inner]), [cell(),cell((.75,0,0),pitch=.1)])
    assert result['status'] == 'proved'
    assert [item['certified']for item in result['cells']] == [False,True]
    assert result['components'] == 2


@pytest.mark.parametrize('triangles',[
    np.concatenate([cube(),cube((.5,.25,.1))]),
    np.concatenate([cube(),cube((2.,2.,2.))]),
    np.concatenate([cube(),cube((2.,0.,0.))]),
    np.concatenate([cube(),np.array([[[4.,0,0],[5.,0,0],[6.,0,0]]])]),
    np.concatenate([cube(),np.array([[[4.,0,0],[5.,0,0],[4.,1,0]],[[4.,1,0],[5.,0,0],[4.,0,0]]])]),
    cube()[:-1],
])
def test_intersections_tangencies_degenerate_membranes_and_open_meshes_never_certify(triangles):
    result = proof(triangles,[cell()])
    assert result['status'] in {'rejected','unknown'}
    assert not any(item['certified']for item in result['cells'])


def test_ray_edge_vertex_ties_and_boundary_points_are_conservative():
    result = proof(cube(),[cell((.3,.3,.3),pitch=.01),cell((1,1,1)),cell((1,0,0))])
    assert [item['certified']for item in result['cells']] == [True,False,False]


def test_exact_double_payload_immutable_and_scale_provenance(tmp_path):
    triangle = cube()
    triangle[triangle==1.] = np.nextafter(1.,2.)
    original = triangle.copy()
    result = proof(triangle,[cell((1.5,0,0))],scale=(2.,1.,1.))
    np.testing.assert_array_equal(triangle,original)
    assert result['status'] == 'proved'
    assert result['cells'][0]['certified'] is True
    assert result['provenance']['scale_hex'] == [float(v).hex()for v in (2.,1.,1.)]
    assert result['provenance']['triangle_positions_preserved'] is True
    assert len(result['provenance']['payload_sha256']) == 64


def native_backend(tmp_path, body):
    import hashlib, json, subprocess
    module=api()
    backend=tmp_path/'backend'
    subprocess.run(['/usr/bin/cc','-x','c','-','-o',str(backend)],input=(
        '#include <stdio.h>\n#include <unistd.h>\nint main(void){'+body+'}').encode(),check=True)
    manifest={'source_sha256':module.REVIEWED_SOURCE_SHA256,'cgal_version':module.CGAL_VERSION,
              'returncode':0,'binary_sha256':hashlib.sha256(backend.read_bytes()).hexdigest()}
    path=tmp_path/'build.json';path.write_text(json.dumps(manifest))
    return dict(binary=backend,build_manifest=path,trusted_build_sha256=hashlib.sha256(path.read_bytes()).hexdigest())


def test_timeout_and_crashed_child_never_admit_cells(tmp_path):
    backend=native_backend(tmp_path,'sleep(10); return 0;')
    result=api().certify_triangles(cube(),[cell()],timeout_s=.05,**backend)
    assert result['status']=='unknown'
    assert result['cells']==[{'certified':False,'reason':'operation_timeout'}]
    backend=native_backend(tmp_path,'return 1;')
    result=api().certify_triangles(cube(),[cell()],**backend)
    assert result['status']=='unknown'
    assert not result['cells'][0]['certified']


@pytest.mark.parametrize('kwargs',[{'timeout_s':0},{'memory_mb':0},{'scale':(0,1,1)}])
def test_invalid_protocol_options_reject_before_child(kwargs):
    with pytest.raises(ValueError):
        api().certify_triangles(cube(),[cell()],binary='/bin/false',**kwargs)


@pytest.mark.parametrize('badcell',[cell(pitch=0),cell(uncertainty=-1),cell((float('nan'),0,0))])
def test_nonfinite_or_invalid_cell_is_rejected_before_backend(badcell):
    with pytest.raises(ValueError):
        api().certify_triangles(cube(),[badcell],binary='/bin/false')


def test_exact_embedding_keeps_subdecimal_surface_clearance():
    triangles=cube()
    edge=1.+4*np.spacing(1.)
    triangles[triangles==1.]=edge
    result=proof(triangles,[cell((1.+2*np.spacing(1.),0,0),pitch=np.spacing(1.)/10)])
    assert result['status']=='proved'
    assert result['cells'][0]['certified'] is True


def test_same_winding_nested_shell_does_not_claim_oriented_material():
    result=proof(np.concatenate([cube(),cube(extent=1.)]),[cell()])
    assert result['status']=='rejected'
    assert result['reason']=='invalid_volume_orientation'
    assert not result['cells'][0]['certified']


def test_cli_bad_cells_or_missing_input_exits_unknown(tmp_path):
    import json
    import subprocess
    output=tmp_path/'result.json'
    proc=subprocess.run([sys.executable,str(MODULE),'--mesh',str(tmp_path/'missing.stl'),
                         '--binary','/bin/false','--trusted-build-sha256','0'*64,'--output',str(output)],capture_output=True)
    assert proc.returncode==2
    result=json.loads(output.read_text())
    assert result['status']=='unknown'
    assert result['production_recovery_qualified'] is False


def test_corrupt_backend_json_and_partial_success_are_unknown(tmp_path):
    backend=native_backend(tmp_path,'puts("{\\\"status\\\":\\\"proved\\\",\\\"cells\\\":[]}");return 0;')
    result=api().certify_triangles(cube(),[cell()],**backend)
    assert result['status']=='unknown'
    assert not result['cells'][0]['certified']


def test_file_snapshot_hash_and_geometry_are_the_same_bytes(tmp_path,monkeypatch):
    require_binary()
    import hashlib
    module=api()
    path=tmp_path/'cube.stl'
    source=trimesh.Trimesh(vertices=cube().reshape(-1,3),faces=np.arange(36).reshape(-1,3),process=False).export(file_type='stl')
    path.write_bytes(source)
    original=module.bounded_read
    def replace_after_snapshot(file_path,*args):
        data=original(file_path,*args)
        if Path(file_path)==path:
            path.write_bytes(trimesh.creation.icosphere(subdivisions=1).export(file_type='stl'))
        return data
    monkeypatch.setattr(module,'bounded_read',replace_after_snapshot)
    result=module.certify_file(path,[cell()],binary=BINARY,trusted_build_sha256=BUILD_PIN)
    assert result['status']=='proved'
    assert result['provenance']['source_sha256']==hashlib.sha256(source).hexdigest()
    assert result['provenance']['faces']==12
    assert result['cells'][0]['certified'] is True


def test_exact_clearance_equality_is_unknown_instead_of_inside():
    # A unit cube translated so distance equals the exact rational radius
    # when pitch=1000 and scale=1: radius867, center distance867.
    result=proof(cube(extent=1734.),[cell(pitch=1000.)])
    assert result['status']=='proved'
    assert result['cells'][0]['certified'] is False


def test_cli_invalid_json_schema_returns_unknown(tmp_path):
    import json
    import subprocess
    mesh=tmp_path/'mesh.stl'
    mesh.write_bytes(trimesh.creation.box().export(file_type='stl'))
    cells=tmp_path/'cells.json';cells.write_text('[null]')
    output=tmp_path/'out.json'
    proc=subprocess.run([sys.executable,str(MODULE),'--mesh',str(mesh),'--cells',str(cells),
                         '--binary',str(BINARY),'--trusted-build-sha256',BUILD_PIN or '0'*64,'--output',str(output)],capture_output=True)
    assert proc.returncode==2
    assert json.loads(output.read_text())['status']=='unknown'


def test_closed_shell_intersection_reaches_the_exact_intersection_gate():
    result=proof(np.concatenate([cube(),cube((.5,.25,.1))]),[cell()])
    assert result['status']=='rejected'
    assert result['reason']=='self_intersection'


def test_unpinned_minimal_fake_success_cannot_authorize(tmp_path):
    import json
    backend=native_backend(tmp_path,'puts('+json.dumps('{"status":"proved","cells":[]}')+'); return 0;')
    result=api().certify_triangles(cube(),[],binary=backend['binary'])
    assert result['status']=='unknown'
    assert result['reason']=='trusted_build_pin_required'


def test_backend_path_swap_executes_only_the_sealed_pinned_bytes(tmp_path,monkeypatch):
    require_binary()
    import hashlib,json
    module=api()
    original_binary=BINARY.read_bytes()
    candidate=tmp_path/'candidate';candidate.write_bytes(original_binary)
    manifest=BINARY.with_name('build.json')
    original_child=module.bounded_child
    def swap_after_verified_snapshot(fd,*args):
        with pytest.raises(OSError):
            os.write(fd,b'corruption')
        candidate.write_bytes(b'not the trusted executable')
        return original_child(fd,*args)
    monkeypatch.setattr(module,'bounded_child',swap_after_verified_snapshot)
    result=module.certify_triangles(cube(),[cell()],binary=candidate,
            build_manifest=manifest,trusted_build_sha256=BUILD_PIN)
    assert result['status']=='proved'
    assert result['cells'][0]['certified'] is True
    assert result['provenance']['backend_sha256']==hashlib.sha256(original_binary).hexdigest()


@pytest.mark.parametrize('field,value',[('source_sha256','0'*64),('cgal_version','0.0'),('returncode',False)])
def test_pinned_unreviewed_build_is_still_rejected(tmp_path,field,value):
    import hashlib,json
    backend=native_backend(tmp_path,'return 0;')
    manifest=json.loads(backend['build_manifest'].read_text());manifest[field]=value
    backend['build_manifest'].write_text(json.dumps(manifest))
    backend['trusted_build_sha256']=hashlib.sha256(backend['build_manifest'].read_bytes()).hexdigest()
    result=api().certify_triangles(cube(),[],**backend)
    assert result['status']=='unknown'


@pytest.mark.parametrize('stream,count',[('stdout',2000),('stderr',16)])
def test_backend_pipe_flood_has_bounded_parent_buffers(tmp_path,stream,count):
    import time
    backend=native_backend(tmp_path,f'char data[8192]={{0}};for(int i=0;i<{count};++i)fwrite(data,1,sizeof(data),{stream});return 0;')
    started=time.monotonic()
    result=api().certify_triangles(cube(),[cell()],timeout_s=2.,**backend)
    assert result['status']=='unknown'
    assert result['reason']=='backend_output_limit'
    assert time.monotonic()-started<2.5
    assert not result['cells'][0]['certified']


def test_oversize_source_and_fifo_are_rejected_without_loading(tmp_path):
    module=api()
    source=tmp_path/'large.stl'
    with source.open('wb')as file:
        file.truncate(module.MAX_MESH_BYTES+1)
    result=module.certify_file(source,[],binary='/bin/false')
    assert result['status']=='unknown'
    fifo=tmp_path/'fifo.stl';os.mkfifo(fifo)
    result=module.certify_file(fifo,[],binary='/bin/false',timeout_s=.1)
    assert result['status']=='unknown'


def test_thousand_digit_center_cli_fails_unknown_without_traceback(tmp_path):
    import json,subprocess
    source=tmp_path/'cube.stl';source.write_bytes(trimesh.creation.box().export(file_type='stl'))
    cells=tmp_path/'cells.json';cells.write_text(json.dumps([cell((10**1000,0,0))]))
    output=tmp_path/'output.json'
    proc=subprocess.run([sys.executable,str(MODULE),'--mesh',str(source),'--binary',str(BINARY),
            '--trusted-build-sha256',BUILD_PIN or '0'*64,'--cells',str(cells),'--output',str(output)],capture_output=True)
    assert proc.returncode==2
    assert b'Traceback'not in proc.stderr
    assert json.loads(output.read_text())['status']=='unknown'


def test_preprocessing_expiry_cannot_reach_backend(tmp_path,monkeypatch):
    import time
    module=api()
    source=tmp_path/'cube.stl';source.write_bytes(trimesh.creation.box().export(file_type='stl'))
    original=module.bounded_read
    def slow_snapshot(*args):
        data=original(*args);time.sleep(.05);return data
    monkeypatch.setattr(module,'bounded_read',slow_snapshot)
    result=module.certify_file(source,[cell()],binary='/bin/false',timeout_s=.01)
    assert result['status']=='unknown'
    assert result['reason']=='operation_timeout'


def test_build_compiles_frozen_bytes_after_source_path_changes(tmp_path,monkeypatch):
    import importlib.util,subprocess,time
    monkeypatch.syspath_prepend(str(MODULE.parent))
    spec=importlib.util.spec_from_file_location('recovery_build',MODULE.with_name('build.py'))
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    source=tmp_path/'source.cpp';source.write_bytes(b'int main(){return 7;}')
    snapshot=source.read_bytes()
    source.write_bytes(b'int main(){return 99;}')
    binary=tmp_path/'compiled'
    code,output,command=module.compile_frozen(snapshot,['/usr/bin/c++','-x','c++','{source}','-o',str(binary)],
            deadline=time.monotonic()+10,limits=lambda:None,env=dict(os.environ))
    assert code==0,output
    assert subprocess.run([str(binary)]).returncode==7
