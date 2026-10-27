"""Frozen OpenCV/retained membership evidence; CPU only, no Vulkan device."""
from pathlib import Path
import hashlib
import json
import shutil
import subprocess
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
P = ROOT/'ros2/src/so101_station_vulkan'
FIXTURE = ROOT/'tests/fixtures/native_geometry/observed-children-v1.json'


@pytest.fixture(scope='module')
def reference(tmp_path_factory):
    binary=tmp_path_factory.mktemp('observed-reference')/'reference'
    built=subprocess.run([shutil.which('c++'),'-std=c++17','-O2','-Wall','-Wextra','-Werror','-I',str(P/'include'),str(P/'test/observed_children_reference.cpp'),'-o',str(binary)],capture_output=True,text=True)
    assert built.returncode==0,built.stderr
    return binary


def execute(binary,case,capacity=0,mutation=None):
    w,h=case['width'],case['height'];roi=case['roi']
    values=[w,h,len(case['records']),capacity,int(roi is not None),*(roi or [0]*6),*case['roots'],*np.asarray(case['board_xyz']).ravel()]
    for r in case['records']:values+=r
    result=subprocess.run([str(binary)]+([] if mutation is None else [str(mutation)]),input=' '.join(map(str,values)),capture_output=True,text=True,timeout=10)
    assert result.returncode==0,result.stderr
    return json.loads(result.stdout)


def cases():return json.loads(FIXTURE.read_text())['cases']


@pytest.mark.parametrize('case',cases(),ids=lambda c:c['id'])
def test_exact_original_closed_memberships_and_roi(reference,case):
    assert execute(reference,case)==case['expected']


def test_overlap_nonclipping_crop_border_and_boundary_fixture_contract():
    import cv2
    for c in cases():
        w,h=c['width'],c['height'];roots=np.asarray(c['roots']).reshape(h,w);board=np.asarray(c['board_xyz']);roi=c['roi'];expected=c['expected']
        for index,(root,x,y,bw,bh) in enumerate(c['records']):
            original=roots==root;inside=np.ones((h,w),bool)
            if roi is not None:
                inside=np.logical_and.reduce([board[:,:,axis]>=roi[axis*2] for axis in range(3)]+[board[:,:,axis]<=roi[axis*2+1] for axis in range(3)])
            count=int((original&inside).sum());closed=np.zeros((h,w),np.uint8)
            if count>=8:closed[y:y+bh,x:x+bw]=cv2.morphologyEx(original[y:y+bh,x:x+bw].astype(np.uint8),cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
            assert expected['records'][index]=={'root':root,'original_pixels':int(original.sum()),'roi_pixels':count,'eligible':count>=8,'closed_pixels':int(closed.sum()),'original':original.astype(int).ravel().tolist(),'closed':closed.astype(int).ravel().tolist()}
    overlap=next(c for c in cases() if c['id']=='overlap-child-tight-box')
    masks=[np.array(r['closed']) for r in overlap['expected']['records']]
    assert np.logical_and(*masks).any()
    boundary=next(c for c in cases() if c['id']=='roi-eight-boundary-nonclip')
    assert boundary['expected']['records'][0]['roi_pixels']==8
    assert boundary['expected']['records'][0]['closed_pixels']>8
    assert next(c for c in cases() if c['id']=='roi-seven-nextafter')['expected']['records'][0]['eligible'] is False


def test_closed_capacity_overflow_rejects_whole_result(reference):
    assert execute(reference,cases()[0],capacity=1)=={'usable':False}


def test_frozen_oracle_source_identity():
    proof=json.loads(FIXTURE.read_text())
    for name,digest in proof['sources'].items():assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==digest


def test_geometry_and_child_borrowed_views_are_additive_and_typed(tmp_path):
    cpp=tmp_path/'typed.cpp'
    cpp.write_text('''#include "so101_station_vulkan/observed_children.hpp"
using namespace so101_station_vulkan;
void contract(GeometryPreparation& g,GeometryLease& lease,ChildLease& child){auto token=g.producer_token();auto config=token.configuration();auto view=lease.resident_view();auto source=lease.source_frame();auto borrowed=child.resident_view();(void)config;(void)view;(void)source;(void)borrowed;lease.belongs_to(token);}
''')
    result=subprocess.run([shutil.which('c++'),'-std=c++17','-fsyntax-only','-I',str(P/'include'),str(cpp)],capture_output=True,text=True)
    assert result.returncode==0,result.stderr


@pytest.fixture(scope='module',params=['native','software'])
def shader_witness(request,tmp_path_factory):
    import os
    compiler=Path(os.environ.get('OBSERVED_SLANGC','/home/bjoern/.local/opt/slang-2026.19/bin/slangc'))
    if not compiler.is_file():pytest.skip('set OBSERVED_SLANGC for explicit AOT CPU arithmetic witness')
    directory=tmp_path_factory.mktemp('observed-'+request.param);obj=directory/'witness.o';reflection=directory/'reflection.json'
    entries=['observed_clear','observed_records','observed_observe','observed_eligible','observed_close','observed_finalize']
    cmd=[str(compiler),str(P/'shaders/observed_children.slang'),'-DOBSERVED_CPU_WITNESS=1']+(['-DOBSERVED_SOFTWARE64=1'] if request.param=='software' else [])
    for name in entries:cmd+=['-entry',name,'-stage','compute']
    cmd+=['-target','object-code','-emit-cpu-via-llvm','-llvm-cpu','generic','-fp-mode','precise','-O2','-reflection-json',str(reflection),'-o',str(obj)]
    result=subprocess.run(cmd,capture_output=True,text=True);assert result.returncode==0,result.stderr
    reflected=json.loads(reflection.read_text());assert [(x['name'],x['binding']['offset'],x['binding']['size']) for x in reflected['parameters']]==[(name,16*i,16) for i,name in enumerate(('roots','children','parents','geometry','roi','arena','compact'))]+[('p',112,8)]
    assert [(e['name'],e['threadGroupSize']) for e in reflected['entryPoints']]==[(e,[256,1,1] if e!='observed_finalize' else [1,1,1]) for e in entries]
    geometry_obj=directory/'geometry.o';geometry_reflection=directory/'geometry.json'
    result=subprocess.run([str(compiler),str(P/('shaders/geometry_soft.slang' if request.param=='software' else 'shaders/geometry.slang')),'-entry','geometry_coordinates','-stage','compute','-target','object-code','-emit-cpu-via-llvm','-llvm-cpu','generic','-fp-mode','precise','-O2','-reflection-json',str(geometry_reflection),'-o',str(geometry_obj)],capture_output=True,text=True);assert result.returncode==0,result.stderr
    r=json.loads(geometry_reflection.read_text());assert [(p['name'],p['binding']['offset'],p['binding']['size']) for p in r['parameters']]==[('source',0,16),('calibration',16,16),('arena',32,16),('counts',48,16),('p',64,8)]
    binary=directory/'witness';built=subprocess.run([shutil.which('c++'),'-std=c++17','-O2','-ffp-contract=off','-I',str(P/'include'),str(P/'test/observed_children_witness.cpp'),str(obj),str(geometry_obj),'-o',str(binary)],capture_output=True,text=True)
    assert built.returncode==0,built.stderr
    return binary


@pytest.mark.parametrize('case',cases(),ids=lambda c:c['id'])
def test_aot_actual_shader_serial_arithmetic_matches_frozen_oracle(shader_witness,case):
    assert execute(shader_witness,case)==case['expected']


def test_actual_module_host_identity_lifetime_and_borrow_validation(tmp_path):
    binary=tmp_path/'host'
    sources=['test/observed_children_host_contract.cpp','src/observed_children.cpp','src/children.cpp','src/parents.cpp','src/geometry.cpp','src/frame_pool.cpp']
    result=subprocess.run([shutil.which('c++'),'-std=c++17','-O2','-pthread','-I',str(P/'include'),*[str(P/s) for s in sources],'-o',str(binary)],capture_output=True,text=True)
    assert result.returncode==0,result.stderr
    groups=[('parent',['parents_init','parents_hook','parents_flatten','parents_validate','parents_scan','parents_offsets','parents_pixels','parents_labels','parents_records']),('child',['child_lookup','child_init','child_hook','child_jump','child_validate','child_reduce','child_gather','child_radix_hist','child_radix_offsets','child_radix_scatter','child_scatter','child_pack_scan','child_pack_offsets','child_jobs','child_packed_init','child_fill_hook','child_fill_jump','child_fill_validate','child_boundary','child_support','child_finalize']),('geometry',['geometry_coordinates','geometry_support_scan','geometry_support_offsets','geometry_support_scatter','geometry_valid_count']),('observed',['observed_clear','observed_records','observed_observe','observed_eligible','observed_close','observed_finalize'])]
    dirs=[]
    for name,entries in groups:
        directory=tmp_path/name;directory.mkdir();dirs.append(str(directory))
        for entry in entries:(directory/(entry+'.spv')).write_bytes(b'\x03\x02\x23\x07')
    result=subprocess.run([str(binary),*dirs],capture_output=True,text=True,timeout=20)
    assert result.returncode==0,result.stdout+result.stderr


@pytest.mark.parametrize('mutation',range(1,16))
def test_actual_shader_rejects_malformed_compact_input(shader_witness,mutation):
    assert execute(shader_witness,cases()[0],mutation=mutation)=={'usable':False}


def test_cmake_builds_and_validates_both_offline_shader_variants(tmp_path):
    import os
    compiler=Path(os.environ.get('OBSERVED_SLANGC','/home/bjoern/.local/opt/slang-2026.19/bin/slangc'))
    validator=Path(os.environ.get('OBSERVED_SPIRV_VAL','/home/bjoern/miniforge3/envs/so101-jazzy/bin/spirv-val'))
    if not compiler.is_file() or not validator.is_file():pytest.skip('set OBSERVED_SLANGC/OBSERVED_SPIRV_VAL for offline compile gate')
    stage=tmp_path/'stage';stage.mkdir()
    for folder in ('src','include','shaders'):(stage/folder).symlink_to(P/folder,target_is_directory=True)
    (stage/'CMakeLists.txt').write_text('cmake_minimum_required(VERSION 3.20)\nproject(observed_fixture LANGUAGES CXX)\nadd_library(resident_children INTERFACE)\nadd_library(resident_geometry INTERFACE)\ninclude("'+str(P/'cmake/observed_children.cmake')+'")\n')
    build=tmp_path/'build';configured=subprocess.run(['cmake','-S',str(stage),'-B',str(build),'-DSLANGC_EXECUTABLE='+str(compiler)],capture_output=True,text=True);assert configured.returncode==0,configured.stdout+configured.stderr
    result=subprocess.run(['cmake','--build',str(build),'--target','observed_children_native_spirv','observed_children_software_spirv','-j2'],capture_output=True,text=True);assert result.returncode==0,result.stdout+result.stderr
    for mode in ('native','software'):
        directory=build/'observed-children-shaders'/mode
        assert len(list(directory.glob('*.spv')))==6
        for spv in directory.glob('*.spv'):
            valid=subprocess.run([str(validator),'--target-env','vulkan1.1',str(spv)],capture_output=True,text=True);assert valid.returncode==0,valid.stderr
            assert (directory/(spv.name+'.sha256')).read_text()==hashlib.sha256(spv.read_bytes()).hexdigest()+'  '+spv.name+'\n'
        assert 'OBSERVED_CPU_WITNESS' not in (directory/'provenance.txt').read_text()
        assert ('OBSERVED_SOFTWARE64=1' in (directory/'provenance.txt').read_text())==(mode=='software')


@pytest.mark.parametrize('axis',range(3))
@pytest.mark.parametrize('sign',[-1,1])
def test_exact_all_six_roi_faces_and_nextafter(shader_witness,axis,sign):
    import copy
    for name in ('roi-eight-boundary-nonclip','roi-seven-nextafter'):
        case=copy.deepcopy(next(c for c in cases() if c['id']==name))
        board=np.asarray(case['board_xyz']);board=board[:,:,[axis,(axis+1)%3,(axis+2)%3]]*sign
        case['board_xyz']=board.tolist()
        # Symmetric ROI: axis permutation/sign preserves frozen exact expected masks.
        assert execute(shader_witness,case)==case['expected']


def _child_helpers():
    import importlib.util
    spec=importlib.util.spec_from_file_location('observed_child_graph_oracle',ROOT/'tests/test_native_child_graph_model.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


@pytest.fixture(scope='module')
def child_model(tmp_path_factory):
    binary=tmp_path_factory.mktemp('observed-child-model')/'model'
    built=subprocess.run([shutil.which('c++'),'-std=c++17','-O2','-I',str(P/'include'),str(P/'test/child_graph_model.cpp'),'-o',str(binary)],capture_output=True,text=True)
    assert built.returncode==0,built.stderr
    return binary


@pytest.mark.parametrize('name',[c['id'] for c in json.loads((ROOT/'tests/fixtures/native_geometry/child-evidence-v1.json').read_text())['cases'] if c['id']!='tall-1025x7'])
def test_combined_existing_child_graph_actual_geometry_and_observed_arithmetic(shader_witness,child_model,name):
    """Existing graph witness -> actual Geometry coordinates -> actual Observed.

    Expectations come from frozen retained child labels/order and retained
    camera_points/transform_points/OpenCV, never from new shader output.
    """
    import copy
    import cv2
    import importlib.util
    helper=_child_helpers();case=copy.deepcopy(next(c for c in helper.cases() if c['id']==name))
    # Full retained analyzer minimum, leaving standalone ChildConfig unchanged.
    minimum=max(24,case['input']['caller_parent_min_pixels'] or case['input']['min_pixels']);case['input']['min_pixels']=minimum
    areas={c['label_id']:c['area'] for c in case['expected']['components']}
    case['expected']['accepted']=[p for p in case['expected']['accepted'] if areas[p['label_id']]>=minimum]
    actual=helper.execute(child_model,case);frozen=helper.expected(case);helper.exact(actual,frozen)
    config,parent,depth=helper.fixture_input(case);w,h,px,py=config[:4];n=w*h
    assert w<=640 and h<=480
    spec=importlib.util.spec_from_file_location('observed_retained_geometry',ROOT/'tools/native_geometry_oracle.py');oracle=importlib.util.module_from_spec(spec);spec.loader.exec_module(oracle)
    yy,xx=np.mgrid[:h,:w];pixels=np.column_stack((xx.ravel(),yy.ravel()));rays=pixels.astype(float)
    transform=np.eye(4);k={'fx':1.,'fy':1.,'ppx':0.,'ppy':0.}
    with np.errstate(invalid='ignore'):
        z=depth.view(np.float32).ravel();valid=np.isfinite(z)&(z>0);camera=oracle.objects.camera_points(pixels[valid],z[valid],k,np.zeros(5));board=np.full((n,3),np.nan);board[valid]=oracle.objects.transform_points(camera,transform);board=board.reshape(h,w,3)
    roi=[0.,float(w//2),0.,float(h),.1,1.2]
    records=[];expected=[];original_roots=np.asarray(frozen['roots']).reshape(h,w)
    for record in frozen['records']:
        root=record['root'];x,y,rw,rh=record['bounds'];x+=px;y+=py;records.append([root,x,y,rw,rh]);mask=original_roots==root
        inside=oracle.objects._inside_roi(board.reshape(-1,3),roi).reshape(h,w);count=int((mask&inside).sum());closed=np.zeros((h,w),np.uint8)
        if count>=8:closed[y:y+rh,x:x+rw]=cv2.morphologyEx(mask[y:y+rh,x:x+rw].astype(np.uint8),cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
        expected.append({'root':root,'original_pixels':int(mask.sum()),'roi_pixels':count,'eligible':count>=8,'closed_pixels':int(closed.sum()),'original':mask.astype(int).ravel().tolist(),'closed':closed.astype(int).ravel().tolist()})
    # Actual Child graph output supplies runtime memberships/records, while all
    # expected arrays above use independent retained frozen labels/regions.
    values=[w,h,len(records),0,1,*roi,*actual['roots'],config[7],*depth.ravel(),*rays.ravel(),*transform[:3].ravel()]
    values += [*config[2:6],int(parent.sum()),*actual['stats'],minimum]
    packed_offset=0
    for record in actual['records']:
        values += [record['root'],record['parent'],record['key'],record['area'],*record['bounds'],*record['halo'],packed_offset,record['cut'],record['support'],0]
        packed_offset+=record['halo'][2]*record['halo'][3]
    result=subprocess.run([str(shader_witness),'geometry'],input=' '.join(map(str,values)),capture_output=True,text=True,timeout=10)
    assert result.returncode==0,result.stderr
    assert json.loads(result.stdout)=={'usable':True,'records':expected}
