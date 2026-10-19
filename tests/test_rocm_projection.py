"""CPU-only contract tests: GPU execution is separately gated by review."""
import copy
import numpy as np
import pytest
from workshop.perception.rocm_projection import calibration_from_oracle, numpy_raw_projection


def metadata():
    return {'schema': 'realsense-cpu-projection-oracle-v1', 'qualified': False,
            'align_name': 'Align', 'pointcloud_name': 'Pointcloud', 'occlusion_option': 2.,
            'depth_size': [8, 4], 'color_size': [8, 4],
            'depth_K': [100., 100., 3.5, 1.5], 'color_K': [100., 100., 3.5, 1.5],
            'rotation_column_major': [1., 0., 0., 0., 1., 0., 0., 0., 1.],
            'translation_m': [0., 0., 0.], 'depth_scale_m': .001}


@pytest.mark.parametrize('field,value', [('align_name', 'Align (SSE3)'),
    ('pointcloud_name', 'Pointcloud (SSE3)'), ('occlusion_option', 0),
    ('depth_size', [True, 4]), ('depth_size', [1281, 720]),
    ('color_size', [1920, 1081]), ('depth_scale_m', float('nan')),
    ('depth_K', [100, 100, float('inf'), 2]), ('color_K', [0, 100, 4, 2]),
    ('rotation_column_major', [0]*9), ('translation_m', [0, 0, float('nan')])])
def test_unsupported_calibration_fails(field, value):
    data = metadata(); data[field] = value
    with pytest.raises(ValueError): calibration_from_oracle(data)


def test_identity_plane_preserves_metric_and_cpu_rectangle_boundary():
    calibration = calibration_from_oracle(metadata())
    raw = np.full((4, 8), 1000, np.uint16)
    aligned, xyz, pixels, flags = numpy_raw_projection(raw, calibration)
    assert flags == 0 and np.all(aligned == 1000)
    assert xyz.dtype == pixels.dtype == np.float32
    assert np.all(xyz[:, :, 2] == 1.)
    yy, xx = np.indices(raw.shape)
    np.testing.assert_allclose(pixels, np.stack((xx, yy), -1), atol=1e-6, rtol=0)


def test_zero_depth_stays_zero_in_raw_projection_and_can_only_receive_real_splats():
    c = calibration_from_oracle(metadata()); raw = np.zeros((4, 8), np.uint16)
    aligned, xyz, pixels, flags = numpy_raw_projection(raw, c)
    assert not flags and not aligned.any() and not xyz.any() and not pixels.any()
    raw[1, 2] = 500
    aligned, xyz, pixels, flags = numpy_raw_projection(raw, c)
    assert np.count_nonzero(aligned) == 4 and np.all(aligned[1:3, 2:4] == 500)
    assert np.count_nonzero(xyz[:, :, 2]) == 1


def test_minimum_is_original_raw_z16_not_transformed_depth():
    data = metadata(); data['color_K'] = [50., 50., 3.5, 1.5]
    c = calibration_from_oracle(data); raw = np.full((4, 8), 1000, np.uint16); raw[1, 2] = 500
    aligned, _, _, flags = numpy_raw_projection(raw, c)
    assert not flags and 500 in aligned and set(np.unique(aligned)) <= {0, 500, 1000}


def test_oversized_footprint_fails_instead_of_clipping():
    data=metadata(); data['color_K']=[1000., 1000., 3.5, 1.5]
    data['color_size']=[80,40]; data['color_K'][2:]=[35.,15.]
    c=calibration_from_oracle(data)
    _,_,_,flags=numpy_raw_projection(np.full((4,8),1000,np.uint16),c)
    assert flags & 1


@pytest.mark.parametrize('raw', [np.zeros((4,8),np.float32),np.zeros((4,9),np.uint16)])
def test_depth_input_is_exact_shape_and_wire_type(raw):
    with pytest.raises(ValueError): numpy_raw_projection(raw,calibration_from_oracle(metadata()))


@pytest.fixture(scope='module')
def compatibility(tmp_path_factory):
    import shutil,subprocess
    from pathlib import Path
    from workshop.perception.rocm_projection import CPUCompatibility
    compiler=shutil.which('g++')
    if not compiler: pytest.skip('C++ compiler unavailable')
    lib=tmp_path_factory.mktemp('projection-compat')/'compat.so'
    source=Path(__file__).resolve().parents[1]/'workshop/perception/projection_compat.cpp'
    subprocess.run([compiler,'-std=c++17','-O2','-shared','-fPIC',str(source),'-o',str(lib)],check=True)
    return CPUCompatibility(lib)


def test_cpu_compat_keeps_uv_on_occlusion_and_dilates_one_valid_pixel(compatibility):
    data=metadata();data['translation_m']=[.02,0.,0.]
    c=calibration_from_oracle(data);xyz=np.ones((4,8,3),np.float32)
    yy,xx=np.indices((4,8));pixels=np.stack((xx,yy),-1).astype(np.float32)
    pixels[0,3,0]=1.5
    colors=np.full((4,8,3),[10,20,30],np.uint8)
    uv,packed,valid=compatibility.finish(xyz,pixels,colors,c)
    assert np.all(xyz[0,3:5]==0) and np.all(xyz[0,5]==1)
    assert uv[0,3,0]==np.float32(1.5/8) and not valid[0,3] and not valid[0,4]
    assert packed[0,2]==0x0a141e


def test_cpu_compat_dilation_skips_source_depth_holes(compatibility):
    data=metadata();data['translation_m']=[.02,0.,0.]
    c=calibration_from_oracle(data);xyz=np.ones((4,8,3),np.float32)
    yy,xx=np.indices((4,8));pixels=np.stack((xx,yy),-1).astype(np.float32)
    pixels[0,3,0]=1.5;xyz[0,4]=0;pixels[0,4]=0
    uv,packed,valid=compatibility.finish(xyz,pixels,np.zeros((4,8,3),np.uint8),c)
    assert not valid[0,3:6].any() and valid[0,6]
    assert not uv[0,4].any()


def test_cpu_compat_malformed_cohort_is_unavailable(compatibility):
    c=calibration_from_oracle(metadata())
    with pytest.raises(ValueError): compatibility.finish(np.ones((4,8,2),np.float32),np.ones((4,8,2),np.float32),np.zeros((4,8,3),np.uint8),c)
    xyz=np.ones((4,8,3),np.float32);xyz[0,0,0]=np.nan
    with pytest.raises(ValueError): compatibility.finish(xyz,np.ones((4,8,2),np.float32),np.zeros((4,8,3),np.uint8),c)


def test_backend_default_off_does_not_import_torch(monkeypatch):
    import builtins
    from workshop.perception.rocm_projection_backend import ROCmProjection
    original=builtins.__import__
    def guarded(name,*args,**kwargs):
        if name=='torch' or 'rocm_projection_kernels' in name:raise AssertionError('GPU activated while disabled')
        return original(name,*args,**kwargs)
    monkeypatch.setattr(builtins,'__import__',guarded)
    with pytest.raises(ValueError,match='explicit'):ROCmProjection(None,None)


def test_backend_static_batching_and_precision_contract():
    import ast,inspect
    from pathlib import Path
    from workshop.perception.rocm_projection_backend import ROCmProjection
    source=inspect.getsource(ROCmProjection)
    tree=ast.parse(source)
    process=next(n for n in tree.body[0].body if isinstance(n,ast.FunctionDef) and n.name=='process')
    calls=[ast.unparse(n.func) for n in ast.walk(process) if isinstance(n,ast.Call)]
    assert calls.count('self.depth.copy_')==1 and calls.count('self.host.copy_')==1
    assert not any(n.endswith(('.cpu','.item','.numpy','.synchronize')) for n in calls)
    assert not any(isinstance(n,(ast.While,ast.ListComp,ast.DictComp)) for n in ast.walk(process))
    loops=[n for n in ast.walk(process) if isinstance(n,ast.For)]
    assert len(loops)==1 and ast.unparse(loops[0].iter)=='(aligned, xyz, uv, packed, valid)'
    assert 'enable_fp_fusion=False' in source and 'device=\'cuda:0\'' in source
    kernel=(Path(__file__).resolve().parents[1]/'workshop/perception/rocm_projection_kernels.py').read_text()
    assert kernel.count('tl.static_range(4)')==2 and 'tl.atomic_min' in kernel
    assert 'tl.div_rn' in kernel and 'bitcast=True' in kernel


@pytest.mark.parametrize('translation',[[0.,.02,0.],[-.02,.02,0.]])
def test_vertical_sdk_occlusion_is_rejected_before_gpu(translation):
    data=metadata();data['translation_m']=translation
    with pytest.raises(ValueError,match='vertical'):calibration_from_oracle(data)


def test_identity_extrinsics_skip_occlusion_exactly_like_sdk(compatibility):
    c=calibration_from_oracle(metadata());assert c.apply_occlusion is False
    xyz=np.ones((4,8,3),np.float32);pixels=np.zeros((4,8,2),np.float32)
    pixels[0,1,0]=3.;pixels[0,2,0]=1.
    compatibility.finish(xyz,pixels,np.zeros((4,8,3),np.uint8),c)
    assert np.all(xyz==1)


def test_sdk_identity_and_vertical_float32_thresholds_are_preserved():
    data=metadata();data['translation_m']=[.001,.02,0.]
    assert calibration_from_oracle(data).apply_occlusion is True
    data=metadata();data['rotation_column_major'][1]=1e-7
    assert calibration_from_oracle(data).apply_occlusion is False
