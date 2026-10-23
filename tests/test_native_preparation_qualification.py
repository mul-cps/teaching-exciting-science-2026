"""Host-only qualification comparator tests. Never instantiate Vulkan."""
import json
from pathlib import Path
import struct

import numpy as np
import pytest
from tools import qualify_native_preparation as q


@pytest.fixture(scope='module')
def fixtures(tmp_path_factory):
    path = tmp_path_factory.mktemp('qualification')/'fixtures'
    q.prepare(path)
    return path


def simulated_arena(fixtures, name, prefix):
    """Independent serialization of oracle arrays; tests decoder, not GPU."""
    e = np.load(fixtures/(name+'.npz'))
    h, w = e['flags'].shape; n = h*w; g = ((w+7)//8)*((h+7)//8); b = (g+255)//256
    selected = (28*n+4*g+8*b+7)&~7
    data = bytearray(selected+28*g)
    data[:24*n] = e['board'].astype('<f8').tobytes()
    data[24*n:28*n] = e['flags'].astype('<u4').tobytes()
    count = len(e['selected_pixels'])
    data[selected:selected+24*count] = e['selected_board'].astype('<f8').tobytes()
    data[selected+24*g:selected+24*g+4*count] = e['selected_pixels'].astype('<u4').tobytes()
    Path(str(prefix)+'.arena').write_bytes(data)
    Path(str(prefix)+'.json').write_text(json.dumps(dict(physical=False, generation=17, metadata_preserved=True, valid_count=int(e['valid_count']), support_count=count)))
    return data


@pytest.mark.parametrize('name', list(q.scenes()))
def test_complete_dense_comparison(fixtures, tmp_path, name):
    prefix = tmp_path/'result'
    simulated_arena(fixtures, name, prefix)
    assert q.compare_case(fixtures, name, prefix)['max_coordinate_error_m'] == 0


def test_strict_discrete_and_coordinate_gate(fixtures, tmp_path):
    prefix = tmp_path/'result'; path = Path(str(prefix)+'.arena')
    data = simulated_arena(fixtures, 'procedural', prefix)
    original = struct.unpack_from('<d', data, 0)[0]
    struct.pack_into('<d', data, 0, original+2e-12); path.write_bytes(data)
    with pytest.raises(AssertionError, match='coordinate error'):
        q.compare_case(fixtures, 'procedural', prefix)
    data = simulated_arena(fixtures, 'procedural', prefix)
    data[24*128*96] ^= 2; path.write_bytes(data)
    with pytest.raises(AssertionError, match='flags'):
        q.compare_case(fixtures, 'procedural', prefix)
    data = simulated_arena(fixtures, 'all_valid', prefix)
    n=640*480; g=4800; b=19; offset=((28*n+4*g+8*b+7)&~7)+24*g
    first, second = struct.unpack_from('<II', data, offset)
    struct.pack_into('<II', data, offset, second, first); path.write_bytes(data)
    with pytest.raises(AssertionError, match='membership/order'):
        q.compare_case(fixtures, 'all_valid', prefix)


def test_alignment_outer_threshold_is_stricter_than_inner():
    header = 'cpu_sdk_mismatch=0\n'*8
    assert q.alignment_result(header+'PASS cases=8 max_xyz_error_m=1e-7 gpu=1 physical=false')['exact_depth']
    for tail in ['1.01e-7 gpu=1', '0 gpu=0', 'nan gpu=1']:
        with pytest.raises(AssertionError):
            q.alignment_result(header+'PASS cases=8 max_xyz_error_m='+tail+' physical=false')


def test_fixture_input_matches_native_contract(fixtures):
    data = (fixtures/'partial_grid.bin').read_bytes()
    magic,w,h,generation,scale,limit = struct.unpack_from('<8sIIQdd', data)
    assert (magic,w,h,generation)==(b'GEOQv1\0\0',137,103,17)
    assert scale==.001 and limit==1.2
    assert len(data)==40+18*8+w*h*24
    e=np.load(fixtures/'partial_grid.npz')
    source=np.frombuffer(data, '<f4', count=w*h, offset=40+18*8+w*h*20)
    assert np.array_equal(source.reshape(h,w),e['depth_z16'].astype(np.float32)*.001)
    assert len(np.load(fixtures/'all_valid.npz')['selected_pixels'])==4800
    assert len(np.load(fixtures/'all_vivid.npz')['selected_pixels'])==0


def test_failed_command_retains_provenance_without_gpu(fixtures, tmp_path):
    import subprocess
    from types import SimpleNamespace
    args = SimpleNamespace(gpu_release='ROOT_RELEASED_GPU_PREPARATION', fixtures=fixtures,
                           output=tmp_path/'failed', geometry=Path('/usr/bin/false'),
                           alignment=None, shaders=tmp_path, expect_fp64_rejection=True)
    with pytest.raises(subprocess.CalledProcessError):
        q.run(args)  # /usr/bin/false is the intentional CPU-only failing stand-in.
    provenance = json.loads((args.output/'provenance.json').read_text())
    assert provenance['physical'] is False
    assert provenance['libraries'] and provenance['sources']
    assert provenance['fixtures_manifest_sha256'] == q.digest(fixtures/'manifest.json')
    assert (args.output/'procedural.log').exists()
