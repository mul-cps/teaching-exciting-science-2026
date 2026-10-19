"""Actual native wire function parity and rejection, CPU only."""
import ast
import ctypes
import os
from pathlib import Path
import shutil
import subprocess

import numpy as np
import pytest

from workshop.perception.rocm_transport import WirePacker, pack_cloud

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope='module')
def packer(tmp_path_factory):
    configured = os.environ.get('SO101_ROCM_WIRE_LIBRARY')
    if configured: return WirePacker(configured)
    compiler = shutil.which('g++')
    if not compiler: pytest.skip('native wire unit probe requires a C++ compiler')
    output = tmp_path_factory.mktemp('rocm-wire')/'probe.so'
    subprocess.run([compiler, '-std=c++17', '-O2', '-shared', '-fPIC',
                    str(ROOT/'workshop/perception/rocm_wire_pack.cpp'), '-o', str(output)], check=True)
    return WirePacker(output)


@pytest.mark.parametrize('all_valid', [False, True])
def test_maximum_complete_wire_bytes_exact_vs_independent_bit_oracle(packer, all_valid):
    random = np.random.default_rng(342)
    xyz = random.uniform(-1, 1, (720, 1280, 3)).astype(np.float32); xyz[..., 2] = np.abs(xyz[..., 2])+.001
    # Preserve arbitrary RGB bits, including patterns that would be NaN if
    # incorrectly treated as float color values rather than copied wire bits.
    rgb = random.integers(0, 2**32, (720, 1280), dtype=np.uint32)
    valid = np.ones((720, 1280), np.uint8) if all_valid else random.integers(0, 2, (720, 1280), dtype=np.uint8)
    output = np.full(720*1280*20, 171, np.uint8)
    count = packer.pack_into(xyz, rgb, valid, output)
    expected = np.zeros((720, 1280, 5), '<u4')
    expected[..., :3] = xyz.view(np.uint32)
    expected[..., 4] = rgb
    expected_bytes = expected[valid.astype(bool)].tobytes()
    assert output[:count].tobytes() == expected_bytes == pack_cloud(xyz, rgb, valid)
    assert not output[count:].size or np.all(output[count:] == 171)


@pytest.mark.parametrize('kind', ['nan', 'inf', 'zero_z', 'negative_z', 'mask_two'])
def test_selected_bad_value_produces_no_success_count(packer, kind):
    xyz = np.ones((2, 2, 3), np.float32); rgb = np.zeros((2, 2), np.uint32); valid = np.ones((2, 2), np.uint8)
    if kind == 'mask_two': valid[-1, -1] = 2
    else: xyz[-1, -1, 2] = dict(nan=np.nan, inf=np.inf, zero_z=0., negative_z=-1.)[kind]
    with pytest.raises(ValueError, match='rejected complete'):
        packer.pack_into(xyz, rgb, valid, np.empty(80, np.uint8))


def test_unknown_invalid_xyz_is_skipped_not_filled_and_padding_exact(packer):
    xyz = np.array([[[np.nan, np.inf, -1.], [-0., 2., 3.]]], np.float32)
    rgb = np.array([[0, 0xff00aabb]], np.uint32); valid = np.array([[False, True]])
    output = np.full(40, 171, np.uint8)
    count = packer.pack_into(xyz, rgb, valid, output)
    assert count == 20 and output[12:16].tobytes() == bytes(4)
    assert output[:12].tobytes() == xyz[0, 1].tobytes()
    assert output[16:20].tobytes() == b'\xbb\xaa\x00\xff'


@pytest.mark.parametrize('kind', ['short', 'strided', 'bigendian', 'readonly', 'shape'])
def test_wrapper_capacity_stride_endianness_and_shapes_before_native(packer, kind):
    xyz = np.ones((2, 2, 3), np.float32); rgb = np.zeros((2, 2), np.uint32); valid = np.ones((2, 2), np.uint8); output = np.empty(80, np.uint8)
    if kind == 'short': output = output[:-1]
    elif kind == 'strided': xyz = np.ones((2, 4, 3), np.float32)[:, ::2]
    elif kind == 'bigendian': xyz = xyz.astype('>f4')
    elif kind == 'readonly': output.setflags(write=False)
    else: valid = np.ones((2, 3), np.uint8)
    with pytest.raises(ValueError): packer.pack_into(xyz, rgb, valid, output)


def test_native_abi_rejects_oversized_count_and_short_capacity_without_writing(packer):
    xyz = np.ones((1, 1, 3), np.float32); rgb = np.zeros((1, 1), np.uint32); valid = np.ones((1, 1), np.uint8)
    output = np.full(20, 171, np.uint8); written = ctypes.c_size_t(100)
    for count, capacity in ((921601, 20), (1, 19)):
        assert packer.function(xyz, rgb, valid, count, output, capacity, ctypes.byref(written)) == -1
        assert written.value == 0 and np.all(output == 171)


def test_wire_wrapper_fixed_one_native_call_no_per_point_python_work():
    tree = ast.parse((ROOT/'workshop/perception/rocm_transport.py').read_text())
    function = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == 'pack_into')
    assert not any(isinstance(n, (ast.For, ast.While, ast.ListComp, ast.DictComp, ast.SetComp)) for n in ast.walk(function))
    calls = [ast.unparse(n.func) for n in ast.walk(function) if isinstance(n, ast.Call)]
    assert calls.count('self.function') == 1
    assert not any(x.endswith(('.copy', '.tobytes', '.astype', '.empty', '.zeros', '.isfinite', '.sha256', '.read_bytes')) for x in calls)
