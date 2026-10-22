"""Independent host-only ownership/fault and allocation guards; no GPU context."""
from pathlib import Path
from types import SimpleNamespace as NS
import ast
import threading
import numpy as np
import pytest
from workshop.perception.rocm_projection_backend import ROCmProjection


class Tensor:
    def __init__(self,array):self.array=array
    def copy_(self,other,non_blocking=False):np.copyto(self.array,other.array);return self


class Kernel:
    def __init__(self,function):self.function=function
    def __getitem__(self,grid):return self.function


def fake_backend():
    b=ROCmProjection.__new__(ROCmProjection);b.calibration=NS(depth_size=(4,2),color_size=(4,2));b.count=b.color_count=8;b.flag=48
    b.raw_array=np.empty(8,np.int32);b.raw_host=Tensor(b.raw_array);b.depth=Tensor(np.empty(8,np.int32));b.rgb_scratch=np.empty((2,4,3),np.uint8);b.output=Tensor(np.empty(49,np.int32));b.host_array=np.empty(49,np.int32);b.host=Tensor(b.host_array);b.rays=b.parameters=None;b.busy=threading.Lock();b.force_fault=False
    def init(output,color_count,flag,block):output.array[:color_count]=65536;output.array[flag]=0
    def project(depth,rays,parameters,output,count,color_count,cw,ch,flag,block,**kwargs):
        raw=depth.array.astype(np.float32);output.array[:color_count]=raw.astype(np.int32);xyz=np.stack((raw,raw,raw),-1);output.array[color_count:color_count+3*count]=xyz.reshape(-1).view(np.int32);output.array[color_count+3*count:flag]=np.zeros(2*count,np.float32).view(np.int32);output.array[flag]=1 if b.force_fault else 0
    def finish(xyz,pixels,rgb,c):return pixels.copy(),np.full((2,4),int(rgb[0,0,0]),np.uint32),np.ones((2,4),np.uint8)
    b.kernels=NS(initialize_output=Kernel(init),project_splat=Kernel(project));b.compatibility=NS(finish=finish);return b


def test_returned_cohorts_readonly_and_independent_after_scratch_reuse():
    b=fake_backend();depth=np.ones((2,4),np.uint16);rgb=np.full((2,4,3),3,np.uint8);first=b.process(depth,rgb)
    arrays=[first.aligned_z16,first.xyz,first.uv,first.rgb_packed,first.valid];saved=[a.copy() for a in arrays]
    depth[:]=2;rgb[:]=9;b.process(depth,rgb)
    for array,copy in zip(arrays,saved):
        np.testing.assert_array_equal(array,copy);assert not array.flags.writeable
    assert not np.shares_memory(first.xyz,b.host_array)


def test_busy_rejects_without_queue_and_fault_releases_lock_without_partial_result():
    b=fake_backend();depth=np.ones((2,4),np.uint16);rgb=np.zeros((2,4,3),np.uint8)
    b.busy.acquire()
    with pytest.raises(RuntimeError,match='no queue'):b.process(depth,rgb)
    b.busy.release();b.force_fault=True
    with pytest.raises(ValueError,match='unsupported'):b.process(depth,rgb)
    assert not b.busy.locked();b.force_fault=False;b.process(depth,rgb)


def test_no_per_frame_device_allocations_or_torch_import():
    root=Path(__file__).resolve().parents[1];tree=ast.parse((root/'workshop/perception/rocm_projection_backend.py').read_text());process=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='process')
    calls=[ast.unparse(n.func) for n in ast.walk(process) if isinstance(n,ast.Call)]
    assert not any('torch' in value or value.endswith(('.empty','.zeros','.to','.numpy','.synchronize','.cpu','.item')) for value in calls)
    kernels=[n for n in ast.walk(process) if isinstance(n,ast.Call) and isinstance(n.func,ast.Subscript)]
    assert [ast.unparse(n.func.value) for n in kernels]==['self.kernels.initialize_output','self.kernels.project_splat']
