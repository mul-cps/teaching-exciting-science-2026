"""Default-off offline ROCm projection backend; no ROS/device/control integration."""
from dataclasses import dataclass
import threading
import sys
import time
import numpy as np
from .rocm_projection import CPUCompatibility


@dataclass(frozen=True)
class ProjectionResult:
    aligned_z16: np.ndarray
    xyz: np.ndarray
    uv: np.ndarray
    rgb_packed: np.ndarray
    valid: np.ndarray
    complete_ms: float
    qualified: bool = False


class ROCmProjection:
    """One job/no queue; explicit GPU activation and no CPU fallback.

    Source identity and original acquisition headers belong to the caller's
    immutable cohort, never synthesized here. This backend grants no authority.
    """
    def __init__(self, calibration, compatibility_library, *, enabled=False):
        if enabled is not True: raise ValueError('ROCm projection requires explicit enabled=True')
        import torch
        from . import rocm_projection_kernels as kernels
        if (not torch.version.hip or not torch.cuda.is_available()
                or not torch.cuda.get_device_properties(0).gcnArchName.startswith('gfx1151')
                or '8060S' not in torch.cuda.get_device_name(0)):
            raise RuntimeError('expected gfx1151 AMD8060S ROCm hardware; no fallback')
        self.torch=torch;self.kernels=kernels;self.calibration=calibration
        self.compatibility=CPUCompatibility(compatibility_library);self.busy=threading.Lock()
        w,h=calibration.depth_size;cw,ch=calibration.color_size
        self.count=w*h;self.color_count=cw*ch;self.flag=self.color_count+5*self.count
        self.depth=torch.empty((self.count,),dtype=torch.int32,device='cuda:0')
        self.raw_host=torch.empty((self.count,),dtype=torch.int32,pin_memory=True)
        self.raw_array=self.raw_host.numpy()
        self.rgb_scratch=np.empty((ch,cw,3),np.uint8)
        self.output=torch.empty((self.flag+1,),dtype=torch.int32,device='cuda:0')
        self.host=torch.empty((self.flag+1,),dtype=torch.int32,pin_memory=True)
        self.host_array=self.host.numpy()
        self.rays=torch.from_numpy(np.stack(calibration.rays).reshape(6,-1)).to('cuda:0')
        parameters=np.concatenate((calibration.rotation,calibration.translation,
                                   calibration.color_k,np.array([calibration.scale],np.float32)))
        self.parameters=torch.from_numpy(parameters).to('cuda:0')
        self.hardware={'python_executable':sys.executable,'python_version':sys.version,'torch_module':torch.__file__,
                       'name':torch.cuda.get_device_name(0),'architecture':torch.cuda.get_device_properties(0).gcnArchName,
                       'torch':torch.__version__,'hip':torch.version.hip}

    def process(self, depth, rgb):
        c=self.calibration;w,h=c.depth_size;cw,ch=c.color_size
        if (not isinstance(depth,np.ndarray) or depth.dtype!=np.uint16 or depth.shape!=(h,w)
                or not isinstance(rgb,np.ndarray) or rgb.dtype!=np.uint8 or rgb.shape!=(ch,cw,3)
                or not depth.flags.c_contiguous or not rgb.flags.c_contiguous):
            raise ValueError('expected complete original nativeZ16/RGB8 cohort')
        if not self.busy.acquire(blocking=False): raise RuntimeError('one GPU job already active; no queue')
        start=time.perf_counter()
        try:
            # Freeze host input ownership before asynchronous device processing.
            np.copyto(self.raw_array,depth.reshape(-1),casting='safe')
            np.copyto(self.rgb_scratch,rgb)
            self.depth.copy_(self.raw_host,non_blocking=False)  # one H2D per cohort
            self.kernels.initialize_output[((self.color_count+255)//256,)](self.output,self.color_count,self.flag,256)
            self.kernels.project_splat[((self.count+255)//256,)](self.depth,self.rays,self.parameters,self.output,
                self.count,self.color_count,cw,ch,self.flag,256,enable_fp_fusion=False)
            self.host.copy_(self.output,non_blocking=False)  # one complete D2H, never per ROI
            if int(self.host_array[self.flag]):
                raise ValueError('GPU cohort unsupported; fault bits='+str(int(self.host_array[self.flag])))
            aligned=self.host_array[:self.color_count]
            aligned[aligned==65536]=0
            aligned=aligned.astype(np.uint16).reshape(ch,cw)
            xyz=self.host_array[self.color_count:self.color_count+3*self.count].view(np.float32).reshape(h,w,3)
            pixels=self.host_array[self.color_count+3*self.count:self.flag].view(np.float32).reshape(h,w,2)
            uv,packed,valid=self.compatibility.finish(xyz,pixels,self.rgb_scratch,c)
            xyz=xyz.copy()  # returned cohorts must survive subsequent scratch reuse
            for result in (aligned,xyz,uv,packed,valid):result.setflags(write=False)
            return ProjectionResult(aligned,xyz,uv,packed,valid,(time.perf_counter()-start)*1000)
        finally:self.busy.release()
