"""Bounded generic-SDK float32 projection contract; no implicit GPU activation.

Only the actual installed CPU ``Align``/``Pointcloud`` algorithm is supported.
SSE changes arithmetic and boundary policy; it is intentionally not guessed.
"""
from dataclasses import dataclass
import numpy as np

MAX_FOOTPRINT = 4


@dataclass(frozen=True)
class ProjectionCalibration:
    depth_size: tuple[int, int]
    color_size: tuple[int, int]
    depth_k: np.ndarray
    color_k: np.ndarray
    rotation: np.ndarray  # column-major flat, exact SDK float32 inputs
    translation: np.ndarray
    rays: tuple[np.ndarray, ...]  # center, top-left, bottom-right X/Y
    scale: np.float32 = np.float32(.001)
    apply_occlusion: bool = True


def _finite_vector(value, size):
    if not isinstance(value, (list, tuple)) or len(value) != size or any(type(x) is bool for x in value):
        raise ValueError('invalid finite-vector shape/type')
    try: array = np.asarray(value, dtype=np.float32)
    except (TypeError, ValueError) as error: raise ValueError('invalid numeric vector') from error
    if not np.isfinite(array).all(): raise ValueError('nonfinite calibration')
    array.setflags(write=False)
    return array


def calibration_from_oracle(metadata):
    if (not isinstance(metadata, dict) or metadata.get('schema') != 'realsense-cpu-projection-oracle-v1'
            or metadata.get('qualified') is not False or metadata.get('align_name') != 'Align'
            or metadata.get('pointcloud_name') != 'Pointcloud' or metadata.get('occlusion_option') != 2.
            or type(metadata.get('occlusion_option')) is bool
            or metadata.get('depth_scale_m') != .001):
        raise ValueError('unsupported CPU oracle/algorithm/scale')
    sizes=[]
    for name, maximum in [('depth_size',(1280,720)),('color_size',(1920,1080))]:
        size=metadata.get(name)
        if (not isinstance(size,(list,tuple)) or len(size)!=2 or
                any(type(x) is not int or not 1<=x<=bound for x,bound in zip(size,maximum))):
            raise ValueError('unsupported bounded image size')
        sizes.append(tuple(size))
    kd=_finite_vector(metadata.get('depth_K'),4); kc=_finite_vector(metadata.get('color_K'),4)
    if any(not 0<float(k[0])<=100000 or not 0<float(k[1])<=100000 or np.max(np.abs(k[2:]))>100000 for k in (kd,kc)):
        raise ValueError('invalid bounded intrinsics')
    r=_finite_vector(metadata.get('rotation_column_major'),9); t=_finite_vector(metadata.get('translation_m'),3)
    matrix=r.reshape((3,3),order='F').astype(np.float64)
    if (not np.allclose(matrix.T@matrix,np.eye(3),atol=1e-5,rtol=0)
            or not np.isclose(np.linalg.det(matrix),1,atol=1e-5,rtol=0)):
        raise ValueError('rotation is not proper SE3')
    if t[0]<np.float32(.001) and t[1]>np.float32(.01):
        raise ValueError('unsupported SDK vertical occlusion scan')
    identity=bool(np.all(t==0) and np.all(np.abs(r.reshape((3,3),order='F')-np.eye(3,dtype=np.float32))<=np.finfo(np.float32).eps))
    w,h=sizes[0]; yy,xx=np.indices((h,w),dtype=np.float32); rays=[]
    for offset in (0.,-.5,.5):
        for values,principal,focal in ((xx,kd[2],kd[0]),(yy,kd[3],kd[1])):
            ray=((values+np.float32(offset))-principal)/focal
            ray.setflags(write=False);rays.append(ray)
    return ProjectionCalibration(*sizes,kd,kc,r,t,tuple(rays),apply_occlusion=not identity)


def _project(x,y,z,c):
    r,t=c.rotation,c.translation
    # Explicit generic rs2_transform_point_to_point left-associative ordering.
    tx=((r[0]*x+r[3]*y)+r[6]*z)+t[0]
    ty=((r[1]*x+r[4]*y)+r[7]*z)+t[1]
    tz=((r[2]*x+r[5]*y)+r[8]*z)+t[2]
    with np.errstate(divide='ignore',invalid='ignore',over='ignore'):
        px=(tx/tz)*c.color_k[0]+c.color_k[2]
        py=(ty/tz)*c.color_k[1]+c.color_k[3]
    return np.stack((px,py),-1),tz


def numpy_raw_projection(raw,c):
    """CPU test oracle for GPU raw geometry, before retained CPU occlusion.

    Returns aligned raw-Z16, native XYZ, projected color pixels and fault bits.
    Fault1: eligible splat exceeds fixed4x4; fault2: unsupported projection.
    Faults mean unavailable; callers must not publish partial output.
    """
    if (not isinstance(raw,np.ndarray) or raw.dtype!=np.uint16
            or raw.shape!=(c.depth_size[1],c.depth_size[0])):
        raise ValueError('expected exact native Z16 shape')
    z=raw.astype(np.float32)*c.scale; active=raw!=0
    x=c.rays[0]*z; y=c.rays[1]*z
    pixels,tz=_project(x,y,z,c)
    pixels=np.where(active[...,None],pixels,np.float32(0))
    xyz=np.stack((x,y,z),-1)
    corners=[]; supported=np.ones(raw.shape,bool)
    for rayx,rayy in ((c.rays[2],c.rays[3]),(c.rays[4],c.rays[5])):
        projected,corner_z=_project(rayx*z,rayy*z,z,c)
        rounded=projected+np.float32(.5)
        valid=np.isfinite(rounded).all(-1)&(corner_z>0)&(np.abs(rounded).max(-1)<2147483520.)
        supported &= valid|~active
        corners.append(np.trunc(np.where(valid[...,None],rounded,0)).astype(np.int32))
    supported &= (~active)|((tz>0)&np.isfinite(pixels).all(-1))
    flags=2 if not supported.all() else 0
    (x0,y0),(x1,y1)=[(a[...,0],a[...,1]) for a in corners]
    cw,ch=c.color_size; widths=x1-x0+1; heights=y1-y0+1
    eligible=active&supported&(x0>=0)&(y0>=0)&(x1<cw)&(y1<ch)&(widths>0)&(heights>0)
    oversized=eligible&((widths>MAX_FOOTPRINT)|(heights>MAX_FOOTPRINT))
    if oversized.any(): flags |= 1
    eligible &= ~oversized
    aligned=np.full(cw*ch,65536,np.int32)
    # A fixed16-way splat, never a per-pixel Python loop or dynamic allocation by footprint.
    for dy in range(MAX_FOOTPRINT):
        for dx in range(MAX_FOOTPRINT):
            use=eligible&(dx<widths)&(dy<heights)
            indices=(y0[use]+dy)*cw+x0[use]+dx
            np.minimum.at(aligned,indices,raw[use])
    aligned=np.where(aligned==65536,0,aligned).astype(np.uint16).reshape(ch,cw)
    return aligned,xyz,pixels,flags


class CPUCompatibility:
    """Load the compiled fixed compatibility stage once, outside frame processing."""
    def __init__(self, path):
        import ctypes
        self.library=ctypes.CDLL(str(path))
        try: abi=self.library.rocm_projection_compat_abi()
        except AttributeError as error: raise ValueError('CPU compatibility ABI missing') from error
        if abi!=1: raise ValueError('unsupported CPU compatibility ABI')
        self.function=self.library.rocm_projection_finish
        array=lambda dtype,ndim,write=False:np.ctypeslib.ndpointer(dtype=dtype,ndim=ndim,
            flags=('C_CONTIGUOUS','ALIGNED','WRITEABLE') if write else ('C_CONTIGUOUS','ALIGNED'))
        self.function.argtypes=[array(np.float32,3,True),array(np.float32,3),array(np.float32,3,True),
            array(np.uint32,2,True),array(np.uint8,2,True),array(np.uint8,3),ctypes.c_size_t,
            ctypes.c_int,ctypes.c_int,ctypes.c_int,ctypes.c_int,ctypes.c_int]
        self.function.restype=ctypes.c_int

    def finish(self, xyz, pixels, rgb_image, calibration):
        w,h=calibration.depth_size;cw,ch=calibration.color_size
        if (xyz.shape!=(h,w,3) or pixels.shape!=(h,w,2) or rgb_image.shape!=(ch,cw,3)
                or xyz.dtype!=np.float32 or pixels.dtype!=np.float32 or rgb_image.dtype!=np.uint8
                or not all(a.flags.c_contiguous for a in (xyz,pixels,rgb_image)) or not xyz.flags.writeable):
            raise ValueError('incompatible complete CPU cohort')
        uv=np.empty((h,w,2),np.float32);packed=np.empty((h,w),np.uint32);valid=np.empty((h,w),np.uint8)
        error=self.function(xyz,pixels,uv,packed,valid,rgb_image,w*h,w,h,cw,ch,int(calibration.apply_occlusion))
        if error: raise ValueError('CPU projection compatibility failed: '+str(error))
        return uv,packed,valid
