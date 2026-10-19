"""Explicit float32 Triton kernels. Imported only by the enabled ROCm backend."""
import triton
import triton.language as tl


@triton.jit
def initialize_output(output, COLOR_COUNT:tl.constexpr, FLAG_INDEX:tl.constexpr, BLOCK:tl.constexpr):
    i=tl.program_id(0)*BLOCK+tl.arange(0,BLOCK)
    tl.store(output+i,65536,i<COLOR_COUNT)
    if tl.program_id(0)==0: tl.store(output+FLAG_INDEX,0)


@triton.jit
def _project(x,y,z,parameters):
    # Exact generic SDK expression order, with launch enable_fp_fusion=False.
    tx=((tl.load(parameters+0)*x+tl.load(parameters+3)*y)+tl.load(parameters+6)*z)+tl.load(parameters+9)
    ty=((tl.load(parameters+1)*x+tl.load(parameters+4)*y)+tl.load(parameters+7)*z)+tl.load(parameters+10)
    tz=((tl.load(parameters+2)*x+tl.load(parameters+5)*y)+tl.load(parameters+8)*z)+tl.load(parameters+11)
    px=tl.div_rn(tx,tz)*tl.load(parameters+12)+tl.load(parameters+14)
    py=tl.div_rn(ty,tz)*tl.load(parameters+13)+tl.load(parameters+15)
    return px,py,tz


@triton.jit
def project_splat(depth,rays,parameters,output,COUNT:tl.constexpr,COLOR_COUNT:tl.constexpr,
                  COLOR_WIDTH:tl.constexpr,COLOR_HEIGHT:tl.constexpr,FLAG_INDEX:tl.constexpr,BLOCK:tl.constexpr):
    i=tl.program_id(0)*BLOCK+tl.arange(0,BLOCK);inside=i<COUNT
    raw=tl.load(depth+i,inside,other=0);active=inside&(raw!=0)
    z=raw.to(tl.float32)*tl.load(parameters+16)
    x=tl.load(rays+i,inside,other=0)*z;y=tl.load(rays+COUNT+i,inside,other=0)*z
    px,py,cz=_project(x,y,z,parameters)
    left_x,left_y,left_z=_project(tl.load(rays+2*COUNT+i,inside,other=0)*z,
                                 tl.load(rays+3*COUNT+i,inside,other=0)*z,z,parameters)
    right_x,right_y,right_z=_project(tl.load(rays+4*COUNT+i,inside,other=0)*z,
                                    tl.load(rays+5*COUNT+i,inside,other=0)*z,z,parameters)
    ax=left_x+0.5;ay=left_y+0.5;bx=right_x+0.5;by=right_y+0.5
    valid=(cz>0)&(left_z>0)&(right_z>0)&(tl.abs(x)<float('inf'))&(tl.abs(y)<float('inf'))
    valid=valid&(tl.abs(px)<float('inf'))&(tl.abs(py)<float('inf'))
    valid=valid&(tl.abs(ax)<2147483520.)&(tl.abs(ay)<2147483520.)&(tl.abs(bx)<2147483520.)&(tl.abs(by)<2147483520.)
    x0=tl.where(valid,ax,0.).to(tl.int32);y0=tl.where(valid,ay,0.).to(tl.int32)
    x1=tl.where(valid,bx,0.).to(tl.int32);y1=tl.where(valid,by,0.).to(tl.int32)
    width=x1-x0+1;height=y1-y0+1
    eligible=active&valid&(x0>=0)&(y0>=0)&(x1<COLOR_WIDTH)&(y1<COLOR_HEIGHT)&(width>0)&(height>0)
    oversized=eligible&((width>4)|(height>4))
    tl.atomic_or(output+FLAG_INDEX,2,mask=tl.sum((active&~valid).to(tl.int32),0)>0)
    tl.atomic_or(output+FLAG_INDEX,1,mask=tl.sum(oversized.to(tl.int32),0)>0)
    eligible=eligible&~oversized
    # Fixed16 atomic candidates; invalid input never changes an output pixel.
    for dy in tl.static_range(4):
        for dx in tl.static_range(4):
            target=(y0+dy)*COLOR_WIDTH+x0+dx
            tl.atomic_min(output+target,raw,mask=eligible&(dx<width)&(dy<height))
    base=COLOR_COUNT
    tl.store(output+base+3*i,x.to(tl.int32,bitcast=True),inside)
    tl.store(output+base+3*i+1,y.to(tl.int32,bitcast=True),inside)
    tl.store(output+base+3*i+2,z.to(tl.int32,bitcast=True),inside)
    pixel_base=base+3*COUNT
    tl.store(output+pixel_base+2*i,tl.where(active,px,0.).to(tl.int32,bitcast=True),inside)
    tl.store(output+pixel_base+2*i+1,tl.where(active,py,0.).to(tl.int32,bitcast=True),inside)
