// RealSense zero-distortion scalar/SSE conventions. Offline precise FP32 only.
#include "exact_divide.slang"
struct Parameters { float r0,r1,r2,r3,r4,r5,r6,r7,r8,t0,t1,t2,fx,fy,cx,cy,scale; uint corners,convention; };
[[vk::push_constant]] ConstantBuffer<Parameters> p;
[[vk::binding(0)]] ByteAddressBuffer raw;
[[vk::binding(1)]] StructuredBuffer<float4> rays;
[[vk::binding(2)]] RWStructuredBuffer<uint> scratch;
[[vk::binding(3)]] RWByteAddressBuffer packed;
[[vk::binding(4)]] RWStructuredBuffer<float4> xyz;
// Explicit ties-to-even; HLSL round() is not our portability contract.
int nearest_even(float v) { float lo=floor(v);float d=v-lo;int i=int(lo);return i+int(d>0.5 || (d==0.5 && (i&1)!=0)); }
int2 project(float2 ray,float z){
 precise float x=ray.x*z,y=ray.y*z;
 precise float tx,ty,tz;
 if(p.convention==1){tx=p.r0*x+(p.r3*y+(p.r6*z+p.t0));ty=p.r1*x+(p.r4*y+(p.r7*z+p.t1));tz=p.r2*x+(p.r5*y+(p.r8*z+p.t2));}
 else {tx=((p.r0*x+p.r3*y)+p.r6*z)+p.t0;ty=((p.r1*x+p.r4*y)+p.r7*z)+p.t1;tz=((p.r2*x+p.r5*y)+p.r8*z)+p.t2;}
 precise float u=((alignment_divide(tx,tz))*p.fx+p.cx)+0.5,v=((alignment_divide(ty,tz))*p.fy+p.cy)+0.5;
 if(!isfinite(u)||!isfinite(v)||abs(u)>2147483000.0||abs(v)>2147483000.0)return int2(-2147483647,-2147483647);
 return p.convention==1?int2(nearest_even(u),nearest_even(v)):int2(u,v);
}
[shader("compute")][numthreads(256,1,1)]
void splat(uint3 id:SV_DispatchThreadID){
 uint i=id.x;if(i>=307200)return;uint word=raw.Load((i/2)*4);uint d=(word>>((i&1)*16))&65535;if(d==0)return;
 precise float z=float(d)*p.scale;float4 ray=rays[i];int2 lo=project(ray.xy,z),hi=p.corners!=0?project(ray.zw,z):lo;
 if(p.convention==0&&(lo.x<0||lo.y<0||hi.x>=640||hi.y>=480))return;
 lo=max(lo,int2(0,0));hi=min(hi,int2(639,479));
 if(hi.x>=lo.x&&hi.y>=lo.y&&(hi.x-lo.x+1)*(hi.y-lo.y+1)>4096){InterlockedOr(scratch[307200],1);return;}
 for(int y=lo.y;y<=hi.y;y++)for(int x=lo.x;x<=hi.x;x++)InterlockedMin(scratch[y*640+x],d);
}
float4 point(uint i,uint d){precise float z=float(d)*p.scale;precise float x=z*(alignment_divide((float(i%640)-p.cx),p.fx)),y=z*(alignment_divide((float(i/640)-p.cy),p.fy));return float4(x,y,z,0);}
[shader("compute")][numthreads(256,1,1)]
void finalize(uint3 id:SV_DispatchThreadID){uint i=id.x*2;if(i>=307200)return;uint a=scratch[i],b=scratch[i+1];a=a==0xffffffff?0:a;b=b==0xffffffff?0:b;packed.Store(i*2,a|(b<<16));xyz[i]=point(i,a);xyz[i+1]=point(i+1,b);}
