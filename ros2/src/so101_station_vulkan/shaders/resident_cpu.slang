// Native LLVM-only whole-frame preparation. One invocation owns each stage.
// Explicit host loops avoid per-pixel callback overhead in CPU dispatch lowering.
#include "resident.slang"
RWStructuredBuffer<uint8_t> cpuMasks;
RWStructuredBuffer<float> cpuDepth;
RWStructuredBuffer<float> cpuXYZ;
RWStructuredBuffer<uint> cpuParents;
[numthreads(1,1,1)]
void cpu_convert(uint3 tid:SV_DispatchThreadID){
 [loop]for(uint i=0;i<p.count;i++){uint at=3*p.count+2*i;uint raw=byteAt(at)|(byteAt(at+1)<<8);
 precise float d=float(raw)*p.scale;
 cpuMasks[i]=uint8_t(maskAt(i));cpuDepth[i]=d;
 precise float x=(float(i%p.width)-p.cx)/p.fx;
 precise float y=(float(i/p.width)-p.cy)/p.fy;
 cpuXYZ[4*i]=x*d;cpuXYZ[4*i+1]=y*d;cpuXYZ[4*i+2]=d;cpuXYZ[4*i+3]=raw!=0?1.0:0.0;
}}
bool cpu_sameDepthNeighbour(uint i,uint j){return cpuMasks[j]==cpuMasks[i]&&valid(cpuDepth[j])&&abs(cpuDepth[i]-cpuDepth[j])>p.jump;}
[numthreads(1,1,1)]
void cpu_candidates(uint3 tid:SV_DispatchThreadID){
 [loop]for(uint i=0;i<p.count;i++){uint m=uint(cpuMasks[i]),x=i%p.width,y=i/p.width;float d=cpuDepth[i];bool ok=m!=0&&valid(d),cut=false;
 if(ok)cut=(x>0&&cpu_sameDepthNeighbour(i,i-1))||(x+1<p.width&&cpu_sameDepthNeighbour(i,i+1))||(y>0&&cpu_sameDepthNeighbour(i,i-p.width))||(y+1<p.height&&cpu_sameDepthNeighbour(i,i+p.width));
 cpuParents[i]=ok&&!cut?i+1:0;
}}
