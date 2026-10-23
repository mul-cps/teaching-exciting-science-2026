// Preparation gate. No contours/PCA/plane-fit acceptance is implied.
// arena: FP64 boardXYZ[N], uint flags[N], uint localRank[G],
// uint blockCount[B], uint blockOffset[B], FP64 selectedXYZ[G], uint pixel[G].
// G=ceil(W/8)*ceil(H/8), B=ceil(G/256). All capacities image bounded.
[[vk::binding(0,0)]] ByteAddressBuffer source;
[[vk::binding(1,0)]] StructuredBuffer<double> calibration;
[[vk::binding(2,0)]] RWByteAddressBuffer arena;
[[vk::binding(3,0)]] RWByteAddressBuffer counts;
struct Parameters { uint width,height,count,gridCount,gridWidth,blocks; };
[[vk::push_constant]] Parameters p;
uint flagsOffset(){return 24*p.count;}
uint ranksOffset(){return 28*p.count;}
uint blockCountsOffset(){return ranksOffset()+4*p.gridCount;}
uint blockOffsetsOffset(){return blockCountsOffset()+4*p.blocks;}
uint selectedOffset(){return (blockOffsetsOffset()+4*p.blocks+7)&~7u;}
uint pixelsOffset(){return selectedOffset()+24*p.gridCount;}
double transform(uint row,double x,double y,double z){
 uint base=2*p.count+4*row;
 precise double value=calibration[base]*x;
 value=value+calibration[base+1]*y;
 value=value+calibration[base+2]*z;
 return value+calibration[base+3];
}
[numthreads(256,1,1)]
void geometry_coordinates(uint3 tid:SV_DispatchThreadID){
 uint i=tid.x;if(i>=p.count)return;
 float z32=asfloat(source.Load(4*(p.count+i)));
 double z=double(z32),x=calibration[2*i]*z,y=calibration[2*i+1]*z;
 double bx=transform(0,x,y,z),by=transform(1,x,y,z),bz=transform(2,x,y,z);
 arena.Store<double>(24*i,bx);arena.Store<double>(24*i+8,by);arena.Store<double>(24*i+16,bz);
 uint base=2*p.count+12;
 bool valid=isfinite(z32)&&z>=double(.10f)&&z<=calibration[base+6];
 // Retained valid depth compares float32 array against a scalar cast by NumPy
 // to float32. Host stores that float32-rounded upper limit as double.
 bool excluded=false;int px=int(i%p.width),py=int(i/p.width);
 [unroll]for(int dy=-3;dy<=3;dy++)[unroll]for(int dx=-3;dx<=3;dx++){
  int sx=px+dx,sy=py+dy;
  if(sx>=0&&sy>=0&&sx<int(p.width)&&sy<int(p.height))excluded|=source.Load(4*(uint(sy)*p.width+uint(sx)))!=0;
 }
 bool inside=bx>=calibration[base]&&bx<=calibration[base+1]&&by>=calibration[base+2]&&by<=calibration[base+3]&&bz>=calibration[base+4]&&bz<=calibration[base+5];
 arena.Store(flagsOffset()+4*i,uint(valid)|(uint(excluded)<<1)|(uint(inside)<<2));
}
// Inclusive Hillis-Steele scan on at most 256 ordered stride-grid samples.
// Separate shared arrays avoid read/write races at each scan step.
groupshared uint scan0[256];groupshared uint scan1[256];
[numthreads(256,1,1)]
void geometry_support_scan(uint3 tid:SV_DispatchThreadID,uint3 lane:SV_GroupThreadID,uint3 group:SV_GroupID){
 uint j=tid.x,k=lane.x,flag=0;
 if(j<p.gridCount){uint x=(j%p.gridWidth)*8,y=(j/p.gridWidth)*8;uint f=arena.Load(flagsOffset()+4*(y*p.width+x));flag=uint((f&7)==5);}
 scan0[k]=flag;GroupMemoryBarrierWithGroupSync();
 uint side=0;
 [unroll]for(uint stride=1;stride<256;stride*=2){
  uint value=side==0?scan0[k]:scan1[k];
  if(k>=stride)value+=side==0?scan0[k-stride]:scan1[k-stride];
  if(side==0)scan1[k]=value;else scan0[k]=value;
  GroupMemoryBarrierWithGroupSync();side=1-side;
 }
 uint rank=side==0?scan0[k]:scan1[k];
 if(j<p.gridCount)arena.Store(ranksOffset()+4*j,flag!=0?rank-1:0xffffffff);
 if(k==255)arena.Store(blockCountsOffset()+4*group.x,rank);
}
[numthreads(1,1,1)]
void geometry_support_offsets(uint3 tid:SV_DispatchThreadID){
 if(tid.x!=0)return;uint total=0;
 for(uint b=0;b<p.blocks;b++){arena.Store(blockOffsetsOffset()+4*b,total);total+=arena.Load(blockCountsOffset()+4*b);}
 counts.Store(4,total);
}
[numthreads(256,1,1)]
void geometry_support_scatter(uint3 tid:SV_DispatchThreadID){
 uint j=tid.x;if(j>=p.gridCount)return;uint rank=arena.Load(ranksOffset()+4*j);if(rank==0xffffffff)return;
 rank+=arena.Load(blockOffsetsOffset()+4*(j/256));
 uint i=(j/p.gridWidth)*8*p.width+(j%p.gridWidth)*8;
 arena.Store<double>(selectedOffset()+24*rank,arena.Load<double>(24*i));
 arena.Store<double>(selectedOffset()+24*rank+8,arena.Load<double>(24*i+8));
 arena.Store<double>(selectedOffset()+24*rank+16,arena.Load<double>(24*i+16));
 arena.Store(pixelsOffset()+4*rank,i);
}
// Exact integer reduction per workgroup; only one atomic per 256 pixels.
groupshared uint validCounts[256];
[numthreads(256,1,1)]
void geometry_valid_count(uint3 tid:SV_DispatchThreadID,uint3 lane:SV_GroupThreadID){
 uint i=tid.x,k=lane.x;uint value=0;if(i<p.count)value=arena.Load(flagsOffset()+4*i)&1;
 validCounts[k]=value;GroupMemoryBarrierWithGroupSync();
 for(uint step=128;step>0;step/=2){if(k<step)validCounts[k]+=validCounts[k+step];GroupMemoryBarrierWithGroupSync();}
 if(k==0){uint ignored;counts.InterlockedAdd(0,validCounts[0],ignored);}
}
