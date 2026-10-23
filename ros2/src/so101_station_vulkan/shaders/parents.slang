// Original-color parent preparation only. Source scratch[0..N) is Core's
// immutable one-hot known-color mask, written independently of invalid depth.
// All offsets below are U32 WORD offsets. M=4*ceil(W/2)*ceil(H/2), G=ceil(B/256).
// arena: occupancy[M], atomicForestA[M], immutableCanonicalB[M], labelRank[M],
// keptRank[M], area[M], xmin[M], ymin[M], xmax[M], ymax[M], fullLabel[N],
// groupCounts[4G], groupOffsets[4G]. compact: 16-word header + 8-word records.
// Header: version, config(lo,hi), calibration(lo,hi), W,H, recordCount,fault,
// allParentCounts[4], reserved[3]. Parent identity is (colorOrdinal,labelID).
[[vk::binding(0,0)]] ByteAddressBuffer source;
[[vk::binding(1,0)]] RWByteAddressBuffer arena;
[[vk::binding(2,0)]] RWByteAddressBuffer compact;
struct Parameters { uint width,height,count,blockWidth,blocks,nodes,scanGroups,minArea,capacity,phase,configLo,configHi,calibrationLo,calibrationHi; };
[[vk::push_constant]] Parameters p;
uint loadWord(uint word){return arena.Load(4*word);}
void storeWord(uint word,uint value){arena.Store(4*word,value);}
void fault(uint value){uint ignored;compact.InterlockedOr(32,value,ignored);}
uint occupancy(uint j){return loadWord(j);}
// Current vs predecessor bits [0,1;2,3]. Occupied blocks alone imply NO edge.
bool edge(uint a,uint b,uint direction){
 if(direction==0)return (a&5)!=0&&(b&10)!=0;
 if(direction==1)return (a&3)!=0&&(b&12)!=0;
 if(direction==2)return (a&1)!=0&&(b&8)!=0;
 return (a&2)!=0&&(b&4)!=0;
}
[numthreads(256,1,1)]
void parents_init(uint3 tid:SV_DispatchThreadID){
 uint key=tid.x;if(key>=p.blocks)return;uint bx=key%p.blockWidth,by=key/p.blockWidth;uint occ[4]={0,0,0,0};
 [unroll]for(uint bit=0;bit<4;bit++){
  uint x=2*bx+bit%2,y=2*by+bit/2;
  if(x<p.width&&y<p.height){uint i=y*p.width+x;uint value=source.Load(4*i);
   if(value!=0&&value!=1&&value!=2&&value!=4&&value!=8)fault(1);
   [unroll]for(uint c=0;c<4;c++)if(value==(1u<<c))occ[c]|=1u<<bit;
  }
 }
 [unroll]for(uint c=0;c<4;c++){
  uint j=c*p.blocks+key,root=occ[c]!=0?j+1:0;storeWord(j,occ[c]);storeWord(p.nodes+j,root);storeWord(2*p.nodes+j,root);
  storeWord(3*p.nodes+j,0);storeWord(4*p.nodes+j,0);storeWord(5*p.nodes+j,0);
  storeWord(6*p.nodes+j,p.width);storeWord(7*p.nodes+j,p.height);storeWord(8*p.nodes+j,0);storeWord(9*p.nodes+j,0);
 }
 if(key==0){compact.Store(0,1);compact.Store(4,p.configLo);compact.Store(8,p.configHi);compact.Store(12,p.calibrationLo);compact.Store(16,p.calibrationHi);compact.Store(20,p.width);compact.Store(24,p.height);}
}
void hookEdge(uint j,uint k,uint direction){
 if(!edge(occupancy(j),occupancy(k),direction))return;
 // Read ONLY immutable B during hooks. A is updated only with atomic minima.
 uint a=loadWord(2*p.nodes+j),b=loadWord(2*p.nodes+k),ignored;
 if(a!=0&&b!=0)arena.InterlockedMin(4*(p.nodes+max(a,b)-1),min(a,b),ignored);
}
[numthreads(256,1,1)]
void parents_hook(uint3 tid:SV_DispatchThreadID){
 uint j=tid.x;if(j>=p.nodes||occupancy(j)==0)return;
 // Fold the previous canonical snapshot into A with the SAME atomic-min
 // operation as edge hooks. A plain Store could overwrite a newer lower hook.
 uint ignored;arena.InterlockedMin(4*(p.nodes+j),loadWord(2*p.nodes+j),ignored);
 uint key=j%p.blocks,x=key%p.blockWidth,y=key/p.blockWidth;
 if(x!=0)hookEdge(j,j-1,0);if(y!=0)hookEdge(j,j-p.blockWidth,1);
 if(x!=0&&y!=0)hookEdge(j,j-p.blockWidth-1,2);
 if(x+1<p.blockWidth&&y!=0)hookEdge(j,j-p.blockWidth+1,3);
}
[numthreads(256,1,1)]
void parents_flatten(uint3 tid:SV_DispatchThreadID){
 uint j=tid.x;if(j>=p.nodes||occupancy(j)==0)return;
 // Separate dispatch: A remains immutable throughout every root walk. Every
 // step must decrease strictly, stay in color/range, and hit a positive root.
 uint root=loadWord(p.nodes+j);
 for(uint steps=0;steps<p.blocks;steps++){
  if(root==0||root>j+1||(root-1)/p.blocks!=j/p.blocks){fault(4);storeWord(2*p.nodes+j,0);return;}
  uint next=loadWord(p.nodes+root-1);if(next==root){storeWord(2*p.nodes+j,root);return;}
  if(next==0||next>=root){fault(4);storeWord(2*p.nodes+j,0);return;}root=next;
 }
 fault(4);storeWord(2*p.nodes+j,0);
}
void validateEdge(uint j,uint k,uint direction){
 if(edge(occupancy(j),occupancy(k),direction)&&loadWord(2*p.nodes+j)!=loadWord(2*p.nodes+k))fault(4);
}
[numthreads(256,1,1)]
void parents_validate(uint3 tid:SV_DispatchThreadID){
 uint j=tid.x;if(j>=p.nodes||occupancy(j)==0)return;uint a=loadWord(p.nodes+j),root=loadWord(2*p.nodes+j);
 if(a==0||a>j+1||(a-1)/p.blocks!=j/p.blocks||root==0||root>j+1||(root-1)/p.blocks!=j/p.blocks){fault(4);return;}
 if(occupancy(a-1)==0||occupancy(root-1)==0||loadWord(2*p.nodes+root-1)!=root)fault(4);
 uint key=j%p.blocks,x=key%p.blockWidth,y=key/p.blockWidth;
 if(x!=0)validateEdge(j,j-1,0);if(y!=0)validateEdge(j,j-p.blockWidth,1);
 if(x!=0&&y!=0)validateEdge(j,j-p.blockWidth-1,2);
 if(x+1<p.blockWidth&&y!=0)validateEdge(j,j-p.blockWidth+1,3);
}
groupshared uint scan0[256];groupshared uint scan1[256];
[numthreads(256,1,1)]
void parents_scan(uint3 lane:SV_GroupThreadID,uint3 group:SV_GroupID){
 uint c=group.x/p.scanGroups,g=group.x%p.scanGroups,k=lane.x,key=256*g+k,j=c*p.blocks+key,flag=0;
 if(key<p.blocks){bool root=loadWord(2*p.nodes+j)==j+1;flag=uint(root&&(p.phase==0||loadWord(5*p.nodes+j)>=p.minArea));}
 scan0[k]=flag;GroupMemoryBarrierWithGroupSync();uint side=0;
 [unroll]for(uint stride=1;stride<256;stride*=2){uint v=side==0?scan0[k]:scan1[k];if(k>=stride)v+=side==0?scan0[k-stride]:scan1[k-stride];if(side==0)scan1[k]=v;else scan0[k]=v;GroupMemoryBarrierWithGroupSync();side=1-side;}
 uint rank=side==0?scan0[k]:scan1[k];
 if(key<p.blocks)storeWord((p.phase==0?3:4)*p.nodes+j,flag!=0?rank:0);
 if(k==255)storeWord(10*p.nodes+p.count+group.x,rank);
}
[numthreads(1,1,1)]
void parents_offsets(uint3 tid:SV_DispatchThreadID){
 uint c=tid.x;if(c>=4)return;uint total=0,base=10*p.nodes+p.count;
 // Each color is independently ranked, including roots below the area gate.
 for(uint g=0;g<p.scanGroups;g++){storeWord(base+4*p.scanGroups+c*p.scanGroups+g,total);total+=loadWord(base+c*p.scanGroups+g);}
 if(p.phase==0)compact.Store(4*(9+c),total);
 else if(c==3){uint all=0;for(uint g=0;g<4*p.scanGroups;g++)all+=loadWord(base+g);compact.Store(28,all);if(all>p.capacity)fault(2);}
}
uint globalRank(uint j,uint which){uint c=j/p.blocks,key=j%p.blocks;return loadWord(which*p.nodes+j)+loadWord(10*p.nodes+p.count+4*p.scanGroups+c*p.scanGroups+key/256);}
[numthreads(256,1,1)]
void parents_pixels(uint3 tid:SV_DispatchThreadID){
 uint i=tid.x;if(i>=p.count)return;uint value=source.Load(4*i),label=0;
 [unroll]for(uint c=0;c<4;c++)if(value==(1u<<c)){
  uint j=c*p.blocks+(i/p.width/2)*p.blockWidth+i%p.width/2,root=loadWord(2*p.nodes+j);
  if(root!=0){j=root-1;label=globalRank(j,3);uint ignored,x=i%p.width,y=i/p.width;
   arena.InterlockedAdd(4*(5*p.nodes+j),1,ignored);arena.InterlockedMin(4*(6*p.nodes+j),x,ignored);arena.InterlockedMin(4*(7*p.nodes+j),y,ignored);arena.InterlockedMax(4*(8*p.nodes+j),x,ignored);arena.InterlockedMax(4*(9*p.nodes+j),y,ignored);
   // Commit canonical-root local label before scan scratch is reused.
   // rank[j] is still a group-local rank until records scatter below.
  }
 }
 storeWord(10*p.nodes+i,label);
}
// Finalize all-root ranks before group-offset scratch is reused for filtering.
[numthreads(256,1,1)]
void parents_labels(uint3 tid:SV_DispatchThreadID){uint j=tid.x;if(j<p.nodes&&loadWord(2*p.nodes+j)==j+1)storeWord(3*p.nodes+j,globalRank(j,3));}
[numthreads(256,1,1)]
void parents_records(uint3 tid:SV_DispatchThreadID){
 uint j=tid.x;if(j>=p.nodes||compact.Load(32)!=0||loadWord(2*p.nodes+j)!=j+1||loadWord(5*p.nodes+j)<p.minArea)return;
 uint c=j/p.blocks,rank=globalRank(j,4)-1,base=10*p.nodes+p.count;
 for(uint g=0;g<c*p.scanGroups;g++)rank+=loadWord(base+g);
 if(rank>=p.capacity){fault(2);return;}
 uint out=64+32*rank,x=loadWord(6*p.nodes+j),y=loadWord(7*p.nodes+j);
 compact.Store(out,c);compact.Store(out+4,loadWord(3*p.nodes+j));compact.Store(out+8,j%p.blocks);compact.Store(out+12,loadWord(5*p.nodes+j));compact.Store(out+16,x);compact.Store(out+20,y);compact.Store(out+24,loadWord(8*p.nodes+j)-x+1);compact.Store(out+28,loadWord(9*p.nodes+j)-y+1);
}
