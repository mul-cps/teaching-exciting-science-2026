// Child preparation only; NO observed-ROI closing/report/plane/PCA stage.
// All offsets in helpers are U32 WORDS, Buffer Load/Store byte-addressed.
// Workspace graph: A[N], B[N], area/xmin/ymin/xmax/ymax/K/parentIndex[7N].
// draftWords=max(9N,32*groups) preserves live 16*C-word drafts during radix.
// Then workspace aliases complement A[S], B[S], boundaryRoot[S].
[[vk::binding(0,0)]] ByteAddressBuffer source;
[[vk::binding(1,0)]] ByteAddressBuffer parentLabels;
[[vk::binding(2,0)]] ByteAddressBuffer parents;
[[vk::binding(3,0)]] RWByteAddressBuffer work;
[[vk::binding(4,0)]] RWByteAddressBuffer persistent;
[[vk::binding(5,0)]] RWByteAddressBuffer lookup;
[[vk::binding(6,0)]] RWByteAddressBuffer packed;
[[vk::binding(7,0)]] RWByteAddressBuffer jobs;
[[vk::binding(8,0)]] RWByteAddressBuffer compact;
[[vk::binding(9,0)]] RWByteAddressBuffer pairs;
[[vk::binding(10,0)]] RWByteAddressBuffer arguments;
struct Parameters{uint width,height,count,blocks,parentCapacity,capacity,packedCapacity,jobCapacity,keyBits,groups,maxBits,jumpBits,minChild,phase,radixShift,radixSide,calLo,calHi,parentLo,parentHi,childLo,childHi,maxLo,maxHi,jumpLo,jumpHi,draftWords,lookupCount,parentLabelWords,reserved0,reserved1,reserved2;};
[[vk::push_constant]] Parameters p;
uint wl(uint i){return work.Load(4*i);}void ws(uint i,uint v){work.Store(4*i,v);}
uint pl(uint i){return persistent.Load(4*i);}void ps(uint i,uint v){persistent.Store(4*i,v);}
uint hl(uint i){return compact.Load(4*i);}void hs(uint i,uint v){compact.Store(4*i,v);}
void fault(uint bit){uint ignored;compact.InterlockedOr(28,bit,ignored);}
uint parentWord(uint j,uint field){return parents.Load(64+32*j+4*field);}
uint recordBase(uint r){return 32+4*p.parentCapacity+16*r;}
uint recordWord(uint r,uint field){return hl(recordBase(r)+field);}
void recordStore(uint r,uint field,uint v){hs(recordBase(r)+field,v);}
uint selectedParent(uint i){uint m=source.Load(4*i),color=0;if(m==1)color=0;else if(m==2)color=1;else if(m==4)color=2;else if(m==8)color=3;else return 0;uint label=parentLabels.Load(4*(p.parentLabelWords+i));if(label==0||label>p.blocks){fault(1);return 0;}return lookup.Load(4*(color*(p.blocks+1)+label));}
bool validDepth(uint i){float z=asfloat(source.Load(4*(p.count+i)));return isfinite(z)&&z>=asfloat(0x3dcccccdu)&&z<=asfloat(p.maxBits);}
bool badEdge(uint a,uint b){return abs(asfloat(source.Load(4*(p.count+a)))-asfloat(source.Load(4*(p.count+b))))>asfloat(p.jumpBits);}
uint localKey(uint x,uint y,uint px,uint py,uint width){return ((y-py)/2)*((width+1)/2)+(x-px)/2;}
bool candidate(uint i){return selectedParent(i)!=0&&(pl(p.count+i)&3)==1;}
uint satAdd(uint a,uint b,uint cap){if(a>cap||b>cap||a>cap-b)return cap+1;return a+b;}
// Shared tables have exclusive lane init, then ATOMIC-only updates, then reads
// after a workgroup barrier. No atomic/plain read/write overlap during updates.
groupshared uint hashKeys[256];groupshared uint sums[256];groupshared uint sums2[256];
groupshared uint xmin[256],ymin[256],xmax[256],ymax[256],keys[256];
groupshared uint groupFault,groupCount;
uint faultSnapshot(){uint value;compact.InterlockedOr(28,0,value);return value;}
void faultGroup(uint lane){if(lane==0)groupFault=faultSnapshot();GroupMemoryBarrierWithGroupSync();}
// groupCount is read only where gather has completed and R is immutable.
void groupState(uint lane){if(lane==0){groupFault=faultSnapshot();groupCount=hl(4);}GroupMemoryBarrierWithGroupSync();}
uint hashSlot(uint key){uint slot=(key*0x9e3779b9u)>>24;for(uint probe=0;probe<256;probe++){uint old;InterlockedCompareExchange(hashKeys[slot],0,key,old);if(old==0||old==key)return slot;slot=(slot+1)&255;}fault(64);return 256;}
[numthreads(256,1,1)]
void child_lookup(uint3 tid:SV_DispatchThreadID){uint i=tid.x,count=parents.Load(28);if(i==0){
 bool ok=parents.Load(0)==1&&parents.Load(32)==0&&parents.Load(20)==p.width&&parents.Load(24)==p.height&&parents.Load(4)==p.parentLo&&parents.Load(8)==p.parentHi&&parents.Load(12)==p.calLo&&parents.Load(16)==p.calHi&&count<=p.parentCapacity;if(!ok)fault(1);
 hs(0,1);hs(1,p.width);hs(2,p.height);hs(3,min(count,p.parentCapacity));hs(8,p.calLo);hs(9,p.calHi);hs(10,p.parentLo);hs(11,p.parentHi);hs(12,p.childLo);hs(13,p.childHi);hs(14,p.maxBits);hs(15,p.jumpBits);hs(16,p.minChild);hs(17,p.capacity);hs(18,p.packedCapacity);hs(19,p.jobCapacity);hs(20,p.parentCapacity);hs(22,p.maxLo);hs(23,p.maxHi);hs(24,p.jumpLo);hs(25,p.jumpHi);
 }if(i>=min(count,p.parentCapacity))return;uint c=parentWord(i,0),label=parentWord(i,1);if(c>=4||label==0||label>p.blocks){fault(1);return;}lookup.Store(4*(c*(p.blocks+1)+label),i+1);
}
[numthreads(256,1,1)]
void child_init(uint3 tid:SV_DispatchThreadID,uint3 lane:SV_GroupThreadID){
 uint i=tid.x,k=lane.x;hashKeys[k]=sums[k]=sums2[k]=0;GroupMemoryBarrierWithGroupSync();
 if(i<p.count){uint par=selectedParent(i);bool valid=validDepth(i),cut=false;
  if(par!=0&&valid){uint x=i%p.width,y=i/p.width;if(x>0&&selectedParent(i-1)==par&&validDepth(i-1)&&badEdge(i,i-1))cut=true;if(x+1<p.width&&selectedParent(i+1)==par&&validDepth(i+1)&&badEdge(i,i+1))cut=true;if(y>0&&selectedParent(i-p.width)==par&&validDepth(i-p.width)&&badEdge(i,i-p.width))cut=true;if(y+1<p.height&&selectedParent(i+p.width)==par&&validDepth(i+p.width)&&badEdge(i,i+p.width))cut=true;}
  ps(p.count+i,uint(valid)|(uint(cut)<<1));uint root=par!=0&&valid&&!cut?i+1:0;ws(i,root);ws(p.count+i,root);ws(2*p.count+i,0);ws(3*p.count+i,p.width);ws(4*p.count+i,p.height);ws(5*p.count+i,0);ws(6*p.count+i,0);ws(7*p.count+i,0xffffffff);ws(8*p.count+i,0);
  // Status valid is precut parent&valid; candidate is the post-cut graph.
  if(par!=0&&valid){uint slot=hashSlot(par);if(slot<256){uint ignored;InterlockedAdd(sums[slot],1,ignored);if(!cut)InterlockedAdd(sums2[slot],1,ignored);}}
 }
 GroupMemoryBarrierWithGroupSync();if(hashKeys[k]!=0){uint ignored;compact.InterlockedAdd(4*(32+4*(hashKeys[k]-1)),sums[k],ignored);compact.InterlockedAdd(4*(33+4*(hashKeys[k]-1)),sums2[k],ignored);}
}
void childEdge(uint i,uint j,uint par,uint root){uint other=wl(p.count+j);if(other==0||selectedParent(j)!=par)return;if(root==0||root>i+1||other>j+1||selectedParent(root-1)!=par||selectedParent(other-1)!=par){fault(4);return;}if(other==root)return;uint ignored;work.InterlockedMin(4*(max(root,other)-1),min(root,other),ignored);}
[numthreads(256,1,1)]
void child_hook(uint3 tid:SV_DispatchThreadID){uint i=tid.x;if(i>=p.count)return;uint root=wl(p.count+i);if(root==0)return;uint par=selectedParent(i),x=i%p.width,y=i/p.width;if(par==0||root>i+1||selectedParent(root-1)!=par){fault(4);return;}uint ignored;work.InterlockedMin(4*i,root,ignored);if(x>0)childEdge(i,i-1,par,root);if(y>0){childEdge(i,i-p.width,par,root);if(x>0)childEdge(i,i-p.width-1,par,root);if(x+1<p.width)childEdge(i,i-p.width+1,par,root);}}
[numthreads(256,1,1)]
void child_jump(uint3 tid:SV_DispatchThreadID){uint i=tid.x;if(i>=p.count)return;uint a=wl(i);if(a==0){ws(p.count+i,0);return;}if(a>i+1||selectedParent(a-1)!=selectedParent(i)){fault(4);ws(p.count+i,0);return;}uint next=wl(a-1);if(next==0||next>a||selectedParent(next-1)!=selectedParent(i)){fault(4);ws(p.count+i,0);return;}ws(p.count+i,next);}
void validateChildEdge(uint i,uint j,uint par,uint root){uint other=wl(p.count+j);if(other!=0&&selectedParent(j)==par&&root!=other)fault(4);}
[numthreads(256,1,1)]
void child_validate(uint3 tid:SV_DispatchThreadID){uint i=tid.x;if(i>=p.count)return;uint root=wl(p.count+i);ps(i,root);if(root==0){if(candidate(i))fault(4);return;}uint par=selectedParent(i),a=wl(i);if(par==0||a==0||a>i+1||root>i+1||selectedParent(a-1)!=par||selectedParent(root-1)!=par||wl(p.count+root-1)!=root){fault(4);return;}uint x=i%p.width,y=i/p.width;if(x>0)validateChildEdge(i,i-1,par,root);if(y>0){validateChildEdge(i,i-p.width,par,root);if(x>0)validateChildEdge(i,i-p.width-1,par,root);if(x+1<p.width)validateChildEdge(i,i-p.width+1,par,root);}}
[numthreads(256,1,1)]
void child_reduce(uint3 tid:SV_DispatchThreadID,uint3 lane:SV_GroupThreadID){uint k=lane.x,i=tid.x;groupState(k);if(groupFault!=0)return;hashKeys[k]=sums[k]=0;xmin[k]=p.width;ymin[k]=p.height;xmax[k]=ymax[k]=0;keys[k]=0xffffffff;GroupMemoryBarrierWithGroupSync();
 if(i<p.count){uint root=pl(i);if(root!=0){uint par=selectedParent(i),x=i%p.width,y=i/p.width;if(par==0||root>i+1||selectedParent(root-1)!=par){fault(4);return;}if(par==0||par>hl(3)){fault(64);}else{
  uint px=parentWord(par-1,4),py=parentWord(par-1,5),pw=parentWord(par-1,6),ph=parentWord(par-1,7);
  if(x<px||y<py||x>=px+pw||y>=py+ph)fault(64);else{uint key=localKey(x,y,px,py,pw),slot=hashSlot(root);if(slot<256){uint ignored;InterlockedAdd(sums[slot],1,ignored);InterlockedMin(xmin[slot],x,ignored);InterlockedMin(ymin[slot],y,ignored);InterlockedMax(xmax[slot],x,ignored);InterlockedMax(ymax[slot],y,ignored);InterlockedMin(keys[slot],key,ignored);}}
  // Exactly one source pixel owns this plain parent-index write.
  if(root==i+1)ws(8*p.count+i,par-1);
 }}}
 GroupMemoryBarrierWithGroupSync();uint root=hashKeys[k];if(root!=0){uint j=root-1,ignored;work.InterlockedAdd(4*(2*p.count+j),sums[k],ignored);work.InterlockedMin(4*(3*p.count+j),xmin[k],ignored);work.InterlockedMin(4*(4*p.count+j),ymin[k],ignored);work.InterlockedMax(4*(5*p.count+j),xmax[k],ignored);work.InterlockedMax(4*(6*p.count+j),ymax[k],ignored);work.InterlockedMin(4*(7*p.count+j),keys[k],ignored);}
}
[numthreads(256,1,1)]
void child_gather(uint3 tid:SV_DispatchThreadID,uint3 lane:SV_GroupThreadID){faultGroup(lane.x);if(groupFault!=0)return;uint i=tid.x;if(i>=p.count||pl(i)!=i+1)return;uint parent=wl(8*p.count+i),area=wl(2*p.count+i),ignored;if(parent>=hl(3)||area==0){fault(64);return;}compact.InterlockedAdd(4*(34+4*parent),1,ignored);if(area<p.minChild)return;compact.InterlockedAdd(4*(35+4*parent),1,ignored);uint slot;compact.InterlockedAdd(16,1,slot);if(slot>=p.capacity){fault(2);return;}uint key=wl(7*p.count+i);if(key>=p.blocks){fault(64);return;}uint x=wl(3*p.count+i)-parentWord(parent,4),y=wl(4*p.count+i)-parentWord(parent,5);uint base=p.draftWords+16*slot;ws(base,i+1);ws(base+1,parent);ws(base+2,key);ws(base+3,area);ws(base+4,x);ws(base+5,y);ws(base+6,wl(5*p.count+i)-wl(3*p.count+i)+1);ws(base+7,wl(6*p.count+i)-wl(4*p.count+i)+1);for(uint n=8;n<16;n++)ws(base+n,0);pairs.Store(8*slot,(parent<<p.keyBits)|key);pairs.Store(8*slot+4,slot);}
groupshared uint digits[256];
[numthreads(256,1,1)]
void child_radix_hist(uint3 lane:SV_GroupThreadID,uint3 group:SV_GroupID){uint k=lane.x,i=group.x*256+k;groupState(k);if(groupFault!=0)return;uint digit=0xffffffff;if(i<groupCount&&i<p.capacity)digit=(pairs.Load(4*(2*p.capacity*p.radixSide+2*i))>>p.radixShift)&15;digits[k]=digit;GroupMemoryBarrierWithGroupSync();if(k<16){uint count=0;for(uint n=0;n<256;n++)count+=uint(digits[n]==k);ws(k*p.groups+group.x,count);}}
[numthreads(16,1,1)]
void child_radix_offsets(uint3 lane:SV_GroupThreadID){faultGroup(lane.x);if(groupFault!=0)return;uint bin=lane.x;if(bin>=16)return;uint total=0;for(uint b=0;b<bin;b++)for(uint g=0;g<p.groups;g++)total+=wl(b*p.groups+g);for(uint g=0;g<p.groups;g++){ws(16*p.groups+bin*p.groups+g,total);total+=wl(bin*p.groups+g);}}
[numthreads(256,1,1)]
void child_radix_scatter(uint3 lane:SV_GroupThreadID,uint3 group:SV_GroupID){uint k=lane.x,i=group.x*256+k;groupState(k);if(groupFault!=0)return;uint key=0,digit=0xffffffff,index=0;if(i<groupCount&&i<p.capacity){key=pairs.Load(4*(2*p.capacity*p.radixSide+2*i));index=pairs.Load(4*(2*p.capacity*p.radixSide+2*i+1));digit=(key>>p.radixShift)&15;}digits[k]=digit;GroupMemoryBarrierWithGroupSync();if(i>=groupCount||i>=p.capacity)return;uint rank=0;for(uint n=0;n<k;n++)rank+=uint(digits[n]==digit);uint out=wl(16*p.groups+digit*p.groups+group.x)+rank;if(out>=groupCount||out>=p.capacity){fault(64);return;}uint base=2*p.capacity*(1-p.radixSide)+2*out;pairs.Store(4*base,key);pairs.Store(4*(base+1),index);}
[numthreads(256,1,1)]
void child_scatter(uint3 tid:SV_DispatchThreadID,uint3 lane:SV_GroupThreadID){groupState(lane.x);if(groupFault!=0)return;uint i=tid.x;if(i>=min(groupCount,p.capacity))return;uint index=pairs.Load(8*i+4);if(index>=hl(4)||index>=p.capacity){fault(64);return;}uint base=p.draftWords+16*index;for(uint n=0;n<16;n++)recordStore(i,n,wl(base+n));uint parent=wl(base+1),x=wl(base+4),y=wl(base+5),w=wl(base+6),h=wl(base+7),pw=parentWord(parent,6),ph=parentWord(parent,7);uint left=x>2?x-2:0,top=y>2?y-2:0;recordStore(i,8,left);recordStore(i,9,top);recordStore(i,10,min(pw,x+w+2)-left);recordStore(i,11,min(ph,y+h+2)-top);}
groupshared uint2 prefix0[256],prefix1[256];
[numthreads(256,1,1)]
void child_pack_scan(uint3 lane:SV_GroupThreadID,uint3 group:SV_GroupID){uint k=lane.x,i=group.x*256+k;groupState(k);if(groupFault!=0)return;uint size=0,tiles=0;if(i<groupCount&&i<p.capacity){size=recordWord(i,10)*recordWord(i,11);tiles=(size+255)/256;}prefix0[k]=uint2(size,tiles);GroupMemoryBarrierWithGroupSync();uint side=0;for(uint stride=1;stride<256;stride*=2){uint2 value=side==0?prefix0[k]:prefix1[k];if(k>=stride){uint2 before=side==0?prefix0[k-stride]:prefix1[k-stride];value=uint2(satAdd(value.x,before.x,p.packedCapacity),satAdd(value.y,before.y,p.jobCapacity));}if(side==0)prefix1[k]=value;else prefix0[k]=value;GroupMemoryBarrierWithGroupSync();side=1-side;}uint2 rank=side==0?prefix0[k]:prefix1[k];if(i<groupCount&&i<p.capacity){pairs.Store(8*i,rank.x<=p.packedCapacity?rank.x-size:0);pairs.Store(8*i+4,rank.y<=p.jobCapacity?rank.y-tiles:0);}if(k==255){ws(group.x,rank.x);ws(p.groups+group.x,rank.y);}}
[numthreads(1,1,1)]
void child_pack_offsets(uint3 tid:SV_DispatchThreadID){if(tid.x!=0)return;uint total=0,jobsCount=0;if(faultSnapshot()==0){for(uint g=0;g<p.groups;g++){ws(2*p.groups+g,total);ws(3*p.groups+g,jobsCount);total=satAdd(total,wl(g),p.packedCapacity);jobsCount=satAdd(jobsCount,wl(p.groups+g),p.jobCapacity);}if(total>p.packedCapacity)fault(8);if(jobsCount>p.jobCapacity)fault(16);}if(faultSnapshot()!=0){hs(5,0);hs(6,0);arguments.Store(0,0);arguments.Store(4,0);arguments.Store(8,0);}else{hs(5,total);hs(6,jobsCount);arguments.Store(0,jobsCount);arguments.Store(4,1);arguments.Store(8,1);}}
[numthreads(256,1,1)]
void child_jobs(uint3 lane:SV_GroupThreadID,uint3 group:SV_GroupID){groupState(lane.x);if(groupFault!=0)return;uint r=group.x;if(r>=min(groupCount,p.capacity))return;uint size=recordWord(r,10)*recordWord(r,11),tiles=(size+255)/256,offset=pairs.Load(8*r)+wl(2*p.groups+r/256),job=pairs.Load(8*r+4)+wl(3*p.groups+r/256);if(offset>p.packedCapacity||size>p.packedCapacity-offset||job>p.jobCapacity||tiles>p.jobCapacity-job){fault(64);return;}if(lane.x==0)recordStore(r,12,offset);for(uint t=lane.x;t<tiles;t+=256){uint base=16*(job+t);jobs.Store(base,r);jobs.Store(base+4,t*256);jobs.Store(base+8,min(256,size-t*256));jobs.Store(base+12,0);}}
bool jobPixel(uint job,uint lane,out uint r,out uint local,out uint index){r=0;local=0;index=0;if(job>=min(hl(6),p.jobCapacity))return false;r=jobs.Load(16*job);if(r>=min(hl(4),p.capacity))return false;uint start=jobs.Load(16*job+4),count=jobs.Load(16*job+8),size=recordWord(r,10)*recordWord(r,11);if(count==0||count>256||lane>=count||start>=size||lane>=size-start)return false;local=start+lane;uint offset=recordWord(r,12);if(offset>=p.packedCapacity||local>=p.packedCapacity-offset)return false;index=offset+local;return index<hl(5);}
[numthreads(256,1,1)]
void child_packed_init(uint3 lane:SV_GroupThreadID,uint3 group:SV_GroupID){faultGroup(lane.x);if(groupFault!=0)return;uint r,local,i;if(!jobPixel(group.x,lane.x,r,local,i))return;uint hw=recordWord(r,10),hh=recordWord(r,11),lx=local%hw,ly=local/hw,parent=recordWord(r,1);uint px=parentWord(parent,4)+recordWord(r,8),py=parentWord(parent,5)+recordWord(r,9),global=(py+ly)*p.width+px+lx,root=recordWord(r,0);bool region=pl(global)==root,ownParent=selectedParent(global)==parent+1,nearby=false;for(int dy=-2;dy<=2;dy++)for(int dx=-2;dx<=2;dx++){int x=int(lx)+dx,y=int(ly)+dy;if(x>=0&&y>=0&&x<int(hw)&&y<int(hh))nearby|=pl((py+uint(y))*p.width+px+uint(x))==root;}bool cut=ownParent&&(pl(p.count+global)&2)!=0;packed.Store(4*i,uint(region)|(uint(ownParent)<<1)|(uint(nearby)<<2)|(uint(cut)<<3));uint a=region?0:i+1;ws(i,a);ws(p.packedCapacity+i,a);ws(2*p.packedCapacity+i,0);}
void fillEdge(uint i,uint j,uint root,uint begin,uint end){uint other=wl(p.packedCapacity+j);if(other==0)return;if(root<=begin||root>end||root>i+1||other<=begin||other>end||other>j+1){fault(32);return;}if(other==root)return;uint ignored;work.InterlockedMin(4*(max(root,other)-1),min(root,other),ignored);}
[numthreads(256,1,1)]
void child_fill_hook(uint3 lane:SV_GroupThreadID,uint3 group:SV_GroupID){faultGroup(lane.x);if(groupFault!=0)return;uint r,local,i;if(!jobPixel(group.x,lane.x,r,local,i))return;uint root=wl(p.packedCapacity+i);if(root==0)return;uint width=recordWord(r,10),begin=recordWord(r,12),end=begin+width*recordWord(r,11);if(root<=begin||root>end||root>i+1){fault(32);return;}uint ignored;work.InterlockedMin(4*i,root,ignored);if(local%width>0)fillEdge(i,i-1,root,begin,end);if(local>=width)fillEdge(i,i-width,root,begin,end);}
[numthreads(256,1,1)]
void child_fill_jump(uint3 lane:SV_GroupThreadID,uint3 group:SV_GroupID){faultGroup(lane.x);if(groupFault!=0)return;uint r,local,i;if(!jobPixel(group.x,lane.x,r,local,i))return;uint a=wl(i);if(a==0){ws(p.packedCapacity+i,0);return;}uint begin=recordWord(r,12),end=begin+recordWord(r,10)*recordWord(r,11);if(a<=begin||a>end||a>i+1){fault(32);ws(p.packedCapacity+i,0);return;}uint next=wl(a-1);if(next<=begin||next>a){fault(32);ws(p.packedCapacity+i,0);return;}ws(p.packedCapacity+i,next);}
[numthreads(256,1,1)]
void child_fill_validate(uint3 lane:SV_GroupThreadID,uint3 group:SV_GroupID){faultGroup(lane.x);if(groupFault!=0)return;uint r,local,i;if(!jobPixel(group.x,lane.x,r,local,i))return;uint root=wl(p.packedCapacity+i);bool region=(packed.Load(4*i)&1)!=0;if(region){if(root!=0)fault(32);return;}uint begin=recordWord(r,12),end=begin+recordWord(r,10)*recordWord(r,11),a=wl(i);if(root<=begin||root>end||root>i+1||a<=begin||a>i+1||wl(p.packedCapacity+root-1)!=root){fault(32);return;}uint width=recordWord(r,10);if(local%width>0){uint other=wl(p.packedCapacity+i-1);if(other!=0&&other!=root)fault(32);}if(local>=width){uint other=wl(p.packedCapacity+i-width);if(other!=0&&other!=root)fault(32);}}
[numthreads(256,1,1)]
void child_boundary(uint3 lane:SV_GroupThreadID,uint3 group:SV_GroupID){uint k=lane.x;groupState(k);if(groupFault!=0)return;hashKeys[k]=0;GroupMemoryBarrierWithGroupSync();uint r,local,i;if(jobPixel(group.x,k,r,local,i)){uint root=wl(p.packedCapacity+i),w=recordWord(r,10),h=recordWord(r,11);if(root!=0&&(local%w==0||local%w+1==w||local/w==0||local/w+1==h))hashSlot(root);}GroupMemoryBarrierWithGroupSync();if(hashKeys[k]!=0){uint ignored;work.InterlockedOr(4*(2*p.packedCapacity+hashKeys[k]-1),1,ignored);}}
[numthreads(256,1,1)]
void child_support(uint3 lane:SV_GroupThreadID,uint3 group:SV_GroupID){uint k=lane.x;groupState(k);if(groupFault!=0)return;uint r=0,local,i;bool active=jobPixel(group.x,k,r,local,i);uint support=0,cut=0;if(active){uint flags=packed.Load(4*i),root=wl(p.packedCapacity+i);bool region=(flags&1)!=0,filled=region||(root!=0&&wl(2*p.packedCapacity+root-1)==0);support=uint((flags&2)!=0&&((flags&4)!=0||filled));cut=uint((flags&8)!=0&&(flags&4)!=0);packed.Store(4*i,flags|(support<<4));}sums[k]=support;sums2[k]=cut;GroupMemoryBarrierWithGroupSync();for(uint step=128;step>0;step/=2){if(k<step){sums[k]+=sums[k+step];sums2[k]+=sums2[k+step];}GroupMemoryBarrierWithGroupSync();}if(k==0){uint ignored;compact.InterlockedAdd(4*(recordBase(r)+13),sums2[0],ignored);compact.InterlockedAdd(4*(recordBase(r)+14),sums[0],ignored);}}
[numthreads(1,1,1)]
void child_finalize(uint3 tid:SV_DispatchThreadID){if(tid.x!=0)return;if(hl(4)>p.capacity)fault(2);if(hl(5)>p.packedCapacity)fault(8);if(hl(6)>p.jobCapacity)fault(16);if(faultSnapshot()!=0){hs(4,0);hs(5,0);hs(6,0);for(uint n=0;n<16;n++)arguments.Store(4*n,0);}}
