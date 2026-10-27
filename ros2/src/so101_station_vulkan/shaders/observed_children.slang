// Ordinary sparse memberships. Preparation only, never accepted objects.
#ifdef OBSERVED_SOFTWARE64
#include "soft_binary64.slang"
#endif
[[vk::binding(0,0)]] ByteAddressBuffer roots;
[[vk::binding(1,0)]] ByteAddressBuffer children;
[[vk::binding(2,0)]] ByteAddressBuffer parents;
[[vk::binding(3,0)]] ByteAddressBuffer geometry;
[[vk::binding(4,0)]] ByteAddressBuffer roi;
[[vk::binding(5,0)]] RWByteAddressBuffer arena;
[[vk::binding(6,0)]] RWByteAddressBuffer compact;
struct Parameters {uint width,height,count,capacity,parentCapacity,closedCapacity,roiEnabled,calLo,calHi,parentLo,parentHi,childLo,childHi,observedLo,observedHi,minChild;};
[[vk::push_constant]] Parameters p;
uint head(uint i){return compact.Load(4*i);}
// CPU witness is explicitly serial test arithmetic, never a production backend.
void fault(uint value){
#ifdef OBSERVED_CPU_WITNESS
 compact.Store(28,compact.Load(28)|value);
#else
 uint ignored;compact.InterlockedOr(28,value,ignored);
#endif
}
void addCount(uint at,uint value){
#ifdef OBSERVED_CPU_WITNESS
 compact.Store(at,compact.Load(at)+value);
#else
 uint ignored;compact.InterlockedAdd(at,value,ignored);
#endif
}
uint insertRoot(uint at,uint value){
#ifdef OBSERVED_CPU_WITNESS
 uint before=arena.Load(at);if(before==0)arena.Store(at,value);return before;
#else
 uint before;arena.InterlockedCompareExchange(at,0,value,before);return before;
#endif
}
uint childWord(uint r,uint j){return children.Load(128+16*p.parentCapacity+64*r+4*j);}
uint outWord(uint r,uint j){return compact.Load(64+80*r+4*j);}
void outStore(uint r,uint j,uint v){compact.Store(64+80*r+4*j,v);}
uint original(uint i){return arena.Load(4*(p.count+i));}
// Only immutable original roots are read to discover at most9 candidates.
[numthreads(256,1,1)]
void observed_clear(uint3 tid:SV_DispatchThreadID){uint i=tid.x;if(i<p.count){arena.Store(4*i,0);arena.Store(4*(p.count+i),0);arena.Store(4*(2*p.count+i),0);for(uint k=0;k<9;k++)arena.Store(4*(3*p.count+9*i+k),0);}
 if(i==0){uint n=children.Load(16),pn=children.Load(12);compact.Store(0,1);compact.Store(4,p.width);compact.Store(8,p.height);compact.Store(12,n);compact.Store(32,p.calLo);compact.Store(36,p.calHi);compact.Store(40,p.childLo);compact.Store(44,p.childHi);compact.Store(48,p.observedLo);compact.Store(52,p.observedHi);compact.Store(56,p.closedCapacity);compact.Store(60,p.capacity);
  if(parents.Load(0)!=1||pn!=parents.Load(28)||children.Load(0)!=1||children.Load(4)!=p.width||children.Load(8)!=p.height||n>p.capacity||pn>p.parentCapacity||children.Load(28)!=0||children.Load(32)!=p.calLo||children.Load(36)!=p.calHi||children.Load(40)!=p.parentLo||children.Load(44)!=p.parentHi||children.Load(48)!=p.childLo||children.Load(52)!=p.childHi||children.Load(64)!=p.minChild)fault(1);
 }
}
// Every potentially malformed index/extent is checked before dependent accesses.
bool validRecord(uint r){
 if(r>=min(head(3),p.capacity))return false;
 uint root=childWord(r,0),parent=childWord(r,1),area=childWord(r,3);
 if(root==0||root>p.count||parent>=children.Load(12)||parent>=p.parentCapacity||area<p.minChild||area>p.count||childWord(r,15)!=0){fault(2);return false;}
 uint b=64+32*parent,px=parents.Load(b+16),py=parents.Load(b+20),pw=parents.Load(b+24),ph=parents.Load(b+28);
 if(px>=p.width||py>=p.height||pw==0||ph==0||pw>p.width-px||ph>p.height-py){fault(2);return false;}
 uint x=childWord(r,4),y=childWord(r,5),w=childWord(r,6),h=childWord(r,7);
 if(x>=pw||y>=ph||w==0||h==0||w>pw-x||h>ph-y||area>w*h){fault(2);return false;}
 uint rx=(root-1)%p.width,ry=(root-1)/p.width;
 if(rx<px+x||ry<py+y||rx>=px+x+w||ry>=py+y+h||roots.Load(4*(root-1))!=root){fault(2);return false;}
 uint left=childWord(r,8),top=childWord(r,9),hw=childWord(r,10),hh=childWord(r,11),offset=childWord(r,12),cut=childWord(r,13),support=childWord(r,14),packed=children.Load(20);
 if(childWord(r,2)>=((pw+1)/2)*((ph+1)/2)||left!=(x>2?x-2:0)||top!=(y>2?y-2:0)||hw!=min(pw,x+w+2)-left||hh!=min(ph,y+h+2)-top||packed>4*p.count||offset>packed||hw*hh>packed-offset||support<area||support>min(hw*hh,parents.Load(b+12))||cut>support||cut>hw*hh-area){fault(2);return false;}
 if(r==0){if(offset!=0){fault(2);return false;}}
 else {uint previous=childWord(r-1,1);if(previous>=children.Load(12)||previous>=p.parentCapacity){fault(2);return false;}
  uint pb=64+32*previous,phw=childWord(r-1,10),phh=childWord(r-1,11),po=childWord(r-1,12);
  if(phw>parents.Load(pb+24)||phh>parents.Load(pb+28)||po>packed||phw*phh>packed-po||offset!=po+phw*phh){fault(2);return false;}
 }
 if(r+1==head(3)&&offset+hw*hh!=packed){fault(2);return false;}
 return true;
}
[numthreads(256,1,1)]
void observed_records(uint3 tid:SV_DispatchThreadID){uint r=tid.x;if(!validRecord(r))return;
 uint root=childWord(r,0),before=insertRoot(4*(root-1),r+1);if(before!=0){fault(2);return;}
 for(uint j=0;j<16;j++)outStore(r,j,childWord(r,j));outStore(r,16,0);outStore(r,17,0);outStore(r,18,4);outStore(r,19,0);
 if(r>0){uint ap=childWord(r-1,1),bp=childWord(r,1);if(ap>bp||(ap==bp&&childWord(r-1,2)>=childWord(r,2)))fault(2);}
}
bool recordValid(uint r){return r<min(head(3),p.capacity)&&(outWord(r,18)&4)!=0;}
bool inside(uint i){
 if(p.roiEnabled==0)return true;
 for(uint axis=0;axis<3;axis++){
#ifdef OBSERVED_SOFTWARE64
  uint64_t v=geometry.Load<uint64_t>(24*i+8*axis),lo=roi.Load<uint64_t>(16*axis),hi=roi.Load<uint64_t>(16*axis+8);
  if(!sf64_less_equal(lo,v)||!sf64_less_equal(v,hi))return false;
#else
  double v=geometry.Load<double>(24*i+8*axis);if(!(v>=roi.Load<double>(16*axis)&&v<=roi.Load<double>(16*axis+8)))return false;
#endif
 }
 return true;
}
[numthreads(256,1,1)]
void observed_observe(uint3 tid:SV_DispatchThreadID){uint i=tid.x;if(i>=p.count)return;uint root=roots.Load(4*i);if(root==0)return;if(root>p.count){fault(2);return;}uint id=arena.Load(4*(root-1));if(id==0)return;if(id>min(head(3),p.capacity)){fault(2);return;}
 uint r=id-1;if(!recordValid(r)){fault(2);return;}
 uint parent=childWord(r,1),base=64+32*parent,x=parents.Load(base+16)+childWord(r,4),y=parents.Load(base+20)+childWord(r,5),w=childWord(r,6),h=childWord(r,7);
 if(i%p.width<x||i%p.width>=x+w||i/p.width<y||i/p.width>=y+h){fault(2);return;}
 arena.Store(4*(p.count+i),id);addCount(64+80*r+76,1);if(inside(i))addCount(64+80*r+64,1);
}
[numthreads(256,1,1)]
void observed_eligible(uint3 tid:SV_DispatchThreadID){uint r=tid.x;if(!recordValid(r))return;
 if(outWord(r,19)!=childWord(r,3)||outWord(r,16)>outWord(r,19)){fault(8);return;}
 if(outWord(r,16)>=8)outStore(r,18,7);outStore(r,19,0);
}
[numthreads(256,1,1)]
void observed_close(uint3 tid:SV_DispatchThreadID){uint i=tid.x;if(i>=p.count)return;int x=int(i%p.width),y=int(i/p.width);uint ids[9]={0,0,0,0,0,0,0,0,0};uint n=0;
 for(int dy=-1;dy<=1;dy++)for(int dx=-1;dx<=1;dx++){int xx=x+dx,yy=y+dy;if(xx<0||yy<0||xx>=int(p.width)||yy>=int(p.height))continue;uint id=original(uint(yy)*p.width+uint(xx));if(id==0)continue;if(id>min(head(3),p.capacity)){fault(2);continue;}bool duplicate=false;for(uint k=0;k<n;k++)duplicate|=ids[k]==id;if(!duplicate){if(n==9){fault(4);return;}ids[n++]=id;}}
 // Canonical sorted record IDs, independent of neighbor traversal or atomics.
 for(uint a=1;a<n;a++){uint v=ids[a],b=a;while(b>0&&ids[b-1]>v){ids[b]=ids[b-1];--b;}ids[b]=v;}
 uint selectedCount=0;
 for(uint k=0;k<n;k++){uint r=ids[k]-1;if(!recordValid(r)||(outWord(r,18)&1)==0)continue;uint parent=childWord(r,1),base=64+32*parent;
  int x0=int(parents.Load(base+16)+childWord(r,4)),y0=int(parents.Load(base+20)+childWord(r,5)),x1=x0+int(childWord(r,6)),y1=y0+int(childWord(r,7));
  if(x<x0||x>=x1||y<y0||y>=y1)continue;bool selected=true;
  for(int ey=-1;ey<=1&&selected;ey++)for(int ex=-1;ex<=1&&selected;ex++){int qx=x+ex,qy=y+ey;if(qx<x0||qx>=x1||qy<y0||qy>=y1)continue;bool dilated=false;
   for(int dy=-1;dy<=1&&!dilated;dy++)for(int dx=-1;dx<=1&&!dilated;dx++){int xx=qx+dx,yy=qy+dy;if(xx<x0||xx>=x1||yy<y0||yy>=y1)continue;dilated=original(uint(yy)*p.width+uint(xx))==ids[k];}selected=selected&&dilated;
  }
  if(selected){arena.Store(4*(3*p.count+9*i+selectedCount),ids[k]);++selectedCount;addCount(64+80*r+68,1);}
 }
 arena.Store(4*(2*p.count+i),selectedCount);
}
[numthreads(1,1,1)]
void observed_finalize(uint3 tid:SV_DispatchThreadID){if(tid.x!=0)return;uint total=0,originals=0,eligible=0;
 for(uint r=0;r<min(head(3),p.capacity);r++){uint orig=outWord(r,3),closed=outWord(r,17),flags=outWord(r,18);if(!validRecord(r)||outWord(r,19)!=0||outWord(r,16)>orig||flags!=((outWord(r,16)>=8)?7:4)||closed>p.count||((flags&1)==0&&closed!=0)){fault(8);continue;}originals+=orig;total+=closed;eligible+=uint((flags&1)!=0);outStore(r,18,flags&3);}
 // Full compact producer consistency, linear P+R; no host record traversal.
 uint cursor=0,validTotal=0,candidateTotal=0,allTotal=0,keptTotal=0,jobs=0,packedEnd=0;
 for(uint parent=0;parent<min(children.Load(12),p.parentCapacity);parent++){
  uint b=128+16*parent,valid=children.Load(b),candidate=children.Load(b+4),all=children.Load(b+8),kept=children.Load(b+12),parentPixels=parents.Load(64+32*parent+12);
  if(valid>parentPixels||candidate>valid||all>candidate||kept>all)fault(8);
  uint begin=cursor,area=0;
  while(cursor<min(head(3),p.capacity)&&childWord(cursor,1)==parent){if(validRecord(cursor)){area+=childWord(cursor,3);uint payload=childWord(cursor,10)*childWord(cursor,11);jobs+=(payload+255)/256;packedEnd=childWord(cursor,12)+payload;}++cursor;}
  if(cursor-begin!=kept||area>candidate)fault(8);validTotal+=valid;candidateTotal+=candidate;allTotal+=all;keptTotal+=kept;
 }
 if(cursor!=head(3)||keptTotal!=head(3)||validTotal>p.count||candidateTotal>validTotal||allTotal>candidateTotal||packedEnd!=children.Load(20)||jobs!=children.Load(24))fault(8);
 if(originals>p.count||total>p.closedCapacity)fault(4);compact.Store(16,eligible);compact.Store(20,originals);compact.Store(24,total);
}
