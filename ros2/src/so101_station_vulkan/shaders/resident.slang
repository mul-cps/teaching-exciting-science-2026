// RGB8/Z16 aligned input. Source pixel centers; no +0.5 deprojection offset.
// Binding layout is deliberately identical for every offline-compiled entry.
[[vk::binding(0,0)]] ByteAddressBuffer input;
[[vk::binding(1,0)]] RWByteAddressBuffer scratch;
[[vk::binding(2,0)]] StructuredBuffer<int> divisions;
[[vk::binding(3,0)]] RWByteAddressBuffer compact;
struct Parameters { uint width,height,count,capacity; float scale,fx,fy,cx,cy,maxDepth,jump; uint round; };
[[vk::push_constant]] Parameters p;
uint byteAt(uint i){return (input.Load(i&~3u)>>((i&3u)*8u))&255u;}
uint maskAt(uint i){
 int r=int(byteAt(3*i)),g=int(byteAt(3*i+1)),b=int(byteAt(3*i+2));
 int v=max(r,max(g,b)),d=v-min(r,min(g,b));int s=(d*divisions[v]+2048)>>12;
 int h=v==r?g-b:(v==g?b-r+2*d:r-g+4*d);h=(h*divisions[256+d]+2048)>>12;if(h<0)h+=180;
 bool vivid=s>=90&&v>=50;
 return !vivid?0u:uint(h<=10||h>=170)|(uint(h>=41&&h<=89)<<1)|(uint(h>=90&&h<=140)<<2)|(uint(h>=15&&h<=40)<<3);
}
uint mask(uint i){return scratch.Load(4*i);}
float depth(uint i){return asfloat(scratch.Load(4*(p.count+i)));}
uint parentOffset(uint i){return 4*(6*p.count+i);}
uint canonicalOffset(uint i){return 4*(17*p.count+i);}
// Forest A is mutated only by tiled initialization or boundary-hook dispatches.
// Every root walk runs in a SEPARATE dispatch, while all of A is immutable.
uint forestParent(uint i){return scratch.Load(parentOffset(i));}
// B is the separately materialized canonical snapshot. Connect/reduce/morphology
// read B only; no stage mutates B concurrently with any of these readers.
uint canonicalParent(uint i){return scratch.Load(canonicalOffset(i));}
uint root(uint i){
 uint r=forestParent(i);
 [loop]for(uint j=0;j<p.count&&r!=0;j++){
  if(r>p.count)break;
  uint next=forestParent(r-1);if(next==r)return r;
  if(next==0||next>=r)break; // positive, strictly decreasing forest invariant
  r=next;
 }
 if(r!=0){uint ignored;compact.InterlockedOr(4,4,ignored);}
 return 0;
}

bool valid(float d){return isfinite(d)&&d>=.10&&d<=p.maxDepth;}
[numthreads(256,1,1)]
void convert(uint3 tid:SV_DispatchThreadID){
 uint i=tid.x;if(i>=p.count)return;uint at=3*p.count+2*i;uint raw=byteAt(at)|(byteAt(at+1)<<8);
 precise float d=float(raw)*p.scale;
 scratch.Store(4*i,maskAt(i));scratch.Store(4*(p.count+i),asuint(d));
 precise float x=(float(i%p.width)-p.cx)/p.fx;precise float y=(float(i/p.width)-p.cy)/p.fy;
 scratch.Store4(4*(2*p.count+4*i),asuint(float4(x*d,y*d,d,raw!=0?1.0:0.0)));
 // per-root reductions: count,xmin,ymin,xmax,ymax,zmin,zmax,color,pad
 uint a=4*(8*p.count+9*i);scratch.Store(a,0);scratch.Store(a+4,0xffffffff);scratch.Store(a+8,0xffffffff);
 scratch.Store(a+12,0);scratch.Store(a+16,0);scratch.Store(a+20,0xffffffff);scratch.Store(a+24,0);scratch.Store(a+28,0);scratch.Store(a+32,0);
}
[numthreads(256,1,1)]
void candidates(uint3 tid:SV_DispatchThreadID){
 uint i=tid.x;if(i>=p.count)return;uint m=mask(i),x=i%p.width,y=i/p.width;float d=depth(i);bool ok=m!=0&&valid(d),cut=false;
 if(ok){
  if(x>0&&mask(i-1)==m&&valid(depth(i-1)))cut|=abs(d-depth(i-1))>p.jump;
  if(x+1<p.width&&mask(i+1)==m&&valid(depth(i+1)))cut|=abs(d-depth(i+1))>p.jump;
  if(y>0&&mask(i-p.width)==m&&valid(depth(i-p.width)))cut|=abs(d-depth(i-p.width))>p.jump;
  if(y+1<p.height&&mask(i+p.width)==m&&valid(depth(i+p.width)))cut|=abs(d-depth(i+p.width))>p.jump;
 }
 scratch.Store(parentOffset(i),ok&&!cut?i+1:0);scratch.Store(4*(7*p.count+i),uint(cut));
}
// All 256 lanes participate, including inactive/out-of-image lanes. The two
// shared arrays alternate immutable readers and disjoint writers each round.
groupshared uint tileLabels0[256];
groupshared uint tileLabels1[256];
groupshared uint tileColors[256];
groupshared uint tileChanged;
[numthreads(16,16,1)]
void tiled_components(uint3 group:SV_GroupID,uint3 lane:SV_GroupThreadID){
 uint local=lane.y*16+lane.x,x=group.x*16+lane.x,y=group.y*16+lane.y,i=y*p.width+x;
 bool inside=x<p.width&&y<p.height;
 uint initial=0,color=0;if(inside){initial=forestParent(i);color=mask(i);}
 tileLabels0[local]=initial;tileColors[local]=color;
 GroupMemoryBarrierWithGroupSync();
 uint source=0;bool converged=false;
 [loop]for(uint iteration=0;iteration<256;iteration++){
  if(local==0)tileChanged=0;
  GroupMemoryBarrierWithGroupSync();
  uint current=source==0?tileLabels0[local]:tileLabels1[local],next=current;
  if(current!=0){
   [unroll]for(int dy=-1;dy<=1;dy++)[unroll]for(int dx=-1;dx<=1;dx++){
    int nx=int(lane.x)+dx,ny=int(lane.y)+dy;if(nx<0||ny<0||nx>=16||ny>=16)continue;
    uint other=uint(ny)*16+uint(nx);if(tileColors[other]!=tileColors[local])continue;
    uint label=source==0?tileLabels0[other]:tileLabels1[other];if(label!=0)next=min(next,label);
   }
  }
  if(source==0)tileLabels1[local]=next;else tileLabels0[local]=next;
  if(next!=current){uint ignored;InterlockedOr(tileChanged,1,ignored);}
  GroupMemoryBarrierWithGroupSync();
  uint changed=tileChanged;
  // Every lane must consume the previous changed flag before lane0 resets it.
  GroupMemoryBarrierWithGroupSync();
  source=1-source;
  if(changed==0){converged=true;break;}
 }
 if(!converged&&local==0){uint ignored;compact.InterlockedOr(4,1,ignored);}
 if(inside)scratch.Store(parentOffset(i),source==0?tileLabels0[local]:tileLabels1[local]);
}
[numthreads(256,1,1)]
void compress(uint3 tid:SV_DispatchThreadID){
 uint i=tid.x;if(i<p.count)scratch.Store(canonicalOffset(i),root(i));
}
[numthreads(256,1,1)]
void connect(uint3 tid:SV_DispatchThreadID){
 uint i=tid.x;if(i>=p.count)return;uint x=i%p.width,y=i/p.width;
 uint a=canonicalParent(i),ignored;if(a==0)return;
 // B[i] is the root of the immutable pre-dispatch A tree, hence B[i]<=A[i].
 // Atomic flattening and boundary hooks all take min with fixed B-derived
 // proposals. They commute, exactly matching a full B->A copy then hooks.
 scratch.InterlockedMin(parentOffset(i),a,ignored);
 // Interior pixels have no edge leaving their tile. Visit every undirected
 // cross-tile 8-neighbor edge exactly once: right, down-left/down/down-right.
 if((x&15u)!=0&&(x&15u)!=15&&(y&15u)!=15)return;
 uint m=mask(i);
 [unroll]for(int dy=0;dy<=1;dy++)[unroll]for(int dx=-1;dx<=1;dx++){
  if(dy==0&&dx!=1)continue;
  int nx=int(x)+dx,ny=int(y)+dy;if(nx<0||ny<0||nx>=int(p.width)||ny>=int(p.height))continue;
  if(uint(nx)/16==x/16&&uint(ny)/16==y/16)continue;
  uint j=uint(ny)*p.width+uint(nx);if(mask(j)!=m)continue;
  uint b=canonicalParent(j);
  if(b!=0&&a!=b)scratch.InterlockedMin(parentOffset(max(a,b)-1),min(a,b),ignored);
 }
}
[numthreads(256,1,1)]
void reduce(uint3 tid:SV_DispatchThreadID){
 uint i=tid.x;if(i>=p.count)return;uint r=canonicalParent(i);if(r==0)return;uint x=i%p.width,y=i/p.width,m=mask(i),ignored;
 // Verify convergence explicitly instead of silently returning partial components.
 [unroll]for(int dy=-1;dy<=1;dy++)[unroll]for(int dx=-1;dx<=1;dx++){
  int nx=int(x)+dx,ny=int(y)+dy;if(nx<0||ny<0||nx>=int(p.width)||ny>=int(p.height))continue;uint j=uint(ny)*p.width+uint(nx);
  uint other=canonicalParent(j);if(other!=0&&mask(j)==m&&other!=r)compact.InterlockedOr(4,1,ignored);
 }
 uint a=4*(8*p.count+9*(r-1));scratch.InterlockedAdd(a,1,ignored);scratch.InterlockedMin(a+4,x,ignored);scratch.InterlockedMin(a+8,y,ignored);
 scratch.InterlockedMax(a+12,x,ignored);scratch.InterlockedMax(a+16,y,ignored);scratch.InterlockedMin(a+20,asuint(depth(i)),ignored);
 scratch.InterlockedMax(a+24,asuint(depth(i)),ignored);
}
// Independent morphology memberships can overlap. Never collapse to one label.
[numthreads(256,1,1)]
void morphology(uint3 tid:SV_DispatchThreadID){
 uint i=tid.x;if(i>=p.count)return;uint x=i%p.width,y=i/p.width;
 uint roots[9];uint count=0,closed=0;
 [loop]for(int dy=-1;dy<=1;dy++)[loop]for(int dx=-1;dx<=1;dx++){
  int nx=int(x)+dx,ny=int(y)+dy;if(nx<0||ny<0||nx>=int(p.width)||ny>=int(p.height))continue;
  uint r=canonicalParent(uint(ny)*p.width+uint(nx));if(r==0)continue;
  bool duplicate=false;[loop]for(uint k=0;k<count;k++)duplicate|=roots[k]==r;
  if(!duplicate)roots[count++]=r;
 }
 [loop]for(uint k=0;k<count;k++){
  uint r=roots[k],a=4*(8*p.count+9*(r-1));if(scratch.Load(a)<24)continue;
  int x0=int(scratch.Load(a+4)),y0=int(scratch.Load(a+8)),x1=int(scratch.Load(a+12)),y1=int(scratch.Load(a+16));
  if(int(x)<x0||int(x)>x1||int(y)<y0||int(y)>y1)continue;
  bool selected=true;
  [loop]for(int ey=-1;ey<=1&&selected;ey++)[loop]for(int ex=-1;ex<=1&&selected;ex++){
   int qx=int(x)+ex,qy=int(y)+ey;
   if(qx<x0||qx>x1||qy<y0||qy>y1)continue; // erosion's outside border is 1
   bool dilated=false;
   [loop]for(int dy=-1;dy<=1&&!dilated;dy++)[loop]for(int dx=-1;dx<=1&&!dilated;dx++){
    int nx=qx+dx,ny=qy+dy;if(nx<x0||nx>x1||ny<y0||ny>y1)continue; // dilation outside=0
    dilated=canonicalParent(uint(ny)*p.width+uint(nx))==r;
   }
   selected&=dilated;
  }
  if(selected){uint ignored;scratch.InterlockedAdd(a+32,1,ignored);
   if(p.round!=0)scratch.Store(4*(18*p.count+10*i+1+closed),r);++closed;
  }
 }
 if(p.round!=0)scratch.Store(4*(18*p.count+10*i),closed);
}
[numthreads(256,1,1)]
void gather(uint3 tid:SV_DispatchThreadID){
 uint i=tid.x;if(i>=p.count)return;uint a=4*(8*p.count+9*i),count=scratch.Load(a);if(count<24)return;
 uint dest,ignored;compact.InterlockedAdd(0,1,dest);if(dest>=p.capacity){compact.InterlockedOr(4,2,ignored);return;}
 uint o=16+dest*40;compact.Store(o,i+1);compact.Store(o+4,mask(i));compact.Store(o+8,count);
 compact.Store4(o+12,uint4(scratch.Load(a+4),scratch.Load(a+8),scratch.Load(a+12),scratch.Load(a+16)));
 compact.Store(o+28,scratch.Load(a+20));compact.Store(o+32,scratch.Load(a+24));compact.Store(o+36,scratch.Load(a+32));
}
