#include "so101_station_vulkan/pipeline.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
namespace so101_station_vulkan {
namespace {
const std::array<int,512>& table(){static auto t=[](){std::array<int,512>a{};for(int i=1;i<256;i++){a[i]=int(std::nearbyint(double(255<<12)/i));a[256+i]=int(std::nearbyint(double(180<<12)/(6*i)));}return a;}();return t;}
uint32_t bits(float f){uint32_t u;std::memcpy(&u,&f,4);return u;}
uint32_t root(std::vector<uint32_t>& a,uint32_t i){uint32_t r=a[i];if(!r)return 0;while(a[r-1]!=r)r=a[r-1];while(a[i]!=r){auto next=a[i]-1;a[i]=r;i=next;}return r;}
}
void cpu_reference(Slot& s,unsigned w,unsigned h){
 auto n=w*h;auto* in=static_cast<const uint8_t*>(s.upload.mapped);auto& t=table();auto& c=s.calibration;
 auto valid=[](float d){return std::isfinite(d)&&d>=.10f&&d<=1.2f;};
 for(unsigned i=0;i<n;i++){
  int r=in[3*i],g=in[3*i+1],b=in[3*i+2],v=std::max(r,std::max(g,b)),d=v-std::min(r,std::min(g,b));
  int saturation=(d*t[v]+2048)>>12,hue=v==r?g-b:(v==g?b-r+2*d:r-g+4*d);hue=(hue*t[256+d]+2048)>>12;if(hue<0)hue+=180;
  s.cpu_masks[i]=saturation<90||v<50?0:unsigned(hue<=10||hue>=170)|(unsigned(hue>=41&&hue<=89)<<1)|(unsigned(hue>=90&&hue<=140)<<2)|(unsigned(hue>=15&&hue<=40)<<3);
  auto raw=unsigned(in[3*n+2*i])|(unsigned(in[3*n+2*i+1])<<8);float z=float(raw)*c.depth_scale;s.cpu_depth[i]=z;
  s.cpu_xyz[4*i]=((float(i%w)-c.cx)/c.fx)*z;s.cpu_xyz[4*i+1]=((float(i/w)-c.cy)/c.fy)*z;s.cpu_xyz[4*i+2]=z;s.cpu_xyz[4*i+3]=raw?1.f:0.f;
 }
 for(unsigned i=0;i<n;i++){
  unsigned m=s.cpu_masks[i],x=i%w,y=i/w;float z=s.cpu_depth[i];bool ok=m&&valid(z),cut=false;
  auto jump=[&](unsigned j){return s.cpu_masks[j]==m&&valid(s.cpu_depth[j])&&std::abs(z-s.cpu_depth[j])>.012f;};
  if(ok)cut=(x&&jump(i-1))||(x+1<w&&jump(i+1))||(y&&jump(i-w))||(y+1<h&&jump(i+w));s.cpu_parents[i]=ok&&!cut?i+1:0;
 }
 for(unsigned i=0;i<n;i++){
  if(!s.cpu_parents[i])continue;unsigned x=i%w,y=i/w;
  for(int dy=-1;dy<=0;dy++)for(int dx=-1;dx<=1;dx++){
   if(dy==0&&dx>=0)continue;int nx=int(x)+dx,ny=int(y)+dy;if(nx<0||ny<0||nx>=int(w))continue;unsigned j=unsigned(ny)*w+unsigned(nx);
   if(!s.cpu_parents[j]||s.cpu_masks[i]!=s.cpu_masks[j])continue;auto a=root(s.cpu_parents,i),b=root(s.cpu_parents,j);s.cpu_parents[std::max(a,b)-1]=std::min(a,b);
  }
 }
 // Reuse the per-pixel xyz allocation's fourth lane as a temporary count map.
 // No per-frame heap allocation; compact values are emitted in canonical root order.
 for(unsigned i=0;i<n;i++){s.cpu_parents[i]=root(s.cpu_parents,i);s.cpu_xyz[4*i+3]=0;}
 for(unsigned i=0;i<n;i++)if(s.cpu_parents[i])s.cpu_xyz[4*(s.cpu_parents[i]-1)+3]+=1;
 s.component_count=0;s.fault=0;
 for(unsigned i=0;i<n;i++)if(s.cpu_xyz[4*i+3]>=24){
  if(s.component_count>=kMaxComponents){s.fault|=2;s.cpu_xyz[4*i+3]=0;continue;}
  auto& a=s.cpu_components[s.component_count++];a={i+1,s.cpu_masks[i],0,UINT32_MAX,UINT32_MAX,0,0,UINT32_MAX,0,0};
  // Temporary root -> compact lookup uses already-reserved depth-count storage.
  s.cpu_xyz[4*i+3]=float(s.component_count);
 }else s.cpu_xyz[4*i+3]=0;
 for(unsigned i=0;i<n;i++)if(s.cpu_parents[i]){unsigned root_index=s.cpu_parents[i]-1,index=unsigned(s.cpu_xyz[4*root_index+3]);if(!index)continue;auto& a=s.cpu_components[index-1];
   ++a.count;a.x0=std::min(a.x0,i%w);a.y0=std::min(a.y0,i/w);a.x1=std::max(a.x1,i%w);a.y1=std::max(a.y1,i/w);auto z=bits(s.cpu_depth[i]);a.min_z_bits=std::min(a.min_z_bits,z);a.max_z_bits=std::max(a.max_z_bits,z);
 }
 cpu_morphology(s,w,h);
 for(unsigned i=0;i<n;i++)s.cpu_xyz[4*i+3]=s.cpu_depth[i]!=0?1.f:0.f;
}
void cpu_morphology(Slot& s,unsigned w,unsigned h){
 (void)h;
 // Each original depth piece retains its own tight bounding box. Border
 // handling matches cv2 morphologyEx(default): dilation outside=0, erosion=1.
 std::fill(s.cpu_morph_counts.begin(),s.cpu_morph_counts.end(),0);
 for(unsigned component=0;component<s.component_count;component++){
  auto& a=s.cpu_components[component];unsigned bw=a.x1-a.x0+1,bh=a.y1-a.y0+1;
  for(unsigned y=0;y<bh;y++)for(unsigned x=0;x<bw;x++)s.cpu_work[y*bw+x]=s.cpu_parents[(a.y0+y)*w+a.x0+x]==a.root;
  for(unsigned y=0;y<bh;y++)for(unsigned x=0;x<bw;x++){
   bool selected=false;for(int dy=-1;dy<=1&&!selected;dy++)for(int dx=-1;dx<=1&&!selected;dx++){
    int nx=int(x)+dx,ny=int(y)+dy;if(nx>=0&&ny>=0&&nx<int(bw)&&ny<int(bh))selected|=s.cpu_work[unsigned(ny)*bw+unsigned(nx)]!=0;
   }s.cpu_dilated[y*bw+x]=selected;
  }
  a.closed_count=0;
  for(unsigned y=0;y<bh;y++)for(unsigned x=0;x<bw;x++){
   bool selected=true;for(int dy=-1;dy<=1&&selected;dy++)for(int dx=-1;dx<=1&&selected;dx++){
    int nx=int(x)+dx,ny=int(y)+dy;if(nx>=0&&ny>=0&&nx<int(bw)&&ny<int(bh))selected&=s.cpu_dilated[unsigned(ny)*bw+unsigned(nx)]!=0;
   }
   if(selected){++a.closed_count;size_t i=size_t(a.y0+y)*w+a.x0+x;auto count=s.cpu_morph_counts[i];if(count>=9){s.fault|=8;continue;}s.cpu_morph_roots[i*9+count]=a.root;++s.cpu_morph_counts[i];}
  }
 }
 }
}
