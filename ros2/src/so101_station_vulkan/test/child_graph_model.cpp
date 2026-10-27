// Offline semantic witness. Dense host loops are deliberately test-only.
// Hook/jump remains an offline transcription; production predicates/layout are shared.
#include "so101_station_vulkan/child_graph.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <vector>

using namespace so101_station_vulkan;
using U = uint32_t;
using V = std::vector<U>;
static void array(const V& a) {
  std::cout << '[';
  for (size_t i=0;i<a.size();++i) { if(i)std::cout<<',';std::cout<<a[i]; }
  std::cout << ']';
}
// Immutable B snapshot, atomicMin-equivalent A updates, then one pointer jump.
// Each pass has constant work per node/edge, with no root walk.
static V graph(const V& active,U w,U h,U rounds,bool reverse,bool diagonal,const V* membership=nullptr) {
  V b(active.size());
  for(U i=0;i<b.size();++i)b[i]=active[i]?i+1:0;
  auto edges=[&](U i,auto fn){
    int x=int(i%w),y=int(i/w);
    for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx){
      if((!dx&&!dy)||(!diagonal&&dx&&dy))continue;
      int xx=x+dx,yy=y+dy;
      if(xx>=0&&yy>=0&&xx<int(w)&&yy<int(h)) {
        U j=U(yy)*w+U(xx);if(active[i]&&active[j]&&(!membership||child_same_parent((*membership)[i],(*membership)[j])))fn(j);
      }
    }
  };
  for(U round=0;round<rounds;++round){
    V a=b;
    for(U k=0;k<b.size();++k){U i=reverse?U(b.size()-1-k):k;
      edges(i,[&](U j){U hi=std::max(b[i],b[j]),lo=std::min(b[i],b[j]);a[hi-1]=std::min(a[hi-1],lo);});
    }
    V next(a.size());
    for(U i=0;i<a.size();++i)if(a[i]){
      if(a[i]>i+1||a[i]>a.size()||!a[a[i]-1])throw std::runtime_error("forest range");
      next[i]=a[a[i]-1];
    }
    b.swap(next);
  }
  for(U i=0;i<b.size();++i)if(b[i]){
    if(b[i]>i+1||b[i]>b.size()||b[b[i]-1]!=b[i])throw std::runtime_error("nonconvergence");
    edges(i,[&](U j){if(b[i]!=b[j])throw std::runtime_error("edge nonconvergence");});
  }
  return b;
}
struct Record {U root,key,area,left,top,right,bottom,hl,ht,hw,hh,cut=0,support=0;V packed;};
int main(int argc,char** argv){try{
  if(argc==2&&std::string(argv[1])=="--round"){uint64_t b;std::cout<<'[';bool comma=false;while(std::cin>>b){double v;std::memcpy(&v,&b,8);if(comma)std::cout<<',';comma=true;std::cout<<child_f32_rne_bits(v);}std::cout<<"]\n";return 0;}
  U w,h,px,py,pw,ph,minimum,rounds,capacity,payload,jobs,reverse;
  double max_d,jump_d;
  if(!(std::cin>>w>>h>>px>>py>>pw>>ph>>minimum>>max_d>>jump_d>>rounds>>capacity>>payload>>jobs>>reverse))return 2;
  if(!w||!h||uint64_t(w)*h>10000000||!std::isfinite(max_d)||max_d<=.1||!std::isfinite(jump_d)||jump_d<=0)throw std::runtime_error("input");
  ChildGraphLayout layout(w,h);
  U n=layout.count;V parent(n),bits(n),flags(n),candidate(n),cut(n);std::vector<float>d(n);
  for(U& p:parent)std::cin>>p;
  for(U i=0;i<n;++i){std::cin>>bits[i];std::memcpy(&d[i],&bits[i],4);}
  U maximum=child_f32_rne_bits(max_d),jump=child_f32_rne_bits(jump_d);U valid_count=0,candidate_count=0;
  for(U i=0;i<n;++i){bool valid=child_depth_valid(bits[i],maximum);flags[i]=valid?1:0;candidate[i]=parent[i]&&valid;if(candidate[i])++valid_count;}
  for(U i=0;i<n;++i)if(candidate[i]){
    for(U j:{i%w+1<w?i+1:n,i/w+1<h?i+w:n})
      if(j<n&&candidate[j]&&child_same_parent(parent[i],parent[j])&&child_bad_edge(bits[i],bits[j],jump))cut[i]=cut[j]=1;
  }
  for(U i=0;i<n;++i){flags[i]|=cut[i]*2;candidate[i]&=!cut[i];candidate_count+=candidate[i];}
  V roots=graph(candidate,w,h,rounds,reverse,true,&parent);
  std::vector<Record> all;
  for(U i=0;i<n;++i)if(roots[i]==i+1)all.push_back({i+1,~U(0),0,w,h,0,0,0,0,0,0,0,0,{}});
  for(auto& r:all)for(U i=0;i<n;++i)if(roots[i]==r.root){U x=i%w,y=i/w;++r.area;r.left=std::min(r.left,x);r.top=std::min(r.top,y);r.right=std::max(r.right,x);r.bottom=std::max(r.bottom,y);r.key=std::min(r.key,child_local_key(x,y,px,py,pw));}
  std::vector<Record> kept;for(auto r:all)if(r.area>=minimum)kept.push_back(r);
  if(kept.size()>capacity)throw std::runtime_error("record capacity");
  std::sort(kept.begin(),kept.end(),[](const Record&a,const Record&b){return a.key<b.key;});
  uint64_t pixels=0,job_count=0;
  for(auto& r:kept){
    U l=std::max(px,r.left>2?r.left-2:0),t=std::max(py,r.top>2?r.top-2:0);
    U right=std::min(px+pw-1,r.right+2),bottom=std::min(py+ph-1,r.bottom+2);
    r.hl=l-px;r.ht=t-py;r.hw=right-l+1;r.hh=bottom-t+1;U size=r.hw*r.hh;
    pixels+=size;job_count+=(size+255)/256;
    if(pixels>payload||job_count>jobs)throw std::runtime_error("packed capacity");
    V complement(size);r.packed.resize(size);
    for(U q=0;q<size;++q){U x=l+q%r.hw,y=t+q/r.hw,i=y*w+x;bool region=roots[i]==r.root,nearby=false;
      for(int dy=-2;dy<=2;++dy)for(int dx=-2;dx<=2;++dx){int xx=int(x)+dx,yy=int(y)+dy;if(xx>=int(l)&&xx<=int(right)&&yy>=int(t)&&yy<=int(bottom)&&roots[U(yy)*w+U(xx)]==r.root)nearby=true;}
      r.packed[q]=U(region)|(parent[i]?2:0)|(nearby?4:0)|(cut[i]?8:0);complement[q]=!region;
      if(nearby&&cut[i])++r.cut;
    }
    V comp=graph(complement,r.hw,r.hh,rounds,reverse,false),boundary(size);
    for(U q=0;q<size;++q)if(comp[q]&&(q%r.hw==0||q%r.hw+1==r.hw||q/r.hw==0||q/r.hw+1==r.hh))boundary[comp[q]-1]=1;
    for(U q=0;q<size;++q){bool filled=(r.packed[q]&1)||!boundary[comp[q]-1];if((r.packed[q]&2)&&((r.packed[q]&4)||filled)){r.packed[q]|=16;++r.support;}}
  }
  std::cout<<"{\"usable\":true,\"roots\":";array(roots);std::cout<<",\"flags\":";array(flags);std::cout<<",\"stats\":";array({valid_count,candidate_count,U(all.size()),U(kept.size())});std::cout<<",\"records\":[";
  bool comma=false;for(auto& r:kept){if(comma)std::cout<<',';comma=true;std::cout<<"{\"root\":"<<r.root<<",\"parent\":0,\"key\":"<<r.key<<",\"area\":"<<r.area<<",\"bounds\":";array({r.left-px,r.top-py,r.right-r.left+1,r.bottom-r.top+1});std::cout<<",\"halo\":";array({r.hl,r.ht,r.hw,r.hh});std::cout<<",\"origin\":";array({px+r.hl,py+r.ht});std::cout<<",\"cut\":"<<r.cut<<",\"support\":"<<r.support<<",\"packed\":";array(r.packed);std::cout<<'}';}
  std::cout<<"]}\n";
}catch(const std::exception&){std::cout<<"{\"usable\":false}\n";}}
