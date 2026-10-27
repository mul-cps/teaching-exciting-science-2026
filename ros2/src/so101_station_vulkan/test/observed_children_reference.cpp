// Independent CPU test reference, NOT a production host fallback. No Vulkan API.
#include <array>
#include <cstdint>
#include <iostream>
#include <vector>
struct Record {unsigned root,x,y,w,h;};
int main(){unsigned w,h,c,capacity,enabled;if(!(std::cin>>w>>h>>c>>capacity>>enabled))return 1;
 if(!w||!h||uint64_t(w)*h>307200||c>4096)return 2;
 std::array<double,6> roi;for(auto& x:roi)std::cin>>x;
 unsigned n=w*h,total=0;capacity=capacity?capacity:4*n;
 std::vector<unsigned> roots(n);for(auto& x:roots)std::cin>>x;
 std::vector<std::array<double,3>> xyz(n);for(auto& v:xyz)for(auto& x:v)std::cin>>x;
 std::vector<Record> records(c);for(auto& r:records)std::cin>>r.root>>r.x>>r.y>>r.w>>r.h;
 struct Result {unsigned original=0,inside=0,closed=0;std::vector<unsigned> a,b;};std::vector<Result> results(c);
 for(unsigned k=0;k<c;k++){const auto& r=records[k];if(!r.root||r.x>=w||r.y>=h||!r.w||!r.h||r.w>w-r.x||r.h>h-r.y)return 3;
  auto& out=results[k];out.a.resize(n);out.b.resize(n);
  for(unsigned i=0;i<n;i++)if(roots[i]==r.root){out.a[i]=1;++out.original;bool inside=true;for(unsigned axis=0;axis<3;axis++)inside&=!enabled||(xyz[i][axis]>=roi[2*axis]&&xyz[i][axis]<=roi[2*axis+1]);out.inside+=inside;}
  if(out.inside<8)continue;
  // Dense two-step reference deliberately differs from sparse candidate shader.
  std::vector<unsigned> dilated(r.w*r.h);
  for(unsigned y=0;y<r.h;y++)for(unsigned x=0;x<r.w;x++)for(int dy=-1;dy<=1;dy++)for(int dx=-1;dx<=1;dx++){int xx=int(x)+dx,yy=int(y)+dy;if(xx>=0&&yy>=0&&xx<int(r.w)&&yy<int(r.h)&&roots[(r.y+unsigned(yy))*w+r.x+unsigned(xx)]==r.root)dilated[y*r.w+x]=1;}
  for(unsigned y=0;y<r.h;y++)for(unsigned x=0;x<r.w;x++){bool selected=true;for(int dy=-1;dy<=1;dy++)for(int dx=-1;dx<=1;dx++){int xx=int(x)+dx,yy=int(y)+dy;if(xx>=0&&yy>=0&&xx<int(r.w)&&yy<int(r.h))selected&=dilated[unsigned(yy)*r.w+unsigned(xx)]!=0;}if(selected){out.b[(r.y+y)*w+r.x+x]=1;++out.closed;++total;}}
 }
 if(total>capacity){std::cout<<"{\"usable\":false}\n";return 0;}
 std::cout<<"{\"usable\":true,\"records\":[";
 for(unsigned k=0;k<c;k++){if(k)std::cout<<',';auto& r=results[k];std::cout<<"{\"root\":"<<records[k].root<<",\"original_pixels\":"<<r.original<<",\"roi_pixels\":"<<r.inside<<",\"eligible\":"<<(r.inside>=8?"true":"false")<<",\"closed_pixels\":"<<r.closed;
  auto array=[&](const char* key,const std::vector<unsigned>& values){std::cout<<",\""<<key<<"\":[";for(unsigned i=0;i<n;i++){if(i)std::cout<<',';std::cout<<values[i];}std::cout<<']';};array("original",r.a);array("closed",r.b);std::cout<<'}';}
 std::cout<<"]}\n";
}
