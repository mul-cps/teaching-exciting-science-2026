// AOT actual Slang arithmetic with explicit SERIAL test-only atomic adapters.
// No Vulkan loader/device; this is not GPU parallelism/runtime qualification.
#include "so101_station_vulkan/observed_children.hpp"
#include <iostream>
#include <cstring>
#include <vector>
using namespace so101_station_vulkan;
struct BufferView {void* data;size_t bytes;};
struct Globals {BufferView roots,children,parents,geometry,roi,arena,compact;observed_detail::Parameters* p;};
struct Grid {uint32_t start[3],end[3];};
static_assert(sizeof(Globals)==120&&sizeof(Grid)==24,"reflected Slang CPU ABI");
extern "C" {
void geometry_coordinates(void*,void*,void*);void observed_clear(void*,void*,void*);void observed_records(void*,void*,void*);void observed_observe(void*,void*,void*);void observed_eligible(void*,void*,void*);void observed_close(void*,void*,void*);void observed_finalize(void*,void*,void*);
}
int main(int argc,char** argv){unsigned w,h,c,capacity,enabled;if(!(std::cin>>w>>h>>c>>capacity>>enabled))return 1;
 ObservedLayout layout(w,h,std::max(1u,c),1,capacity);unsigned n=w*h;
 std::array<double,6> roi;for(auto& x:roi)std::cin>>x;
 std::vector<uint32_t> roots(2*n);for(unsigned i=0;i<n;i++)std::cin>>roots[i];
 std::vector<double> xyz(3*n);bool prepare_geometry=argc>1&&std::string(argv[1])=="geometry";
 if(prepare_geometry){
  double maximum;std::cin>>maximum;std::vector<uint32_t> source(2*n),storage(7*n);for(unsigned i=0;i<n;i++)std::cin>>source[n+i];
  std::vector<double> calibration(2*n+19);for(unsigned i=0;i<2*n+12;i++)std::cin>>calibration[i];
  for(unsigned axis=0;axis<3;axis++){calibration[2*n+12+2*axis]=-1e10;calibration[2*n+13+2*axis]=1e10;}calibration[2*n+18]=double(float(maximum));
  uint32_t counts[4]{},parameters[]={w,h,n,((w+7)/8)*((h+7)/8),(w+7)/8,((((w+7)/8)*((h+7)/8))+255)/256};
  struct GeometryGlobals{BufferView source,calibration,arena,counts;uint32_t* p;};static_assert(sizeof(GeometryGlobals)==72,"reflected Geometry CPU ABI");
  GeometryGlobals globals{{source.data(),source.size()*4},{calibration.data(),calibration.size()*8},{storage.data(),storage.size()*4},{counts,16},parameters};Grid grid{{0,0,0},{(n+255)/256,1,1}};geometry_coordinates(&grid,nullptr,&globals);std::memcpy(xyz.data(),storage.data(),xyz.size()*8);
 }else for(auto& x:xyz)std::cin>>x;
 struct Record {unsigned root,x,y,w,h;};std::vector<Record> records(c);std::vector<std::array<uint32_t,16>> native_records(c);unsigned parent_data[5]={0,0,w,h,n},status_data[4]{},minimum=8;
 if(prepare_geometry){for(auto& v:parent_data)std::cin>>v;for(auto& v:status_data)std::cin>>v;std::cin>>minimum;for(unsigned k=0;k<c;k++){auto& v=native_records[k];for(auto& value:v)std::cin>>value;records[k]={v[0],parent_data[0]+v[4],parent_data[1]+v[5],v[6],v[7]};}}
 else for(auto& r:records)std::cin>>r.root>>r.x>>r.y>>r.w>>r.h;
 std::vector<uint32_t> children((128+16+64*layout.capacity)/4),parents((64+32)/4),arena(12*n),compact(layout.compact_bytes/4);
 children[0]=1;children[1]=w;children[2]=h;children[3]=1;children[4]=c;children[8]=1;children[10]=2;children[12]=3;children[16]=minimum;
 parents[0]=1;parents[7]=1;parents[16+3]=parent_data[4];parents[16+4]=parent_data[0];parents[16+5]=parent_data[1];parents[16+6]=parent_data[2];parents[16+7]=parent_data[3];
 unsigned packed=0;for(unsigned k=0;k<c;k++){auto& r=records[k];auto* v=children.data()+36+16*k;if(prepare_geometry){std::copy(native_records[k].begin(),native_records[k].end(),v);packed+=v[10]*v[11];continue;}v[0]=r.root;v[1]=0;v[2]=k;v[3]=unsigned(std::count(roots.begin(),roots.begin()+n,r.root));v[4]=r.x;v[5]=r.y;v[6]=r.w;v[7]=r.h;v[8]=r.x>2?r.x-2:0;v[9]=r.y>2?r.y-2:0;v[10]=std::min(w,r.x+r.w+2)-v[8];v[11]=std::min(h,r.y+r.h+2)-v[9];v[12]=packed;v[14]=v[3];packed+=v[10]*v[11];}children[5]=packed;unsigned originals=0,jobs=0;for(unsigned k=0;k<c;k++){auto* v=children.data()+36+16*k;originals+=v[3];jobs+=(v[10]*v[11]+255)/256;}children[6]=jobs;children[32]=children[33]=originals;children[34]=children[35]=c;if(prepare_geometry)std::copy(std::begin(status_data),std::end(status_data),children.begin()+32);
 unsigned mutate=argc>1&&!prepare_geometry?unsigned(std::atoi(argv[1])):0;auto* first=children.data()+36;
 if(mutate==1)children[35]++;if(mutate==2)children[33]=0;if(mutate==3)children[34]=0;if(mutate==4)children[6]++;if(mutate==5)children[5]++;if(mutate==6&&c)first[0]=n+1;if(mutate==7&&c)first[1]=1;if(mutate==8&&c>1)children[36+16]=first[0];if(mutate==9&&c)first[2]=n;if(mutate==10&&c)first[12]++;if(mutate==11&&c)first[14]=n+1;if(mutate==12&&c)first[15]=1;if(mutate==13)children[7]=1;if(mutate==14)roots[0]=n+1;if(mutate==15&&c)first[3]++;
 observed_detail::Parameters p{w,h,n,layout.capacity,1,layout.closed_capacity,enabled,1,0,2,0,3,0,4,0,minimum};
 Globals g{{roots.data(),roots.size()*4},{children.data(),children.size()*4},{parents.data(),parents.size()*4},{xyz.data(),xyz.size()*8},{roi.data(),48},{arena.data(),arena.size()*4},{compact.data(),compact.size()*4},&p};
 using Entry=void(*)(void*,void*,void*);Entry entries[]={observed_clear,observed_records,observed_observe,observed_eligible,observed_close,observed_finalize};
 unsigned groups[]={(n+255)/256,(layout.capacity+255)/256,(n+255)/256,(layout.capacity+255)/256,(n+255)/256,1};
 for(unsigned k=0;k<6;k++){Grid grid{{0,0,0},{groups[k],1,1}};entries[k](&grid,nullptr,&g);}
 if(compact[7]){std::cout<<"{\"usable\":false}\n";return 0;}
 std::cout<<"{\"usable\":true,\"records\":[";
 for(unsigned k=0;k<c;k++){if(k)std::cout<<',';auto* r=compact.data()+16+20*k;std::cout<<"{\"root\":"<<r[0]<<",\"original_pixels\":"<<r[3]<<",\"roi_pixels\":"<<r[16]<<",\"eligible\":"<<((r[18]&1)?"true":"false")<<",\"closed_pixels\":"<<r[17];
  for(unsigned closed=0;closed<2;closed++){std::cout<<",\""<<(closed?"closed":"original")<<"\":[";for(unsigned i=0;i<n;i++){if(i)std::cout<<',';bool member=arena[n+i]==k+1;if(closed){member=false;for(unsigned j=0;j<arena[2*n+i];j++)member|=arena[3*n+9*i+j]==k+1;}std::cout<<unsigned(member);}std::cout<<']';}std::cout<<'}';}
 std::cout<<"]}\n";
}
