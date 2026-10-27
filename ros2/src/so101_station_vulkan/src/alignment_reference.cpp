#include "so101_station_vulkan/alignment.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>
namespace so101_station_vulkan {
void validate_alignment(const AlignmentCalibration& c){
 for(const auto* k:{&c.depth,&c.color}){
  const bool standard=(k->width==640&&k->height==480)||(k->width==1280&&k->height==720)||(k==&c.color&&k->width==1920&&k->height==1080);
  if(!standard)throw std::invalid_argument("alignment requires bounded standard depth/color extents");
  if(!std::isfinite(k->fx)||!std::isfinite(k->fy)||k->fx<=0||k->fy<=0||!std::isfinite(k->ppx)||!std::isfinite(k->ppy))throw std::invalid_argument("invalid alignment intrinsics");
  if(k->model!=RS2_DISTORTION_NONE&&k->model!=RS2_DISTORTION_BROWN_CONRADY&&k->model!=RS2_DISTORTION_INVERSE_BROWN_CONRADY&&k->model!=RS2_DISTORTION_MODIFIED_BROWN_CONRADY)throw std::invalid_argument("unsupported alignment distortion model");
  for(float v:k->coeffs)if(v!=0)throw std::invalid_argument("nonzero distortion requires separate numerical qualification");
 }
 if(!std::isfinite(c.depth_units)||c.depth_units<=0)throw std::invalid_argument("invalid metric depth units");
 for(float v:c.depth_to_color.rotation)if(!std::isfinite(v))throw std::invalid_argument("invalid alignment rotation");
 for(float v:c.depth_to_color.translation)if(!std::isfinite(v))throw std::invalid_argument("invalid alignment translation");
 if(c.convention!=AlignmentConvention::Scalar&&c.convention!=AlignmentConvention::Sse)throw std::invalid_argument("unknown SDK alignment convention");
}
bool alignment_uses_corners(const AlignmentCalibration& c){
 if(c.convention==AlignmentConvention::Scalar)return true;
 float df[2],cf[2];rs2_fov(&c.depth,df);rs2_fov(&c.color,cf);
 return c.depth.width/df[0]<c.color.width/cf[0]||c.depth.height/df[1]<c.color.height/cf[1];
}
void alignment_reference(const AlignmentCalibration& c,const uint16_t* raw,uint32_t* aligned,float* xyz){
 validate_alignment(c);const unsigned n=unsigned(c.color.width)*unsigned(c.color.height);std::fill(aligned,aligned+n,UINT32_MAX);bool corners=alignment_uses_corners(c);
 auto project=[&](int x,int y,float offset,float z,int& ox,int& oy){
  float px=z*((float(x)+offset-c.depth.ppx)/c.depth.fx),py=z*((float(y)+offset-c.depth.ppy)/c.depth.fy);const auto& e=c.depth_to_color;
  float tx,ty,tz;
  if(c.convention==AlignmentConvention::Sse){tx=e.rotation[0]*px+(e.rotation[3]*py+(e.rotation[6]*z+e.translation[0]));ty=e.rotation[1]*px+(e.rotation[4]*py+(e.rotation[7]*z+e.translation[1]));tz=e.rotation[2]*px+(e.rotation[5]*py+(e.rotation[8]*z+e.translation[2]));}
  else {float p[3]{px,py,z},q[3];rs2_transform_point_to_point(q,&e,p);tx=q[0];ty=q[1];tz=q[2];}
  float u=(tx/tz)*c.color.fx+c.color.ppx+.5f,v=(ty/tz)*c.color.fy+c.color.ppy+.5f;
  if(!std::isfinite(u)||!std::isfinite(v)||u<-2147483000.f||u>2147483000.f||v<-2147483000.f||v>2147483000.f){ox=oy=INT32_MIN;return;}
  ox=c.convention==AlignmentConvention::Sse?int(std::nearbyint(u)):int(u);oy=c.convention==AlignmentConvention::Sse?int(std::nearbyint(v)):int(v);
 };
 for(int y=0;y<c.depth.height;y++)for(int x=0;x<c.depth.width;x++){auto d=raw[y*c.depth.width+x];if(!d)continue;float z=c.depth_units*d;int x0,y0,x1,y1;project(x,y,-.5f,z,x0,y0);if(corners)project(x,y,.5f,z,x1,y1);else{x1=x0;y1=y0;}
  if(c.convention==AlignmentConvention::Scalar&&(x0<0||y0<0||x1>=c.color.width||y1>=c.color.height))continue;
  if(x1>=x0&&y1>=y0&&int64_t(std::max(0,std::min(c.color.width-1,x1)-std::max(0,x0)+1))*std::max(0,std::min(c.color.height-1,y1)-std::max(0,y0)+1)>4096)throw std::runtime_error("alignment projection footprint exceeds bounded qualification");
  for(int yy=std::max(0,y0);yy<=std::min(c.color.height-1,y1);yy++)for(int xx=std::max(0,x0);xx<=std::min(c.color.width-1,x1);xx++)aligned[yy*c.color.width+xx]=std::min(aligned[yy*c.color.width+xx],uint32_t(d));
 }
 for(unsigned i=0;i<n;i++){if(aligned[i]==UINT32_MAX)aligned[i]=0;float z=c.depth_units*aligned[i];xyz[i*4]=z*((float(i%unsigned(c.color.width))-c.color.ppx)/c.color.fx);xyz[i*4+1]=z*((float(i/unsigned(c.color.width))-c.color.ppy)/c.color.fy);xyz[i*4+2]=z;xyz[i*4+3]=0;}
}
}
