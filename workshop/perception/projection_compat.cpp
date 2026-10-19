// CPU compatibility stage for the fixed D435 horizontal occlusion policy.
// Policy copied from librealsense v2.58.3 src/proc/occlusion-filter.cpp:124-165
// (Apache-2.0); kept on CPU rather than substitute the different GLSL heuristic.
#include <cmath>
#include <cstddef>
#include <cstdint>

extern "C" int rocm_projection_compat_abi(){return 1;}

extern "C" int rocm_projection_finish(float* xyz,const float* pixels,float* uv,
    uint32_t* rgb,uint8_t* valid,const uint8_t* color,
    size_t count,int width,int height,int color_width,int color_height,int apply_occlusion) {
    if(!xyz||!pixels||!uv||!rgb||!valid||!color||width<1||height<1||width>1280||height>720
        ||color_width<1||color_height<1||color_width>1920||color_height>1080
        ||(apply_occlusion!=0&&apply_occlusion!=1)||count!=size_t(width)*size_t(height)) return 1;
    for(int y=0;y<height;++y) {
        float max_in_line=-1.f,max_z=0.f;int dilation_left=0;
        for(int x=0;x<width;++x) {
            const size_t i=size_t(y)*width+x;
            float* point=xyz+3*i;const float* pixel=pixels+2*i;
            if(!std::isfinite(point[0])||!std::isfinite(point[1])||!std::isfinite(point[2])
                ||point[2]<0.f||!std::isfinite(pixel[0])||!std::isfinite(pixel[1])) return 2;
            // CPU SDK creates UV before occlusion and retains it on invalidation.
            uv[2*i]=point[2]?pixel[0]/float(color_width):0.f;
            uv[2*i+1]=point[2]?pixel[1]/float(color_height):0.f;
            if(apply_occlusion&&point[2]) {
                if(pixel[0]<max_in_line||(pixel[0]==max_in_line&&(point[2]-max_z)>.1f)) {
                    point[0]=point[1]=point[2]=0.f;dilation_left=1;
                } else {
                    max_in_line=pixel[0];max_z=point[2];
                    if(dilation_left>0){point[0]=point[1]=point[2]=0.f;--dilation_left;}
                }
            }
            // Same safe half-open texture policy as the audited CPU wrapper.
            const float u=uv[2*i],v=uv[2*i+1];
            const float px=u*color_width,py=v*color_height;
            const bool texture=std::isfinite(u)&&std::isfinite(v)&&u>=0.f&&u<1.f&&v>=0.f&&v<1.f
                &&px>=0.f&&double(px)<color_width&&py>=0.f&&double(py)<color_height;
            valid[i]=point[2]>0.f&&texture;rgb[i]=0;
            if(valid[i]) {
                const int ix=int(px),iy=int(py);
                if(ix<0||iy<0||ix>=color_width||iy>=color_height)return 3;
                const size_t offset=(size_t(iy)*color_width+ix)*3;
                rgb[i]=(uint32_t(color[offset])<<16)|(uint32_t(color[offset+1])<<8)|color[offset+2];
            }
        }
    }
    return 0;
}
