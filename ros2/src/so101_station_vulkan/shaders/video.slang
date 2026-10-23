// BT.601 limited-range RGB8 -> two-plane NV12. One invocation owns four
// horizontal pixels (and, on even rows, two disjoint 2x2 chroma groups).
[[vk::binding(0,0)]] StructuredBuffer<uint> rgb;
[[vk::binding(1,0)]] RWStructuredBuffer<uint> nv12;
struct Params {uint width;uint height;};
[[vk::push_constant]] Params params;
uint byte_at(uint n){return (rgb[n/4]>>((n%4)*8))&255;}
int3 pixel(uint x,uint y){uint n=(y*params.width+x)*3;return int3(byte_at(n),byte_at(n+1),byte_at(n+2));}
uint luma(int3 p){return uint(clamp(((66*p.x+129*p.y+25*p.z+128)>>8)+16,16,235));}
uint2 chroma(uint x,uint y){int3 p=(pixel(x,y)+pixel(x+1,y)+pixel(x,y+1)+pixel(x+1,y+1)+2)/4;return uint2(clamp(((-38*p.x-74*p.y+112*p.z+128)>>8)+128,16,240),clamp(((112*p.x-94*p.y-18*p.z+128)>>8)+128,16,240));}
[shader("compute")]
[numthreads(16,16,1)]
void rgb_to_nv12(uint3 id:SV_DispatchThreadID){uint x=id.x*4,y=id.y;if(x>=params.width||y>=params.height)return;uint value=0;for(uint j=0;j<4;j++)value|=luma(pixel(x+j,y))<<(8*j);nv12[(y*params.width+x)/4]=value;if((y&1)==0){uint2 a=chroma(x,y),b=chroma(x+2,y);nv12[(params.width*params.height+(y/2)*params.width+x)/4]=a.x|(a.y<<8)|(b.x<<16)|(b.y<<24);}}
