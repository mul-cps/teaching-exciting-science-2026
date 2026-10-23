// Offline arithmetic diagnostic only. No station pipeline uses these bindings.
#include "soft_binary64.slang"
[[vk::binding(0,0)]] StructuredBuffer<uint64_t> probe_input;
[[vk::binding(1,0)]] RWStructuredBuffer<uint64_t> probe_output;
struct ProbeParameters { uint count; };
[[vk::push_constant]] ConstantBuffer<ProbeParameters> p;
[shader("compute")][numthreads(256,1,1)]
void main(uint3 id:SV_DispatchThreadID) {
    uint i=id.x;
    if(i>=p.count) return;
    uint64_t a=probe_input[i*3], b=probe_input[i*3+1];
    probe_output[i*4]=sf64_add_bits(a,b);
    probe_output[i*4+1]=sf64_mul_bits(a,b);
    probe_output[i*4+2]=sf64_from_f32_bits(uint(probe_input[i*3+2]));
    uint flags=(sf64_less(a,b)?1u:0u)|(sf64_less_equal(a,b)?2u:0u)|
        (sf64_is_finite(a)?4u:0u)|(sf64_is_finite(b)?8u:0u);
    probe_output[i*4+3]=uint64_t(flags);
}
