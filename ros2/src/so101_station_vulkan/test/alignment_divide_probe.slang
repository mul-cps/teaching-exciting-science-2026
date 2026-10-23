// Qualification only: execute the actual production helper and bindings.
#include "../shaders/alignment.slang"
[shader("compute")][numthreads(256,1,1)]
void divide_probe(uint3 id:SV_DispatchThreadID) {
    uint i=id.x;
    if(i>=p.corners) return;
    packed.Store(i*4,alignment_divide_bits(raw.Load(i*8),raw.Load(i*8+4)));
}
