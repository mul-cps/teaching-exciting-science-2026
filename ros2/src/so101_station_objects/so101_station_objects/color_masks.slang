// OpenCV's uint8 RGB->HSV integer semantics (hue range 180, shift 12).
// No floating-point feature is needed, including shaderFloat64.
ByteAddressBuffer rgb;
StructuredBuffer<int> divisions;
RWStructuredBuffer<uint> masks;
uniform uint pixelCount;
uint byteAt(uint i) { return (rgb.Load(i & ~3u) >> ((i & 3u) * 8u)) & 255u; }
uint maskAt(uint i) {
    if (i >= pixelCount) return 0u;
    int r = int(byteAt(3u*i)), g = int(byteAt(3u*i+1u)), b = int(byteAt(3u*i+2u));
    int v = max(r, max(g,b)), lo = min(r,min(g,b)), d = v-lo;
    int s = (d * divisions[v] + 2048) >> 12;
    int h = v == r ? g-b : (v == g ? b-r+2*d : r-g+4*d);
    h = (h * divisions[256+d] + 2048) >> 12;
    if (h < 0) h += 180;
    bool vivid = s >= 90 && v >= 50;
    return !vivid ? 0u : uint(h <= 10 || h >= 170)
        | (uint(h >= 41 && h <= 89) << 1)
        | (uint(h >= 90 && h <= 140) << 2)
        | (uint(h >= 15 && h <= 40) << 3);
}

// Four independent pixel results per word: one byte per pixel readback.
[numthreads(256, 1, 1)]
void main(uint3 tid : SV_DispatchThreadID) {
    uint i = tid.x * 4u;
    if (i >= pixelCount) return;
    masks[tid.x] = maskAt(i) | (maskAt(i+1u) << 8u)
                | (maskAt(i+2u) << 16u) | (maskAt(i+3u) << 24u);
}
