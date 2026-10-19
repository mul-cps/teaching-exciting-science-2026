# Default-off ROCm projection/alignment prototype

Base4e47649c9cd33934ecee7eec092cfc67ac9ab197, branchfeat/rocm-projection-20261019. No ROS subscriptions, SDK physical-device access, owner calls, default activation or source-control actions. Root alone integrates/deploys.

## Decision

Pinned SDKGLSL is rejected as a metric drop-in after strict frozen synthetic/native tests: float16 textures, differing zero-depth UV/occlusion and depth-footprint behavior. Preserve failed artifacts in gpu-perception/GLSL-DECISION.md; never relax their parity gates.

Prototype ROCm PyTorch storage/transfer with explicit Triton float32 arithmetic kernels. Distinguish GPU projection/alignment from the retained exact CPU occlusion compatibility stage. CPU installed SDK2.58.3 software-frame output is the oracle, including actual generic/SSE processing-block identity. No inference or robot control.

## Frozen semantics and bounds

- Inputs: original native Z16 little-endian depth, original color RGB8, exact K/D and column-major depth-to-color R/T; original headers and immutable file hashes. Zero distortion only; dimensions <=1280x720 native/1920x1080color; fixed .001ROSscale declaration, no physical scale qualification.
- Match selected CPU generic/SSE float32 operation order, pixel corners +/-0.5, truncation after +0.5, whole-rectangle boundary rejection and minimum ORIGINAL raw Z16 for overlap. Zero-depth remains unknown. No bilinear depth, nearest-surface replacement, hole filling or precision reduction.
- Fixed4x4 splat footprint budget. Any eligible rectangle larger than this, nonfinite projection or unsupported behind-camera case reports unavailable, never clips, drops or silently falls back.
- Raw XYZ/projection retains float32. CPU compatibility uses pinned SDK whole-row monotonic occlusion with one-pixel dilation, not GL10sample behavior. SDKvertical predicate(tx<.001f &&ty>.01f) rejects before GPU. SDKidentity T==0/Rdelta<=float32epsilon correctly skips occlusion. CPU zero-depth UV remains zero. RGB copying uses finite half-open UV/int/stride/storage guards.
- One bounded depth upload, fixed output initialization plus one projection/splat kernel, one complete packed cohort readback. No per-ROI/per-point Python/device synchronization. CPU occlusion/colorization operates once over native arrays in compiled code. Allocation/cache costs and compiler cold cost reported separately from warm entire cohort.
- Original exposure headers are carried separately; benchmark repeats are never new acquisitions. Output remains diagnostic/unqualified; no motion or calibration authority.

## Implementation order

1. Freeze installed-CPU software-frame oracle, output binary/meta hashes and selected block names. Test synthetic plane/holes/overlap/out-of-frame and root frozen native cohort.
2. Pure CPU contract/calibration validation and NumPy arithmetic oracle tests; explicit bounded failure cases. Add Triton source without GPU execution.
3. Independent separate performance-refinement review: allocations, fixed kernel/transfer count, precision flags, output volume, no per-item loops/syncs; AST guards and whole-cohort benchmark.
4. Only after ACK, GPU hardware tests: exact raw-Z16 and unknown masks, exact zero-mask semantics; XYZ/UV1e-5 thresholds unchanged. Cold/warm upload+kernel+readback+CPUcompatibility and full output materialization, both synthetic and frozen native.
5. If parity or budget fails, preserve failure; no deployment. If it passes, root reviews a separate optional bounded ROS3.12/worker3.13 interface design rather than import incompatible Python Torch into ROS.

## Risks / unresolved before implementation

Installed CPU may select SSE arithmetic differing from generic source; inspect actual block names and tie-boundary behavior first. Triton AMD exact division/FMA controls need installed-source verification, no assumed API. The4x4 cap is an explicit unavailable condition, not extended SDK support. Actual ROS serialization/pointcloud publishing may still dominate; saved-frame speed does not establish live throughput. Physical depth scale, tag shape, model/TCP and pick-place remain unqualified.
