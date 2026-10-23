# Native geometry qualification boundary

The native preparation library is an **unqualified first stage**, not an object detector replacement. `GeometryPreparation` consumes a completed aligned RGB8/Z16 resident frame, preserves the source frame until its consumers finish, and computes calibrated FP64 board coordinates, valid-depth flags, radius-three vivid-color exclusion, and row-major stride-eight support samples on the shared Vulkan device. Its ordinary completion reads 16 bytes of counts. Sample coordinates remain resident; three fixed arenas bound memory and lifetime. At 640×480, each arena is 8,755,352 bytes and contains at most 4,800 support samples. No per-frame Vulkan allocation or queue/device wait is used. Dropped in-flight leases are collected nonblockingly on scheduler idle ticks with `collect()`.

The caller supplies a ray LUT once per immutable calibration generation. Generate it using the retained OpenCV `undistortPoints` convention for full-image pixel centers. The calibration contains the affine board transform, inclusive support ROI and observation depth limit. A different generation requires a new preparation instance; stale frames, insufficient buffer capacity, source faults and a shared device without **enabled** `shaderFloat64` are explicit failures. There is no CPU fallback hidden behind this API. The supplied LUT must not be reused for grasp footprint rays: the retained footprint algorithm intentionally uses pinhole rays.

The preparation does not fit a support plane, apply seed-17 point selection, validate depth components, construct children, trace ordered contours, compute exact quantiles/PCA or assemble object reports. All these remain necessary before native object admission can be enabled. Full report equivalence, strict distortion parity and throughput are open gates. Neither software tests nor a preparation result qualify physical grasping.

The offline oracle `tools/native_geometry_oracle.py` freezes complete retained reports together with parent/piece/support memberships, tight-bbox closing, ordered simplified contours, filled children intersected with original observations, both local/global calibrated rays and seed-17 support selection. Each lossless array carries its shape, dtype and SHA-256; the manifest pins dependencies and source hashes. `restore_arrays()` resolves references for report comparison. Discrete schemas, reasons, ordering and integer/boolean evidence must match exactly. Existing numeric limits remain absolute 1e-9 for published reports and 1e-12 for local/global distorted rays.

Run a public procedural freeze with:

```sh
python tools/native_geometry_oracle.py --output /tmp/full-geometry-oracle
```

An existing complete saved capture can be supplied with `--capture`. The capture must include an observed board/world transform; the tool refuses to invent one. The known October 16 lighting capture contains RGB/depth intrinsics and depth-to-color extrinsics but lacks the board transform, so it cannot independently qualify a complete world-space geometry report.

The host-only contract executable and Python oracle/static tests do not open devices. Before any GPU validation of a changed preparation path, a separate performance review must examine allocations, synchronization, lease cleanup and dense readbacks, add a benchmark and preserve a static guard. GPU qualification is not implied by successful offline SPIR-V compilation.
