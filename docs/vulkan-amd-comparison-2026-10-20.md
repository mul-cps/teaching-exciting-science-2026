# AMD Vulkan projection comparison, 2026-10-20

Decision: retain ROCm as the workstation default.

Hardware: AMD Radeon 8060S, gfx1151. The paired comparison used the same frozen
real D435i 1280×720 depth and 1920×1080 color fixture, five warmups, and 200 timed
frames per backend in alternating blocks.

| Complete owned projection processing | Median | p95 | p99 |
| --- | ---: | ---: | ---: |
| Existing ROCm | 8.609 ms | 10.109 ms | 11.025 ms |
| Exact AMD Vulkan diagnostic variant | 176.669 ms | 187.432 ms | 191.233 ms |

The timed operation includes input conversion/upload, dispatch, output readback,
native occlusion and RGB processing, and owned result buffers. It excludes camera
capture, ROS transport, and object perception. Vulkan was 20.52 times slower by
median; no full-pipeline or N100 speedup follows from this experiment.

The original portable Vulkan path failed exact parity on 18 aligned-depth and 10
RGB pixels. An isolated Slang 2026.19 compiler plus Float64 division followed by
Float32 conversion produced exact checked depth, holes, XYZ, UV, RGB and validity
outputs on AMD. This variant was used for the table and was not deployed. It is
not an N100 profile because that GPU lacks the required Float64 support.

GPU completion/readback/owned-copy timing dominated the corrected variant at
167 ms median. A separate Float32 diagnostic was also slow (134 ms complete) and
failed parity, so Float64 alone does not explain the slowdown. No default or live
processing backend was changed.

Local evidence is under the station evidence directory in
`2026-10-19-sorting/vulkan-current-benchmark-20261020/`: `decision.json`,
`paired-amd-exact.json`, `compiler-loader-proof.json`, and the 57-file
`SHA256SUMS.json` manifest. The independent source review is in the sibling
`vulkan-current-review-20261020/` directory.
