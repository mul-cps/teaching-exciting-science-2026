# Component locality: frozen parity and performance proof

Baseline: `522a6584106a9650811e9a238a540d6c0068a842` (immutable default in benchmark, explicit override supported). This patch changes CPU buffer locality; admission thresholds, depth holes, support dilation, piece ordering and global geometry stay unchanged. Legacy splitter output remains the default. Local outputs carry explicit global pixel origins, and cropped intrinsics subtract that same origin.

## Verification

`python -m pytest -q tests/test_colored_component_locality.py tests/test_colored_object_validation.py tests/test_colored_observation_depth.py tests/test_ros2_objects.py`:86 passed,1 existing native-environment skip in0.77s. Synthetic legacy comparison explicitly skips if immutable baseline history is absent.

Independent author-separated review (Intel integration worker) replayed86 passed/1 skip and all four frozen captures with `--baseline-ref 522a6584106a9650811e9a238a540d6c0068a842 --samples 3`. Exact classifications/counts/diagnostics; equivalent numeric geometry tolerance1e-9 absolute. Object counts9/9/9/10. No GPU/readback/new per-pixel Python work.

| Frozen capture | Before mean ms | After mean ms |
|---|---:|---:|
| H00 |83.54|46.33|
| H01 |88.16|51.09|
| H02 |87.95|45.56|
| HD registration baseline |1004.13|188.37|

Inputs under `2026-10-19-sorting/home-series-model-calibration/{H00,H01,H02}` and `commissioning-live/registration-baseline-bound`. Full hashes and diagnostics: station evidence `gpu-perception/component-locality-{benchmark,independent}.json`. Build co-residency retained; these are saved-frame analyzer timings, not live ROS original-age/throughput qualification. Whole analyzer includes image validation and output materialization; no GPU speedup claimed.

Reproduce: `python tools/benchmark_component_locality.py --capture <H00> --capture <H01> --capture <H02> --capture <HD> --samples 3 --output <artifact.json>`. All input file hashes and runtime source hashes are recorded.

## Frozen source hashes

- `ros2/src/so101_station_objects/so101_station_objects/colored_object_validation.py`: `5c102ac55c561a93e6b934f920e5cae7a0f2abb70309433706328c24a57ffa99`
- `ros2/src/so101_station_objects/so101_station_objects/objects.py`: `62a85375cb922041c7a1d6326b0eb8ead66259cf11c003afd20988512e1d3ac1`
- `tests/test_colored_component_locality.py`: `66ade3c91cfbd9441d92f0408b8410c2dd3eaab774570c6bc3d88e8ff2e9d410`
- `tools/benchmark_component_locality.py`: `3ecbb9acb750a5f21b39843302f9912e74e060600cf59516d498158b689a2cd2`
