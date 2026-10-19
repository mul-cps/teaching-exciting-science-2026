# Board-plane color sampling

The board plane fit already selects at most approximately 8,000 image pixels before limiting depth candidates to 2,000. Color rejection now evaluates the existing channel-spread rule at those same sample coordinates. It previously computed channel minima, maxima and saturation over the whole image.

The board polygon, erosion, sample grid, thresholds, depth bounds, deterministic RANSAC, coverage gates and output metrics are unchanged. Color shape/type validation remains before geometric processing. The change creates no workspace crop or new acceptance rule.

## Evidence

Author checks: 10 focused native tests passed. A traced-array regression fails against the original full-image reductions and passes with sampled reductions. Three malformed color cases retain their rejection.

The offline benchmark pins original source revision `370d8ee82489d58a1bcce9e6a401dbef7738237d`. It compares plane centers and normals with exact array equality and all metric fields with exact equality across 12 alternating runs per saved capture.

| Saved capture | Original mean ms | Sampled mean ms |
| --- | ---: | ---: |
| H00, 640×480 | 14.33 | 2.16 |
| H01, 640×480 | 13.63 | 1.95 |
| H02, 640×480 | 13.34 | 2.05 |
| Registration baseline, 1920×1080 | 77.35 | 2.56 |

These are function timings on frozen images, not complete callback timings or a live freshness qualification. Each call uses the same recorded canonical board transform converted into the OpenCV board convention as its seed. The benchmark does not qualify a new marker detection or PnP result. An initial attempt to redetect the board in every old capture lacked enough corners and was abandoned rather than substituting an inferred detection.

Reproduce with `python tools/benchmark_board_plane_locality.py --capture <capture-directory> --samples 12 --output <new-json-path>`. The output records immutable source/input hashes, runtime versions, every timing sample and exact parity.

Artifacts are under the station evidence directory `2026-10-19-sorting/overnight/board-plane-locality-benchmark.json`. Independent review and any live deployment evidence must be recorded separately.

Independent review repeated all 10 native tests and four captures with six alternating runs each. All centers, normals and metric fields matched exactly. Independent mean times were 15.99→2.44, 13.91→2.28, 13.63→2.12 and 77.10→3.05 ms. The reviewer confirmed unchanged input validation, sample grid, arithmetic and geometric gates, with no new I/O, GPU execution or per-pixel Python loop.
