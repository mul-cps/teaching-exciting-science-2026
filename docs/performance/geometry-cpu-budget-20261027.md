# Retained CPU geometry budget — 2026-10-27

The unchanged retained CPU path passes all 12 frozen synthetic reports locally. **The fwmini4 whole-report parity gate remains closed: 10/12 pass with pinned numerical packages.** This checkpoint does not admit the rejected QR factor probe or qualify full native geometry, camera processing, station throughput or physical execution. Sources bind immutable commit `3416f2b9a08306afb13db94f787538e9d668d55b`.

| Case | Frame | Wall p50/p95 ms | Process CPU p50/p95 ms |
|---|---|---:|---:|
| base | 128×96 | 4.054/4.157 | 4.034/4.112 |
| invalid-hole | 128×96 | 3.351/3.445 | 3.333/3.428 |
| touching-depth-children | 128×96 | 3.451/3.538 | 3.433/3.520 |
| nonplanar-two-face | 128×96 | 4.236/4.324 | 4.219/4.304 |
| sensor-edge | 128×96 | 3.764/3.972 | 3.748/3.949 |
| sparse-concave | 128×96 | 3.371/3.520 | 3.353/3.503 |
| partial-observation-roi | 128×96 | 4.117/4.209 | 4.096/4.191 |
| partial-pickup-roi | 128×96 | 4.153/4.379 | 4.132/4.352 |
| distortion-4 | 128×96 | 4.176/4.286 | 4.153/4.261 |
| distortion-5 | 128×96 | 4.171/4.320 | 4.148/4.297 |
| distortion-8 | 128×96 | 4.166/4.352 | 4.143/4.315 |
| large-gray-support | 640×480 | 7.847/8.069 | 7.818/8.035 |

The primary local run used Python 3.13.12, NumPy 2.5.4, OpenCV 4.14.0 and SciPy 1.17.1; OpenBLAS/OMP/OpenCV threads=1, three warmups and 20 measurements per case. All complete before/instrumented/after reports have identical hashes, and frozen comparison retains its 1e-9 tolerance. No affinity, power or workload reservation was imposed. Eleven cases are only 128×96; VGA gray contains no colored objects. This is not a full-resolution camera geometry workload.

Separate nested instrumentation and cProfile runs provide attribution, not throughput. Exclusive CPU categories sum exactly to each instrumented root; inclusive costs overlap. Base support fitting is 2.601 ms inclusive, with 2.258 ms exclusive in RANSAC/robust residual/report work. Nine weighted fits total .248 ms, including .078 ms in nine SVDs. VGA support fitting is 3.592 ms; nine SVDs .316 ms, masks 1.233 ms and four parent CCL calls 1.989 ms. The retained RANSAC/robust path is a stronger measured optimization candidate than rewriting those SVDs.

The budget covers support sampling/cap/fitting, depth-child preparation, observed projection/ROI/morphology/contours, component validation/PCA/percentiles, pickup and grasp/report work. It does not call native Child/Observed modules. Report assembly remains in root/geometry residual categories; JSON serialization is measured separately. Machine data retains all stage attribution and populations.

On fwmini4, system NumPy 1.26.4/OpenCV 4.6/SciPy 1.11.4 first failed base validation extent. A private wheels-only Python 3.12.3 environment with the same NumPy/OpenCV/SciPy versions as local still fails `nonplanar-two-face` grasp closing axis and `sensor-edge` validation extent. Partial target timing logs are diagnostic history, not an accepted whole-matrix benchmark.

Read-only tracing finds the failing matrices and centers bit-identical across hosts. Nonplanar contour SVD has leading relative singular-value gap 1.22e-16 locally/zero remotely; valid bases differ by up to 1.68. Sensor-edge validation has gap 5.15e-16; its basis differs .006157 and the reported extent differs 5.50e-5 m. Reconstruction errors remain about 1e-16. Base is a matching positive control. The bundled OpenBLAS64 bytes match; Python-ABI-specific NumPy extensions differ. This supports an underdetermined/nearly repeated PCA/grasp orientation correctness issue; it does not isolate a CPU instruction or justify numerical retries, dispatch forcing or tolerance changes.

The focused observer accidentally shadowed its case-label field; retained comparison correlates verified call order, callers and same-host report hashes without changing arrays. Initial transfer/import and manifest-check failures, system/pinned refusals and all raw arrays/reports remain in the private archive. No source or algorithm changed. The owner service was untouched.

[Machine readout](geometry-cpu-budget-20261027.json) · [Archive inventory](evidence/geometry-cpu-budget-20261027/private-archive.json). Recheck selected evidence and immutable source blobs with `python3 docs/performance/evidence/geometry-cpu-budget-20261027/selfcheck.py`; this never reruns a fitter or touches hardware.
