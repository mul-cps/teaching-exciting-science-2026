# Canonical principal directions — 2026-10-27

Numerically tied principal values previously exposed an arbitrary SVD/eigenvector basis through grasp axes and percentile extents. With bit-identical inputs, the retained fwmini4 CPU reports matched only 10 of 12 historical cases. The new explicit direction policy makes **all twelve v2 complete reports pass locally and on fwmini4** at unchanged tolerances. Historical v1 payloads and the old target 10/12 refusal remain intact. Historical v1 replay here was **local only**, using immutable `1d088c7` sources. Production baseline is `47002d05b81d23feb87afcaccbdd72122705e7c3`; the reviewed legacy source-epoch migration is `8744239c4c367696ae00dcebb7365b0eaaef641c`. The final canonical CPU run passes with all 934 frozen files unchanged.

The `canonical-principal-axes-v1` policy keeps centering, SVD/eigh, support fitting/RANSAC and admission thresholds. A cluster's complete spread must fit `8 * eps64 * max(sample_count, dimension) * max(abs(spectrum))`; adjacent gaps are not chained. SVD spectra use singular-value units, covariance spectra eigenvalue units. Full-space ties return coordinate identity. For a tied pair in 3D, `I - outer(untied_row, untied_row)` defines the projector; projected-anchor squared norms within `64 * eps64` tie in X/Y/Z order. Fixed-dimension orthogonalization preserves positive anchor alignment. Singleton rows retain their bytes and caller sign conventions; visible geometry retains its right-handed cross product. Grasp ranking retains the first candidate within `32 * eps64 * max(abs(widths))`, after the exact maximum-opening filter. This defines numerical boundaries, not a universal LAPACK bound or cross-platform bit-identity promise.

V2 oracle/procedural/matrix references explicitly record the policy and all eight geometry sources, including the helper. Read-only v1 loading retains historical source claims. New observed-v2 cases are exactly the old membership/ROI/morphology cases with current provenance. Every v1 fixture digest is unchanged; all twelve inputs retain identical scalar metadata and lossless array bytes. Coordinate-only preparation and the GEOQv1 interface are unchanged.

The exhaustive report audit records **64 changed numeric leaves**: 56 tied footprint/grasp axis/opening/contact/rank values and eight tied validation-extent values. Every changed field has an actual caller spectrum, tied mask and old/new axis witness. No unexplained changes or roundoff-ranking-only changes occur in this matrix. Every other field, discrete gate, flag, reason, count, order and ordered membership stays equal. Full reports retain absolute 1e-9; ordered evidence/rays/trace/preparation retain 1e-12. Independent analytic tests establish axes/projectors and boundary behavior rather than using regenerated references as the sole proof.

Focused verification: author geometry/tracking tests 142 passed/2 skipped; independent analytic tests 36 passed; root analytic/resource tests 46 passed at the interim freeze; final resource guards 12 passed after adding AugAssign/NamedExpr mutation coverage; oracle/matrix/observed tests 162 passed. These sets overlap and are not a summed total. The new write-form mutants first exposed two failures/ten passes in the resource test itself; the corrected guard passes all twelve without changing production arithmetic or benchmarks. The first broad CPU attempt (4606 passed, six failed) is preserved. A separately approved baseline-epoch migration and four test/tool updates have 57 targeted passes; the final broad run passed **4616 tests, 133 skipped and 48 subtests in 262.17 s**. The before/after 934-file manifests have identical hashes; the final receipt is `0301ea2145641f01ecb56c32866313daae1075ea4ee7010e07ec7e75fa59f84f`. The two skips are missing-rclpy tests, not native ROS acceptance. This CPU result does not close native ROS, complete resident report integration, deployment, physical cycles or release gates.

A separate deployment-prerequisite replay also passes all twelve cases on fwmini4 system Python 3.12.3 with NumPy 1.26.4, OpenCV 4.6.0 and SciPy 1.11.4, using exactly the frozen private source/input tree and unchanged tolerances. It makes no timing claim. The known owner PID 181501/start tick 17719784 is unchanged, but its executable is `/usr/bin/bash`; the actual owner Python is unavailable from the bounded executable-only probe. Neither system replay nor the private wheel environment proves the installed station runtime or deployment admission. The source pins in machine data identify the actual Task2 replay/cost snapshot, preceding the separately reviewed baseline-epoch migration.

The helper resource review checks bounded dimension-at-most-three work, no added solvers/per-point loops/frame-sized storage/readback/synchronization. Python fixed-size validation/output allocations remain included in its measured cost. Seven samples of 2000 calls after 256 warmups use preallocated float64 inputs. The controlled repeat sets OpenBLAS/OMP/MKL threads=1 and records one actual process thread; wall/processCPU p50 below are amortized sample means, not individual-call latency.

| Helper case | Wall µs/call | Process CPU µs/call |
|---|---:|---:|
| distinct2 | 4.691 | 4.681 |
| tied2 | 5.810 | 5.796 |
| distinct3 | 5.077 | 5.067 |
| tied_top_pair3 | 19.913 | 19.856 |
| tied_bottom_pair3 | 20.003 | 19.955 |
| tied_all3 | 5.766 | 5.750 |
| rank_deficient3 | 20.969 | 20.909 |

The original uncontrolled run is retained, including distinct2 processCPU p95 284.901 µs versus wall p95 9.270 µs. That outlier cannot establish isolated helper CPU cost; the same-source controlled repeat is not an algorithm speedup claim. Neither run reserves CPU affinity or power state.

Complete-report costs below measure unchanged `analyze_rgbd`, without tracing wrappers, three warmups and twenty calls per frozen input. All comparisons/serialization are outside timing; every measured output and untimed before/after report passes v2 parity, with bit-exact untimed repeats. Source/input pins remain stable. Local Python is 3.13.12; fwmini4 private Python is 3.12.3; both use NumPy 2.5.4/OpenCV 4.14.0, OpenBLAS/OMP/OpenCV threads=1. No affinity, power or host workload reservation was imposed.

| Case | Frame | Local wall p50/p95 ms | Local CPU p50/p95 ms | fwmini4 wall p50/p95 ms | fwmini4 CPU p50/p95 ms |
|---|---|---:|---:|---:|---:|
| base | 128×96 | 3.935/4.376 | 3.922/4.352 | 13.541/13.592 | 13.543/13.594 |
| invalid-hole | 128×96 | 2.966/2.985 | 2.953/2.971 | 10.870/10.896 | 10.868/10.897 |
| touching-depth-children | 128×96 | 3.203/3.220 | 3.193/3.210 | 11.526/11.600 | 11.529/11.602 |
| nonplanar-two-face | 128×96 | 3.883/3.918 | 3.870/3.902 | 13.445/13.782 | 13.443/13.785 |
| sensor-edge | 128×96 | 3.431/3.458 | 3.418/3.444 | 11.950/13.388 | 11.953/13.392 |
| sparse-concave | 128×96 | 3.100/3.138 | 3.086/3.126 | 11.218/11.691 | 11.220/11.695 |
| partial-observation-roi | 128×96 | 3.957/4.021 | 3.946/3.993 | 13.527/13.572 | 13.527/13.575 |
| partial-pickup-roi | 128×96 | 3.924/3.977 | 3.906/3.959 | 13.545/14.173 | 13.545/13.971 |
| distortion-4 | 128×96 | 3.942/3.964 | 3.927/3.946 | 13.454/13.536 | 13.456/13.532 |
| distortion-5 | 128×96 | 3.920/3.999 | 3.906/3.979 | 13.476/13.901 | 13.482/13.898 |
| distortion-8 | 128×96 | 3.952/4.090 | 3.939/4.078 | 13.468/13.532 | 13.471/13.535 |
| large-gray-support | 640×480 | 7.740/7.861 | 7.715/7.839 | 20.855/20.925 | 20.855/20.920 |

Eleven cases are only 128×96; the 640×480 gray case has no colored objects. These are synthetic CPU case costs, not camera FPS, control budget or station headroom evidence. The earlier v1 budget is historical and supports no matched speedup ratio; no old estimator variant was rerun. The original cost-driver metadata-key failures are retained and corrected outside the timed numerical body.

[Machine readout](canonical-principal-axes-20261027.json) retains actual field differences, tie witnesses, all timing samples, source/fixture pins, runtime settings, the bounded cost driver and private receipt hashes. Private replay/audit/cost files are under `/tmp/canonical-principal-axes-v2-20261027/`; helper samples are under `/tmp/canonical-principal-axes-resource-20261027/`. No GPU, ROS, camera or serial hardware was used; the existing owner service was only observed through PID/start-tick/executable metadata and was not changed.

The durable private archive is `/home/bjoern/.local/share/exciting-science/evidence/2026-10-27/canonical-principal-axes-20261027.tar.gz`: **14,115,366 bytes**, SHA256 `bcfe13218ef923ec51016821e200d8fba0916ffb86561faf465c904b91011632`. Its 1910 files were round-trip hash-verified. The adjacent `.receipt.json` has SHA256 `24c55b13fe55d92533f4430fd3b3017767f344a6961fb0aa9d48ffc443627151`. It retains the six proof/replay directories, system-runtime result, original failures/outliers and canonical logs/XML/manifests/receipts; numerical input/output bodies remain outside Git.
