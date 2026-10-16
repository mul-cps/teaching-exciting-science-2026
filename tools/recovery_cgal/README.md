# Offline exact mesh prototype

This standalone tool proves mathematical properties of frozen triangle payloads. It is not imported by the resident, collision model, scene provider or calibration pipeline. Every result keeps `production_recovery_qualified=false`. No hardware access occurs.

## Proof and limits

1. Preserve every source triangle position. Send binary64 coordinates as hexadecimal tokens; construct EPECK scalars directly from those doubles. Weld exactly equal coordinate triples only. No repair, normalization, deletion, convex fill or simplification.
2. Require nondegenerate triangles and an unchanged face count in a valid closed triangle mesh. Reject failed face insertion/nonmanifold topology or inconsistent winding.
3. Run `PMP::does_self_intersect` **before** APIs whose preconditions exclude intersections.
4. Require exact nonzero signed volume for every connected component, then `PMP::does_bound_a_volume`. This rejects membranes and invalid shell orientation.
5. Query the **whole mesh** with `Side_of_triangle_mesh`: odd/even shell parity preserves cavities. Never union components. CGAL handles ray edge/vertex ties; boundary points fail admission.
6. For a cubic cell of side `pitch`, center strictly inside material and exact `squared_distance > (867/1000 * pitch + boundary_uncertainty)^2` certify the entire uncertain cell. The rational factor strictly exceeds `sqrt(3)/2`. Equality fails. All pass/fail comparisons retain EPECK exact scalars; no conversion to double.

Coordinates are source units after the recorded positive XYZ scale. A mathematically exact input embedding does not prove physical CAD accuracy, camera registration, uncertainty bounds or encoder-to-jaw angle. A zero uncertainty supplied in a mathematical fixture is not a physical uncertainty qualification. Source registration and jaw mapping remain unqualified, the live provider remains absent, and single-view unobserved padding remains unknown. The invalid motor-holder mesh remains unsupported.

The operation deadline defaults to30seconds (maximum120), starting before input loading/preprocessing; late computation never returns proof. The child has4GiB virtual memory by default (allowed1..8192MiB), a CPU cap, no core dumps and nice10. Deadline/pipe overflow kills its entire process group and discards partial results. Input caps:250000faces,100000cells,12,500,084mesh bytes,16MiB cell JSON,64KiB manifest,16MiB executable,80MiB protocol. Only unchanged binarySTL is supported. Streaming stdout/stderr caps are12MiB/64KiB; stderr, crashes, incomplete/full-schema mismatches all return unknown. Local filesystem/native calls cannot be forcibly interrupted in-process; elapsed time is checked after bounded reads/stages and before positive return. This is offline tooling and must never execute in the actuator motion loop.

## Dependencies and build

CGAL6.2.1 is header-only. The tested external overlay contains these conda-forge packages (SHA256):

| Package | SHA256 |
| --- | --- |
| [cgal-cpp6.2.1](https://conda.anaconda.org/conda-forge/linux-64/cgal-cpp-6.2.1-hec4b13c_0.conda) | `232ea63938c3582460c5767e118fc205963bc0d8c331885b2020ef1c5d0c83a5` |
| [mpfr4.2.2](https://conda.anaconda.org/conda-forge/linux-64/mpfr-4.2.2-ha2cb11d_1.conda) | `0be79ef2ee536e0c5d3ddb4a52bbae9f26d08560623299f33db2f4c4a17c525a` |

Download these only from the linked conda-forge URLs, verify the hashes before extraction, then extract both with `conda_package_handling.api.extract(package_path, dest_dir=overlay_path)` using the existing Miniforge Python. This prototype uses an external overlay rather than activating or modifying the live ROS environment. GCC15.3, GMP6.3 and Boost1.90 are read from the existing conda prefix. Python needs NumPy and Trimesh. No Eigen/Qt/demo compilation is performed.

```sh
python tools/recovery_cgal/build.py \
  --dependency-prefix "$HOME/.local/share/exciting-science/recovery-cgal/deps" \
  --base-prefix "$HOME/miniforge3/envs/so101-jazzy" \
  --output-dir "$HOME/.local/share/exciting-science/recovery-cgal/bin"
```

The build compiles a sealed immutable snapshot of the bytes whose source hash is recorded. The reviewed C++sourceSHA is `2ba8cc76e23ae144da2ec1a0a6224fadcd8ca24c1400de2a40ffbf7c09adfef4`. The build uses one compiler process, 8 GiB virtual-memory/170CPU-second/180wall-second caps and `-std=c++17 -O1 -DNDEBUG -frounding-math`. `build.json` records exact command, source/binary hashes, compiler and base dependency package metadata, elapsed time and peak child RSS. The observed first build took13.53s wall/1,432,932KiB RSS; reproducible script rebuild13.97s/1,434,140KiB. The trusted snapshot rebuild took18.19s/1,431,844KiB RSS. No swap was used.

## JSON CLI

A reviewed build-manifestSHA must be supplied explicitly, never derived automatically from the candidate manifest. The manifest must bind the exact executable bytes, reviewed sourceSHA and CGAL6.2.1. Integration must keep this pin in trusted configuration and never accept it from a motion caller. The wrapper executes the verified ELF bytes through Linuxsealedmemfd, so replacing the executable pathname after hashing cannot change what runs. Unsupported sealing fails unknown. Cells are a JSON list; each item has exactly `center:[x,y,z]`, positive `pitch`, and nonnegative `boundary_uncertainty`, all in the same scaled source coordinates. Omit `--cells` for mesh topology/intersection proof only.

```sh
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
python tools/recovery_cgal/certify.py \
  --mesh /absolute/frozen/mesh.stl \
  --binary "$HOME/.local/share/exciting-science/recovery-cgal/bin/recovery-cgal" \
  --trusted-build-sha256 REVIEWED_BUILD_MANIFEST_SHA256 \
  --scale 1 1 1 --timeout-s 30 --memory-mb 4096 \
  --cells /absolute/cells.json --output /absolute/result.json
```

Exit0 means the mesh proof completed; inspect each cell's `certified` field. Exit2 means rejected/unknown. `provenance` records source bytes, exact triangle-array and protocol-payload SHA256, scale as hex doubles and executable SHA256. Backend version, component count, status/reason, elapsed-time type, every cell schema/reason/cardinality and certified booleans are checked. The expected manifestSHA is an external review decision; no self-asserted candidate checksum is trusted. Results are offline evidence, never self-authenticating owner permissions or a production cache.

Internal stdin protocol: magic `recovery-cgal-v1`; decimal face/cell counts; three hex scale tokens; nine hex coordinate tokens per original triangle; five hex tokens per cell (`x y z pitch uncertainty`); EOF. Hard counts:250000 faces/100000 cells. Backend emits one JSON object. Wrapper validates cardinality and booleans, and overrides production qualification false.

## Evidence and verification

External evidence directory:
`~/.local/share/exciting-science/station/evidence/2026-10-16/recovery-cgal/`

It contains frozen13STLs, prior source audit/effective URDF identity and per-instance scales, per-mesh proof JSON and GNU time records, dependencies/build provenance and `summary.json`. All current URDF scales are1,1,1. The original unpinned run is retained in `initial-unpinned-run/`; only the v2trusted summary and its pinned build are current evidence. The full13 run is topology/intersection proof only; it does not certify a physical scene or a set of motion cells. Twelve meshes passed, including the moving jaw with its two nested shells; `motor_holder_so101_base_v1.stl` failed unchanged topology. No timeout. The trusted rerun took0.016–0.651s per successful backend proof and0.106–0.966s per full CLI invocation with thread counts1. Moving jaw:0.408s backend,0.536s complete operation,0.62s whole CLI and55,808KiB peak RSS. The full trusted test run passed1096tests with51skipped; dedicated CGAL tests passed36.

```sh
OPENBLAS_NUM_THREADS=1 ROS_DOMAIN_ID=75 \
RECOVERY_CGAL_BINARY="$HOME/.local/share/exciting-science/recovery-cgal/bin/recovery-cgal" \
RECOVERY_CGAL_BUILD_SHA256=REVIEWED_BUILD_MANIFEST_SHA256 \
python -m pytest -q tests/test_recovery_cgal.py tests/test_recovery_mesh_audit.py
```

Without an explicit trusted build pin or installed default executable, geometry fixture tests skip; wrapper failure tests still run. Explicitly supplying a missing `RECOVERY_CGAL_BINARY` fails verification.

Primary API contracts: [EPECK](https://doc.cgal.org/latest/Kernel_23/classCGAL_1_1Exact__predicates__exact__constructions__kernel.html), [self intersections](https://doc.cgal.org/latest/Polygon_mesh_processing/group__PMP__intersection__grp.html), [volume orientation](https://doc.cgal.org/latest/PMP_Mesh_repair/group__PMP__orientation__grp.html), [mesh parity](https://doc.cgal.org/latest/Polygon_mesh_processing/classCGAL_1_1Side__of__triangle__mesh.html), [exact AABB distance](https://doc.cgal.org/latest/AABB_tree/classCGAL_1_1AABB__tree.html).
