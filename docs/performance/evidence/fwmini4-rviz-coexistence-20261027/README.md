# RViz coexistence evidence, 27 October 2026

The unchanged `fwmini4-rviz-coexistence-raw-20261027.tar.gz` is 344,176 bytes,
SHA-256 `1f33b4e14ab0db33da14031ca242e624e3b95e8fe00d0a55793034122f9a26b7`.
`raw-archive-inventory.json` pins all 87 regular members; publication checked each
against the supplied extracted bytes. It retains six camera collector/runner sets,
shared per-profile RViz/sampler logs, all three failed compositor setups, scripts,
Weston configuration, native fake-GL host benchmark and the post-run empty library
maps attempt. The latter is not loaded-SDK identity proof.

The raw archive has no actual image, calibration, mesh asset or ELF payload.
Published supplements likewise contain no such payload. Public generated model
URDF/YAML is retained; referenced meshes are omitted and hash-bound below.
Screenshots and any later UI capture remain private and outside the timed matrix.
The Qt/rootless-X11 method was black/invalid and remains a failed capture.
A separate post-matrix VGA-only Weston13 screenshooter capture was inspected by
root: actual RGB, robot, synthetic world surface and bricks visible, Global Status
OK and 31fps status bar. `root-UI-manual-inspection.json` retains the reported
inspection and private screenshot hash, not the image. The unchanged supplemental
`fwmini4-rviz-ui-metadata-20261027.tar.gz` (SHA-256
`db9bdee33df0eb5de5c9b421fe7cd5c331509f37d38b883455555f6bd3371167`) retains 11
metadata/log/script members, pinned by `ui-metadata-archive-inventory.json`.
Its Qt JSON correctly says child GL content unverified; the successful root
inspection used the later compositor capture. Debug/screenshooter readback was
outside all timed windows. This does not independently inspect all three profiles.

`analysis/` retains the exact offline analyzer, six-run result and its synthetic
17 host tests. `review/analyzer/` retains the independent bounded metadata/input-guard
review and unchanged-six-run proof. `probe/` retains the independently reviewed GLX C implementation,
compiler scripts, fakeGL/heap audit harness, host timing/UBSan/static checks and
five rejected variants; all executable/shared-library ELF files are omitted.
`review/` retains independent source pins, sampler cost/static tests and source
review. Native host cost proof within the raw archive used 500k calls per sample:
median 25.635ns/call with probe versus 2.386ns baseline, incremental 23.249ns on the
fake real-swap body. It excludes first/per-interval metadata/logging, actual driver
work and display performance. Zero measured hot allocations/queries is a host
instrumentation statement, not a claim that drivers or ROS never allocate.

`scene/` retains the generator/configuration/model, visual load source and its
CPU-only test/cost records. It publishes actual resident RGB through the RViz
Image subscription; robot poses, world points and brick markers are synthetic.
`native-source/` binds source checkpoint
`5344d665692ab2dddb0de32ef58c8adf6aedadb8` through the prior 103-file manifest and
publication-time commit verification. The earlier high-resolution checkpoint
contains native build, SDK hash and alignment/CPU-suite evidence; no new full CPU
suite is claimed for this documentation-only publication.

To reproduce offline math without native runtime:

```sh
mkdir /tmp/rviz-evidence-reanalysis
# First inspect archive names and preserve source hashes.
tar -xzf fwmini4-rviz-coexistence-raw-20261027.tar.gz \
  -C /tmp/rviz-evidence-reanalysis
python3 analysis/rviz-coexistence-analysis-20261027.py \
  --base /tmp/rviz-evidence-reanalysis --output /tmp/rviz-reanalysis.json
```

Absolute `inputs` paths change; numeric/window fields must match `analysis/results.json`.
The artifact selfcheck performs safe member-name validation and temporary extraction
itself, verifies all inventory hashes and reanalyzes every run:

```sh
PYTHONDONTWRITEBYTECODE=1 python3 selfcheck.py
```

No script launches a native process automatically. Retained runtime launch scripts
are historical/reconstruction evidence; reproducing a camera/display run requires
separate authorization and the original native environment. The production marker,
control, geometry and owner-service paths were excluded. Return code 0 from the
analyzer means structurally valid analysis, not station acceptance. Qualification
and execution eligibility remain false.

`selfcheck-result.json` is the retained artifact-only check. `inventory.json` pins
all evidence files except itself and the generated selfcheck result. The main
Markdown/JSON note provides the human summary; raw per-run JSON is the authority.
GLX rates are per drawable, only fully contained steady intervals; they are never
summed. Observer process CPU uses separately reported nearest-sample windows,
explicit endpoint offsets and stable PID/starttime. Combined CPU is approximate.
The observer itself is excluded from targeted CPU sums, but included in host busy.
DRM engine counters are per-process/client, not full GPU occupancy or power.

## Reconstructing the omitted model assets

`scene/description-source-manifest.json` pins 70 actual description files and
records their total bytes and exact SHA-256. All were compared with the local
upstream cache used by the bundle generator. The origin was
`https://github.com/legalaspro/so101-ros-physical-ai.git` at
`58318c905a2c61289fa907de85cb8473322fbe68`, with two local xacro edits retained in
`scene/upstream-local.patch.gz`. The upstream commit alone is insufficient.

Patch/log gzip files preserve original bytes, including historical source/test
whitespace; decompress before use. The manifest records compressed and uncompressed
patch hashes. Independent review proof filenames refer to the original uncompressed
logs; their `.gz` forms are retained here.

Start with that checkout and apply the decompressed patch (or reuse the original
already-patched cache), then verify every description file against the manifest.
Copy `so101_description/` to
`/tmp/rviz-coexistence-scene-20261027/share/so101_description/`, preserving the
absolute path embedded by the generator. Retained `robot.urdf`, `rsp.yaml` and
`rviz-coexistence.rviz` bind the exact evaluated model/layout used in the run.
`build_bundle.py --station-root CHECKPOINT --description-source PATCHED_DESCRIPTION`
explains reconstruction using the source checkpoint and original Qt/xacro tools;
regeneration with different Qt versions may change dock-state bytes. Actual
retained generated hashes are authoritative. Assets remain omitted from this
publication rather than copying the 19MB model package. No private URDF or mesh
asset is published.
