# Offline RGB-D station digital twin

The viewer combines saved RealSense RGB-D front surfaces with the official
SO101 native MuJoCo model. Its original STL meshes, six actuators, inertials,
contacts and joint defaults are preserved. EV3 mechanical parts behind the
visible surfaces remain estimated primitives, hidden by default. The stationary front white clamp is the follower’s native base and is
retained as a registration anchor. A separately connected leader is not assumed
to be visible merely because it appears in USB evidence. Nothing imports a camera, serial or device SDK; sliders change
only the virtual model.

```bash
python3.13 -m venv .venv-twin
.venv-twin/bin/python -m pip install '.[digital-twin]'
STATION_TWIN_PYTHON=.venv-twin/bin/python tools/launch_digital_twin.sh \
  --snapshot /path/to/realsense-station \
  --calibration /path/to/hs_rig01_follower.json \
  --home /path/to/measured-home.json \
  --output /path/to/twin-output --port 8096
```

The viewer can compose saved calibration and controller evidence without
opening the camera or arm:

```bash
STATION_TWIN_PYTHON=.venv-twin/bin/python tools/launch_digital_twin.sh \
  --snapshot /path/to/realsense-station \
  --calibration /path/to/hs_rig01_follower.json \
  --camera-arm-calibration /path/to/branch-consistency-report.json \
  --camera-target-calibration /path/to/charuco-table-7x5/camera-from-board.json \
  --phase-file /tmp/cps-pick-place-phase.json \
  --rgbd-keyframes /path/to/realsense-station/tag-calibration-held/keyframes \
  --output /path/to/twin-output --port 8096
```

For the resident arm state and rolling camera capture, use the saved live
evidence directory explicitly:

```bash
STATION_TWIN_PYTHON=.venv-twin/bin/python tools/launch_digital_twin.sh \
  --snapshot /home/bjoern/.local/share/exciting-science/station/evidence/2026-10-14/realsense-station \
  --calibration /home/bjoern/git/soarm101-highschool-demos/.calibration/hs_rig01_follower.json \
  --phase-file /home/bjoern/.local/share/exciting-science/station/evidence/2026-10-14/arm-resident-state.json \
  --rgbd-live-folder /home/bjoern/.local/share/exciting-science/station/evidence/2026-10-14/realsense-station/resident-pick-live-v2 \
  --output /home/bjoern/.local/state/exciting-science/digital-twin --port 8096
```

`--rgbd-live-folder` reads `latest.png`, `latest-depth.npy` and `latest.json`
(with a bounded `timestamps.jsonl` fallback). It requires a qualified camera
and arm sample pair within 160 ms, and all source files must be no older than
0.5 seconds at startup. The viewer then polls the same saved files at 5 Hz and
labels stale resident state or changed raw values as unpaired. A changed,
synchronized RGB-D pair regenerates the measured surface with the complete
scene registration; the compiled MuJoCo mesh and photograph texture are updated
in place, with the previous visible surface retained unless the replacement
builds successfully. The refreshed frame bounds the rendered camera around both
the tabletop and robot. `live-status.json` reports the measured surface as
`snapshot`, `fresh-live`, or `stale-live` separately from the point-cloud cache,
so an old cache is never reported as the current surface. `--phase-file`
accepts both legacy phase records (`pose_raw`) and resident-state records
(`raw`, `timestamp_utc`, `monotonic`). The displayed raw encoder values use the
supplied calibration file. This is a read-only file bridge and does not access
the camera, arm or serial port.

The board-up frame is derived with `camera_from_world = camera_from_target @
diag(1,-1,-1,1)`, then `world_from_base = world_from_camera @
inverse(base_from_camera)`. The complete transform re-expresses the existing
RGB-D table scene before placing the native URDF base. Both input transforms are
kept **provisional and unqualified**; held-out branch closure, board pose and
camera-to-arm placement do not establish physical calibration. The viewer polls
the saved phase or resident-state file and selected RGB/depth source at 5 Hz.
`--rgbd-keyframes` selects and watches a manifest-paired folder; when omitted,
the viewer uses `tag-calibration-held/keyframes` under `--snapshot` if present.
Startup selects once from the full manifest. The 5 Hz watcher reads only its
last 8 KiB (up to 128 rows), tolerates an incomplete trailing append, and
accepts only a synchronized pair no older than 0.5 seconds; it never scans the
historical frame directory when the rolling manifest is invalid or stale. The
manifest's capture monotonic time and pose-event delta determine whether the
image/pose were synchronized and how old the recorded frame is. A phase sample
older than 0.5 seconds is shown as stale while retaining its last raw values.
RGB-D point-cloud caches are reused only when their source hashes and capture
timestamp match; otherwise the cloud is rebuilt from the saved depth pair.
`live-status.json` records pose age, camera age, synchronization delta and
calibration status. It does not read controller, camera or serial devices.

The optional wrist-tag comparison uses only the saved paired frame and its
qualified raw pose. Enable it with the held candidate fit:

```bash
STATION_TWIN_PYTHON=.venv-twin/bin/python tools/launch_digital_twin.sh \
  --snapshot /home/bjoern/.local/share/exciting-science/station/evidence/2026-10-14/realsense-station \
  --calibration /home/bjoern/git/soarm101-highschool-demos/.calibration/hs_rig01_follower.json \
  --camera-arm-calibration /home/bjoern/.cache/cps-tag-calibration-held/branch-consistency-report.json \
  --camera-target-calibration /home/bjoern/.local/share/exciting-science/station/evidence/2026-10-15/gripper-direction-retry/analysis-2026-10-15/camera-from-board-candidate.json \
  --phase-file /home/bjoern/.local/share/exciting-science/station/evidence/2026-10-15/arm-resident-state.json \
  --rgbd-live-folder /home/bjoern/.local/share/exciting-science/station/evidence/2026-10-15/arm-tag-registration-retry \
  --fiducial-calibration /home/bjoern/.cache/cps-tag-calibration-held/depth-pnp-hybrid-analysis.json \
  --output /home/bjoern/.local/state/exciting-science/digital-twin --port 8096
```

The detector uses AprilTag 25h9 ID 0 (19.4 mm), bootstraps only inside the
saved wrist ROI x=330–440, y=200–290, and follows the nearest candidate within
40 px. Ambiguous duplicates are rejected. Cyan expected and yellow observed
axes/outlines report translation in mm, projected center error in px and
rotation in degrees. A paired depth plane from known white tag modules can
provisionally anchor translation while retaining the nominal 19.4 mm tag size
and the chosen PnP orientation. `live-status.json` reports nominal PnP residual,
depth-versus-PnP translation delta, plane RMS and valid-module count; if depth
quality fails, the UI labels the comparison PnP-only and unqualified. The side
length inferred from depth is reconciliation evidence, not proof that the
printed tag size differs from its nominal value. Every comparison is labeled
**SOURCE UNQUALIFIED**; the visual overlay does not fit calibration, alter model
pose or authorize motion.
The tag-pose prediction uses the held `base_from_camera` and `gripper_from_tag`
fit in the supplied fiducial JSON. The current board-pose candidate is used
only for the scene's camera/world placement; neither transform is upgraded by
the live residual.

Open http://127.0.0.1:8096. The default surface mesh is built from measured
RGB-D triangles and photograph texture; holes and unobserved backs remain
missing. Layer toggles expose the cropped cloud, room context and small hidden
camera frustum. Dark labels identify estimates. Photograph and MuJoCo render
panels are distinct. `--render-only` writes PNGs without starting a server.
When valid tabletop RGB has mean luminance below 15/255, the surface and sample
panel switch to an explicitly labeled `DEPTH_VIEW` using a vectorized 0.3–0.9 m
depth colormap; invalid depth pixels remain black. This visualization texture
does not alter captured RGB/depth files or point-cloud RGB used for perception.
When the tabletop is lit again, the photograph texture returns automatically.
Headless EGL is selected before importing MuJoCo; `MUJOCO_GL=osmesa` is another
supported backend where available. The launcher installs nothing.

The base snapshot directory requires `capture.json`, `color.png`,
`aligned-depth.npy` and `arm-readonly.json`. When a synchronized keyframe folder
is selected, its `manifest.jsonl` pairs RGB, depth and encoder/phase timestamps;
unsynchronized records are ignored. `--home` selects an explicit immutable encoder capture;
`--original-snapshot` retains the original image/depth. Latest captures reuse
`capture.json` intrinsics, assuming the same camera profile. An image and home
file are not automatically synchronous. After a layout change, supply paired
`--image` and `--depth`, the corresponding `--home`, and `--no-pose-series`
to avoid reusing old absolute registration. `--base-yaw` retains a user visual
orientation prior and may anchor translation to an approximately segmented
rear motor face; it remains unqualified. `--model-convention old` compares the
separate official old-calibration native model without changing hardware.

Camera intrinsics and metric depth scale are measured. Nonzero distortion
requires SDK deprojection and is rejected by this pinhole implementation. A
fixed tabletop ROI is fitted using seeded RANSAC/SVD. Table z faces the camera,
x follows projected optical right. The fitted table frame is not surveyed.

Registration runs once before the viewer starts. A white rear-follower ROI and
area-weighted official CAD samples feed a robust surface fit. If saved
`scene-calibration-recorded-result.json` and corresponding timestamped RGB-D
frames exist, the multi-pose fit uses actual dwell-end joint readings, host
monotonic nearest-frame matching and a distinct wrist pose held out from fitting.
It tries mount yaw and planar joint-zero candidates. Pan zero remains fixed
because mount yaw and pan zero have a gauge ambiguity. **These candidates remain
unqualified**: segmentation, occlusion, mount priors and small dithers can yield
incorrect correspondences even with small surface residuals.

A separate gripper-feature tracker exports recorded camera displacements and
local finite-difference Jacobian columns. It tests native joint-sign candidates
with a bounded offset around the official black gripper-motor mesh center.
The tracked feature is not a surveyed grasp TCP. Its local pseudoinverse is
numerical evidence near the recorded home, with conditioning and holdout error
reported; it does not authorize hardware motion or establish global IK.

`scene-evidence.json` records hashes, camera/table transforms, raw readings,
nominal normalization and fitted visual corrections separately. `kinematics`
uses the exact original calibrated-midpoint normalization without clipping;
`visual_fit_kinematics` records corrected model qpos separately. Both include
arm-base-local link frames, joint axes and the MuJoCo gripper-site Jacobian.
`tracked-tcp-registration.json` includes feature tracks, sign variants, local
Jacobians and uncertainty. `measured-camera-render.png` and
`robot-silhouette-overlay.png` provide silhouette/depth comparison diagnostics.
Photograph-textured static surfaces reconstruct the input; this is distinct
from predictive robot holdout validation. No grasp, contact dynamics or
sim-to-real qualification is claimed.

For a reusable stationary-base candidate, `icp.oriented_cad_surface` accepts
`body_name='arm_base'`, excluding all moving links. `icp.fit_fixed_base` accepts
those CAD samples/normals, temporally segmented base points in world coordinates,
the measured `world_from_camera`, a translation initializer and the user mount
yaw prior. It exports `camera_from_arm_base`, alternating-point holdout residuals
and sensitivity to three starting angles within the same hard yaw bound. These
are diagnostics: seed sensitivity is not a confidence interval, and same-capture
withheld pixels are not independent pose validation. Moving-link depth residuals
must be evaluated on distinct synchronized poses with the transform held fixed.

`kinematics.export_pinch_geometry` selects fixed and moving jaw vertex patches
once from the compiled native CAD meshes at canonical gripper joint `q=0`.
Every later frame transforms those same physical mesh vertices through the
current geom and body transforms; the patch is not reselected as the jaw rotates.
The export includes world, jaw-body and native-site coordinates, mesh vertex
indices, CAD triangle normals, the opening-dependent midpoint and gap, and a
geometric contact-eligibility result from face orientation and the direction
between patches. It also reports the current native joint angle and derives the
current normalized opening from that model joint's limits. A pivoting jaw can
make the gap grow nonlinearly and turn its face away from the fixed jaw across
the sweep, so gap alone does not imply contact. This is
`OBSERVED_CAD_ESTIMATE`; eligibility is geometric only and physical calibration
remains `UNQUALIFIED`. The official site is near the fixed
face, not the midpoint. The calibrated-percent mapping to native gripper angular
limits remains a visual estimate; raw encoder angle and actual jaw opening
require independent comparison. Neither this candidate transform nor the CAD
pinch estimate enables hardware IK.

The next independent camera/base check should observe a fixed mounting-board
target or several known mount edges with different surface normals. Use held-out
features and synchronized distinct arm poses to check the unchanged transform;
do not qualify it from a height-filtered training patch alone. Separately compare
actual jaw spans at several encoder openings against the CAD fixed/moving faces
before treating their modeled midpoint as a calibrated grasp target.

Official assets are pinned to
[TheRobotStudio SO101 native MuJoCo source](https://github.com/TheRobotStudio/SO-ARM100/blob/a758567c3978dfeefe282ede0500085a48fe8f78/Simulation/SO101/so101_new_calib.xml),
commit `a758567c3978dfeefe282ede0500085a48fe8f78`, Apache-2.0.
Exact asset hashes and license are bundled. White plastic recoloring and scene
mount placement are local display changes. Outputs and photographs stay outside
the repository. Physical configuration/calibration files are never modified.

Dependencies are an optional extra pinned separately from workshop hardware
packages. Rendering has been exercised with Python 3.13, MuJoCo 3.14.0, Viser
1.1.0 and EGL. The viewer performs forward kinematics, not a hardware controller.
