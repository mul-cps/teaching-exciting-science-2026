# Offline RGB-D station digital twin

The viewer combines saved RealSense RGB-D front surfaces with the official
SO101 native MuJoCo model. Its original STL meshes, six actuators, inertials,
contacts and joint defaults are preserved. EV3 mechanical parts behind the
visible surfaces remain estimated primitives, hidden by default. The second,
stationary white arm remains part of the measured scene; fitting targets the
rear follower. Nothing imports a camera, serial or device SDK; sliders change
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

Open http://127.0.0.1:8096. The default surface mesh is built from measured
RGB-D triangles and photograph texture; holes and unobserved backs remain
missing. Layer toggles expose the cropped cloud, room context and small hidden
camera frustum. Dark labels identify estimates. Photograph and MuJoCo render
panels are distinct. `--render-only` writes PNGs without starting a server.
Headless EGL is selected before importing MuJoCo; `MUJOCO_GL=osmesa` is another
supported backend where available. The launcher installs nothing.

The snapshot directory requires `capture.json`, `color.png`,
`aligned-depth.npy` and `arm-readonly.json`. A complete `home-recheck` set is
preferred when present; the last paired `initial-align-retry/frames` image and
depth takes precedence. `--home` selects an explicit immutable encoder capture;
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

Official assets are pinned to
[TheRobotStudio SO101 native MuJoCo source](https://github.com/TheRobotStudio/SO-ARM100/blob/a758567c3978dfeefe282ede0500085a48fe8f78/Simulation/SO101/so101_new_calib.xml),
commit `a758567c3978dfeefe282ede0500085a48fe8f78`, Apache-2.0.
Exact asset hashes and license are bundled. White plastic recoloring and scene
mount placement are local display changes. Outputs and photographs stay outside
the repository. Physical configuration/calibration files are never modified.

Dependencies are an optional extra pinned separately from workshop hardware
packages. Rendering has been exercised with Python 3.13, MuJoCo 3.14.0, Viser
1.1.0 and EGL. The viewer performs forward kinematics, not a hardware controller.
