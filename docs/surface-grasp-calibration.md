# RGB-D grasp planning and camera-to-arm calibration

The station now has an offline path from RGB-D surfaces to SO101 inverse
kinematics. It does not issue motor commands. Numerical IK convergence,
camera registration, contact geometry, and a successful pickup are separate
results; the current physical registration and finger-opening model remain
provisional.

## Installation

First install the station environment using the repository station installer.
Then add the optional geometry and IK packages to that same environment:

```sh
uv pip install --python ~/.local/share/exciting-science/station/venv/bin/python '.[camera,digital-twin,ik]'
```

The existing CPU PyTorch installation also runs GG-CNN. IK pins Placo 0.9.15,
urdfdom 4.0.1 and tinyxml2 10.0.0; the tinyxml2 pin avoids a missing shared
library. LeRobot's installed solver is reused with explicit convergence checks.

## Board calibration

The board placed in this station is **7 × 5 squares, 25 mm squares, 18 mm
markers, DICT_6X6_250**. Its camera pose was detected from the D435i image.
Keep these dimensions in the board JSON; they differ from the example
printable targets in `artifacts/calibration-targets`.

```sh
python -m tools.estimate_board_pose --image FRAME.png --metadata BOARD.json \
  --intrinsics EPISODE/metadata.json --output camera-from-board.json
```

The output maps board coordinates into camera coordinates, in metres. A
board on the table defines a table reference, not the robot base. To obtain
`base_from_camera`, either measure a rigid `base_from_target` transform and
use `base_from_camera_from_board`, or rigidly attach a target to the wrist
and use `eye_to_hand` with synchronized actual joint poses and target poses.
The latter explicitly inverts base-to-gripper FK for OpenCV's eye-to-hand
configuration. Use varied rotations around at least two axes and independent
held-out poses near the pickup area. Synthetic solver tests do not qualify
physical calibration.

## Surfaces and grasp proposals

`grasp_geometry` fuses valid metric depth samples, fits robust local support
planes, intersects calibrated rays with specified height planes, and extracts
two footprint contact directions. Mat and wood require separate support
planes. Mixed front/top silhouettes and assumed brick heights are explicitly
reported; contact points above a plane do not establish finger clearance.

`tools.propose_rgbd_grasp` combines an episode, support ROI, color mask,
height prior, explicit camera/base transform, calibrated joint seed and pinch
offset into numerical IK proposals. Run its `--help` for input paths. Reports
include input hashes, residuals and assumptions. They remain non-executable.

For optional learned pixel proposals:

```sh
python -m workshop.digital_twin.grasp_ggcnn --depth EPISODE/latest-depth.npy \
  --download-weights --roi 310 200 510 400 --object-mask OBJECT_MASK.npy \
  --threads 4 --output proposals.json
```

The SHA256-pinned Cornell checkpoint is downloaded on request. CPU-only,
tensor-only loading, valid-depth/object filtering and periodic angle smoothing
are used. Scores are not calibrated pickup probabilities. Pixel contact
endpoints must be lifted through the correct local height plane and transformed
to the robot base before IK. The example ROI must be adjusted to the object.

## Frame conventions and performance

`urdf:gripper_frame_link` and MuJoCo `gripper_frame_site` have almost the
same position but differ by **90 degrees about Y**. `frame_conventions`
provides the explicit native-to-URDF conversion. Never feed a native-site
pinch offset directly into URDF IK. The approximate CAD pinch provider is
restricted to 0–30% opening and remains uncalibrated physically.

Use a fixed commanded opening for each planned segment; changing the model
opening reconstructs the solver and takes seconds. Warm IK takes fractions of
a millisecond. Local cropped depth fusion and plane fitting take a few
milliseconds each; the full episode pipeline can take over 100 ms. Registration,
perception and model construction belong outside the servo loop. No solver
success overrides orientation, surface-clearance or measured calibration checks.

Sources: [LeRobot IK](https://github.com/huggingface/lerobot/blob/main/src/lerobot/model/kinematics.py),
[official SO101 assets](https://github.com/TheRobotStudio/SO-ARM100/tree/main/Simulation/SO101),
[GG-CNN](https://github.com/dougsm/ggcnn),
[OpenCV ChArUco](https://docs.opencv.org/4.x/df/d4a/tutorial_charuco_detection.html).
