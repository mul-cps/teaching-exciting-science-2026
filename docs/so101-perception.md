# SO-ARM101 native RGB-D perception

`so101_station_perception` launches the Jazzy `realsense2_camera` driver for the
Rig01 D435i (`243322071783`) and publishes color, aligned depth, camera info,
and a color-textured point cloud. Its conservative USB2 profile is 640x480 at
15 Hz for both streams. Aligned depth uses the color optical frame. The normal
driver TF tree stays intact and has no parent attached to the arm's planning
base.

Build the workspace in the provided container, then start the camera on the
host only when the station owner has handed off the device:

```bash
ros2/scripts/so101.sh build
ros2/scripts/runtime.sh run -- bash -lc '
  source /opt/ros/jazzy/setup.bash
  source /ws/install/setup.bash
  ros2 launch so101_station_perception perception.launch.py \
    serial_no:=243322071783 \
    rviz:=true
'
```

The standard topics are `/camera/color/image_raw`,
`/camera/color/camera_info`, `/camera/aligned_depth_to_color/image_raw`,
`/camera/aligned_depth_to_color/camera_info`, and
`/camera/depth/color/points`. `align_depth.enable` and
`pointcloud.enable` are set in [`realsense.yaml`](../ros2/config/realsense.yaml).
The camera profile can be inspected before connection with:

```bash
ros2/scripts/runtime.sh run -- bash -lc '
  source /opt/ros/jazzy/setup.bash
  ros2 launch realsense2_camera rs_launch.py --show-args
'
```

The launch defaults to publishing two repeated AprilTag 25h9 ID-0 observations
as short-lived `visualization_msgs/MarkerArray` entries on
`/so101_perception/tag_observations`. Each observation is a depth-backed center
point in the image's optical frame; it is not a full tag pose and never creates
a TF frame. The wrist and upper-arm ROIs have separate temporal trackers. If a
region contains multiple ID-0 candidates, or a detection jumps beyond its
track gate, that region is suppressed. Image/depth stamps must agree within
40 ms, and markers expire after 0.4 s so stopped or stale input cannot leave a
marker displayed.

## Calibration transforms

Calibration paths are explicit launch arguments. For the saved unqualified
candidates used in the current analysis:

```bash
ros2/scripts/runtime.sh run -- bash -lc '
  source /opt/ros/jazzy/setup.bash
  source /ws/install/setup.bash
  ros2 launch so101_station_perception perception.launch.py \
    serial_no:=243322071783 \
    camera_target_calibration:=/home/bjoern/.cache/cps-tag-calibration-held/joint-registration-followup-analysis-2026-10-15/new-camera-from-board-candidate.json \
    rviz:=true \
    base_frame:=base_link \
    camera_arm_calibration:=/home/bjoern/.local/share/exciting-science/station/evidence/2026-10-15/ros-control-runtime/camera-arm-wider-unqualified.json
'
```

The target file must declare `camera-target-pose-v1`, direction
`camera_from_target`, and a rigid 4x4 metre transform. The arm file must declare
`camera-arm-calibration-v1`, direction `base_from_camera`, and a rigid SE(3)
matrix. Matrices are consumed as written; no implicit inverse is guessed. The
current fits use the OpenCV/RealSense optical convention (x right, y down, z
forward). Accordingly, the calibration source frame is
`camera_color_optical_frame`; qualified attachment composes through the
official camera-link transform supplied by RealSense.

Unqualified hand-eye candidates publish `base_link ->
so101_diag_base` as an identity **visual-only anchor**, then the candidate
`so101_diag_base -> so101_diag_camera_optical` transform. This does not qualify
or alter the planning transform. The optional diagnostic cloud node looks up
the official RealSense transform from the source point-cloud frame into
`camera_color_optical_frame`, applies that vectorized transform, and republishes
the same points on `/so101_perception/diagnostic/points` using the diagnostic
camera alias. It is bounded to 5 Hz and 640x480 points. The default native
point-cloud topic is untouched. Tag centers use the same diagnostic alias when
an unqualified arm calibration is loaded. The ChArUco candidate is a child
frame under that alias. RViz labels this cloud as an unqualified candidate
overlay, making residuals and possible misregistration visible without
changing the robot's planning TF tree. The calibration publisher validates
matrix shape, finiteness, orthogonality, determinant, schema, and direction
before broadcasting.

Qualified attachment is deliberately opt-in:

```bash
... use_qualified_frames:=true base_frame:=base_link
```

This mode fails before starting the camera unless the arm JSON contains the
exact boolean `"qualified": true`. It keeps the RealSense TF tree enabled and
waits for the official `camera_link -> camera_color_optical_frame` transform.
It composes the calibrated base-to-optical matrix with that measured vendor
edge, then publishes `base_link -> camera_link`; this preserves the
camera's internal depth/color transforms and avoids assigning a second parent
to the optical frame. The current camera-arm candidate is explicitly
unqualified, so this mode refuses it. ChArUco target frames remain under the
native optical frame when qualified mode is active.

`station_live.rviz` shows the follower `RobotModel`, native camera point cloud,
diagnostic point cloud, RGB image, depth-backed tag centers, and TF/calibration
frames. The camera image is visible independently. In an unqualified preview,
RViz can overlay the diagnostic candidate cloud with the live arm model; the
native camera TF branch remains separately visible. A qualified calibration
uses the native cloud against the model.

## Bounded rolling evidence (optional)

The camera launch can also subscribe to the resident state file and maintain a
small twin-compatible rolling folder. Choose a new empty destination; the sink
does not overwrite nonempty evidence or open the camera itself:

```bash
ros2/scripts/runtime.sh run -- bash -lc '
  source /opt/ros/jazzy/setup.bash
  source /ws/install/setup.bash
  ros2 launch so101_station_perception perception.launch.py \
    serial_no:=243322071783 \
    evidence_folder:=/home/bjoern/.local/share/exciting-science/station/evidence/2026-10-15/ros2-native-rgbd \
    resident_state_file:=/home/bjoern/.local/share/exciting-science/station/evidence/2026-10-15/arm-resident-state.json
'
```

The sink accepts only synchronized color/aligned-depth samples within 40 ms,
writes at most 5 samples/s, and caps resident-state reads and `latest.json` at
8 KiB. Atomic replacements maintain only `latest.png`,
`latest-depth.npy`, `latest.json`, `capture.json`, and a 320x240-per-view
`overview.jpg`; no history log or per-frame files accumulate. Metadata contains
the source camera stamps, intrinsics, depth scale, raw resident joint values,
and their signed temporal delta. `arm.qualified_sync` is true only for a
complete resident sample with consistent host clocks, age at most 0.5 s, and
absolute camera/arm delta at most 0.16 s. That field describes temporal pairing
only, never geometric calibration. A missing, stale, malformed, or mismatched
sample is marked unpaired so the existing live-twin selector rejects it. The
overview's depth color map is visualization only; the saved depth remains raw
uint16 samples.

The build and tests are offline and do not enumerate or open a camera. Physical
topic rate, serial identity, RGB/depth pairing, and TF behavior still need an
operator-owned handoff test on the station before use.
