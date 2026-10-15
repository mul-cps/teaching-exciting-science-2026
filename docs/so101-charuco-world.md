# Live ChArUco world frame

`ros2 launch so101_station_perception charuco_world.launch.py` tracks the fixed
7 by 5 ChArUco board (25 mm squares, 18 mm markers, `DICT_6X6_250`) as a world
reference while the camera moves. It consumes timestamp-paired color and
`CameraInfo` messages, uses the supplied camera distortion model in calibrated
PnP, and publishes at no more than 5 Hz.

The `charuco_board` frame is the root of this visual scene. Each valid
observation publishes `charuco_board -> camera_link` at the observation stamp;
the camera's vendor `camera_link -> camera_color_optical_frame` transform is
left intact. Fewer than six board corners, stale input, invalid intrinsics,
negative depth, or reprojection RMS above 2.5 pixels suppresses the transform.
The status topic `/so101_perception/charuco_world/status` reports tracking,
occlusion, and freshness without keeping old transforms alive.

The board observation establishes camera pose relative to the board only. It
does not establish robot-base calibration. Optional initial transform files
can seed a `charuco_board -> base_link` transform using `base_from_camera` and
`camera_from_board` 4x4 matrices, but that transform is explicitly an
**INITIAL ESTIMATE** and must not be treated as qualified calibration.
