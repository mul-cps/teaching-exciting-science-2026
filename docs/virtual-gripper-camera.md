# Virtual gripper camera

The passive ROS twin browser now has **Follow virtual gripper camera**. It
moves the existing browser camera with fresh `gripper_frame_link` TF, rendering
the same depth point cloud and diagnostic scene. This uses the existing browser
GPU renderer; it adds no camera stream, CPU rasterizer or depth reconstruction.
Unchecking returns to the station overview. Placement camera framing exits ego
mode. Controls are shared by connected viewer clients, like the placement view.

The virtual eye defaults to tool +Z, tool -Y up, 40 mm behind the tool origin and
60 degrees vertical field of view. Select another tool axis and a retreat from
0 to 150 mm to inspect the approach. These are visualization settings, not a
measured camera mount or a change to TF/calibration. Existing TF axes remain
visible. Looking along tool Y uses tool +Z up to avoid a degenerate view.

The view follows only with fresh robot inputs, a fresh tool TF chain and a depth
cloud received within 0.5 seconds. Otherwise its pose is held and the panel says
WAITING/STALE; stale cloud points are hidden. The synthetic RGB image plane and
its projected placement outline are hidden while following so an image from the
fixed camera cannot be mistaken for a gripper-camera image.

## What the current representation can show

A single RGB-D point cloud is sufficient for a useful alternate perspective of
**surfaces observed by the fixed depth camera**. The current ROS bridge samples
up to 12,000 XYZ points, displayed in a uniform color. The virtual camera does
not reveal additional measured detail, fill holes or infer hidden geometry.
Sparse splats can show through gaps and do not certify free space or occlusion.
The apparent alignment is only as accurate as the existing camera/robot TF.
Detected object boxes/contact markers are diagnostic overlays, not measured
surface reconstructions. The separate MuJoCo image remains a model view.

For denser, textured views, preserve RGB per depth sample first. For fewer gaps,
accumulate observations with trustworthy camera poses and remove stale dynamic
objects; a surfel or TSDF mesh can provide more consistent surface occlusion.
Neither accumulation nor meshing can recover surfaces never observed. A real
wrist camera or additional registered depth viewpoints are needed for those
surfaces. This feature does not qualify robot registration or enable motion.

## Optional ROS camera feed

A separate ROS-only node can publish the synthetic view without any browser:

```bash
ROS_DOMAIN_ID=76 ROS_LOCALHOST_ONLY=1 ros2 launch so101_station_perception \
  virtual_gripper_camera.launch.py enabled:=true
```

It is disabled by default. The launch applies the existing process-local GPU
library preload and AMD renderer settings. The EGL renderer rejects software
renderers, including llvmpipe; it never silently falls back to CPU rendering.
Native EGL/GLES libraries are used through ctypes, without a new Python GPU
package. If EGL cannot initialize on hardware, the node refuses startup.

Topics under `/so101/synthetic_gripper_camera`:

| Topic | Meaning |
|---|---|
| `image_raw` | `sensor_msgs/Image`, `rgba8`, uniform-color observed point samples |
| `camera_info` | Virtual pinhole intrinsics, no distortion |
| `status` | Synthetic/unqualified flags, original stamp, age, renderer, point count and timings |
| `image_annotated` | Optional `rgb8` display image; annotations are not scene observations |
| `annotation_mask` | Optional `mono8`; nonzero pixels identify annotation strokes/text |

The corresponding TF child is `synthetic_gripper_camera_optical`, parent
`gripper_frame_link`. Optical axes are +X right, +Y down, +Z forward and match
the browser's selected axis/retreat convention (both default +Z/40 mm). Browser
controls do not change ROS node parameters. Image, CameraInfo and mount TF share
the **original input cloud acquisition stamp**. The cloud-to-tool transform is
looked up at that exact stamp. Missing TF, duplicate/future/zero/stale stamps or
age exceeding 0.5 seconds after rendering drop the acquisition without publishing
a replacement image. A held image in a downstream viewer must be checked for age.

The default image is 320×240 at at most 3 Hz, using at most 20,000 finite XYZ
samples from each admitted PointCloud2. Other ROS parameters can reduce the
rate/point count or set dimensions up to 640×480; larger values are rejected.
Depth input is bounded to 1280×720 points. Frames are rate gated before decoding,
then vectorized sampling decodes XYZ once. Native EGL draws one-pixel points
with a depth buffer, reusing its pbuffer/VBO/shaders. One RGBA readback per frame
is bounded to 307,200 bytes at the default resolution (1,228,800 at the maximum).
It does not accumulate observations or infer triangles between points.

**Raw alpha semantics:** alpha 255 means an observed rasterized sample; alpha 0
means unknown. All unknown RGBA channels are zero. A point image has gaps and
cannot certify occlusion or free space. Standard viewers that ignore alpha may
show unknown pixels as black. Raw imagery never contains crosshair/text strokes.

### Nominal gripping-center crosshair

Provide an opening-specific CAD artifact to enable a diagnostic crosshair:

```bash
ROS_DOMAIN_ID=76 ROS_LOCALHOST_ONLY=1 ros2 launch so101_station_perception \
  virtual_gripper_camera.launch.py enabled:=true \
  pinch_artifact:=/absolute/path/nominal-pinch.json publish_annotation:=true
```

The JSON schema is `so101-virtual-pinch-overlay-v1`, with parent_frame
`urdf:gripper_frame_link`, a proper 4×4 `parent_from_pinch` matrix in metres,
`opening_angle_rad` matching the ROS gripper visual proxy, and `qualified:false`.
Retain CAD/source hashes and derivation provenance in that artifact. This is a
**NOMINAL / UNCALIBRATED** center, not a physically measured TCP. The artifact
SHA and projected pixel are reported in status.

The crosshair projects the configured 3D pinch center through the virtual
optical transform and CameraInfo K. It is not assumed to lie at image center.
It is hidden if behind/outside the virtual camera, if the fresh ROS `gripper`
joint angle differs by more than 1e-5 rad, or if the joint acquisition stamp is
more than 50 ms from the depth stamp. Changed opening requires a matching new
artifact; an old nominal center is not silently reused. Unknown or changed
opening produces `NOMINAL CENTER UNAVAILABLE` text without a crosshair.
Annotation pixels are tracked separately and never change raw unknown-alpha
semantics. The browser follow view does not add this image crosshair.

The station RViz configuration includes **NOMINAL Virtual Gripper Camera** as an
enabled Image panel subscribed to `image_annotated` with Best Effort/Volatile
sensor QoS. It waits until the optional feed is started with annotations enabled.
A second **Virtual Gripper Camera raw samples** Image display is disabled by
default. The synthetic optical frame is included in the selected TF axes.
RViz continues to use the station's existing GPU startup guard; adding an Image
display does not introduce a CPU renderer or change the robot's camera stream.
