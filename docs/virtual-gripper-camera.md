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
up to 12,000 XYZ points with measured packed RGB/RGBA colors when available. A field-absent uniform fallback is explicitly identified. The virtual camera does
not reveal additional measured detail, fill holes or infer hidden geometry.
Sparse splats can show through gaps and do not certify free space or occlusion.
The apparent alignment is only as accurate as the existing camera/robot TF.
Detected object boxes/contact markers are diagnostic overlays, not measured
surface reconstructions. The separate MuJoCo image remains a model view.

Measured RGB is retained per depth sample; it does not invent surface texture between samples. For fewer gaps,
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
| `image_raw` | `sensor_msgs/Image`, `rgba8`, observed RGB point samples; no model pixels |
| `camera_info` | Virtual pinhole intrinsics, no distortion |
| `image_scene` | `rgba8`, observed points plus explicitly NOMINAL gripper CAD |
| `provenance_mask` | `mono8`: 0 unknown, 1 observed point, 2 nominal model |
| `status` | Synthetic/unqualified flags, color source, model hashes, original stamp, age, FOV, renderer and timings |
| `image_annotated` | Optional `rgb8` display image; annotations are not scene observations |
| `annotation_mask` | Optional `mono8`; nonzero pixels identify annotation strokes/text |

The corresponding TF child is `synthetic_gripper_camera_optical`, parent
`gripper_frame_link`. Optical axes are +X right, +Y down, +Z forward and match
the browser's selected axis/retreat convention (both default +Z/40 mm). Browser
controls do not change ROS node parameters. The shared mount includes a fixed
180° roll about optical forward: right/down are reversed together, while the
40 mm retreat, eye position, forward axis, 320×240 image and 60° vertical FOV
remain unchanged. ROS TF, browser up vector, mesh/point projection and nominal
crosshair use this same mount; no image-only flip or top-side translation is applied. Image, CameraInfo and mount TF share
the **original input cloud acquisition stamp**. The cloud-to-tool transform is
looked up at that exact stamp. Missing TF, duplicate/future/zero/stale stamps or
age exceeding 0.5 seconds after rendering drop the acquisition without publishing
a replacement image. A held image in a downstream viewer must be checked for age.

The default image is 320×240 at at most 3 Hz, using at most 20,000 finite XYZ
samples from each admitted PointCloud2. Other ROS parameters can reduce the
rate/point count or set dimensions up to 640×480; larger values are rejected.
Depth input is bounded to 1280×720 points. Frames are rate gated before decoding,
then vectorized sampling decodes XYZ once. Native EGL draws one-pixel points
with a depth buffer, reusing its pbuffer/VBO/shaders. One atlas readback per frame
is bounded to 614,400 bytes at the default resolution (2,457,600 at the maximum).
It does not accumulate observations or infer triangles between points.

**Raw alpha semantics:** alpha 255 means an observed rasterized sample; alpha 0
means unknown. All unknown RGBA channels are zero. A point image has gaps and
cannot certify occlusion or free space. Standard viewers that ignore alpha may
show unknown pixels as black. Raw imagery never contains CAD, crosshair or text strokes. Scene images have a separate provenance mask and must not be treated as depth observations.

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
The annotated image displays the scene layer plus text/crosshair. Annotation pixels are tracked separately and never change raw unknown-alpha
semantics. The browser follow view does not add this image crosshair.

The station RViz configuration includes **NOMINAL Virtual Gripper Camera** as an
enabled Image panel subscribed to `image_annotated` with Best Effort/Volatile
sensor QoS. It waits until the optional feed is started with annotations enabled.
A second **Virtual Gripper Camera raw samples** Image display is disabled by
default. The synthetic optical frame is included in the selected TF axes.
RViz continues to use the station's existing GPU startup guard; adding an Image
display does not introduce a CPU renderer or change the robot's camera stream.

The node retains at most 64 accepted gripper-joint samples. For delayed depth
frames it selects the temporally nearest sample within the unchanged 50 ms
acquisition window, rather than the latest received joint state. Every selected
sample must still be nonfuture and at most 0.5 seconds old at annotation time.
Out-of-order arrivals are matched by their original acquisition stamps. Opening
value never affects which timestamp is selected: if the nearest opening differs
from the nominal artifact, the crosshair is hidden. Annotation status reports the
selected joint stamp and its difference from the depth acquisition stamp.


## Observed RGB and nominal gripper mesh layers

PointCloud2 `rgb`/`rgba` fields declared FLOAT32 or UINT32 are decoded as packed
integer bits with the message's byte order. XYZ sampling and nonfinite filtering
use exactly the same indices for colors. Present malformed colors reject the
acquisition; only a missing color field permits an explicit uniform fallback.
Color source is reported in status. Source RGBA alpha is not a depth-validity
signal: output unknown alpha comes from rasterized depth samples, not packed
color alpha. There is no texturing, interpolation across holes, or invented RGB.
The browser uses the same bounded decoder. Its color payload is RGB8 base64,
limited to 48,000 characters for 12,000 points, and decoded once per cloud change.

The optional ROS feed loads **every visual mesh** in `gripper_link` and
`moving_jaw_so101_v1_link` from the effective `/follower/robot_description`.
URDF visual origins and scales are applied exactly once. Binary STL loading,
source hashes, baked gray diagnostic shading and GPU uploads happen only when
the robot description changes. The bounds are 16 visuals and 200,000 triangles.
The current pinned description has three visuals: servo, fixed-finger housing,
and moving jaw. These are gray NOMINAL CAD, not observed or physically calibrated
surfaces. No other robot/workspace geometry is inferred into this feed.

Each model link uses TF at the exact depth acquisition stamp. Missing model TF
omits the model layer and reports that reason; it never substitutes latest TF.
A GPU atlas draws cloud-only on the left and cloud plus CAD on the right, sharing
one EGL context, depth allocation and bounded readback. `image_raw` retains the
unoccluded observed cloud-only view. `image_scene` contains the nearest rendered
point/model surface; its alpha means a rendered layer, not a physical observation.
`provenance_mask` distinguishes their winning pixels. `annotation_mask` identifies
crosshair/text and is a different mask. Both masks and both images share the
original acquisition header and CameraInfo. Unknown regions remain unknown.

The default is **320×240, vertical FOV 60°, horizontal FOV 75.178°**, tool +Z
forward and a 40 mm retreat from the URDF tool origin. CameraInfo and status
report the exact rendered geometry. Launch arguments expose `width`, `height`,
`vertical_fov_deg`, `forward_axis`, `backoff_m`, `max_hz` and `render_gripper_mesh`.
Changing the viewpoint is explicit; no automatic retreat/FOV adjustment hides
occlusion or changes calibration. Browser and ROS use the same mount formula,
but changing a browser control does not reconfigure the ROS feed.


Offline replay of the frozen real cloud/model/TF cohort compared 40, 80 and
120 mm retreats at the same CameraInfo. At 40 mm the jaws/housing frame a visible
workspace gap; 80 mm lies inside opaque CAD and 120 mm is almost fully occluded
by the gripper body. The default remains 40 mm. CAD is rendered opaque with a
depth buffer: no opacity trick turns hidden samples into observations. The
cloud-only raw stream remains available even where the nominal model occludes
the scene stream. Those replay images evaluate framing, not physical calibration.
