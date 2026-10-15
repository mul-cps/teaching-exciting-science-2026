# RGB-D tabletop objects and geometric grasp candidates

Build the workspace and source its install space, then start the RealSense and
the `so101_station_perception/charuco_world.launch.py` transform publisher.
That publisher must provide `charuco_board <- camera_color_optical_frame` at
the image timestamp. Start the detector with:

```sh
ros2 launch so101_station_objects objects.launch.py
```

The node pairs best-effort RGB, aligned depth, and CameraInfo messages, rejects
stale or mismatched inputs, and processes at no more than 5 Hz. By default it
requires a recent valid observation on
`/so101_perception/charuco_world/status`; retained camera TF alone does not
count as fresh localization. When registration is missing or stale, the node
clears detections and grasp proposals and reports the reason in
`/so101/grasp_status`. Set `require_registration_tracking:=false` only when
using a separately validated static board-to-camera transform.
When tracking drops briefly, the node stops publishing fresh detections and
grasp proposals; the last messages keep their original image stamp so
age-aware consumers can reject them. RViz markers retain that stamp and expire
after 1 second. At expiry, the node sends one empty detection/grasp update and
one marker clear. A valid observation with no objects clears immediately.

It fits the observed table from at most 2,000 depth samples in a bounded
board-frame ROI (`x=-30..210 mm`, `y=-30..160 mm`, `z=-50..60 mm`). The
measured support plane is reused for at most 2 seconds if a frame lacks enough
table samples; this cache is expressed in the fixed board frame. The board's
z=0 plane is used only when the measured table agrees within 5 degrees and
15 mm. Candidate footprints must be 10–65 mm across, no more than 2.5:1 in
aspect, and between 2 and 25 mm above the support plane. The lower height
allowance retains low-profile bricks while the aspect bound removes long,
narrow color fragments. Red, green, blue, and yellow regions produce spatially
associated detections; gray and black regions are ignored.

`/so101/objects` is `vision_msgs/Detection3DArray` in `charuco_board`.
Each detection box is centered halfway up its observed height, so its bottom
face lies on the measured support plane. `/so101/object_markers` displays that
box plus a short single-token track label above its top; the label pose is a
copy and does not change the detection or box center.

`/so101/grasp_contacts` is a `geometry_msgs/PoseArray` at the estimated
mid-height contact pose. `/so101/grasp_candidates` retains its existing
contract as the top-down pregrasp pose, 60 mm above the matching contact pose.
This offset is deliberate and can make pregrasp axes appear above a brick; use
the contact topic to inspect where the fingers would meet the object. Both
topics share the Detection3DArray timestamp and frame. `/so101/grasp_status`
is JSON in `std_msgs/String`, including IDs, dimensions and their uncertainty,
measured support-plane diagnostics, jaw opening, and contact/pregrasp
coordinates.

These are image-derived geometric proposals only. Dimensions describe visible
RGB-D extents, not CAD dimensions or necessarily the whole object. Grasp
finger clearance, IK, collision checking, and execution are not validated;
the package does not publish a MoveIt planning-scene update or command the arm.
Track IDs are temporary spatial associations and are not semantic identities.

### Surface anchoring and RViz display

Object geometry is fitted relative to a measured, upward-facing RGB-D support
plane. For unit normal `n`, plane offset `d`, box height `h`, and box center `c`,
the box is constrained by `n·c + d = h/2`: its lower face lies on the fitted
surface. This is a geometric constraint, not proof that the depth plane or
camera calibration has zero physical error.

`/so101/grasp_contacts` displays contact poses on the measured objects. The
existing `/so101/grasp_candidates` remains the **pregrasp** stream, 60 mm away
from contact along the reverse approach direction; MoveIt consumes that stream.
RViz shows contact axes by default and hides the approach axes and text labels.
The namespace switches can enable labels when needed. Compact labels contain
no spaces because small RViz text markers have an upstream
[space-width scaling issue](https://github.com/ros2/rviz/issues/1336).

Label markers hold independent pose messages. Moving a label for readability
must never change the Detection3D, object cube, or collision-box pose.
