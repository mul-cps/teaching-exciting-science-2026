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

It fits the observed table from at most 2,000 depth samples in a bounded
board-frame ROI (`x=-30..210 mm`, `y=-30..160 mm`, `z=-50..60 mm`). The
measured support plane is reused for at most 2 seconds if a frame lacks enough
table samples; this cache is expressed in the fixed board frame. The board's
z=0 plane is used only when the measured table agrees within 5 degrees and
15 mm. Candidate footprints must be 10–65 mm across and lie within 25 mm of
the support plane. Red, green, blue, and yellow regions produce spatially
associated detections; gray and black regions are ignored.

`/so101/objects` is `vision_msgs/Detection3DArray` in `charuco_board`.
`/so101/object_markers` shows the observed boxes and labels. `/so101/grasp_candidates`
is `geometry_msgs/PoseArray`, with one top-down pregrasp per proposal, 60 mm
above its estimated contact pose. `/so101/grasp_status` is JSON in
`std_msgs/String`, including IDs, dimensions and their uncertainty, measured
support-plane diagnostics, jaw opening, and contact/pregrasp coordinates.

These are image-derived geometric proposals only. Dimensions describe visible
RGB-D extents, not CAD dimensions or necessarily the whole object. Grasp
finger clearance, IK, collision checking, and execution are not validated;
the package does not publish a MoveIt planning-scene update or command the arm.
Track IDs are temporary spatial associations and are not semantic identities.
