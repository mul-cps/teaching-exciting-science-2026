# Virtual gripper image validity

The synthetic feed is a reprojection of observed points plus explicitly nominal CAD;
its pose and pinch geometry remain unqualified. Torque off alone does not invalidate
an otherwise fresh measured pose.

A 5 Hz metadata watchdog uses the existing `max_age_s` (at most 0.5 seconds), both
the **original cloud ROS timestamp** and the monotonic start of the last successful
render callback. Expiry, backward/nonfinite monotonic time, backward ROS time,
rejected acquisition stamps, unavailable exact acquisition-time tool/mesh TF, or
render failures invalidate the display. Timer dispatch can itself be delayed by a
blocked single-threaded callback; this is no independent hard real-time watchdog.

On each valid-to-invalid transition, and once before any first valid render:

- `image_raw` and `image_scene` become blank RGBA with alpha zero.
- `provenance_mask` and `annotation_mask` become zero: no observed/model/pinch pixels.
- `image_annotated` becomes black RGB with **STALE / INVALID** text. This is a display
  diagnostic, not geometry or an observed image. No cached scene pixels are reused.
- Every diagnostic image retains the last successful **virtual optical image frame
  and original acquisition timestamp**. Before first success its stamp is zero and
  frame is `synthetic_gripper_camera_optical`. Neither CameraInfo nor TF is renewed.

A valid accepted render restores current outputs. `publish_annotation=false` keeps
pinch annotations disabled; `image_annotated` still carries the scene RGB so it can
restore the diagnostic display. The renderer/readback count does not change. This
adds a bounded RGB materialization when pinch annotation is disabled.

The existing `/so101/synthetic_gripper_camera/status` String adds `visual_valid`,
`checked_ros_ns`, `last_valid_acquisition_stamp_ns`, `last_valid_tool_tf_stamp_ns`,
and `last_valid_acquisition_age_s`. Check time is not acquisition time. Invalid
status is repeated at 5 Hz; blank images are emitted once per invalid transition.
Rejected inputs cannot refresh the last successful timestamp or receipt lease.

## Operator display

The configured RViz Image display uses `image_annotated`. It has no existing bound
status overlay/expiry policy; previously it could retain old pixels indefinitely.
The replacement diagnostic is intended to make invalidity visible without giving
old geometry a current timestamp. Any image client that rejects repeated or zero
headers must also consume `status`; actual RViz rendering still requires the root's
separate integration check. A stopped/crashed synthetic-camera process cannot issue
a blank and requires external process/status monitoring. The standalone browser
MuJoCo twin has its own STALE / WAITING state; this patch does not change that client.

## CPU verification

`python -m pytest -q tests/test_virtual_camera_stale.py tests/test_virtual_gripper_camera.py`

The dedicated test extracts and executes the production methods with fake ROS
message/publisher/TF/renderer objects, never initializing ROS, EGL, physical cameras
or devices. Tests cover no cloud, exact-TF failure, render failure, late render,
clock discontinuities, restoration, semantic masks, unchanged headers, disabled
pinch overlay and watchdog cancellation before renderer teardown. Existing EGL
tests remain opt-in and are not run for this change before independent review.
