# Offline SO-101 gripper registration

`tools/fit_ros_gripper_registration.py` compares local wrist-roll mount hypotheses against saved RGB-D bundles while holding the recorded base transform and all body joints fixed. It applies the candidate only to the URDF joint from `wrist_roll` to `gripper_link`, then reports both observed-depth-to-CAD residuals on a per-snapshot population frozen from the union of candidate CAD visibility masks and CAD-frontface-to-depth overlap. The last bundle is the held-out pose by default; pass `--holdout-index` to change it.

For the current configured mount of +90 degrees, the default `0,90,180,-90` deltas test total mount rotations of +90, +180, +270, and 0 degrees. A candidate JSON file and held-out image overlay are written to the requested output directory. The overlay marks CAD samples whose projected depth agrees within 60 mm; it is a visual aid, not an acceptance criterion.

The report includes every detected AprilTag 25h9 observation (including repeated code ID 0), depth-derived provisional tag size, track association and planar pose branches. Size and movement are clues for assigning the ~19.4 mm moving gripper tag and ~36 mm stationary EV3-base tag; track IDs are temporal indices, never permanent identity. A constant `gripper_from_tag` transform has an absolute-roll gauge: tag observations alone cannot select one of the constant roll candidates. CAD face geometry and depth returns are required to compare absolute mounting orientation. The station's gripper joint state is an opening-percent proxy, so its current value does not calibrate jaw angle or servo mapping.

This utility is strictly an offline diagnostic. It does not invoke ROS, access hardware, change the live model, or apply a candidate. It does not jointly optimize global pose, body joint zeros, jaw geometry, or tag mounting. Treat all outputs as unqualified until independent review of held-out per-link metrics and overlays. It refuses a bundle set with fewer than three training poses, insufficient independent motion, or a held-out pose too close to training.

Example:

```sh
python tools/fit_ros_gripper_registration.py \
  evidence/baseline evidence/wrist-roll-plus evidence/holdout-combination \
  --output /tmp/gripper-registration --samples 10000
```
# Resident `GripperCommand` action

The resident trajectory bridge exposes the standard
`control_msgs/action/GripperCommand` action at
`/follower/gripper_controller/gripper_cmd`. Its `command.position` and result
`position` are jaw opening in metres. Internally, the bridge converts through a
qualified jaw-gap calibration into the station owner's native gripper percent
and then into the identity-matched raw encoder interval. It shares the existing
ArmSession command owner and admission lock; it never starts a second bus owner.

The bridge requires the `gripper_jaw_calibration` parameter to name a JSON file
with this schema:

```json
{
  "schema": "so101-jaw-gap-calibration-v1",
  "qualified": true,
  "closed_gap_m": 0.004,
  "open_gap_m": 0.075,
  "closed_percent": 0.0,
  "open_percent": 100.0
}
```

The example values are illustrative and are not a station calibration. Do not
enable this action with them. `max_effort` is only accepted as zero because the
servo has no calibrated force limit. A successful result means the jaw reached
the requested calibrated gap; it does not mean an object was grasped. Pickup
verification requires separate scene evidence that the selected tracked object
moved with the gripper during lift.
