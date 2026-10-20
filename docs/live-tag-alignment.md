# Continuous tag alignment

This optional ROS node matches seeded physical marker instances to model links and renders a separate aligned robot. It never commands the arm or writes canonical robot/camera TF. Every output remains `qualified: false`: surface-initialized marker mounts are provisional, and a fit to those same mounts is not independent calibration evidence.

## Inputs and correspondence file

The existing `marker_pipeline` calls `TagObservations._images` once on its admitted RGB-D pair. That callback publishes `/so101_perception/tag_detections` after its existing admission check. The compact JSON contains original exposure stamp, optical frame, image dimensions, code IDs, corners, pixel centers, depth-backed camera centers, provisional black-edge sizes and measurement source. Derived messages also carry the checked source consumer epoch, pin digest, producer/source generations and camera serial, so a fast same-process source reset invalidates the association until explicit reseeding. It reuses the detector result and aligned depth; it does not run another detector or copy the RGB image. White-module depth-plane centers are preferred; a consistent 3x3 center-depth patch is labeled as a fallback. No ambiguous planar PnP orientation is asserted.

Provide a private JSON file with schema `so101-tag-bindings-v1`, exact `model_sha256`, `camera_serial`, `image_width`, `image_height`, `camera_from_base_reference` (4x4), and `bindings`. Each binding needs:

- `instance_name`, `link`, `code_id`, `size_m` (outer black-border edge, metres);
- `seed_px` and `seed_radius_px` in original image coordinates;
- `link_point_m` (marker center expressed in that link), `moving` boolean;
- `qualified: false` and measurement/initialization provenance.

All repeated code IDs need explicit distinct instance seeds. A moving-finger tag must use `moving: true`: it reports a residual and is excluded from the rigid alignment fit. At least three non-collinear fixed-link centers, with sufficient baseline, must be available. Files are limited to 64 KiB and 16 bindings; messages to 64 KiB and 64 observations.

## Run

Build/install the updated perception package in a separate overlay and source it in the existing ROS environment. The existing marker pipeline also needs that updated package; restarting it requires the station's perception recovery procedure. This launch does not start or restart the arm owner, camera or detector.

```bash
ROS_DOMAIN_ID=76 ROS_LOCALHOST_ONLY=1 \
python tools/run_live_tag_alignment.py --bindings-file /absolute/path/to/bindings.json
```

Or use `ros2 launch so101_station_perception live_tag_alignment.launch.py bindings_file:=/absolute/path/to/bindings.json`. Domain 0 is rejected. The wrapper explicitly sets localhost transport and one BLAS/OMP thread.

The node validates the latched `/follower/robot_description` SHA and independently queries `/camera/device_info` for the configured serial. Original-exposure TF lookups resolve each marker's link to `base_link`. Position-only Kabsch fits are bounded to 3 cm / 5 degrees against both the original reference and last accepted candidate, with maximum fixed-point residual 8 mm. These numerical bounds are guards, not calibration accuracy claims.

## RViz and ROS outputs

- `/so101_perception/live_tag_alignment/status`: compact JSON status, fit residuals, timing and provenance. A 1 Hz heartbeat makes the latest availability visible to late subscribers, including while awaiting input. It preserves the original exposure stamp, reports its current age, and never renews a stale candidate.
- `/so101_perception/live_tag_alignment/candidate`: timestamped candidate `TransformStamped`; not a canonical TF authority.
- `/so101_perception/live_tag_alignment/markers`: cyan measured centers, magenta model-predicted centers, yellow connecting residuals and labels.
- `/so101_perception/aligned_robot_description`: transient-local visual-only URDF, with link/joint names prefixed `live_aligned/`.
- `/tf`: only `live_aligned/*` children, with camera-to-aligned-base candidate and exposure-time canonical joint transforms copied into that separate tree.

Add a RobotModel using the aligned description topic and a MarkerArray using the markers topic. Retain the original RobotModel as a comparison. When unavailable, the aligned description contains a valid, geometry-free link in the existing camera frame, hiding the aligned robot without triggering an RViz URDF parse error. This hidden description is also published during startup. No fresh aligned TF is broadcast while unavailable. TF listeners can still retain old samples in their own caches, so status and exposure time determine availability.

Loss, ambiguity, stale stream gaps and compact publisher/frame/admission changes stop current output. Recovery of a lost instance requires explicit reseeding, never automatic association with a same-ID tag after occlusion:

```bash
ROS_DOMAIN_ID=76 ROS_LOCALHOST_ONLY=1 ros2 service call \
  /so101_perception/live_tag_alignment/reseed std_srvs/srv/Trigger '{}'
```

This restores the original file's pixel seeds. If the arm or camera has moved beyond those seeds, prepare a new reviewed correspondence file and restart only this node. `derived_checked` records that the shared pipeline's derived-source admission check ran; `native_unattested` is explicitly not an attested derived source. The alignment node rejects `native_unattested` for usable alignment. Neither permits qualification or autonomous motion.

## Stationary camera damping

The main ChArUco camera-pose filter defaults to `pose_filter_alpha=0.05`,
`pose_translation_deadband_m=0.002`, and `pose_rotation_deadband_deg=0.3`.
Translation and rotation inside their respective magnitude deadbands retain the
previous estimate; larger small changes use the low-pass gain. Changes above
30 mm or 0.12 rad retain the existing immediate camera-movement behavior, and
stale observations still force reacquisition. Only the camera extrinsic is
filtered: object points, arm joints, and their timestamps are not smoothed.
Tracking status reports the configured values and filter scope. Override these
ROS parameters at node startup when a different camera-mount behavior is needed.

## Restoring measured arm telemetry

The trajectory bridge supports `startup_observe_only:=true`. This starts its
owner without Goal_Position, torque, profile, or calibration-register writes,
retains the measured torque state, and keeps the bridge fault/STOP admission
latch set. Fresh measured joint states remain available. A verified STOP and the
normal commissioning gates are still required before motion.

Source the updated control overlay before launching, then verify
`/follower/trajectory_server` reports `startup_observe_only=true` and that the
owner state actually says `observe_only=true`. A parameter override passed to an
older package does not activate this behavior. Long-lived hardware owners should
run under a persistent process supervisor, independently of a terminal session.
