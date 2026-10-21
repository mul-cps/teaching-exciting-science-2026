# Camera versus encoder reversal measurement

This is a small **offline, read-only** measurement tool. It does not start ROS,
open a camera/serial port, send commands, change calibration, or enable
compensation. `physical_qualified` and `compensation_allowed` always remain false.
The synthetic example validates the mathematics only. No physical backlash value
has been measured by this change.

## What is missing for a physical result

The current `tag_detections` compact publisher supplies exposure stamps, optical
frame, source identity, code IDs, corners, provisional sizes and depth-backed
centers. It deliberately does **not** publish a qualified orientation. Existing
`live_tag_alignment` mounts initialized from model/FK surfaces cannot independently
measure that model's joint error. A moving-finger tag is not a gripper-body tag.

For the tested joint, identify two rigid marker instances, one on each adjacent
link, plus a fixed table/wall witness visible in the same exposure. Different
physical instances must remain distinct even when they share a code ID. Measure
`link_from_tag` transforms, the joint axis in the parent link, and the zero
`parent_from_child_zero` orientation independently of servo counts/FK. A rigid
multi-tag board or independently reconstructed link frame can replace one marker
instance, provided its pose and uncertainty are externally observable. Do not use
an encoder-initialized camera-to-base transform to invent link poses.

Use RGB poses from measured black-border sizes and calibrated RGB intrinsics and
distortion. Resolve planar PnP ambiguity with independent multiview/rigid-board
information; reject unresolved hypotheses. Existing
`workshop.digital_twin.tag_tracking.pose_candidates` exposes candidates but its
provisional depth-supported selection is **not** this qualification. The D435i
1080p/30 Hz RGB stream is potentially useful; small 7.5/19.4 mm tags do not imply
subdegree accuracy. Measure stationary repeatability at each pose/distance/view,
mount rigidity and print scale, camera calibration residuals and clock/exposure
uncertainty (including rolling shutter). Record the uncertainty evidence source.
Depth and registered RGB-D pose estimates are rejected by this tool, excluding
registration/depth bias from this measurement path.

## Minimal experiment for the existing owner (ROOT runs it)

1. Keep the existing single controller owner, collision checks, calibration hashes,
   motion bounds, torque and STOP safeguards. Pin the controller/model/collision/
   calibration/source hashes and record owner and goal IDs. The collector only
   receives snapshots from that owner; it creates no second owner or service.
2. Fix the camera, illuminate tags, retain raw images/corners and record a fixed
   witness pose in every accepted exposure. Check stationary repeatability before
   and after the run. If the witness moves beyond the configured angular or metric
   bound, abort analysis. Common-camera rotation cancels in simultaneous relative
   link rotations; witness checks still guard camera movement and changed views.
3. Select a safe small reversal range around one pose. Keep other joints and load
   identical; assign a `load_pose_id`. ROOT performs at least three slow cycles
   with the existing motion API, recording every command/goal and encoder sample.
   Prefer dwell frames before/after each reversal and at least 30 frames per
   direction. Freeze limits before capture; reduce speed so timing error is well
   below the uncertainty budget. Repeat at another speed to expose dynamic lag,
   then at another load separately; do not pool unlike conditions.
4. Retain raw encoder counts with both acquisition and camera exposure mapped into
   one documented clock domain. Record clock conversion evidence in
   `uncertainty_source`, including offset/drift bounds. Each exposure needs encoder
   samples on both sides within `max_sync_gap_s`. Do not substitute callback
   arrival time or target joint position for measured counts. Unwrap encoder counts
   before analysis and preserve the original raw stream/hash alongside that export.
5. Analyze the export below. Unresolved motion, ambiguity, insufficient matched
   frames, source changes, excessive uncertainty, off-axis movement and camera
   drift all block. Repeat runs must agree within their bounds before interpreting
   an effect as mechanical. Elastic deformation, friction and dynamic lag can also
   produce hysteresis; this script does not isolate gear backlash or fit a correction.

## Files and capture API

`tools/backlash_measurement/analyze.py` validates schema
`so101-camera-backlash-v1`. Run the fixture generator to obtain a complete concrete
manifest and JSONL schema example; replace **every synthetic value** for a physical
export. The manifest contains:

- `run_id`, `owner`, `goal_id`, `load_pose_id`, `clock_domain`, camera serial and
  optical frame; SHA256 strings for model, calibration, mounts, source, collision
  contract and controller; uncertainty and camera-calibration evidence sources.
- Parent/child instance identity, rigid and moving-finger flags, independently
  measured homogeneous `link_from_tag` transforms; fixed witness identity;
  `parent_from_child_zero`, `axis_parent`, encoder sign/zero/degrees per count.
- Explicit synchronization, speed, reprojection, off-axis, uncertainty, camera
  drift, visible-motion and bidirectional sample/bin/reversal gates. Example
  thresholds are synthetic settings, **not hardware acceptance thresholds**.

Every JSONL row repeats the run/owner/goal/load/clock identity and six hashes.
Encoder rows have `kind: encoder`, `t_s` and integer `raw_count`. Frame rows have
`kind: frame`, exposure `t_s`, `camera_serial`, `optical_frame`, `image_sha256` and
`observations` keyed by physical instance. Each observation supplies a rigid
`camera_from_tag` 4x4, original `corners_px`, conservative `sigma_deg`,
`reprojection_rms_px`, `ambiguous: false`, and `independent: true`. These flags are
assertions by the capture adapter, not proof; retain their calibration/ambiguity
evidence. The manifest source hash should identify the exact adapter/estimator
implementation; archive native source epochs/pin digests and images alongside the
export. Split runs when source admission epochs or identities change.

ROOT can call the file-only API from the existing read-only snapshot collector:

```python
from tools.backlash_measurement.analyze import append_record
# manifest is loaded once; snapshot is a schema-complete measured row.
append_record('/absolute/new-evidence/records.jsonl', manifest, snapshot)
```

Use a single file writer. `append_record` validates before appending and raises on
invalid data. It polls nothing and writes only the supplied evidence file. A new
capture adapter is still required to populate independent orientation observations;
the existing center-only ROS topic cannot be directly relabeled into this schema.

## Runnable offline commands

From the repository root, using Python with NumPy:

```bash
python tools/backlash_measurement/check_math.py
python tools/backlash_measurement/synthetic_fixture.py /tmp/backlash-math-example
python tools/backlash_measurement/analyze.py \
  /tmp/backlash-math-example/manifest.json \
  /tmp/backlash-math-example/records.jsonl \
  --output /tmp/backlash-math-example/report.json
# After ROOT has captured actual independent observations:
python tools/backlash_measurement/analyze.py \
  /absolute/new-evidence/manifest.json /absolute/new-evidence/records.jsonl \
  --output /absolute/new-evidence/report.json
```

The generator refuses an existing directory. Analyzer exit code 2 means blocked;
its report records the reason and input file hashes. Successful analysis retains
all sample angles/bounds plus matched-bin hysteresis widths and reversal detection
travel. The latter is an **upper bound on encoder travel to resolved external
motion**, including frame spacing, uncertainty threshold and dynamic response;
it is not a fitted deadband constant.

Angles use `R_camera_parent = R_camera_tag_parent R_parent_tag_parent.T`, and the
same expression for the child. Project
`R_camera_parent.T R_camera_child R_parent_child_zero.T` onto the measured parent
axis, rejecting residual rotation off that axis. Unwrap external angles over time.
Encoder samples interpolate only within exposure brackets. Angular uncertainty
adds both pose bounds, mount uncertainty, measured witness angular change and
speed times (clock uncertainty plus half encoder bracket spacing); systematic
terms are never divided by sample count. Treat `sigma_deg` as a conservative
single-sample bound; the report's 3-times comparisons are resolution guards,
not statistical confidence intervals. Compare median external-minus-encoder
residuals from both directions within encoder bins with at least three samples
per direction. Bin width also contributes to the reported difference bound.

## Frame references

OpenCV documents object-to-camera transforms and optical axes, plus the required
IPPE square corner ordering in its [PnP documentation](https://docs.opencv.org/4.11.0/d5/d1f/calib3d_solvePnP.html).
AprilRobotics documents measured tag size, pose estimation and its coordinate
convention in the [official AprilTag repository](https://github.com/AprilRobotics/apriltag).
Record the exact estimator convention and explicitly transform mounts to match it;
never mix OpenCV square axes and another library's tag axes implicitly.
