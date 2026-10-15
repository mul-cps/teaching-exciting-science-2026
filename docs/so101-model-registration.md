# Offline SO-101 model registration

`tools/fit_ros_robot_joint_offsets.py` fits a reviewable candidate from saved
ROS RGB-D bundles. It estimates one common bounded six-degree base correction
and selected URDF joint-zero offsets, with `shoulder_lift`, `elbow_flex`, and
`wrist_flex` selected by default. The shoulder-pan zero stays fixed because it
cannot be separated from base yaw in this fit. The input URDF's CAD meshes are
sampled once per snapshot and kept in each link's local frame; forward
kinematics moves those cached points as the optimizer changes joint offsets.

Provide at least four bundle directories: three or more training poses and one
held-out pose. By default the last bundle is held out. Training snapshots must
excite at least two selected joints by more than three degrees and span two
independent motion directions. The held-out pose must differ from the training
poses. Repeated-pose captures are rejected before fitting. Use a different
held-out index with `--holdout-index` when the final bundle is not the desired
holdout.

For example:

```bash
python tools/fit_ros_robot_joint_offsets.py \
  /path/to/baseline /path/to/lift /path/to/elbow /path/to/wrist /path/to/heldout \
  --output /tmp/so101-registration-candidate
```

The tool writes `candidate.json` and a side-by-side `heldout-registration.png`
to the requested output directory. Scores include fixed observed-to-CAD ROI
residuals and visible-CAD-to-depth matching coverage and residuals, each with
per-link detail for training and held-out snapshots. Observed ROI populations
are selected from the initial URDF z-buffer and optional tag-pose seed, then
remain fixed during fitting. The holdout is only scored; it is never optimized.
The visible-CAD direction reports both a loose 60 mm residual population and
the fraction of projected front-face CAD samples with depth within 10 mm.
Parameter standard errors come from a local linearized fit and are diagnostic
only; the residual model does not capture correlated pixel noise, mesh error,
or calibration uncertainty. The report also lists any parameter bound hits.

The output always remains an unqualified candidate. It does not update ROS TF,
write into input bundles, start ROS, or move hardware. Review residuals,
coverage, per-link overlays, pose provenance, and calibration assumptions
before any separate promotion process. Base yaw and shoulder-pan zero remain a
gauge pair, so the pan zero is held at nominal. The station's gripper angle is
currently derived from an opening-percent proxy rather than a calibrated
encoder-angle mapping; jaw residuals at that aperture do not calibrate the
gripper linkage or jaw zero.
