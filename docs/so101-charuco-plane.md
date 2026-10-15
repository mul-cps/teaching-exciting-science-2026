# ChArUco depth-plane world

`marker_pipeline` is the canonical source of the station world transform. It
synchronizes color, aligned-to-color depth, and `CameraInfo`, converts the color
frame to grayscale once, then uses the same frame for ChArUco and tag detection.
The standalone `charuco_world` node can still calculate an RGB PnP estimate for
diagnostics, but reports `plane_source: pnp_unqualified` and does not mark that
observation valid.

The combined pipeline projects the complete 175 mm by 125 mm ChArUco outer
rectangle from calibrated PnP, erodes its image hull slightly, and samples at
most 2,000 valid aligned-depth points. Depth is interpreted from ROS encoding
(`16UC1` millimetres or `32FC1` metres), deprojected with
`cv2.undistortPoints`, filtered for high-saturation color directly from the
original RGB/BGR message, then constrained to a 25 mm band around the PnP plane.
The color samples use a zero-copy NumPy view, so no second CvBridge conversion
is needed. This rejects vivid LEGO pieces and most raised objects before a deterministic
32-trial RANSAC. A 2.5 mm inlier threshold and SVD refinement recover the plane;
the fit must have at least 80 inliers over 8 of 12 board image regions.

The fitted camera-facing normal defines world +Z. World +X is the PnP board X
axis projected onto the plane, +Y is `Z × X`, and the PnP board origin is
projected onto the fitted plane. The node compares reprojection RMS before and
after this correction and refuses fits tilted more than 12 degrees from PnP or
with an inconsistent reprojection. Thus a missing or contaminated depth fit
cannot silently fall back to PnP in the canonical pipeline.

Status messages expose `plane_source`, inlier/sample counts, inlier ratio,
median and 95th-percentile residual in millimetres, coverage regions, normal,
vivid-color rejection count, PnP tilt, and reprojection RMS before and after correction. TF retains its
existing held-transform behavior during occlusion; the status distinguishes a
held last valid transform from a fresh depth-plane observation. These values
qualify the board plane only. Robot base registration remains a separate
calibration and qualification step.
