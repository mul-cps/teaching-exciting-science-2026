# Wrist flex model envelope correction

The trajectory helper and MoveIt override now use the existing official wrist
flex envelope **[-1.65806, +1.65806] radians** (approximately ±94.999840 degrees).
Pan, lift, elbow, wrist roll and all velocity/acceleration/jerk limits are unchanged.

This removes a narrower generic ±90 degree software cap. It does not change the
URDF, encoder mapping, calibration, firmware, physical geometry or runtime station
configuration. Execution still intersects the model envelope with the owner's
configured, calibrated and firmware admission; a narrower owner limit stays strict.
The IK model already uses the same ±1.65806 rad envelope.

## Evidence and provenance

- Source base: `1a6832a534a8d50c027bf05d53a1c5135cbec150`.
- Official model: `workshop/digital_twin/assets/so101_new_calib.urdf`, SHA256
  `3a65d2d35e68a8d2f0c2cc176d19b884506543c93ba72980145b80abe276022c`.
- Existing effective expanded model SHA256:
  `45532e4ae4c6b6ae2c67461f7a43bb1bdb20114ff5c2512ec967598118926ecc`.
- Existing follower calibration SHA256:
  `bc688147848f386899f8f34eb09cdb285fcdb0fc8de576b78a50a8efaf7f424c`.
  Wrist raw range820..3171, homing offset−947, midpoint1995.5 are retained.
- Existing mapping is `q=(raw-1995.5)*2*pi/4095`. Its intersection with the
  model admits integer raw **915..3076** (approximately ±94.989011 degrees).
  Raw914/3077 and calibration endpoints820/3171 remain outside this model envelope.
- The passive sweep recorded raw826..3172 with torque off and no wrap. Its lower
  endpoint pause was only0.421s, while the upper pause was45.252s. This supports
  checking the existing envelope but does not qualify new geometry or promote
  the approximately103 degree calibrated endpoints into a new model limit.

## Enforcement and validation

`ros_trajectory._position_limits` intersects owner-provided bounds with this model
cap; trajectory knots and analytic spline extrema retain their checks.
`prepare_supervised_calibration` uses that same helper. MoveIt inherits unchanged
motion dynamics through the existing YAML anchor. The owner also intersects
configuration, calibration and firmware raw endpoints before execution.
Runtime station configuration and regenerated MoveIt installation require separate
supervised deployment; these source changes alone do not widen owner admission.

Focused regressions cover model identity, inherited dynamics, the quantized edges,
continuous bounds, narrower owner admission, spline overshoot, and the supervised
preparation path. No hardware or ROS calls are needed for these tests.
