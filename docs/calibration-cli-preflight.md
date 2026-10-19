# Calibration CLI preflight

`tools/ros_plan_current.py --execute --calibration` reads
`<namespace>/trajectory_server/get_parameters` and requires
`calibration_endpoint_mode` to be a typed ROS Boolean `true` before requesting
a MoveIt plan or submitting an action. The default namespace is `/follower`.

Missing, false, incorrectly typed, unavailable, or timed-out parameter results
reject the command. Discovery and the parameter response share the existing
`--timeout` deadline with initial joint-state acquisition and planning.

`--calibration` recognizes the resident bridge's measured-endpoint calibration
result; it does not enable that mode. This preflight never changes ROS
parameters, operating limits, commissioning gates, or a failed endpoint hold.
Plan-only commands and execution without `--calibration` do not query this
parameter. A successful read is admission evidence at that instant; the
resident owner remains responsible for execution admission.
