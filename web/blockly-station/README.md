# Blockly station frontend

The local UI uses vendored Blockly and the bounded program engine in `workshop/blockly_program.py`. Start simulation with `./station sim`. An instructor can start the prepared station, RViz and physical Blockly with `./station start --physical --instructor`. See [startup and shutdown](../../docs/station-startup.md).

`tools/run_blockly_station.py` serves this directory on loopback port 8877. The frontend submits validated program data, not arbitrary Python execution. Physical commands reuse resident ROS services/actions; no browser component opens a motor or camera device.

The [public API reference](../../docs/ros-station-api.md) documents both `/station/program/command` and HTTP routes, recordings, generation checks and failure semantics. `examples/` contains importable bounded programs. Physical recording requires the existing owner and fixed jaw; replay retains the normal trajectory admission checks. Calibrated jaw gap commands use millimetres in program data and metres in the ROS action.

Third-party sources and licenses are recorded in [THIRD_PARTY.md](THIRD_PARTY.md). Keep vendored licenses with distributed learner bundles. Student instructions are in [the Blockly guide](../../docs/blockly-student.md).
