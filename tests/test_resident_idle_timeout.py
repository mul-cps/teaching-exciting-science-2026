"""Hardware-free resident timeout default and command propagation checks."""
import ast
import math
from pathlib import Path
import pytest
SOURCE = Path(__file__).resolve().parents[1] / "ros2/src/so101_station_control/so101_station_control/trajectory_server.py"

def test_resident_idle_default_is_one_hour_and_propagates_to_owner():
    tree = ast.parse(SOURCE.read_text())
    values = [call.args[1].value for call in ast.walk(tree)
              if isinstance(call, ast.Call) and isinstance(call.func, ast.Attribute)
              and call.func.attr == "declare_parameter" and len(call.args) >= 2
              and isinstance(call.args[0], ast.Constant)
              and call.args[0].value == "owner_idle_timeout_s"]
    assert values == [3600.0]
    function = next(node for node in tree.body if isinstance(node, ast.FunctionDef)
                    and node.name == "_arm_session_command")
    namespace = {"math": math}
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(SOURCE), "exec"), namespace)
    command = namespace["_arm_session_command"]("python", "config", "state", "logs", "lock", values[0])
    assert command[command.index("--idle-timeout") + 1] == "3600.0"
    assert "--persistent-idle" in command
    assert "--recover-sag" not in command

@pytest.mark.parametrize("duration", [0, -1, float("nan"), float("inf")])
def test_invalid_idle_duration_still_rejected(duration):
    tree = ast.parse(SOURCE.read_text())
    function = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "_arm_session_command")
    namespace = {"math": math}
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(SOURCE), "exec"), namespace)
    with pytest.raises(ValueError):
        namespace["_arm_session_command"]("python", "config", "state", "logs", "lock", duration)
