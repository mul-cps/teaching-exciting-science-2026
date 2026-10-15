"""Offline structural checks for the combined launch (no ROS graph/hardware)."""
import ast
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT / "ros2/src/so101_station_control/launch/leader_follower_teleop.launch.py"


def test_integrated_launch_starts_both_existing_launches_and_keeps_arm_opt_in():
    tree = ast.parse(LAUNCH.read_text())
    source = LAUNCH.read_text()
    assert 'DeclareLaunchArgument("enable_hardware", default_value="false"' in source
    assert '"moveit_resident.launch.py"' in source
    assert '"leader_teleop.launch.py"' in source
    assert '"external_state_only": "false"' in source
    assert '"station_python": station_python' in source
    assert '"station_root": station_root' in source
    assert any(isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
               and node.name == "generate_launch_description" for node in ast.walk(tree))
