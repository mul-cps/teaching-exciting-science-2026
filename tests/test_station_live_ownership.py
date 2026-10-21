"""Source composition checks; no ROS installation or devices required."""
import ast
from pathlib import Path

SOURCE = Path(__file__).resolve().parents[1] / 'ros2/src/so101_station_bringup/launch/station_live.launch.py'


def test_integrated_launch_has_one_leader_and_forwards_contract():
    tree = ast.parse(SOURCE.read_text())
    assignments = {node.targets[0].id: node.value for node in ast.walk(tree)
                   if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name)}
    defaults = {key.value: ast.literal_eval(value) for key, value in
                zip(assignments['defaults'].keys, assignments['defaults'].values)
                if isinstance(key, ast.Constant) and key.value == 'joint_limit_contract'}
    assert defaults['joint_limit_contract'] == ''
    assert 'joint_limit_contract' in ast.literal_eval(assignments['robot_keys'])
    robot = assignments['robot']
    assert isinstance(robot, ast.Call)
    assert ast.literal_eval(robot.args[1]) == 'moveit_resident.launch.py'
    arguments = robot.args[2]
    assert isinstance(arguments, ast.Dict)
    assert any(isinstance(key, ast.Constant) and key.value == 'leader' and
               ast.literal_eval(value) == 'false' for key, value in zip(arguments.keys, arguments.values))
    leaders = [node for node in ast.walk(tree) if isinstance(node, ast.Call) and
               isinstance(node.func, ast.Name) and node.func.id == 'include' and
               len(node.args) > 1 and isinstance(node.args[1], ast.Constant) and
               node.args[1].value == 'leader_teleop.launch.py']
    assert len(leaders) == 1
