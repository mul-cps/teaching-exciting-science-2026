import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    description_share = get_package_share_directory("so101_description")
    config_share = get_package_share_directory("so101_moveit_config")
    xacro_path = os.path.join(description_share, "urdf", "so101_arm.urdf.xacro")
    moveit_config = (
        MoveItConfigsBuilder("so101_arm", package_name="so101_moveit_config")
        .robot_description(
            file_path=xacro_path,
            mappings={"variant": "follower", "use_ros2_control": "false"},
        )
        .robot_description_semantic()
        .robot_description_kinematics()
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .pilz_cartesian_limits(file_path="config/pilz_cartesian_limits.yaml")
        .joint_limits(file_path=os.path.join(config_share, "config", "joint_limits.yaml"))
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml",
            moveit_manage_controllers=False,
        )
        .moveit_cpp(
            file_path=os.path.join(config_share, "config", "moveit_py_config.yaml")
        )
        .to_moveit_configs()
    )
    return LaunchDescription(
        [
            Node(
                name="so101_plan_only_smoke",
                package="so101_station_bringup",
                executable="plan_only_smoke",
                output="screen",
                parameters=[moveit_config.to_dict()],
            )
        ]
    )
