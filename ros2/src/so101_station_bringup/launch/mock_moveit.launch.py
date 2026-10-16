from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    upstream = PathJoinSubstitution(
        [FindPackageShare("so101_bringup"), "launch", "follower_moveit_demo.launch.py"]
    )
    return LaunchDescription(
        [
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="world_to_base_link",
                arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(upstream),
                launch_arguments={
                    "hardware_type": "mock",
                    "namespace": "follower",
                    "joint_config_file": "",
                    "use_cameras": "false",
                    "use_rviz": "false",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare("so101_station_bringup"),
                        "launch", "guarded_moveit_rviz.launch.py",
                    ])
                ),
                launch_arguments={"namespace": "follower", "variant": "follower"}.items(),
            ),
        ]
    )
