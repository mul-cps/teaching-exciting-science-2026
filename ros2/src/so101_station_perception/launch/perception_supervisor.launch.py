"""Default-off metadata-only automatic perception recovery client."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arguments=[DeclareLaunchArgument('enabled',default_value='false')]
    parameters={'enabled':ParameterValue(LaunchConfiguration('enabled'),value_type=bool)}
    for name,default in (('expected_source_cohort',''),('expected_serial','243322071783'),
                         ('journal_directory',''),('recovery_artifact_directory','')):
        arguments.append(DeclareLaunchArgument(name,default_value=default))
        parameters[name]=ParameterValue(LaunchConfiguration(name),value_type=str)
    return LaunchDescription(arguments+[Node(package='so101_station_perception',executable='perception_supervisor',
        condition=IfCondition(LaunchConfiguration('enabled')),parameters=[parameters],output='screen')])
