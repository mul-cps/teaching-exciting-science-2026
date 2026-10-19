"""Default-off read-only stationary proposals; no browser or control dependency."""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from so101_station_perception.derived_lineage import configure_ros_domain


def generate_launch_description():
    configure_ros_domain(os.environ)
    options={'enabled':('false',bool),'source_cohort':('',str),'owner_state_file':('',str),
             'allow_derived_source':('false',bool),'cloud_topic':('/camera/depth/color/points',str),
             'camera_info_topic':('/camera/depth/camera_info',str),'lineage_topic':('/so101_perception/rocm/lineage',str),
             'marker_mount':('',str),'max_samples':('4000',int),'fit_budget_s':('0.2',float)}
    return LaunchDescription([
        *[DeclareLaunchArgument(key,default_value=value[0]) for key,value in options.items()],
        Node(package='so101_station_perception',executable='periodic_registration',
             condition=IfCondition(LaunchConfiguration('enabled')),output='screen',
             parameters=[{key:ParameterValue(LaunchConfiguration(key),value_type=value[1])
                          for key,value in options.items()}],
             additional_env={'ROS_DOMAIN_ID':os.environ.get('ROS_DOMAIN_ID','76'),
                             'ROS_LOCALHOST_ONLY':os.environ.get('ROS_LOCALHOST_ONLY','1'),
                             'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1'})])
