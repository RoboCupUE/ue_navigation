import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml, ReplaceString

def generate_launch_description():
    ld = LaunchDescription()

    pkg_dir = get_package_share_directory('tb3_slam')
    slam_dir = get_package_share_directory('slam_toolbox')
    slam_params_file = os.path.join(pkg_dir, 'config/slam', 'slam_params.yaml')

    remappings = [('/tf', 'tf'),
        ('/tf_static', 'tf_static')]

    slam_cmd_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(slam_dir, 'launch', 'online_sync_launch.py')),
            launch_arguments={
                'params_file': slam_params_file
            }.items()
        )
    ])
    ld.add_action(slam_cmd_group)

    return ld