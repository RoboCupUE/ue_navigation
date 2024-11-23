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

    pkg_dir = get_package_share_directory('ue_navigation')
    map_server_params = os.path.join(pkg_dir, 'config/nav2', 'map_server_params.yaml')
    amcl_params = os.path.join(pkg_dir, 'config/nav2', 'amcl_params.yaml')
    controller_server_params = os.path.join(pkg_dir, 'config/nav2', 'controller_params.yaml')
    planner_params = os.path.join(pkg_dir, 'config/nav2', 'planner_params.yaml')
    bt_navigator_params = os.path.join(pkg_dir, 'config/nav2', 'bt_navigator_params.yaml')
    behavior_server_params = os.path.join(pkg_dir, 'config/nav2', 'behavior_server_params.yaml')

    remappings = [('/tf', 'tf'),
        ('/tf_static', 'tf_static')]
    
    lifecycle_nodes = ['map_server',
                       'amcl',
                       'controller_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator']
    
    nav2_cmd_group = GroupAction([
        Node(
            package='nav2_map_server',
            executable='map_server',
            output='screen',
            respawn_delay=2.0,
            parameters=[map_server_params],
            remappings=remappings),
        Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{'use_sim_time': False},
            {'autostart': True},
            {'node_names': lifecycle_nodes},
            {'attempt_respawn_reconnection': True}],
        remappings=remappings),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            respawn_delay=2.0,
            parameters=[amcl_params],
            remappings=remappings),
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            respawn_delay=2.0,
            parameters=[controller_server_params],
            remappings=remappings),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            respawn_delay=2.0,
            parameters=[planner_params],
            remappings=remappings),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            respawn_delay=2.0,
            parameters=[behavior_server_params],
            remappings=remappings),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            respawn_delay=2.0,
            parameters=[bt_navigator_params],
            remappings=remappings)
    ])
    
    ld.add_action(nav2_cmd_group)

    return ld