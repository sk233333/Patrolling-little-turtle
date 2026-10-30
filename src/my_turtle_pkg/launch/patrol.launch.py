import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PRESETS = {'slow': (0.5, 0.8), 'fast': (1.5, 2.0)}


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration('mode').perform(context)
    if mode not in PRESETS:
        raise RuntimeError(f'未知 mode: {mode}，可选 slow/fast')
    linear, angular = PRESETS[mode]

    share = get_package_share_directory('my_turtle_pkg')
    params = os.path.join(share, 'config', 'patrol_params.yaml')

    return [
        Node(package='turtlesim', executable='turtlesim_node',
             name='turtlesim', output='screen'),
        Node(package='my_turtle_pkg', executable='patrol_server',
             name='patrol_server', output='screen',
             parameters=[params,
                         {'linear_speed': linear,
                          'angular_speed': angular}]),
        Node(package='my_turtle_pkg', executable='patrol_client',
             name='patrol_client', output='screen',
             parameters=[params],
             condition=IfCondition(LaunchConfiguration('client'))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='slow',
                              description='slow=慢速预设, fast=快速预设'),
        DeclareLaunchArgument('client', default_value='true',
                              description='是否自动启动巡逻客户端'),
        OpaqueFunction(function=launch_setup),
    ])
