# Copyright 2026 barn-2027-prep contributors. MIT License.
#
# Track A vertical slice: goal adapter + robot adapter + goal-seeker + safety.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = get_package_share_directory('barn_bringup')
    common = os.path.join(cfg, 'config', 'common.yaml')
    params = os.path.join(cfg, 'config', 'classical.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    cmd_vel_type = LaunchConfiguration('cmd_vel_type')
    sim = {'use_sim_time': use_sim_time}

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('cmd_vel_type', default_value='twist_stamped'),

        Node(
            package='barn_goal_adapter', executable='goal_adapter_node',
            name='goal_adapter_node', output='screen',
            parameters=[common, params, sim]),
        Node(
            package='barn_robot_adapter', executable='robot_adapter_node',
            name='robot_adapter_node', output='screen',
            parameters=[common, params, sim, {'cmd_vel_type': cmd_vel_type}]),
        Node(
            package='barn_classical', executable='goal_seeker_node',
            name='goal_seeker_node', output='screen',
            parameters=[common, params, sim]),
        Node(
            package='barn_safety', executable='safety_node',
            name='safety_node', output='screen',
            parameters=[common, params, sim]),
    ])
