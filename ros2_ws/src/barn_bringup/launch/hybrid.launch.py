# Copyright 2026 barn-2027-prep contributors. MIT License.
#
# Track C: classical goal-seeker (nominal) + RL runtime (residual) + hybrid
# arbiter + safety, plus the dynamic-obstacle tracker. In static worlds the
# arbiter gate is 0, so behaviour matches the classical slice.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = get_package_share_directory('barn_bringup')
    common = os.path.join(cfg, 'config', 'common.yaml')
    params = os.path.join(cfg, 'config', 'hybrid.yaml')

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
            package='barn_rl_runtime', executable='rl_runtime_node',
            name='rl_runtime_node', output='screen',
            parameters=[common, params, sim]),
        Node(
            package='barn_dynamic_tracking', executable='tracker_node',
            name='tracker_node', output='screen',
            parameters=[common, params, sim]),
        Node(
            package='barn_hybrid', executable='hybrid_node',
            name='hybrid_node', output='screen',
            parameters=[common, params, sim]),
        Node(
            package='barn_safety', executable='safety_node',
            name='safety_node', output='screen',
            parameters=[common, params, sim]),
    ])
