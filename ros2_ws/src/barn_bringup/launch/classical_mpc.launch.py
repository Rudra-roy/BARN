"""Static BARN footprint-aware mapping, MPC navigation, and final safety shield."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup = get_package_share_directory('barn_bringup')
    tracking_cfg = os.path.join(
        get_package_share_directory('barn_dynamic_tracking'), 'config', 'tracking.yaml')
    common = os.path.join(bringup, 'config', 'common.yaml')
    parameters = os.path.join(bringup, 'config', 'classical_mpc.yaml')
    rviz_config = os.path.join(bringup, 'rviz', 'barn.rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    cmd_vel_type = LaunchConfiguration('cmd_vel_type')
    planner_rviz = LaunchConfiguration('planner_rviz')
    enable_tracking = LaunchConfiguration('enable_tracking')
    sim = {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('cmd_vel_type', default_value='twist_stamped'),
        DeclareLaunchArgument('planner_rviz', default_value='false', choices=['true', 'false']),
        # Dynamic-obstacle tracker (DynaBARN). Off leaves /barn/tracks unpublished,
        # so the MPC transparently falls back to static-only behavior.
        DeclareLaunchArgument('enable_tracking', default_value='true',
                              choices=['true', 'false']),

        Node(
            package='barn_goal_adapter', executable='goal_adapter_node',
            name='goal_adapter_node', output='screen',
            parameters=[common, parameters, sim]),
        Node(
            package='barn_robot_adapter', executable='robot_adapter_node',
            name='robot_adapter_node', output='screen',
            parameters=[common, parameters, sim, {'cmd_vel_type': cmd_vel_type}]),
        Node(
            package='barn_mapping', executable='mapping_node',
            name='mapping_node', output='screen',
            parameters=[common, parameters, sim]),
        Node(
            package='barn_classical', executable='classical_mpc_node',
            name='classical_mpc_node', output='screen',
            parameters=[common, parameters, sim]),
        Node(
            package='barn_safety', executable='safety_node',
            name='safety_node', output='screen',
            parameters=[common, parameters, sim]),
        Node(
            package='barn_dynamic_tracking', executable='tracker_node',
            name='tracker_node', output='screen',
            parameters=[common, tracking_cfg, sim],
            condition=IfCondition(enable_tracking)),
        Node(
            package='rviz2', executable='rviz2', name='barn_planner_rviz',
            output='log', arguments=['-d', rviz_config], parameters=[sim],
            condition=IfCondition(planner_rviz)),
    ])
