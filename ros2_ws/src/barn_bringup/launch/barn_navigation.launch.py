# Copyright 2026 barn-2027-prep contributors. MIT License.
#
# Future single entrypoint for our BARN navigation stack. Once an algorithm is
# ready, the official evaluator's documented launch_navigation_stack() hook may
# manually include this file. The evaluator package name and all simulation,
# goal, collision, timeout, and scoring behavior must remain unchanged.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EqualsSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare


def _mode_include(filename, mode_name):
    bringup = FindPackageShare('barn_bringup')
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup, 'launch', filename])),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'cmd_vel_type': LaunchConfiguration('cmd_vel_type'),
        }.items(),
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('mode'), mode_name)),
    )


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='classical',
            description='Navigation track: classical | classical_mpc | e2e_rl | hybrid'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the Gazebo /clock (must be true under the evaluator)'),
        DeclareLaunchArgument(
            'cmd_vel_type', default_value='twist_stamped',
            description='Final /cmd_vel message type: twist | twist_stamped'),
        DeclareLaunchArgument('planner_rviz', default_value='false'),

        _mode_include('slice_classical.launch.py', 'classical'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('barn_bringup'), 'launch', 'classical_mpc.launch.py'])),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'cmd_vel_type': LaunchConfiguration('cmd_vel_type'),
                'planner_rviz': LaunchConfiguration('planner_rviz'),
            }.items(),
            condition=IfCondition(EqualsSubstitution(
                LaunchConfiguration('mode'), 'classical_mpc'))),
        _mode_include('e2e_rl.launch.py', 'e2e_rl'),
        _mode_include('hybrid.launch.py', 'hybrid'),
    ])
