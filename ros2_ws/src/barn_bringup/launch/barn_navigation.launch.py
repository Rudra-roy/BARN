# Copyright 2026 barn-2027-prep contributors. MIT License.
#
# Single entrypoint for the BARN navigation stack. The BARN evaluator's patched
# launch_navigation_stack() includes THIS file with `mode:=<track>`. The
# evaluator owns Gazebo, the Jackal spawn, the LiDAR remap, and all
# collision/goal/timeout monitoring; we own only the navigation nodes below.

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
            description='Navigation track: classical | e2e_rl | hybrid'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the Gazebo /clock (must be true under the evaluator)'),
        DeclareLaunchArgument(
            'cmd_vel_type', default_value='twist_stamped',
            description='Final /cmd_vel message type: twist | twist_stamped'),

        _mode_include('slice_classical.launch.py', 'classical'),
        _mode_include('e2e_rl.launch.py', 'e2e_rl'),
        _mode_include('hybrid.launch.py', 'hybrid'),
    ])
