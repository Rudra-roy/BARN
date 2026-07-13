"""Launch the movement_and_odom_test planner."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='true'),
    DeclareLaunchArgument('movement_duration', default_value='4.0'),
    DeclareLaunchArgument('forward_velocity', default_value='0.25'),
    DeclareLaunchArgument('rotation_speed', default_value='0.5'),
]


def generate_launch_description():
    """Create the fixed-duration movement test node."""
    planner = Node(
        package='barn_movement_test',
        executable='movement_and_odom_test',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': ParameterValue(
                LaunchConfiguration('use_sim_time'), value_type=bool),
            'movement_duration': ParameterValue(
                LaunchConfiguration('movement_duration'), value_type=float),
            'forward_velocity': ParameterValue(
                LaunchConfiguration('forward_velocity'), value_type=float),
            'rotation_speed': ParameterValue(
                LaunchConfiguration('rotation_speed'), value_type=float),
        }],
    )
    return LaunchDescription(ARGUMENTS + [planner])
