"""DynaBARN variant: the unmodified classical MPC stack + the moving-obstacle
tracker feeding /barn/tracks. The static classical_mpc.launch.py is included
verbatim so the competition stack stays untouched; this file only adds the
tracker on top. Use for dynamic (DynaBARN) worlds; the plain classical_mpc
launch remains the stable static/competition entrypoint.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup = get_package_share_directory('barn_bringup')
    common = os.path.join(bringup, 'config', 'common.yaml')
    tracking_cfg = os.path.join(
        get_package_share_directory('barn_dynamic_tracking'), 'config', 'tracking.yaml')
    static_launch = os.path.join(bringup, 'launch', 'classical_mpc.launch.py')
    use_sim_time = LaunchConfiguration('use_sim_time')
    cmd_vel_type = LaunchConfiguration('cmd_vel_type')
    planner_rviz = LaunchConfiguration('planner_rviz')
    sim = {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('cmd_vel_type', default_value='twist_stamped'),
        DeclareLaunchArgument('planner_rviz', default_value='false', choices=['true', 'false']),

        # The full, unmodified competition stack (goal/robot adapters, mapping,
        # MPC, safety shield, optional RViz).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([static_launch]),
            launch_arguments=[
                ('use_sim_time', use_sim_time),
                ('cmd_vel_type', cmd_vel_type),
                ('planner_rviz', planner_rviz),
            ]),
        # Moving-obstacle tracker. output_frame is forced to 'map' to match the
        # MPC planning frame (robot_adapter already publishes /barn/pose in map),
        # so track positions and the MPC's constraints share one frame.
        Node(
            package='barn_dynamic_tracking', executable='tracker_node',
            name='tracker_node', output='screen',
            parameters=[common, tracking_cfg, sim, {'output_frame': 'map'}]),
    ])
