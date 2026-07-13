#!/usr/bin/env python3
"""Add this workspace's algorithm dispatcher to a fresh BARN evaluator clone."""

from __future__ import annotations

import argparse
from pathlib import Path


MARKER = "    # BARN_ALGO_DISPATCH_V3_BEGIN"


def replace_once(text: str, old: str, new: str, description: str) -> str:
    """Replace one known upstream fragment or report a useful compatibility error."""
    count = text.count(old)
    if count != 1:
        raise RuntimeError(
            f"expected one {description}, found {count}; upstream layout changed"
        )
    return text.replace(old, new, 1)


def _configure_arguments(text: str) -> str:
    world_argument = """    DeclareLaunchArgument('world_idx', default_value='0',
                          description='BARN World Index: [0-299].'),
"""
    algorithm_arguments = """    DeclareLaunchArgument('algo_type', default_value='builtin',
                          description='Navigation algorithm: builtin, movement_and_odom_test, or classical_mpc.'),
    DeclareLaunchArgument('planner_rviz', default_value='false', choices=['true', 'false'],
                          description='Start the custom planner RViz view.'),
    DeclareLaunchArgument('movement_duration', default_value='4.0',
                          description='Test planner forward-drive time in simulation seconds.'),
    DeclareLaunchArgument('forward_velocity', default_value='0.25',
                          description='Test planner forward velocity in m/s.'),
    DeclareLaunchArgument('rotation_speed', default_value='0.5',
                          description='Test planner rotation speed in rad/s.'),
"""

    algo_start = text.find("    DeclareLaunchArgument('algo_type'")
    if algo_start >= 0:
        world_start = text.find("    DeclareLaunchArgument('world_idx'", algo_start)
        if world_start < 0:
            raise RuntimeError('could not locate world_idx after algo_type')
        return text[:algo_start] + algorithm_arguments + text[world_start:]

    return replace_once(
        text,
        world_argument,
        algorithm_arguments + world_argument,
        'world_idx launch argument',
    )


def _navigation_function_prefix() -> str:
    return """def launch_navigation_stack(context, *args, **kwargs):

    # BARN_ALGO_DISPATCH_V3_BEGIN
    algo_type = LaunchConfiguration('algo_type').perform(context)
    supported_algorithms = ('builtin', 'movement_and_odom_test', 'classical_mpc')
    if algo_type not in supported_algorithms:
        return [
            LogInfo(msg=(
                f">>>>>>>>> Algorithm '{algo_type}' is a placeholder; "
                "no planner with that name has been connected yet."
            )),
            Shutdown(reason=f"Algorithm '{algo_type}' is not implemented"),
        ]
    # BARN_ALGO_DISPATCH_V3_END

    stack_exit_handlers = []
    if algo_type == 'builtin':
        nav2_launch_path = PathJoinSubstitution([
            get_package_share_directory('jackal_helper'),
            'launch',
            'nav2_bringup.launch.py',
        ])
        navigation_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_path]),
            launch_arguments=[
                ('use_sim_time', 'true'),
                ('setup_path', LaunchConfiguration('setup_path')),
                ('scan_topic', '/front/scan'),
                ('nav2_params_file', 'nav2.yaml'),
                ('log_level', 'WARN'),
            ]
        )
        stack_exit_handlers.append(RegisterEventHandler(
            OnProcessExit(
                target_action=lambda action: action.name == 'bt_navigator',
                on_exit=[Shutdown()]
            )
        ))
    elif algo_type == 'movement_and_odom_test':
        movement_launch_path = PathJoinSubstitution([
            get_package_share_directory('barn_movement_test'),
            'launch',
            'movement_and_odom_test.launch.py',
        ])
        navigation_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([movement_launch_path]),
            launch_arguments=[
                ('use_sim_time', 'true'),
                ('movement_duration', LaunchConfiguration('movement_duration')),
                ('forward_velocity', LaunchConfiguration('forward_velocity')),
                ('rotation_speed', LaunchConfiguration('rotation_speed')),
            ]
        )
    else:
        classical_launch_path = PathJoinSubstitution([
            get_package_share_directory('barn_bringup'),
            'launch',
            'classical_mpc.launch.py',
        ])
        navigation_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([classical_launch_path]),
            launch_arguments=[
                ('use_sim_time', 'true'),
                ('cmd_vel_type', 'twist_stamped'),
                ('planner_rviz', LaunchConfiguration('planner_rviz')),
            ]
        )

"""


def _configure_navigation_function(text: str) -> str:
    function_start = text.find("def launch_navigation_stack(context, *args, **kwargs):")
    goal_comment = text.find(
        "    # Get goal distance from world_idx and make a string to send to the "
        "/navigate_to_pose action server",
        function_start,
    )
    if function_start < 0 or goal_comment < 0:
        raise RuntimeError(
            'could not locate launch_navigation_stack boundaries; upstream layout changed'
        )

    text = text[:function_start] + _navigation_function_prefix() + text[goal_comment:]
    stock_return = "    return [nav2_launch, nav2_exit_handler, publish_goal]"
    configured_return = "    return [navigation_launch, *stack_exit_handlers, publish_goal]"
    if stock_return in text:
        text = replace_once(
            text, stock_return, configured_return, 'navigation action return')
    elif configured_return not in text:
        raise RuntimeError('could not locate navigation action return')
    return text.replace(
        'LogInfo(msg=">>>>>>>>> Publishing Nav2 goal... ")',
        'LogInfo(msg=">>>>>>>>> Publishing navigation goal... ")',
        1,
    )


def _configure_timer(text: str) -> str:
    generic_log = (
        'LogInfo(msg=[">>>>>>>>> Launching navigation algorithm: ", '
        "LaunchConfiguration('algo_type')])"
    )
    if generic_log in text:
        return text

    old_timer = (
        "    nav_stack = TimerAction(\n"
        "        period=15.0,\n"
        "        actions=[LogInfo(msg=\">>>>>>>>> Launching Nav2...\"), "
        "OpaqueFunction(function=launch_navigation_stack)]\n"
        "    )\n"
    )
    new_timer = (
        "    nav_stack = TimerAction(\n"
        "        period=15.0,\n"
        "        actions=[\n"
        "            LogInfo(msg=[\">>>>>>>>> Launching navigation algorithm: \", "
        "LaunchConfiguration('algo_type')]),\n"
        "            OpaqueFunction(function=launch_navigation_stack),\n"
        "        ]\n"
        "    )\n"
    )
    return replace_once(text, old_timer, new_timer, 'navigation timer')


def configure(path: Path) -> bool:
    """Configure one evaluator runner; return false when it is already current."""
    text = path.read_text()
    if MARKER in text:
        return False

    text = _configure_arguments(text)
    text = _configure_navigation_function(text)
    text = _configure_timer(text)
    path.write_text(text)
    return True


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('runner', type=Path)
    args = parser.parse_args()

    state = 'configured' if configure(args.runner) else 'already configured'
    print(f'[configure_barn_eval_dispatch] {state}: {args.runner}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
