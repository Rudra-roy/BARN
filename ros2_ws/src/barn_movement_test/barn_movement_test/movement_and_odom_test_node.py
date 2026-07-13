"""Fixed-duration movement test exposed as a NavigateToPose action server.

This node deliberately does not read a laser scan or avoid obstacles. On each
goal it uses filtered odometry to rotate toward the goal, drives straight for a
configured amount of simulation time, stops, and completes the action.
"""

import math
import threading
import time

from geometry_msgs.msg import TwistStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from barn_movement_test.motion_math import (
    heading_to_goal,
    normalize_angle,
    quaternion_to_yaw,
)


class MovementAndOdomTest(Node):
    """Rotate toward a received goal, then drive straight for a fixed time."""

    def __init__(self):
        """Declare parameters and create the action, odometry, and command endpoints."""
        super().__init__('movement_and_odom_test')

        self.declare_parameter('action_name', '/navigate_to_pose')
        self.declare_parameter('odom_topic', '/platform/odom/filtered')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('cmd_frame', 'base_link')
        self.declare_parameter('movement_duration', 4.0)
        self.declare_parameter('forward_velocity', 0.25)
        self.declare_parameter('rotation_speed', 0.5)
        self.declare_parameter('heading_tolerance', 0.05)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('odom_wait_timeout', 10.0)
        self.declare_parameter('rotation_timeout', 30.0)

        self._movement_duration = self._positive_parameter('movement_duration', allow_zero=True)
        self._forward_velocity = self._positive_parameter('forward_velocity', allow_zero=True)
        self._rotation_speed = self._positive_parameter('rotation_speed')
        self._heading_tolerance = self._positive_parameter('heading_tolerance')
        self._control_rate_hz = self._positive_parameter('control_rate_hz')
        self._odom_wait_timeout = self._positive_parameter('odom_wait_timeout')
        self._rotation_timeout = self._positive_parameter('rotation_timeout')
        self._cmd_frame = self.get_parameter('cmd_frame').value

        self._odom = None
        self._odom_lock = threading.Lock()
        self._execution_lock = threading.Lock()

        odom_callbacks = MutuallyExclusiveCallbackGroup()
        action_callbacks = ReentrantCallbackGroup()
        self._cmd_pub = self.create_publisher(
            TwistStamped, self.get_parameter('cmd_vel_topic').value, 10)
        self._odom_sub = self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self._on_odom,
            20,
            callback_group=odom_callbacks,
        )
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            self.get_parameter('action_name').value,
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=action_callbacks,
        )

        self.get_logger().warning(
            'movement_and_odom_test is ready: NO obstacle avoidance; '
            f'forward={self._forward_velocity:.3f} m/s for '
            f'{self._movement_duration:.3f} s, rotation={self._rotation_speed:.3f} rad/s'
        )

    def _positive_parameter(self, name, allow_zero=False):
        value = float(self.get_parameter(name).value)
        minimum_ok = value >= 0.0 if allow_zero else value > 0.0
        if not math.isfinite(value) or not minimum_ok:
            comparison = 'non-negative' if allow_zero else 'greater than zero'
            raise ValueError(f"parameter '{name}' must be finite and {comparison}")
        return value

    def _on_odom(self, message):
        with self._odom_lock:
            self._odom = message

    def _odom_snapshot(self):
        with self._odom_lock:
            return self._odom

    def _on_goal(self, _goal_request):
        if self._execution_lock.locked():
            self.get_logger().warning('rejecting goal because a movement test is already active')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_cancel(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _publish_command(self, linear_x=0.0, angular_z=0.0):
        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = self._cmd_frame
        command.twist.linear.x = linear_x
        command.twist.angular.z = angular_z
        self._cmd_pub.publish(command)

    def _sleep_one_cycle(self):
        time.sleep(1.0 / self._control_rate_hz)

    def _wait_for_odom(self, goal_handle):
        deadline = time.monotonic() + self._odom_wait_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if goal_handle.is_cancel_requested or self._odom_snapshot() is not None:
                return self._odom_snapshot()
            self._sleep_one_cycle()
        return self._odom_snapshot()

    def _publish_feedback(self, goal_handle, odom, goal_x, goal_y, start_time):
        feedback = NavigateToPose.Feedback()
        feedback.current_pose.header = odom.header
        feedback.current_pose.pose = odom.pose.pose
        dx = goal_x - odom.pose.pose.position.x
        dy = goal_y - odom.pose.pose.position.y
        feedback.distance_remaining = math.hypot(dx, dy)
        feedback.navigation_time = (self.get_clock().now() - start_time).to_msg()
        goal_handle.publish_feedback(feedback)

    def _cancel_if_requested(self, goal_handle):
        if not goal_handle.is_cancel_requested:
            return False
        self._publish_command()
        goal_handle.canceled()
        self.get_logger().info('movement test canceled; stop command published')
        return True

    def _rotate_toward_goal(self, goal_handle, goal_x, goal_y, start_time):
        deadline = time.monotonic() + self._rotation_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if self._cancel_if_requested(goal_handle):
                return False

            odom = self._odom_snapshot()
            if odom is None:
                self._sleep_one_cycle()
                continue

            pose = odom.pose.pose
            orientation = pose.orientation
            current_yaw = quaternion_to_yaw(
                orientation.x, orientation.y, orientation.z, orientation.w)
            target_yaw = heading_to_goal(
                pose.position.x, pose.position.y, goal_x, goal_y)
            heading_error = normalize_angle(target_yaw - current_yaw)
            self._publish_feedback(goal_handle, odom, goal_x, goal_y, start_time)

            if abs(heading_error) <= self._heading_tolerance:
                self._publish_command()
                return True

            direction = 1.0 if heading_error > 0.0 else -1.0
            self._publish_command(angular_z=direction * self._rotation_speed)
            self._sleep_one_cycle()

        self._publish_command()
        self.get_logger().error('rotation timed out before reaching the goal heading')
        return False

    def _drive_for_duration(self, goal_handle, goal_x, goal_y, start_time):
        drive_start = self.get_clock().now()
        duration_ns = int(self._movement_duration * 1_000_000_000)
        while rclpy.ok() and (self.get_clock().now() - drive_start).nanoseconds < duration_ns:
            if self._cancel_if_requested(goal_handle):
                return False

            odom = self._odom_snapshot()
            if odom is not None:
                self._publish_feedback(goal_handle, odom, goal_x, goal_y, start_time)
            self._publish_command(linear_x=self._forward_velocity)
            self._sleep_one_cycle()

        self._publish_command()
        return True

    def _execute(self, goal_handle):
        with self._execution_lock:
            goal_pose = goal_handle.request.pose
            goal_x = goal_pose.pose.position.x
            goal_y = goal_pose.pose.position.y
            self.get_logger().info(
                f'received goal ({goal_x:.3f}, {goal_y:.3f}) in '
                f"frame '{goal_pose.header.frame_id}'"
            )

            start_time = self.get_clock().now()
            odom = self._wait_for_odom(goal_handle)
            if self._cancel_if_requested(goal_handle):
                return NavigateToPose.Result()
            if odom is None:
                self._publish_command()
                self.get_logger().error('no filtered odometry received; aborting movement test')
                goal_handle.abort()
                return NavigateToPose.Result()

            if not self._rotate_toward_goal(
                    goal_handle, goal_x, goal_y, start_time):
                if not goal_handle.is_cancel_requested:
                    goal_handle.abort()
                return NavigateToPose.Result()

            self.get_logger().info(
                f'heading aligned; driving at {self._forward_velocity:.3f} m/s '
                f'for {self._movement_duration:.3f} simulation seconds'
            )
            if not self._drive_for_duration(
                    goal_handle, goal_x, goal_y, start_time):
                return NavigateToPose.Result()

            goal_handle.succeed()
            self.get_logger().info('movement duration complete; stop command published')
            return NavigateToPose.Result()

    def destroy_node(self):
        """Stop the robot before releasing ROS entities."""
        if rclpy.ok():
            self._publish_command()
        self._action_server.destroy()
        return super().destroy_node()


def main(args=None):
    """Run the planner with enough executor threads to receive odometry while moving."""
    rclpy.init(args=args)
    node = MovementAndOdomTest()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
