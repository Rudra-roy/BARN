# Copyright 2026 barn-2027-prep contributors. MIT License.
#
# Standalone integration smoke test for the classical vertical slice. It does
# NOT need Gazebo or the evaluator: it launches goal_seeker + safety + robot
# adapter, feeds them a latched goal, a pose, and a clear scan, and asserts that
# a non-zero forward command reaches /cmd_vel.
#
# Run inside the distrobox after building + sourcing the overlay:
#     launch_test tests/integration/test_slice_smoke.py

import time
import unittest

import launch
import launch_testing.actions
import pytest
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from launch_ros.actions import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


@pytest.mark.launch_test
def generate_test_description():
    """Launch the three slice nodes that produce /cmd_vel."""
    common = [{'use_sim_time': False}]
    nodes = [
        Node(package='barn_classical', executable='goal_seeker_node',
             name='goal_seeker_node', parameters=common),
        Node(package='barn_safety', executable='safety_node',
             name='safety_node', parameters=common),
        Node(package='barn_robot_adapter', executable='robot_adapter_node',
             name='robot_adapter_node',
             parameters=common + [{'cmd_vel_type': 'twist_stamped'}]),
    ]
    return launch.LaunchDescription(nodes + [launch_testing.actions.ReadyToTest()]), {}


class TestSliceMotion(unittest.TestCase):
    """Assert the slice commands forward motion toward the goal."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('slice_smoke')
        self.max_forward = 0.0
        latched = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.goal_pub = self.node.create_publisher(PoseStamped, '/barn/goal', latched)
        self.pose_pub = self.node.create_publisher(PoseStamped, '/barn/pose', 10)
        self.scan_pub = self.node.create_publisher(LaserScan, '/barn/scan', 10)
        self.node.create_subscription(TwistStamped, '/cmd_vel', self._on_cmd, 10)

    def tearDown(self):
        self.node.destroy_node()

    def _on_cmd(self, msg):
        self.max_forward = max(self.max_forward, msg.twist.linear.x)

    def _publish_inputs(self):
        goal = PoseStamped()
        goal.header.frame_id = 'odom'
        goal.pose.position.x = 10.0
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)

        pose = PoseStamped()
        pose.header.frame_id = 'odom'
        pose.pose.orientation.w = 1.0  # at origin, facing +x (toward goal)
        self.pose_pub.publish(pose)

        scan = LaserScan()
        scan.angle_min = -1.57
        scan.angle_increment = 1.57 / 90.0
        scan.range_min = 0.1
        scan.range_max = 30.0
        scan.ranges = [10.0] * 180  # wide open, no obstacle
        self.scan_pub.publish(scan)

    def test_cmd_vel_moves_forward(self):
        """Within a few seconds a positive linear.x should appear on /cmd_vel."""
        deadline = time.time() + 8.0
        while time.time() < deadline and self.max_forward <= 0.05:
            self._publish_inputs()
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertGreater(
            self.max_forward, 0.05,
            'slice did not command forward motion on /cmd_vel')
