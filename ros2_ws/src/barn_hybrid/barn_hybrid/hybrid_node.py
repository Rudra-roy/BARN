"""Hybrid arbiter node (Track C).

Subscribes the classical command and the RL residual, computes the dynamic-risk
gate, fuses them, and publishes the desired command for barn_safety. In static
worlds the gate is 0, so the output equals the classical command and static
navigation performance is preserved.

This node is NOT the real-time authority: barn_safety (C++) still clamps the
final command. That is why the arbiter is written in Python (fast to iterate).
"""

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node

from barn_hybrid.fusion import fuse
from barn_hybrid.risk_gate import RiskGate


class HybridNode(Node):
    """Fuses classical + RL commands under a dynamic-risk gate."""

    def __init__(self):
        """Declare parameters and wire pub/sub/timer."""
        super().__init__('hybrid_node')

        self.declare_parameter('classical_topic', '/barn/cmd_classical')
        self.declare_parameter('rl_topic', '/barn/cmd_rl')
        self.declare_parameter('cmd_topic', '/barn/cmd_desired')
        self.declare_parameter('cmd_frame', 'base_link')
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('ttc_full', 1.0)
        self.declare_parameter('ttc_zero', 3.0)

        self._cmd_frame = self.get_parameter('cmd_frame').value
        self._gate = RiskGate(
            self.get_parameter('ttc_full').value, self.get_parameter('ttc_zero').value)

        self._classical = (0.0, 0.0)
        self._rl = (0.0, 0.0)
        self._min_ttc = float('inf')  # updated from the tracker when implemented

        self._cmd_pub = self.create_publisher(
            TwistStamped, self.get_parameter('cmd_topic').value, 10)
        self.create_subscription(
            TwistStamped, self.get_parameter('classical_topic').value, self._on_classical, 10)
        self.create_subscription(
            TwistStamped, self.get_parameter('rl_topic').value, self._on_rl, 10)

        rate = self.get_parameter('control_rate_hz').value
        self.create_timer(1.0 / rate, self._step)
        self.get_logger().info('hybrid_node ready (alpha=0 in static worlds)')

    def _on_classical(self, msg):
        self._classical = (msg.twist.linear.x, msg.twist.angular.z)

    def _on_rl(self, msg):
        self._rl = (msg.twist.linear.x, msg.twist.angular.z)

    def _step(self):
        """Fuse the latest commands and publish."""
        alpha = self._gate.alpha(self._min_ttc)
        residual = (self._rl[0] - self._classical[0], self._rl[1] - self._classical[1])
        v, w = fuse(self._classical, residual, alpha)

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = self._cmd_frame
        cmd.twist.linear.x = v
        cmd.twist.angular.z = w
        self._cmd_pub.publish(cmd)


def main(args=None):
    """Entry point for `ros2 run barn_hybrid hybrid_node`."""
    rclpy.init(args=args)
    node = HybridNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
