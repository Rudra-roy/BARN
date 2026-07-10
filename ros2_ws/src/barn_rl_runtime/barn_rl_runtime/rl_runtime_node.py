"""End-to-end RL policy runtime node (Track B).

Subscribes the latched goal, the robot pose, and the relayed scan; builds the
observation, runs the (CPU) policy, scales the action, and publishes a desired
velocity command for barn_safety. With the stub model this publishes zero motion
until a trained policy is exported.

The policy receives ONLY allowed inputs (LiDAR/goal/velocity/previous action) —
never privileged simulator information.
"""

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan

from barn_rl_runtime.action_scale import scale_action
from barn_rl_runtime.model_loader import PolicyModel
from barn_rl_runtime.normalization import Normalizer
from barn_rl_runtime.observation import build_observation


class RlRuntimeNode(Node):
    """Runs a policy at a fixed rate and publishes /barn/cmd_desired."""

    def __init__(self):
        """Declare parameters, load the model, and wire pub/sub/timer."""
        super().__init__('rl_runtime_node')

        self.declare_parameter('goal_topic', '/barn/goal')
        self.declare_parameter('pose_topic', '/barn/pose')
        self.declare_parameter('scan_topic', '/barn/scan')
        self.declare_parameter('cmd_topic', '/barn/cmd_desired')
        self.declare_parameter('cmd_frame', 'base_link')
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('model_path', '')
        self.declare_parameter('v_max', 2.0)
        self.declare_parameter('w_max', 1.5)

        self._cmd_frame = self.get_parameter('cmd_frame').value
        self._v_max = self.get_parameter('v_max').value
        self._w_max = self.get_parameter('w_max').value

        self._model = PolicyModel(self.get_parameter('model_path').value or None)
        self._normalizer = Normalizer()

        self._goal = None
        self._pose = None
        self._scan = None
        self._prev_action = (0.0, 0.0)

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._cmd_pub = self.create_publisher(
            TwistStamped, self.get_parameter('cmd_topic').value, 10)
        self.create_subscription(
            PoseStamped, self.get_parameter('goal_topic').value, self._on_goal, latched)
        self.create_subscription(
            PoseStamped, self.get_parameter('pose_topic').value, self._on_pose, 10)
        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value, self._on_scan,
            qos_profile_sensor_data)

        rate = self.get_parameter('control_rate_hz').value
        self.create_timer(1.0 / rate, self._step)
        self.get_logger().info('rl_runtime_node ready (STUB policy)')

    def _on_goal(self, msg):
        self._goal = msg

    def _on_pose(self, msg):
        self._pose = msg

    def _on_scan(self, msg):
        self._scan = msg

    def _step(self):
        """Run one inference tick and publish the scaled command."""
        if self._goal is None or self._pose is None:
            return
        ranges = list(self._scan.ranges) if self._scan is not None else []
        goal_rel = (
            self._goal.pose.position.x - self._pose.pose.position.x,
            self._goal.pose.position.y - self._pose.pose.position.y,
        )
        obs = build_observation(ranges, goal_rel, (0.0, 0.0), self._prev_action)
        obs = self._normalizer.apply(obs)
        action = self._model.infer(obs)
        v, w = scale_action(action, v_min=0.0, v_max=self._v_max, w_max=self._w_max)
        self._prev_action = (v, w)

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = self._cmd_frame
        cmd.twist.linear.x = v
        cmd.twist.angular.z = w
        self._cmd_pub.publish(cmd)


def main(args=None):
    """Entry point for `ros2 run barn_rl_runtime rl_runtime_node`."""
    rclpy.init(args=args)
    node = RlRuntimeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
