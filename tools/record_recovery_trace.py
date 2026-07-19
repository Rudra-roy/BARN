#!/usr/bin/env python3
# Copyright 2026 barn-2027-prep contributors. MIT License.
"""Record a recovery-focused navigation trace to JSONL (run INSIDE the distrobox).

This is a standalone rclpy node — it needs no colcon build and no changes to the
navigation packages. Run it in parallel with a live run (standalone slice or
under the evaluator); it subscribes to the internal topics, samples them at a
fixed rate, and writes one time-aligned JSON object per tick.

The resulting ``recovery_trace.jsonl`` is plain text with NO ROS dependency, so
it can be handed to an offline analyzer (``tools/analyze_recovery_trace.py``) or
read directly. One row per sample; fields are the latest value seen on each
topic at sample time (None until the first message arrives).

Usage (in the distrobox, workspace sourced):
    python3 tools/record_recovery_trace.py                       # -> recovery_trace.jsonl
    python3 tools/record_recovery_trace.py -o results/run12.jsonl --rate 30

Topics consumed (all optional; missing ones simply stay None):
    /barn/navigation_diagnostics  diagnostic_msgs/DiagnosticArray  (classical + safety)
    /barn/pose                    geometry_msgs/PoseStamped
    /barn/goal                    geometry_msgs/PoseStamped        (latched)
    /barn/cmd_desired             geometry_msgs/TwistStamped
    /barn/cmd_safe                geometry_msgs/TwistStamped
    /barn/safety_veto             std_msgs/Bool
    /barn/scan                    sensor_msgs/LaserScan
"""

import argparse
import json
import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


def yaw_from_quat(q):
    """Planar yaw (rad) from a geometry_msgs Quaternion."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def sector_min(scan, lo, hi):
    """Minimum finite range (m) over [lo, hi] radians, or None if empty."""
    if scan is None:
        return None
    best = None
    for i, r in enumerate(scan.ranges):
        if not math.isfinite(r) or r < scan.range_min or r > scan.range_max:
            continue
        angle = scan.angle_min + i * scan.angle_increment
        # wrap into [-pi, pi] for a stable comparison
        a = math.atan2(math.sin(angle), math.cos(angle))
        if lo <= a <= hi and (best is None or r < best):
            best = r
    return best


class RecoveryTraceRecorder(Node):
    """Samples internal navigation topics and writes one JSONL row per tick."""

    def __init__(self, out_path, rate_hz):
        super().__init__('recovery_trace_recorder')
        self._diag = {}          # {status.name: {key: value}}
        self._pose = None
        self._goal = None
        self._cmd_desired = None
        self._cmd_safe = None
        self._veto = None
        self._scan = None
        self._t0 = time.monotonic()
        self._count = 0
        self._file = open(out_path, 'w')
        self._out_path = out_path

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        best_effort = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(
            DiagnosticArray, '/barn/navigation_diagnostics', self._on_diag, 10)
        self.create_subscription(PoseStamped, '/barn/pose', self._on_pose, 10)
        self.create_subscription(PoseStamped, '/barn/goal', self._on_goal, latched)
        self.create_subscription(
            TwistStamped, '/barn/cmd_desired', self._on_desired, 10)
        self.create_subscription(TwistStamped, '/barn/cmd_safe', self._on_safe, 10)
        self.create_subscription(Bool, '/barn/safety_veto', self._on_veto, best_effort)
        self.create_subscription(
            LaserScan, '/barn/scan', self._on_scan, qos_profile_sensor_data)

        self.create_timer(1.0 / rate_hz, self._sample)
        self.get_logger().info(
            f'recording recovery trace -> {out_path} at {rate_hz:g} Hz '
            '(Ctrl-C to stop)')

    @staticmethod
    def _level_int(level):
        """DiagnosticStatus.level is a byte field; rclpy hands it back as bytes."""
        if isinstance(level, (bytes, bytearray)):
            return int.from_bytes(level, 'little') if level else 0
        return int(level)

    def _on_diag(self, msg):
        for status in msg.status:
            self._diag[status.name] = {kv.key: kv.value for kv in status.values}
            # message field carries the human-readable control/veto status
            self._diag[status.name]['_message'] = status.message
            self._diag[status.name]['_level'] = self._level_int(status.level)

    def _on_pose(self, msg):
        self._pose = msg

    def _on_goal(self, msg):
        self._goal = msg

    def _on_desired(self, msg):
        self._cmd_desired = msg

    def _on_safe(self, msg):
        self._cmd_safe = msg

    def _on_veto(self, msg):
        self._veto = bool(msg.data)

    def _on_scan(self, msg):
        self._scan = msg

    @staticmethod
    def _fnum(d, key):
        """Parse a diagnostic value as float, tolerating missing/garbage."""
        if d is None or key not in d:
            return None
        try:
            return float(d[key])
        except (TypeError, ValueError):
            return None

    def _sample(self):
        classical = self._diag.get('barn/classical_mpc', {})
        safety = self._diag.get('barn/safety_shield', {})

        row = {
            't': round(time.monotonic() - self._t0, 4),
            # --- classical control / recovery state ---
            'control_status': classical.get('_message'),
            'planner_status': classical.get('planner_status'),
            'recovery_state': self._fnum(classical, 'recovery_state'),
            'recovery_attempts': self._fnum(classical, 'recovery_attempts'),
            'mpc_ms': self._fnum(classical, 'mpc_ms'),
            'planner_ms': self._fnum(classical, 'planner_ms'),
            'clearance_m': self._fnum(classical, 'clearance_m'),
            'cmd_v': self._fnum(classical, 'selected_speed'),
            'cmd_w': self._fnum(classical, 'selected_yaw_rate'),
            # --- safety shield ---
            'safety_reason': safety.get('safety_veto_reason'),
            'safety_scale': self._fnum(safety, 'command_scale'),
            'safety_clearance': self._fnum(safety, 'minimum_clearance'),
            'veto': self._veto,
        }

        if self._pose is not None:
            p = self._pose.pose
            row['pose_x'] = round(p.position.x, 4)
            row['pose_y'] = round(p.position.y, 4)
            row['pose_yaw'] = round(yaw_from_quat(p.orientation), 5)
        if self._goal is not None:
            g = self._goal.pose.position
            row['goal_x'] = round(g.x, 4)
            row['goal_y'] = round(g.y, 4)
            if self._pose is not None:
                row['goal_dist'] = round(
                    math.hypot(g.x - self._pose.pose.position.x,
                               g.y - self._pose.pose.position.y), 4)
        if self._cmd_desired is not None:
            row['desired_v'] = round(self._cmd_desired.twist.linear.x, 4)
            row['desired_w'] = round(self._cmd_desired.twist.angular.z, 4)
        if self._cmd_safe is not None:
            row['safe_v'] = round(self._cmd_safe.twist.linear.x, 4)
            row['safe_w'] = round(self._cmd_safe.twist.angular.z, 4)
        if self._scan is not None:
            # Coarse clearance sectors relative to base_link: front, sides, rear.
            row['front_m'] = sector_min(self._scan, -0.35, 0.35)
            row['left_m'] = sector_min(self._scan, 0.35, 1.40)
            row['right_m'] = sector_min(self._scan, -1.40, -0.35)
            rear = sector_min(self._scan, math.pi - 0.45, math.pi)
            rear2 = sector_min(self._scan, -math.pi, -math.pi + 0.45)
            row['rear_m'] = min([v for v in (rear, rear2) if v is not None],
                                default=None)

        self._file.write(json.dumps(row) + '\n')
        self._file.flush()
        self._count += 1

    def close(self):
        self._file.close()
        self.get_logger().info(
            f'wrote {self._count} samples to {self._out_path}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('-o', '--out', default='recovery_trace.jsonl',
                        help='output JSONL path (default: recovery_trace.jsonl)')
    parser.add_argument('--rate', type=float, default=20.0,
                        help='sampling rate in Hz (default: 20)')
    args = parser.parse_args()

    rclpy.init()
    node = RecoveryTraceRecorder(args.out, args.rate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
