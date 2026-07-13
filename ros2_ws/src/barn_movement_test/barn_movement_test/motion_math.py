"""ROS-independent planar motion helpers."""

import math


def normalize_angle(angle):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(x, y, z, w):
    """Return planar yaw from a quaternion."""
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(sin_yaw, cos_yaw)


def heading_to_goal(current_x, current_y, goal_x, goal_y):
    """Return the world-frame heading from the current pose to the goal."""
    return math.atan2(goal_y - current_y, goal_x - current_x)
