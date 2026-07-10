"""Build the policy observation from allowed sensor inputs.

STUB. The observation must NOT contain privileged information (world index,
Gazebo poses, reference path). See docs/benchmark/barn_2026_contract.md and
docs/architecture/e2e_rl.md for the frozen observation contract.
"""

DEFAULT_LIDAR_BEAMS = 720
# downsampled lidar + [goal_dist, sin(bearing), cos(bearing), v, w, prev_v, prev_w]
DEFAULT_OBS_DIM = DEFAULT_LIDAR_BEAMS + 7


def build_observation(scan_ranges, goal_rel, velocity, prev_action, num_beams=DEFAULT_LIDAR_BEAMS):
    """Assemble the observation vector.

    Args:
        scan_ranges: sequence of LiDAR ranges (metres).
        goal_rel: (dx, dy) goal offset in the robot frame.
        velocity: (v, w) current body velocity.
        prev_action: (v, w) previous commanded velocity.
        num_beams: number of downsampled beams to keep.

    Returns:
        A list of floats of length ``num_beams + 7`` (STUB: currently zeros).
    """
    # STUB: downsample scan_ranges to num_beams, append goal/velocity/action
    # features, and normalize. For now return a correctly shaped zero vector so
    # the runtime plumbing can be exercised end to end.
    _ = (scan_ranges, goal_rel, velocity, prev_action)
    return [0.0] * (num_beams + 7)
