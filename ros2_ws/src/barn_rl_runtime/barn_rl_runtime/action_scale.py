"""Map a bounded policy action to a physical velocity command.

This is real (and unit-tested): a policy emits actions in [-1, 1]; we scale them
to [v_min, v_max] x [-w_max, w_max]. Keeping it pure makes it testable without a
model or a ROS graph.
"""


def scale_action(action, v_min=0.0, v_max=2.0, w_max=1.5):
    """Scale a 2-vector policy action to (linear, angular) velocity.

    Args:
        action: iterable of length 2, each component expected in [-1, 1].
        v_min: linear velocity mapped from action[0] == -1.
        v_max: linear velocity mapped from action[0] == +1.
        w_max: angular velocity magnitude at action[1] == +/-1.

    Returns:
        (v, w) tuple of floats, clipped to the configured bounds.
    """
    a0 = _clip(float(action[0]), -1.0, 1.0)
    a1 = _clip(float(action[1]), -1.0, 1.0)
    v = v_min + (a0 + 1.0) * 0.5 * (v_max - v_min)
    w = a1 * w_max
    return v, w


def _clip(value, lo, hi):
    """Clamp value into [lo, hi]."""
    return max(lo, min(hi, value))
