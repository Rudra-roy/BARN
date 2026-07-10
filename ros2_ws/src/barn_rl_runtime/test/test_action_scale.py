"""Unit tests for the pure action-scaling helper."""

from barn_rl_runtime.action_scale import scale_action


def test_neutral_action_is_stationary():
    """action[0] == -1 maps to v_min (0), action[1] == 0 maps to w == 0."""
    v, w = scale_action([-1.0, 0.0], v_min=0.0, v_max=2.0, w_max=1.5)
    assert v == 0.0
    assert w == 0.0


def test_full_forward():
    """action[0] == +1 maps to v_max."""
    v, w = scale_action([1.0, 0.0], v_min=0.0, v_max=2.0, w_max=1.5)
    assert abs(v - 2.0) < 1e-9
    assert w == 0.0


def test_angular_sign_and_magnitude():
    """action[1] scales linearly to +/- w_max."""
    _, w_left = scale_action([0.0, 1.0], w_max=1.5)
    _, w_right = scale_action([0.0, -1.0], w_max=1.5)
    assert abs(w_left - 1.5) < 1e-9
    assert abs(w_right + 1.5) < 1e-9


def test_out_of_range_is_clipped():
    """Actions outside [-1, 1] are clipped before scaling."""
    v, w = scale_action([5.0, -5.0], v_min=0.0, v_max=2.0, w_max=1.5)
    assert abs(v - 2.0) < 1e-9
    assert abs(w + 1.5) < 1e-9
