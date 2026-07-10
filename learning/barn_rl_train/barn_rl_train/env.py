"""Fast 2-D BARN-style training environment (STUB).

A Gymnasium-compatible environment used to train policies without Gazebo. It
must expose ONLY the competition-faithful observation (downsampled LiDAR, goal
distance, sin/cos bearing, velocity, previous action) — never privileged
world information. Keep the observation/action layout identical to
barn_rl_runtime so exported policies transfer.
"""


class BarnEnv2D:
    """Minimal Gymnasium-style API placeholder.

    Implement reset()/step() over a procedurally generated 2-D obstacle field
    with a differential-drive kinematic model. Kept dependency-light here so the
    scaffold imports without gymnasium/numpy installed.
    """

    metadata = {'render_modes': []}

    def __init__(self, num_beams=720, max_speed=2.0):
        """Store the observation/action shape parameters."""
        self.num_beams = num_beams
        self.max_speed = max_speed
        self.observation_dim = num_beams + 7
        self.action_dim = 2

    def reset(self, seed=None):
        """Return (observation, info). STUB: zeros."""
        _ = seed
        return [0.0] * self.observation_dim, {}

    def step(self, action):
        """Return (observation, reward, terminated, truncated, info). STUB."""
        _ = action
        return [0.0] * self.observation_dim, 0.0, False, False, {}
