"""Policy network definition (STUB).

Defines the MLP architecture shared by training and export. Keep it small enough
for CPU inference on the i3-class BARN physical target. Torch is imported lazily
so the scaffold imports without it installed.
"""

DEFAULT_HIDDEN = (256, 256)


def build_policy(observation_dim, action_dim, hidden=DEFAULT_HIDDEN):
    """Construct the policy network.

    STUB: returns a description dict. Replace with a torch.nn.Module (tanh MLP,
    action in [-1, 1]) once torch is a dependency.
    """
    return {
        'observation_dim': observation_dim,
        'action_dim': action_dim,
        'hidden': list(hidden),
        'activation': 'tanh',
    }
