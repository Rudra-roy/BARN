"""Command fusion for the hybrid track.

Real and unit-tested. The classical command is nominal; the RL contributes a
residual gated by ``alpha`` in [0, 1]:

    v = v_c + alpha * dv
    w = w_c + alpha * dw

At alpha == 0 the output is exactly the classical command, which is the required
behaviour in static BARN worlds (see docs/architecture/hybrid.md and the static
regression rule in docs/roadmap.md).
"""


def fuse(classical, rl_residual, alpha):
    """Blend a classical command with a gated RL residual.

    Args:
        classical: (v, w) classical/nominal command.
        rl_residual: (dv, dw) residual proposed by the RL policy.
        alpha: gate in [0, 1]; 0 -> pure classical, 1 -> full residual.

    Returns:
        (v, w) fused command tuple.
    """
    a = max(0.0, min(1.0, float(alpha)))
    v = classical[0] + a * rl_residual[0]
    w = classical[1] + a * rl_residual[1]
    return v, w
