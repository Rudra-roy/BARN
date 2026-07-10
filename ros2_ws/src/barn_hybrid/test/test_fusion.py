"""Unit tests for the hybrid fusion and risk gate."""

from barn_hybrid.fusion import fuse
from barn_hybrid.risk_gate import RiskGate


def test_alpha_zero_is_pure_classical():
    """At alpha == 0 the fused command equals the classical command."""
    v, w = fuse((0.7, -0.2), (5.0, 5.0), 0.0)
    assert v == 0.7
    assert w == -0.2


def test_alpha_one_adds_full_residual():
    """At alpha == 1 the full residual is added."""
    v, w = fuse((0.7, -0.2), (0.1, 0.3), 1.0)
    assert abs(v - 0.8) < 1e-9
    assert abs(w - 0.1) < 1e-9


def test_alpha_is_clamped():
    """Alpha outside [0, 1] is clamped."""
    v, _ = fuse((1.0, 0.0), (1.0, 0.0), 5.0)
    assert abs(v - 2.0) < 1e-9


def test_gate_closed_when_safe():
    """No dynamic risk (TTC == inf) closes the gate."""
    gate = RiskGate(ttc_full=1.0, ttc_zero=3.0)
    assert gate.alpha(float('inf')) == 0.0


def test_gate_open_when_imminent():
    """A short TTC opens the gate fully; mid-range ramps."""
    gate = RiskGate(ttc_full=1.0, ttc_zero=3.0)
    assert gate.alpha(0.5) == 1.0
    assert abs(gate.alpha(2.0) - 0.5) < 1e-9
