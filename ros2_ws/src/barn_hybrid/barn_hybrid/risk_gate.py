"""Dynamic-risk gate for the hybrid track.

Maps the minimum time-to-collision among dynamic tracks to a gate alpha in
[0, 1]: fully classical when the scene is safe, ramping in the RL residual as a
collision becomes imminent. With no dynamic tracks (static worlds) TTC is
+infinity and alpha is 0, so the hybrid stack reduces exactly to the classical
stack. Hysteresis (separate enter/exit thresholds) is a STUB to be completed
alongside the tracker (M18/M19).
"""


class RiskGate:
    """Computes the RL residual gate from the nearest time-to-collision."""

    def __init__(self, ttc_full=1.0, ttc_zero=3.0):
        """Store the TTC thresholds.

        Args:
            ttc_full: at/below this TTC (s) the gate is fully open (alpha = 1).
            ttc_zero: at/above this TTC (s) the gate is closed (alpha = 0).
        """
        self._ttc_full = ttc_full
        self._ttc_zero = ttc_zero

    def alpha(self, min_ttc):
        """Return the gate value in [0, 1] for the given minimum TTC (seconds)."""
        if min_ttc >= self._ttc_zero:
            return 0.0
        if min_ttc <= self._ttc_full:
            return 1.0
        span = self._ttc_zero - self._ttc_full
        return (self._ttc_zero - min_ttc) / span
