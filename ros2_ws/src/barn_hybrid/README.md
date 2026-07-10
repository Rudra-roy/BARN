# barn_hybrid — Track C

Hybrid arbiter: classical command (nominal) + dynamic-risk-gated RL residual.

```
/barn/cmd_classical ─┐
                     ├─▶ hybrid_node ──▶ /barn/cmd_desired ──▶ barn_safety
/barn/cmd_rl        ─┘   (alpha from risk gate)
```

## Design
- `fusion.py` (real): `v = v_c + alpha·dv`, `w = w_c + alpha·dw`.
- `risk_gate.py` (real): maps minimum time-to-collision → `alpha ∈ [0, 1]`.
  With no dynamic tracks (static worlds) TTC is ∞ → `alpha = 0` → output equals
  the classical command. Hysteresis is stubbed pending the tracker (M18/M19).

## Why Python
This is **not** the real-time authority — `barn_safety` (C++) still clamps the
final `/cmd_vel`. The arbiter/gate logic changes frequently during research, so
Python's iteration speed wins. If profiling later shows the gate on the critical
path, promote it into `barn_safety`; the topic contract is unchanged.

## Static-regression rule
Hybrid static success rate must stay within the allowed margin of the classical
stack (see [`docs/roadmap.md`](../../../docs/roadmap.md)). `alpha = 0` in static
worlds makes this hold by construction until the gate/residual are trained.

## Tests
`test/test_fusion.py` covers fusion + gate; ament flake8/pep257 lint.
