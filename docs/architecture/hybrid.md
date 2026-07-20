# Track C — Hybrid (Classical + Gated RL Residual)

> Purpose: document the hybrid track — a classical nominal command corrected by a dynamic-risk-gated RL residual, with a C++ tracker feeding the gate and C++ safety remaining the final authority.

Related documents:
[Architecture Overview](./overview.md) ·
[Track A — Classical](./classical.md) ·
[Track B — End-to-end RL](./e2e_rl.md) ·
[Robot Interface Contract](../robot_interface.md)

Track C keeps the reliable classical controller in charge and lets RL contribute
a **correction** only when a dynamic hazard warrants it. The arbiter lives in the
Python package `barn_hybrid`; obstacle tracking lives in the C++ package
`barn_dynamic_tracking`. In purely static worlds — which is what BARN scores —
the gate closes and the output equals the classical command exactly.

---

## 1. Command math

Let `[v_c, w_c]` be the classical nominal command and `[dv, dw]` the RL residual.
The arbiter fuses them with a scalar gate `alpha in [0, 1]`:

```
   v = v_c + alpha * dv
   w = w_c + alpha * dw
```

- `alpha = 0` -> output is pure classical (`v = v_c`, `w = w_c`).
- `alpha = 1` -> RL residual applied in full.
- `alpha` is set by the **dynamic-risk gate** from time-to-collision (TTC).

RL contributes a *residual*, not a full command, so even at `alpha = 1` it can
only nudge the trusted classical command — it cannot replace it. In static
worlds there is no closing dynamic obstacle, so `alpha ~ 0` and Track C is,
by construction, Track A.

---

## 2. The risk gate (`risk_gate.py`)

The gate maps the most-threatening track's TTC to `alpha` with a linear ramp
between two thresholds, plus hysteresis to avoid chattering near the boundary.

```
   alpha
     1 │────────────●
       │            ╲
       │             ╲          (linear ramp)
       │              ╲
     0 │               ●──────────────────────
       └───────────────┼──────────┼──────────── TTC
                    ttc_full   ttc_zero
                     (1.0 s)    (3.0 s)
```

| Parameter  | Value | Meaning                                                        |
|------------|-------|----------------------------------------------------------------|
| `ttc_full` | 1.0 s | At/below this TTC the gate is fully open (`alpha = 1`).        |
| `ttc_zero` | 3.0 s | At/above this TTC the gate is closed (`alpha = 0`).           |

Between the thresholds `alpha` ramps linearly. **Hysteresis:** the gate opens
readily but closes reluctantly — once engaged it stays engaged through brief TTC
fluctuations, so `alpha` does not oscillate and the command stays smooth. In a
static world no track ever produces a TTC below `ttc_zero`, so `alpha` stays at 0.

Parameters live in `barn_hybrid/config/hybrid.yaml`.

---

## 3. The tracker pipeline (`barn_dynamic_tracking`)

> **🚧 Status:** the tracker described here is **implemented** (clustering,
> association, per-track KF, `tracker_node`) and already runs — but currently
> feeding the **classical** MPC's moving-obstacle constraints via a separate
> dynamic launch, not this hybrid gate. Wiring `/barn/tracks` into `risk_gate.py`
> is still pending (M19). See
> [DynaBARN Dynamic-Obstacle Support (WIP)](../features/dynabarn_dynamic_obstacles.md).

The gate needs relative velocity and TTC, which come from a classical
multi-object tracker driven by LiDAR — no ground-truth obstacle states.

```
   /barn/scan ──▶ [ CLUSTER ] ──▶ [ ASSOCIATE ] ──▶ [ PER-TRACK KALMAN (CV) ] ──▶ [ REL. VEL / TTC ]
   /barn/pose ──▶  clustering.cpp   association.cpp     kalman.cpp                    ttc.cpp
                                                                                        │
                                                                                        ▼
                                                                    /barn/tracks (MarkerArray) + TTC to gate
```

| Stage       | Source            | Responsibility                                              |
|-------------|-------------------|-------------------------------------------------------------|
| Cluster     | `clustering.cpp`  | Group scan returns into candidate obstacles (`cluster_distance_threshold = 0.3 m`). |
| Associate   | `association.cpp` | Match clusters to existing tracks (`association_gate_distance = 0.5 m`). |
| Kalman      | `kalman.cpp`      | Per-track constant-velocity filter (`process_noise_q`, `measurement_noise_r`). |
| Rel. vel / TTC | `ttc.cpp`      | Compute relative velocity and time-to-collision per track.  |

`tracker_node` publishes `/barn/tracks` (`barn_msgs/ObstacleTrackArray`) plus
`/barn/track_markers` (`visualization_msgs/MarkerArray` for RViz) at
`publish_rate_hz = 10.0`, and will feed the minimum TTC to the gate once wired.
Because BARN worlds are static, tracks appear stationary, TTC stays large, and the
residual stays gated off — exactly the intended behavior.

---

## 4. Why Python arbiter, C++ safety

The arbiter's job is orchestration and gating logic — light math at 20 Hz —
where Python's clarity and quick iteration pay off and its latency cost is
negligible. The **final authority stays in C++ `barn_safety`**, deliberately:

| Concern            | Arbiter (`barn_hybrid`, Python)          | Authority (`barn_safety`, C++)          |
|--------------------|------------------------------------------|------------------------------------------|
| Role               | Fuse classical + gated residual          | Clamp / accel-limit / stale-gate         |
| Runs at            | 20 Hz                                     | 50+ Hz                                    |
| Trust level        | Convenience layer                         | Non-bypassable safety layer              |
| Language rationale | Fast iteration on gating policy           | Deterministic, low-latency, always last  |

The fused `/barn/cmd_desired` from the arbiter is still just a *desired*
command. It flows to `barn_safety` like every other track's output; neither the
RL residual nor the arbiter can reach `/cmd_vel` directly. See the
[Robot Interface Contract](../robot_interface.md).

---

## 5. Command flow (mode = hybrid)

```
   goal_seeker_node ──▶ /barn/cmd_classical ┐
                                            ├─▶ hybrid_node ──▶ /barn/cmd_desired ──▶ safety_node ──▶ /cmd_vel
   rl_runtime_node  ──▶ /barn/cmd_rl        ┘        ▲
                                                     │ alpha (TTC gate)
   tracker_node ──▶ /barn/tracks + min TTC ──────────┘
```

---

## 6. Static-regression rule

Track C must not degrade the static-world performance we already trust from
Track A. The rule:

> **Hybrid static success rate must stay within ~1 percentage point of
> classical.**

Because `alpha ~ 0` in static worlds, hybrid *should* equal classical; the ~1 pp
band tolerates only tracker/gate noise, not real regression.

**Worked example:**

| Track                    | Static success rate | Verdict                      |
|--------------------------|---------------------|------------------------------|
| Classical (Track A)      | 97.4%               | baseline                     |
| Hybrid (Track C)         | `>= ~96.4%`         | **acceptable** (within 1 pp) |
| Hybrid (Track C)         | `< 96.4%`           | **regression — investigate** |

If hybrid drops below ~96.4%, the residual is leaking into static commands
(gate not fully closing, or tracker producing spurious low-TTC tracks) and must
be fixed before the track is used. Reliability and consistency come before any
dynamic-world benefit.
