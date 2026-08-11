# Codebase review — six specialist passes (2026-08-10)

Scope: `ros2_ws/src/*` and `barn_bringup` config. Excludes `evaluation/` and `learning/`.
Six parallel reviews: global planner, local planner + MPC, mapping/distance field,
safety shield, system integration, recovery.

## The meta-finding

A full day of parameter tuning produced **seven rejected changes and zero adopted**.
The reviews show most of those rejections were measuring defects, not the changes:

| tuning conclusion | what the review found |
|---|---|
| "shield escape costs time for no gain" | the escape latch cancels itself; it emitted ~5 mm of motion per 1.5 s |
| "obstacle_margin 0.10 causes shield traps" | traps come from a hard veto on PRE-EXISTING intrusions, fixable separately; 0.10 removed freezes 42->0 and cut AT sd 8x |
| "the map is 85% optimistic vs the scan" | artifact: compared an at-pose value against the shield's SWEPT-ENVELOPE minimum. Corrected: 42.6% optimistic, p50 -0.016 |
| "the freeze is unfixable inside the QP" | the QP is fed a padded clearance (10 cm max-pooled grid, no interpolation, centre-to-centre EDT) |
| "planner_status shows 22-26% retained_cooldown" | the status is only assigned on ACCEPTED plans; failures during cooldown keep the previous label |
| "recovery timeouts are as configured" | `recovery_.step(0.033, ...)` hardcodes dt while the loop achieves ~50 ms |

**Conclusion: fix the measurement-invalidating defects before re-testing anything.**

---

## Tier 1 — bugs. No tuning risk, unit-testable, fix first

1. **Scan matcher injects unbudgeted frame jitter** — `barn_mapping/src/mapping_node.cpp:295,331,334`.
   xy search has exactly 3 levels (`{-0.04, 0, +0.04}`), the acceptance gate is an
   absolute 0.75 summed over <=150 points (~0.5%, open on noise), and `corr_rate_budget_`
   debits **yaw only**. Permits 1.6 m/s of frame slew at 40 Hz.
   An offline sim against world 66's real obstacles reproduces the measured map-vs-scan
   distribution ONLY when this jitter is injected (perfect pose: p90 +0.066; +0.08 m
   jitter: p90 +0.109 vs +0.101 measured live). **Root cause of the map/scan disagreement.**

2. **Shield escape latch self-cancels** — `barn_safety/src/safety_node.cpp:159-171`.
   Any non-zero output clears `stopped_latched_`, so the next cycle vetoes to zero and the
   1.5 s wait restarts: one 33 ms command per 1.5 s. **Invalidates the measurement that
   `shield_escape_enable` is disabled on.**

3. **Shield hard-vetoes motions that would undo an intrusion** — `swept_footprint_shield.cpp:44-59`.
   A return inside the pose-0 box fails every scale including reverse. Cancels the planner's
   own `ReverseToClearance`. Fix: monotonicity constraint (never dig deeper) for pre-existing
   intruders, hard veto for everyone else. Pure attenuation — needs no gate.

4. **A* heuristic truncated in COST units, not metres** — `global_planner_astar.cpp:113`.
   `top.dist > start_dist + 5.0` compares a cost against a distance. Beyond it cells keep
   `h = inf`, every node ties on `f`, and A* degenerates to unordered blind search.

5. **`planner_status_` only assigned when a plan is accepted** — `classical_mpc_node.cpp:589-594`.
   Failures during cooldown silently keep the previous label.

6. **`recovery_.step(0.033, ctx)` hardcodes dt** — `classical_mpc_node.cpp:802`.
   Every recovery timeout stretches with the achieved rate.

7. **Dead 600k-cell EDT rebuilt at 15 Hz** — `mapping_node.cpp:409`. `distance_field_` is
   written and never read. 5.76 ms per rebuild inside `grid_mutex_` = ~8.6% of a core, wasted.

8. **Single callback group** — `classical_mpc_node.cpp:1186`. A 3-thread executor with no
   callback groups serialises everything; the control timer is set to 30 Hz and achieves 20.

9. **`widest_gap_heading` sweeps the rear blind cone** — `recovery.cpp:43`. The laser is
   ~270°; `min_range_in_sector` returns `range_max` for empty sectors, so the rear always
   "wins" and `kRotateToGap` targets ~180 degrees, which also exceeds its own 2.5 s timeout.

10. **Path smoother runs after collision checking and is never re-validated** —
    `global_planner_astar.cpp:284-307`. Displaces corners up to ~5 cm into walls; the node
    then discards the ENTIRE plan.

## Tier 2 — missing capability. Structural, moderate risk

11. **No backward braking pass in the velocity profile** — `local_planner.cpp:152-211`.
    The max-over-3 m window is a crude substitute for a proper decel sweep: too slow behind
    every corner, and no pre-deceleration for a corner just outside the window. Predicted
    mean v_ref 1.25 -> ~2.0-2.5. **Largest single speed item.**

12. **`clearance_scale` is pinned at 0.85** — `local_planner.cpp:216-222`. Its interpolation
    range never activates at BARN clearances, so it is a permanent, unconditional ~17.6% AT tax.

13. **Axis-wise attenuation before declaring a veto** — `swept_footprint_shield.cpp:97-112`.
    Try `(0, w*s)` and `(v*s, 0)` before the escape. Componentwise bounded by the planner's
    command, so no gate needed. This is the step skipped between "scaling can't change
    direction" and the escape.

14. **Distance field accuracy** — 10 cm max-pooled grid, no interpolation, centre-to-centre
    EDT. Up to ~0.12 m of phantom padding; the measured freeze gap was 0.056 m. Building at
    5 cm costs 1.57 -> 1.74 ms.

15. **Log-odds asymmetry** — misses are 1.89x hits, so a cell needs >65% hit rate to stay
    occupied and 2 misses declassify a saturated obstacle. Fix ONLY after #1.

## Tier 3 — behaviour changes. Need 30+ trials per arm

16. Re-test `obstacle_margin: 0.10` after #2, #3, #14 — the largest measured effect in the repo.
17. `distance_weight 0.3 -> 1.0` — the cost function currently prices 1 m of tight corridor
    as worth a 4.5 m detour, a candidate for the 1.44x path-length overshoot.
18. `recovery_reverse_speed 0.35 -> 0.9` on the breadcrumb branch ONLY (rear is blind).
19. Attempt-budget refund by decrement over ~1 m, not reset after 0.18 m.

## Do not ship without a dedicated collision campaign

- `emergency_margin_y 0.02 -> 0.005` (safety review #4). The only proposal that genuinely
  reduces a safety margin. Baseline collision rate is ~1 in 155.

## Stale comments (behaviour claimed but not implemented)

`local_planner.cpp:231` "drop v_ref to 0 / rotate in place" (floors at 0.3, never rotates) ·
`local_planner.cpp:152` "scan ahead ~1.5 m" (uses 3.0) · `recovery.cpp:59` "a rear obstacle
scales it down" (rear is blind) · `classical_mpc_node.cpp:268` "20 Hz MPC" (timer is 30 Hz) ·
`mapping_node.hpp:84-89` decay described as active (both rates are 0.0) ·
`controller.cpp:399` "per-pass time check bounds wall-clock" (only between passes; OSQP
`time_limit` is inside a `#ifdef PROFILING` never defined) · `distance_field.hpp:3` "exact
Euclidean ... in metres" (centre-to-centre, not distance-to-obstacle).

## Dead config that will waste a tuning batch

`barn_safety/config/safety.yaml` and `barn_mapping/config/mapping.yaml` are loaded by
nothing; both carry values contradicting the live `barn_bringup/config/classical_mpc.yaml`.
