# Classical MPC & Recovery Updates

This document details the latest features, bug fixes, and robust improvements added to the **Classical MPC** navigation stack (`barn_classical` and `barn_mapping`).

## 1. Robust 6-Phase Recovery State Machine
The recovery system was completely rewritten to handle extremely tight, dead-end corridors and complex obstacles. It now follows a structured, progressive escalation to safely extract the robot from stuck states:
- **Phase 1 (Rotate Opposite):** Immediately rotates away from the closest LiDAR obstacle.
- **Phase 2 (Back Up):** Reverses slightly if physical rear clearance permits.
- **Phase 3 (Rotate Opposite 2):** Uses the newly gained space to continue rotating away from the obstacle.
- **Phase 4 (Rotate to Gap):** Identifies the widest open gap in the LiDAR scan and steers towards it.
- **Phase 5 (Back Up 2):** Backs up further if the gap rotation fails.
- **Phase 6 (Last Resort - 1m Backup & Replan):** Reverses a full meter and aggressively increases the A* clearance heuristic to force the global planner to find an entirely new, wider corridor.

**Key Recovery Fixes:**
- Fixed a bug where the safety node's velocity vetoes were artificially short-circuiting the recovery attempts. Recovery phases now wait out their full timeouts even if velocity commands are momentarily clamped.
- Added strict `rear_clearance` physical checks using the LiDAR during backup phases to prevent the robot from reversing into unseen walls.

## 2. Global Planner Path Stability Cooldown
Fixed severe path oscillation where the robot would rapidly jitter left and right in tight spaces. 
- The global planner now enforces a **2.0-second cooldown** between path swaps. 
- It actively checks if the path ahead is still collision-free (ignoring the immediate footprint which constantly updates). If the path ahead is valid, it locks onto the current trajectory instead of flipping to a marginally cheaper A* route.

## 3. High-Speed Local Planner Bottlenecks Resolved
The local planner was previously constrained by stacking speed limiters, preventing the robot from reaching the intended 4.0 m/s max speed.
- Extended the **lookahead curvature distance** from 1.5m to 3.0m for smoother, less aggressive deceleration profiles.
- Relaxed the **side-clearance penalty** from a 70% minimum to an 85% minimum speed scale.
- Relaxed the **entry-heading gate** to only significantly brake when the heading error is extreme (>86 degrees).
- Properly exposed and aligned the Safety Node's `v_max` and `max_lin_accel` with the MPC configuration.

## 4. Advanced Map Decay & 3D Quaternions
- **Map Decay:** Added an online map decay rate to `barn_mapping` to automatically clear out transient "leftover" cells that were falsely blocking the robot's path.
- **3D Quaternion Raycasting:** Fixed a critical bug in LiDAR projection. The mapper now uses full 3D quaternion rotation matrices instead of 1D yaw extraction, correctly handling "upside-down" or tilted sensor mounts (e.g., pitch/roll = 180 degrees).

## 5. Tunable MPC Prediction and Obstacle Margin
To prevent the MPC from prematurely slamming on the brakes or cutting corners too tightly, several prediction parameters were exposed and tuned:
- **`mpc_horizon` and `mpc_dt`:** Now fully exposed in `classical_mpc.yaml`. Users can directly tune the lookahead time/distance.
- **`obstacle_margin`:** Increased to explicitly instruct the MPC to start bending its predicted footprint away from walls *before* it physically scrapes them.
- **`max_obstacle_slack`:** Relaxed to allow the QP solver to remain feasible when navigating through extremely narrow doorways.

## 6. Safety Shield: the freeze that timed out trials

The 500-trial campaign scored **0.3759** with **95.1 % success and 1 collision**. Every remaining failure was a *timeout*, and the dominant cause was the robot standing perfectly still until the 100 s clock expired. Root cause, in two layers:

**Layer 1 — the veto box was wider than the benchmark's own corridors.** `SweptFootprintShield` hard-vetoes when an obstacle falls inside `half_extent + emergency_margin`, and the scale search scales `v` and `w` together. As the scale approaches zero the swept envelope collapses onto the box at the robot's *current* pose, so a swept intersection can always be scaled away — but an obstacle already **inside** the pose-0 box makes every scale unsafe, including an in-place rotation. At `emergency_margin: 0.05` the box half-width was 0.265 m against a tightest reference-path clearance of 0.225 m. Reduced to **0.02**, which shrinks the trap shell from 4 cm to 1 cm (`safety_node` already discards returns inside `half + 1 cm` as self-hits). Worlds 114 and 288 went from 8/10 with 2 timeouts each to **12/12**.

**Layer 2 — a scale search cannot change direction.** Shrinking the margin made the trap rarer, not impossible; worlds 216 and 222 still froze. Scaling changes a command's magnitude, never its heading, so a blocked direction is blocked at every scale even when rotating in place is free. Traced on world 216: the robot sat at `(-2.51, 8.56)` for **89 s** with a signed clearance of `+0.001 m` — an obstacle a millimetre *outside* the box, dead ahead — while `ReverseToClearance` recovery ran and produced 0.00 m of motion, because the shield vetoed the reverse too.

What shipped for layer 2:

- **`find_escape()`** — when every scale fails, search eight slow motions (rotations first, then reverse, then forward) and accept one only if it *strictly* increases signed clearance and never decreases it anywhere along the path, so an "escape" can never clip a second obstacle. Needs a new `signed_clearance()` that goes negative inside the box (penetration depth), since the clamped clearance stops varying once trapped.
- **Gated behind `shield_escape_after_s` (1.5 s of sustained dead stop).** The escape *adds* motion the planner never requested; firing it on transient vetoes turned the shield from a filter into an actuator and shoved the robot off its start pose during the evaluator's reset, adding ~30 s to every trial.
- **Default `shield_escape_enable: false`.** On world 216 (12 trials/arm) it was 11/12 either way, median AT 16.2 → 18.3 s. It costs time for no measurable reliability gain, and n=12 cannot resolve a 1-in-12 failure rate. Needs 30+ trials per arm before adoption.
- **`emergency_trapped` as a distinct reason** from `emergency_veto`, plus throttled WARN logs for `emergency_trapped`, `emergency_escape` and the watchdog's `publish_zero`. A shield-frozen robot previously produced *zero* log output, which is why an 89 s freeze took so long to find. Plain `emergency_veto` stays unlogged — it is the routine stop in front of an obstacle and floods a tight corridor.
- Shield unit tests extended 6 → 14, covering both trap variants, the "don't back into a second obstacle" case, the disabled path, and the ungated path.

See [`evaluation/tuning/RESULTS.md`](../../evaluation/tuning/RESULTS.md) for the full measurement log and [Chapter 05](../tutorials/05-the-safety-shield.md) for the explanation.

## 7. Tuning changes tried and REVERTED

Recorded here and inline in `classical_mpc.yaml` so they are not retried blind. All measured at 12 trials per config, headless, against the `emergency_margin: 0.02` config.

| change | result | why it failed |
|---|---|---|
| `obstacle_margin` 0.20 → 0.12 | score 0.4469 → **0.4077**; sd 1.6 → 5.0 | Lets the MPC plan into the band where `barn_safety`'s shield vetoes commands. Buys stalls, not speed. The MPC's margin should stay at or above what the shield allows. |
| `clearance_penalty_radius` 1.4 → 0.8 | score 0.4469 → **0.3044**; success 12/12 → 9/12 | Restores a gradient near walls but stops biasing the route toward corridor centres, so A\* hugs walls into unrecoverable pinches. The wide band is doing reliability work, not just centering. |
| `max_lateral_accel` 3.0 → 5.0 | mean AT 16.55 → 15.83 s (sd 2.0 → 3.1) | Inside the noise at n=12; the batch score rose only because one timeout flipped. Not shown harmful — reverted to avoid confounding the next campaign. Still the best speed candidate, since it reduces no clearance and cannot reach the shield's veto band. |

**Where the score actually is:** campaign-wide, failures cost 0.0244 of score while slowness costs **0.0997** — 63 % of successful trials finish above the `2·OT` clip, at a mean 0.92 m/s against a 2.0 m/s reference. Fixing the freeze converts hard zeros (worth roughly +0.01 to +0.03 overall); the remaining headroom is speed.
