# BARN Navigation System — Architecture, Change Log & Recovery Guide

This document tracks all system enhancements, architectural mechanics, and provides an in-depth guide on the navigation stack, local planner comparison (Elastic Band vs. B-Spline + TOPP-RA), the Anti-U-Turn heading guard, the 100s freeze recovery system, and the traversed breadcrumb tracking memory.

---

## 1. System Architecture Overview

```
                      +-----------------------------+
                      |   A* Global Planner (24 bins)|
                      |    (Grid + Soft Clearance)  |
                      +--------------+--------------+
                                     |  global_path (Waypoints)
                                     v
                 +---------------------------------------+
                 |       Path-Swap Heading Guard         |
                 |  (Rejects >90° flips in tight gaps)   |
                 +-------------------+-------------------+
                                     |  Gated global_path
                                     v
  +---------------------------------------------------------------------+
  |                       Local Planner Module                          |
  |                                                                     |
  |  [Choice A: Elastic Band]            [Choice B: B-Spline + TOPP-RA] |
  |  - Discrete waypoint smoothing       - Uniform Cubic B-Spline (C²)  |
  |  - Distance field gradient push      - Obstacle/Curvature Opt       |
  |  - Windowed curvature profiling      - Time-Optimal Wheel Profiling |
  +----------------------------------+----------------------------------+
                                     |  LocalTrajectory (x, y, yaw, v_ref, clearance)
                                     v
                      +-----------------------------+
                      |   QP Model Predictive       |
                      |   Controller (MPC, 10 steps)|
                      +--------------+--------------+
                                     |  cmd_desired (v, w)
                                     v
                      +-----------------------------+
                      |   Swept-Footprint Safety    |
                      |   Shield (Emergency Veto)   |
                      +--------------+--------------+
                                     |  cmd_safe (v, w)
                                     v
                               [Robot Base]
```

---

## 2. Complete Summary of Changes

| No. | Date | Component / File | Old State | New State | Why / Benefit |
|:---:|:---:|:---|:---|:---|:---|
| **Fix 1** | 2026-08-27 | `classical_mpc_node.cpp:L685` | Blind path swap on replan | **Anti-U-Turn Heading Guard** | Rejects mid-run >90° heading flips in narrow corridors (`clearance < 0.40m`) and triggers breadcrumb reverse instead of colliding during in-place spins. |
| **Fix 2** | 2026-08-27 | `classical_mpc_node.cpp:L1028` & `classical_mpc.yaml` | `\|w\| < 0.15` angular filter blocked recovery | **Prompt Freeze Recovery (1.2s)** | Triggers breadcrumb reverse whenever physical motion stops for >1.2s, regardless of angular twitches. |
| **Fix 3** | 2026-08-27 | `swept_footprint_shield.cpp:L57` | Step 0 intrusion blocked all motion | **Allowed Reverse Escapes Away From Obstacles** | Prevents safety shield from vetoing recovery reverse commands with `SHIELD TRAPPED` when touching front obstacles. |
| **Fix 4** | 2026-08-27 | `recovery.cpp:L221` | Attempt 2+ forced in-place spin even in tight corridors | **Strict Physical Rotation Clearance Gating** | In-place rotation (`kRotateToGap`) is strictly gated on `clearance >= 0.40m`. If space is lost, it aborts spin and reverses. |
| **Fix 5** | 2026-08-27 | `classical_mpc_node.cpp:L635` | Full footprint check broke cooldown | **Path Occupancy Check & Cooldown Lock** | Eliminates left-right path flip-flops and limit-cycle wiggling in narrow corridors. |
| **Fix 6** | 2026-08-27 | `recovery.cpp:L153` & `L195` | Backed up only 5cm before premature abort on `!veto_active` | **Full Reverse Guarantee (`reversed >= 0.40m`)** | Prevents 5cm twitch loops by requiring the robot to reverse deeply into open space before replanning. |
| **Fix 7** | 2026-08-27 | `classical_mpc.yaml:L230` | `obstacle_margin: 0.20` demanded 0.46m clearance | **`obstacle_margin: 0.10` (0.36m Demanded Clearance)** | Fits within BARN narrow corridors (0.35-0.45m), eliminating false MPC freezes and replanning loops. |
| **Fix 8** | 2026-08-28 | `classical_mpc.yaml` & scripts | 15.0s lateral sweep timeouts & relative output paths | **Fast Finish Sweep (2.0s) & 1.0m Goal Overshoot** | Eliminates 60s finish line wiggles and file not found errors. |
| **Fix 9** | 2026-08-28 | `local_plannerBspline.cpp:L160` | Only wheel velocity bound | **TOPP-RA Yaw-Rate & Centripetal Limits** | Pre-decelerates into tight curves ($\kappa = 2.0$), preventing corner understeer collisions (e.g. World 228). |
| **Fix 10** | 2026-08-29 | `classical_mpc_node.cpp` & `recovery.cpp` | Over-constrained safety logic caused regressions | **Balanced Motion Recovery & Dead-End Acceptance** | Accepts 180° escape routes, drives through unblocked 38cm corridors, and bounds tunnel reverses to 40cm. |
| **Fix 11** | 2026-08-29 | `classical_mpc_node.cpp` & `recovery.cpp` | Twitching masked stalls & front veto aborted reverses | **Veto-Aware Motion Watchdog & Non-Aborting Reverse** | Progress requires unvetoed speed; `blocked_timeout` does not abort reverse escapes; recovery always backs out first. |
| **Fix 12** | 2026-08-29 | `recovery.cpp:L79` | Diagonal reverse translation struck rear obstacles | **Pivot-First Alignment in Reverse Recovery** | Misaligned rear pivots in place ($v = 0$) toward breadcrumb before reversing, preventing rear collision vetoes. |
| **Fix 13** | 2026-08-30 | `path_validator.cpp:L8` | Rigid footprint box rejected narrow passages | **Centerline Cell Validation for Global Plans** | Validates path centerline cells without rigid box rejection, allowing narrow escape paths to be accepted. |
| **Fix 14** | 2026-09-04 22:14:10 | `local_planner.cpp`, `local_plannerBspline.cpp`, `classical_mpc_node.cpp`, `classical_mpc.yaml` | High speed during turns in narrow corridors caused wall collisions | **Clearance-Aware Turning Speed & Real-Time Yaw Governor** | In narrow corridors (clearance < 0.35m) slows to a crawl (0.35 m/s) and pivots in place for >20° turns; in open ground (clearance > 0.80m) takes sweeping turns at full speed (1.5-2.0+ m/s). |
| **Fix 15** | 2026-09-04 22:25:00 | `classical_mpc_node.cpp:L637` & `classical_mpc.yaml:L116` | Periodic replanner repeatedly switched between two near-equal paths | **Path Hysteresis (Commitment)** | If current path ahead is clear, commits to it and refuses to swap unless candidate path is at least 15%–20% better (`path_improvement_ratio = 0.85`). |
| **Fix 16** | 2026-09-04 22:30:00 | `local_planner.cpp`, `local_plannerBspline.cpp`, `classical_mpc_node.cpp`, `classical_mpc.yaml` | Distant lookahead and low w thresholds slowed down turns in open spaces | **High-Speed Open Cornering & Strict Dual-Condition Gating** | Enforces fast cornering (1.5–2.5+ m/s, 4.0 m/s² lateral budget) in open/moderate space; slows to crawl or pivots only when BOTH space is really narrow (clearance < 0.38m) AND turn is very sharp. |
| **Fix 17** | 2026-09-04 22:50:00 | `path_validator.cpp`, `global_planner_astar.cpp`, `collision_checker.cpp`, `classical_mpc_node.cpp` | Centerline-only collision checks and start-pose footprint abortion caused planner to retain paths with no clearance | **Clearance-Aware Path Invalidation, Permissive Start-Pose & Dead-End Recovery** | Validates physical clearance (clearance < 0.22m) along path to bypass hysteresis and accept detours; allows A* to expand from start pose near obstacles; triggers immediate reverse recovery when path has no clearance and no forward route exists. |
| **Fix 18** | 2026-09-04 23:05:00 | `local_planner.cpp`, `local_plannerBspline.cpp`, `classical_mpc_node.cpp`, `classical_mpc.yaml` | Hard boolean condition (narrow AND sharp) charged hot into moderate corridor bends at 2.0 m/s, causing wall drift and breadcrumb reverse triggers | **Continuous Clearance Scaling & Extended Lookahead (2.0m)** | Scales allowable lateral acceleration budget continuously from 0.60 m/s^2 (tight pinch) to 3.50 m/s^2 (open field) based on distance-field clearance; dynamically regulates cornering speed, pinch speed ceiling, and real-time yaw governor without sharp boolean cutoffs; extends lookahead window to 2.0m for advance braking into corners. |
| **Fix 19** | 2026-09-04 23:55:00 | `global_planner_astar.cpp`, `classical_mpc_node.cpp`, `classical_mpc.yaml` | Dubins 360-degree keyhole loops, corner-pinning against obstacles, false path invalidation on 10cm grid, and 25x narrow gap penalty caused the robot to freeze and refuse clear corridors | **Global Path Shortcutting, Loop Pruning, Corner Rotation Gating, and Clearance Recalibration** | Eliminates Dubins teardrop loops via line-of-sight shortcutting and loop pruning; gates in-place rotation away from obstacle faces (clearance >= 0.35m); recalibrates 10cm grid clearance threshold from 0.22m to 0.09m to prevent false path invalidation; normalizes clearance_weight from 1.2 to 0.15 in configuration to remove artificial 25x narrow corridor penalty. |
| **Fix 20** | 2026-09-08 14:20:00 | `classical_mpc.yaml`, `classical_mpc_node.cpp` | Under-penalizing clearance (0.15) and artificial footprint shrinkage (-0.02) caused A* to choose impassably narrow slits over wide open corridors | **Restoring Physical Footprint Margin, Wide-Corridor Preference, and Clearance Validation** | Restores clearance_weight to 1.2 so A* actively favors wide corridors over narrow pinches; sets global_planner_footprint_margin to 0.00 to match real 0.432m chassis width; restores kMinPassableClearance to 0.21m (physical half-width) to reject corridors narrower than the robot while retaining Fix 19 shortcutting and loop pruning. |
| **Fix 21** | 2026-09-15 18:38:00 | `path_validator.hpp`, `path_validator.cpp`, `classical_mpc_node.cpp` | Global clearance checks over the entire 15m path falsely flagged distant pinches as blocked, bypassing hysteresis and causing continuous flip-flop replanning and in-place spinning | **Horizon-Bounded Clearance Validation & Anti-Thrashing Hysteresis Lock** | Restricts clearance validation (`c < 0.21m`) to a 2.5m local lookahead horizon in both `path_validator` and `ahead_blocked`, while strictly checking hard obstacle occupancy (`kOccupied`) along the entire path length; eliminates 15 Hz replan thrashing and prevents premature hysteresis bypassing from distant sensor noise. |
| **Fix 22** | 2026-09-15 19:28:00 | `classical_mpc_node.cpp` | Over-strict clearance threshold (0.21m) falsely flagged 0.42m corridors as blocked, and geometric-only length comparison allowed 180-degree backward path swaps while moving forward | **Passable Clearance Recalibration (0.16m) & Forward Heading Continuity Hysteresis** | Recalibrates kMinPassableClearance to 0.16m so physically drivable 0.40m-0.44m narrow corridors are accepted without false blockage triggers; enforces forward heading continuity (heading_err < 90 degrees) in path hysteresis so the rover never executes a sudden backward U-turn swap while its current forward path is clear. |

---

## 3. Deep Dive: Core Fixes & Live Case Studies

### Bug 1 / Fix 1 (2026-08-27): Anti-U-Turn & Heading Discontinuity Guard (360° Spin Collisions)

#### The Problem:
* During mid-run navigation, dynamic map updates trigger $A^*$ replanning.
* If $A^*$ finds that an alternative route behind the robot is marginally shorter or unobstructed, the candidate path starts pointing backwards ($>90^\circ$ heading error relative to the robot's current heading).
* Previously, `planner_loop` blindly accepted the candidate and assigned `global_path_ = candidate`.
* The MPC immediately tried to track this backward-pointing path by commanding maximum angular velocity ($\omega = 3.0\,\text{rad/s}$) to spin the robot around on the spot.
* The Jackal robot chassis ($0.50\,\text{m} \times 0.43\,\text{m}$) has a rotation diagonal of $\approx 0.66\,\text{m}$. When pinched in a narrow BARN corridor of width $0.50 - 0.60\,\text{m}$, the robot cannot rotate in place without its corners violently striking the obstacles.

#### The Solution (`classical_mpc_node.cpp:L685`):
Before accepting any new candidate path mid-run, we evaluate both the initial heading discontinuity and the local distance-field clearance:
```cpp
if (accepted && !global_path_.empty() && candidate.size() >= 2) {
  const double cand_yaw = std::atan2(candidate[1].y - candidate[0].y, candidate[1].x - candidate[0].x);
  const double heading_err = std::abs(barn_core::wrap_angle(cand_yaw - pose.yaw));
  const double cur_clearance = distance_field_ ? distance_field_->distance_world(pose.x, pose.y) : 0.0;
  
  // If candidate requires a >90° turn and clearance is insufficient to rotate in place (< 0.40m):
  if (heading_err > (M_PI / 2.0) && (!std::isfinite(cur_clearance) || cur_clearance < rotation_clearance_m_)) {
    request_reverse_recovery_ = true;
    planner_status_ = "rejected_uturn_low_clearance";
    replan_completed_ = true;
    goto done;
  }
}
```
* **Result:** The path swap is rejected. The robot backs up along its known-safe breadcrumb trail into open ground (`clearance >= 0.40m`). Once in open space, the follow-up replan executes the turn safely without collisions.

---

### Bug 2 / Fix 2 (2026-08-27): Prompt Freeze Recovery & Timeout Reduction (100s Timeouts)

#### The Problem:
* In `classical_mpc.yaml`, `obstacle_margin` is configured at $0.20\,\text{m}$. Combined with the robot's half-width ($0.2159\,\text{m}$) and footprint safety margin ($0.04\,\text{m}$), the MPC demands a total clearance of $0.4559\,\text{m}$.
* In BARN corridors of width $0.35 - 0.45\,\text{m}$, the clearance constraint is mathematically infeasible at the robot's current pose.
* Because every forward control candidate violates the constraint by the same margin, the QP solver's minimum-cost slack solution is $v = 0.0\,\text{m/s}$.
* Because the robot twitches with small angular oscillations ($\pm 0.43\,\text{rad/s}$), standard `no_progress` checks (which checked $|\omega| < 0.15$) failed to trigger.
* With `freeze_recovery_enable: false`, the robot remained permanently stationary until hitting the simulation's 100s timeout.

#### The Solution (`classical_mpc.yaml` & `classical_mpc_node.cpp:L1028`):
1. **Enable `freeze_recovery_enable: true`**:
   The `FreezeDetector` module continuously monitors robot velocity and displacement. When $v < 0.05\,\text{m/s}$ for $>0.75\,\text{s}$ while MPC reports "solved", it flags a freeze state and triggers `recovery_.trigger(...)`.
2. **Reduce `no_progress_timeout_s: 1.2`** (from 3.0s):
   Stalled or blocked episodes initiate recovery in $1.2\,\text{s}$ instead of waiting $3\,\text{s}$.
3. **Unblock Angular Twitch Suppression (`classical_mpc_node.cpp:L1028`)**:
   Removed the restrictive angular velocity check `(command.v > 0.25 || (command.v < 0.08 && std::abs(command.w) < 0.15))` that prevented recovery whenever the stalled robot twitched its steering ($\omega \approx \pm 0.35\text{ rad/s}$).

* **Result:** The robot immediately backs out of tight pinches where MPC constraints are unsatisfiable, preventing silent 100s timeouts.

---

### Bug 3 / Fix 3 (2026-08-27): Allowed Reverse Escapes in Safety Shield (`SHIELD TRAPPED` Fix)

#### Observed Issue in RViz / Gazebo:
* The robot approaches a narrow opening and halts completely ($v = 0\text{ m/s}$).
* In RViz, the global/local path (green) points straight through the gap, and the historical breadcrumb trail (blue) is visibly drawn behind the robot.
* Despite the breadcrumbs being present, the robot remains frozen and never reverses, eventually timing out at 100s.

#### Root Cause Breakdown:
1. **Why $v = 0$:**
   The corridor width is narrower than the MPC's total demanded clearance ($0.4559\text{ m}$). The QP solver cannot satisfy the clearance constraint at the current pose, so its minimum-cost slack solution is $v = 0\text{ m/s}$.
2. **Why Breadcrumbs Did Not Trigger:**
   In `classical_mpc_node.cpp:L1029`, the `no_progress` recovery timer contained the following guard:
   ```cpp
   // OLD CODE:
   else if ((command.v > 0.25 || (command.v < 0.08 && std::abs(command.w) < 0.15)) &&
            (stamp - last_progress_time_).seconds() > no_progress_timeout_s_)
   ```
   While forward velocity $v$ was 0, the solver was outputting small steering corrections ($\omega \approx \pm 0.35\text{ rad/s}$). Because $|\omega| \ge 0.15$, the condition `std::abs(command.w) < 0.15` evaluated to **`false`**. The system concluded the robot was "still turning", silently discarding the `no_progress` timer every single tick!

3. **Safety Shield Interception (`SHIELD TRAPPED`):**
   Even when `classical_mpc_node` triggered `ReverseToClearance` and issued $v = -0.35\text{ m/s}$, the downstream `safety_node` (the emergency safety shield) intercepted the command.
   Because an obstacle point was $1\text{ mm}$ inside the safety boundary at the current pose, `safe_at_scale` tested the current pose (`step = 0`), saw the existing penetration, and rejected every velocity command—including the reverse command that was moving *away* from the obstacle!
   `safety_node` then outputted $v = 0.0\text{ m/s}$ with the log:
   `SHIELD TRAPPED: an obstacle is inside the veto box ... and no escape motion increases it.`

#### How It Was Fixed:
1. **Unblocked `no_progress` recovery timer** in `classical_mpc_node.cpp:L1028`:
   ```cpp
   // NEW CODE:
   else if ((stamp - last_progress_time_).seconds() > no_progress_timeout_s_) {
     recovery_.trigger(recovery_context(state.pose, field, scan, veto_active));
     status = "no_progress_recovery";
     RCLCPP_INFO(get_logger(), "[Recovery] Triggered due to: no_progress.");
   }
   ```
2. **Allowed Reverse Escapes in Safety Shield** in `swept_footprint_shield.cpp:L57`:
   If an obstacle point was already intruding at `step = 0`, and the commanded reverse motion is strictly moving away from this obstacle ($d_k \ge d_0$), `safe_at_scale` allows the motion across all steps of the stopping horizon rather than rejecting it at `step = 0`.

Now, when `classical_mpc_node` commands reverse breadcrumb recovery, `safety_node` allows the reverse motion to pass through to the wheels, safely pulling the robot out of the pinch!

---

### Bug 4 / Fix 4 (2026-08-27): Strict Physical Rotation Clearance Gating in Recovery (`kRotateToGap`)

#### The Problem:
* In `recovery.cpp`, recovery attempts $\ge 2$ escalated to `rotate_after_reverse_ = true`, attempting to rotate toward the widest lidar gap (`kRotateToGap`).
* If the reverse maneuver reached `max_reverse_distance` ($1.5\,\text{m}$) before reaching open space, or if clearance was lost during the turn, the robot would still attempt an in-place rotation, clipping surrounding obstacles.

#### The Solution (`recovery.cpp:L221`):
Enforce hard clearance verification in all rotation states:
```cpp
case RecoveryState::kRotateToGap: {
  // Never spin in place if physical clearance is below rotation_radius (0.40m)
  if (ctx.clearance < ctx.rotation_radius) {
    if (!breadcrumb_exhausted(ctx)) {
      reverse_start_pose_ = ctx.pose;
      state_ = RecoveryState::kReverseToClearance;
    } else {
      state_ = boost_after_ ? RecoveryState::kRequestReplanClearance : RecoveryState::kRequestReplan;
    }
    state_elapsed_ = 0.0;
    return {0.0, 0.0};
  }
  ...
}
```
* **Result:** In-place rotations are strictly forbidden unless physical clearance $\ge 0.40\,\text{m}$ is guaranteed.

---

### Bug 5 / Fix 5 (2026-08-27): Path Commitment & Anti-Oscillation Lock (Left/Right Path Flip-Flop)

#### Observed Issue in RViz / Gazebo:
* After backing up or approaching a corridor with multiple candidate gaps (e.g. Left Gap vs Right Gap around an obstacle island), the robot rapidly twitches and oscillates its heading left and right in place without moving forward, eventually clipping the walls.
* In RViz, the green global path rapidly flips back and forth between the Left corridor and the Right corridor every $0.5\text{ seconds}$.

#### Root Cause:
* In `classical_mpc_node.cpp:L638`, the planner has a cooldown mechanism (`path_cooldown_s_ = 2.0s`) meant to prevent path flip-flopping.
* However, the cooldown check evaluated:
  ```cpp
  // OLD CODE:
  ahead_blocked = !path_validator_.is_path_clear(ahead, *grid_, false);
  ```
* `path_validator_.is_path_clear()` checks the vehicle's full rectangular footprint against the obstacle grid. Because corridor walls in BARN are narrow ($0.40 - 0.50\text{ m}$), the full footprint checker **always returns false** inside any narrow corridor!
* As a result, `ahead_blocked = true` was triggered on **every single replan cycle**, permanently bypassing the 2-second cooldown lock!
* The planner ping-ponged between the Left and Right corridors at $2\text{ Hz}$. The MPC commanded $\omega > 0$ to turn toward Path 1, then $\omega < 0$ to turn toward Path 2, trapping the robot in a destructive limit-cycle wiggle.

#### The Fix (`classical_mpc_node.cpp:L635`):
Changed `ahead_blocked` to check actual obstacle cell occupancy along the path centerline rather than whole-body footprint collision:
```cpp
// NEW CODE:
bool ahead_blocked = false;
if (global_path_.size() > 3 && grid_) {
  for (std::size_t i = 3; i < global_path_.size(); ++i) {
    const auto cell = grid_->world_to_cell(global_path_[i].x, global_path_[i].y);
    if (grid_->in_bounds(cell) && grid_->classify(cell) == barn_core::CellState::kOccupied) {
      ahead_blocked = true;
      break;
    }
  }
}
```
* **Result:** In narrow corridors, the robot now commits to its selected path throughout the cooldown duration and will not flip back and forth, completely eliminating left-right wiggling oscillations!

---

### Bug 6 / Fix 6 (2026-08-27): Full Reverse Guarantee (5cm Premature Reverse Abort)

#### Observed Issue in Gazebo Logs:
```text
Time: 19.24 (s), x: -1.66 (m), y: 7.51 (m)
[Recovery] Triggered due to: safety_veto. Action taken: ReverseToClearance
Time: 20.25 (s), x: -1.66 (m), y: 7.49 (m)
Time: 21.26 (s), x: -1.67 (m), y: 7.46 (m)   <--- Robot reversed 5 cm
Time: 24.27 (s), x: -1.66 (m), y: 7.50 (m)   <--- Drove forward 5 cm back into obstacle!
Time: 25.27 (s), x: -1.66 (m), y: 7.51 (m)   <--- Stuck again in stationary freeze!
```

#### Root Cause:
* In `recovery.cpp:L153`, the recovery state machine had an early exit for veto-escapes:
  ```cpp
  // OLD CODE:
  if (veto_escape_ && !ctx.veto_active && state_elapsed_ >= params_.veto_clear_min_rotate) {
    state_ = RecoveryState::kRequestReplan;
  }
  ```
* When `safety_veto` triggered reverse recovery, backing up just $5\text{ cm}$ cleared the front bumper from the obstacle, which immediately caused `!ctx.veto_active` to evaluate to `true`.
* The recovery machine concluded the escape was complete after only $0.2\text{s}$ ($5\text{ cm}$), long before the robot reached open space.
* The planner immediately re-planned a forward route through the same gap, driving forward $5\text{ cm}$ back into the pinch.

#### The Fix (`recovery.cpp:L153` & `L195`):
1. Restricted the early veto exit to only apply to rotation states (`kRotateToGap`), allowing `kReverseToClearance` to always run its full course.
2. Guaranteed that `kReverseToClearance` reverses at least **$0.40\text{ m}$** (`reversed >= 0.40`) before exiting to replan:
  ```cpp
  // NEW CODE:
  const double reversed = std::hypot(
    ctx.pose.x - reverse_start_pose_.x, ctx.pose.y - reverse_start_pose_.y);
  if (ctx.clearance >= ctx.rotation_radius && reversed >= 0.40) {
    state_ = RecoveryState::kRequestReplan;
    return {0.0, 0.0};
  }
  ```
* **Result:** The robot is guaranteed to back out deeply into open space before replanning, completely eliminating 5cm twitch loops.

---

### Bug 7 / Fix 7 (2026-08-27): MPC Clearance Margin Calibration (Fits Narrow Corridors)

#### Observed Issue in Gazebo Logs:
```text
[classical_mpc_node-31] [WARN] [classical_mpc_node]: MPC FREEZE: stopped with nothing blocking -- clearance 0.04 m, shield passive, status 'solved'. Demanded clearance is half_width + footprint_margin + obstacle_margin; if the corridor is narrower the constraint cannot be satisfied at this pose.
[classical_mpc_node-31] [WARN] [classical_mpc_node]: [Recovery] Triggered due to: mpc_freeze. Action taken: RotateToGap
```

#### Root Cause:
* In `classical_mpc.yaml`, the MPC safety constraint demands:
  $$\text{Total Demanded Clearance} = \text{half\_width}\,(0.2159\text{m}) + \text{footprint\_margin}\,(0.04\text{m}) + \text{obstacle\_margin}\,(0.20\text{m}) = \mathbf{0.4559\text{ m}}$$
* BARN narrow corridors are typically between $0.35\text{ m}$ and $0.45\text{ m}$ wide.
* Because the corridor is narrower than $0.4559\text{ m}$, the MPC QP solver finds the clearance constraint mathematically infeasible with $v > 0$ and clamps forward velocity to $v = 0.0\text{ m/s}$ (`MPC FREEZE`).
* This false freeze triggers `freeze_recovery`, causing the robot to turn toward a side gap (`RotateToGap`), replan, drive forward, hit the corridor again, and repeat the cycle endlessly.

#### The Fix (`classical_mpc.yaml:L230`):
Calibrated `obstacle_margin` from `0.20` down to `0.10`:
$$\text{Total Demanded Clearance} = 0.2159 + 0.04 + 0.10 = \mathbf{0.3559\text{ m}}$$
* **Result:** The demanded clearance now fits within BARN narrow corridors. The MPC drives straight through the corridor smoothly without freezing or triggering unnecessary recovery loops!

---

## 4. Subsystem Deep Dive: The Breadcrumb Memory & Reverse Tracking

### Why Breadcrumbs?
In narrow obstacle fields, executing arbitrary reverse motions or turning around blind risks collisions. However, the space the robot **just drove through** is physically guaranteed to be collision-free. By recording its traversed poses, the robot can backtrack along its exact historical trajectory.

### 1. Breadcrumb Recording & Memory Layout (`classical_mpc_node.cpp`)
* **Data Structure:** `std::vector<barn_core::Pose2D> breadcrumb_`
* **Recording Frequency:** Sampled in `control_step` at $20\,\text{Hz}$.
* **Spatial Spacing Gate (`breadcrumb_spacing_m = 0.10m`):**
  A new pose is only recorded if the robot has translated at least 0.10 m from the previous crumb:
  ```
  delta_d = sqrt((x_curr - x_last)^2 + (y_curr - y_last)^2) >= 0.10 m
  ```
* **Ring Buffer Capping (`breadcrumb_max_points = 160`):**
  Holds up to 160 points (16 m of historical travel). When full, oldest points at index 0 are popped:
  ```cpp
  if (breadcrumb_.size() > breadcrumb_max_) {
    breadcrumb_.erase(breadcrumb_.begin());
  }
  ```
* **Active Protection:** Poses are **only recorded during normal forward navigation** (`!recovery_.active()`). During recovery maneuvers, recording is frozen so reversing does not overwrite the forward trail.

### 2. Reverse Pure Pursuit Controller (`recovery.cpp`)

```
 [Target Crumb (goal)] <---- Accumulated Arc = 0.50m ---- [Nearest Crumb]
        ^                                                        ^
        |                                                        |
   Virtual Heading (pose.yaw + PI) ------------------------ Robot Current Pose
```

When `RecoveryState::kReverseToClearance` is active:
1. **Find Nearest Breadcrumb:** Scans `breadcrumb_` for index `i` minimizing Euclidean distance to `ctx.pose`.
2. **Lookahead Accumulation:** Walks backwards from index `i` toward older crumbs (`i-1, i-2, ...`) until the cumulative Euclidean arc length reaches `reverse_lookahead` (0.50 m), selecting point `goal`.
3. **Virtual-Rear Differential Kinematics:**
   Treats the rear of the differential robot as a virtual forward-facing vehicle:
   ```
   virtual_heading = wrap_angle(pose.yaw + PI)
   bearing = atan2(y_goal - y_robot, x_goal - x_robot)
   alpha = wrap_angle(bearing - virtual_heading)
   ```
4. **Steering Command Generation:**
   ```
   yaw_rate = clamp(2.0 * v_reverse * sin(alpha) / lookahead_dist, -max_yaw_rate, max_yaw_rate)
   v_cmd = -0.35 m/s
   ```
5. **Termination Condition:**
   * Stops when `clearance >= rotation_clearance_m_` (0.40 m).
   * Or when `reversed_distance >= max_reverse_distance` (1.5 m) or `timeout >= 6.0 s`.

---

## 5. How to Configure and Switch Planners

In [`ros2_ws/src/barn_bringup/config/classical_mpc.yaml`](file:///home/masuk/barn-2027-prep/ros2_ws/src/barn_bringup/config/classical_mpc.yaml):

### To Run the Original Elastic-Band Planner:
```yaml
classical_mpc_node:
  ros__parameters:
    use_bspline_planner: false
```

### To Run the B-Spline + TOPP-RA Planner:
```yaml
classical_mpc_node:
  ros__parameters:
    use_bspline_planner: true
    n_control_points: 20
    spline_opt_iterations: 30
    spline_smooth_weight: 0.40
    spline_anchor_weight: 0.15
    spline_obstacle_weight: 0.30
    spline_curvature_weight: 0.50
    track_width: 0.4318
    wheel_v_max: 3.5
    wheel_a_max: 2.5
    topp_ra_samples: 100
```
```

---

## 6. Bug 8 / Fix 8 (2026-08-28): Finish Line Lateral Sweep & Goal Overshoot Fixes

| File | Parameter / Line | Change | Why / Benefit |
|------|------------------|--------|---------------|
| **`classical_mpc.yaml`** | `goal_overshoot_m` | `0.75` $\rightarrow$ `1.00` | Drives the robot 1.0m past believed goal, ensuring it crosses the evaluator's ground-truth 1.0m finish circle on the first pass without short-stopping. |
| **`classical_mpc.yaml`** | `finish_sweep_leg_timeout_s` | *(added)* `2.0` (was 15.0s default) | Prevents 60s wiggling timeouts at the finish line by capping each lateral sweep leg at 2.0s instead of 15.0s. |
| **`classical_mpc.yaml`** | `finish_sweep_max_m` / `step_m` | `2.0 / 1.0` $\rightarrow$ `1.5 / 0.5` | Finer 0.5m steps for faster, tighter lateral finish sweeps. |
| **`run_dev_suite.sh`** | `RESULTS_DIR` / `OUT_FILE` | Relative $\rightarrow$ **Absolute Paths** (`${REPO_ROOT}/...`) | Prevents Python `FileNotFoundError` in `barn_runner.py` when writing `raw_results.txt`. |
| **`run_single_world.sh`** | `RESULTS_DIR` / `OUT_FILE` | Relative $\rightarrow$ **Absolute Paths** (`${REPO_ROOT}/...`) | Ensures single-world evaluation runs output to valid absolute directories. |

### Case Study: "Why did the robot sweep left/right at the finish line and time out?"

#### Problem:
1. In open space past the obstacles, the robot arrived at what its onboard mapping/odometry believed to be the goal ($\le 1.0\,\text{m}$).
2. Due to small residual mapping/odometry drift, the robot was slightly outside the evaluator's ground-truth 1.0m finish circle, so the evaluator did not immediately end the trial.
3. `goal_adapter_node` initiated `run_finish_sweep()`, publishing goals +1.0m to the right, then -1.0m to the left.
4. Because `finish_sweep_leg_timeout_s` defaulted to **15.0 seconds** per leg, waiting on 4 sweep legs took $4 \times 15 = 60\text{ seconds}$ of wiggling at the finish line, causing trial timeouts at 100s.

#### Fix:
1. Set `goal_overshoot_m: 1.00` in `classical_mpc.yaml` so the robot aims 1.0m further forward, crossing the evaluator's ground-truth circle on its first pass.
2. Set `finish_sweep_leg_timeout_s: 2.0` so any required sweep takes only ~4 seconds total instead of 60 seconds.

---

## 7. Bug 9 / Fix 9 (2026-08-28): TOPP-RA Dynamic Cornering Limits (Fixing Tight-Turn Collisions)

| File | Component / Line | Change | Why / Benefit |
|------|------------------|--------|---------------|
| **`local_plannerBspline.hpp`** | `LocalPlannerBsplineParams` | Added `max_lateral_accel: 1.5` | Exposes maximum lateral (centripetal) acceleration limit for velocity profiling. |
| **`local_plannerBspline.cpp`** | `topp_ra()` velocity ceiling | Added `max_yaw_rate / kappa` and `sqrt(max_lateral_accel / kappa)` bounds | Pre-decelerates the robot into sharp turns so steering demands never exceed physical motor limits. |
| **`classical_mpc_node.cpp`** | `LocalPlannerBspline` initialization | Bound `bspline.max_lateral_accel = local.max_lateral_accel` | Ensures the B-Spline profiler receives the calibrated lateral acceleration budget from configuration. |

### Case Study: "Why did the robot collide in tight corners during high-speed runs (e.g. World 228)?"

#### Problem:
* In testing (such as World 228), traversal speeds were fast (14–15s), but collisions occurred when rounding narrow corners.
* **Root Cause:** TOPP-RA's velocity ceiling was previously constrained only by single-wheel maximum velocity ($v_{\text{wheel\_max}} = 3.5\text{ m/s}$).
* For a sharp curve of curvature $\kappa = 2.0\text{ rad/m}$, TOPP-RA computed that the outer wheel only spun at $2.44\text{ m/s}$ ($< 3.5\text{ m/s}$), so it commanded full linear speed ($2.0\text{ m/s}$) through the turn.
* Rounding a $\kappa = 2.0$ corner at $2.0\text{ m/s}$ requires:
  1. An angular velocity of $\omega = v \cdot \kappa = 2.0 \times 2.0 = 4.0\text{ rad/s}$ (robot hardware limit is $1.5\text{–}2.5\text{ rad/s}$).
  2. A sideways lateral acceleration of $a_{\text{lat}} = v^2 \cdot \kappa = 2.0^2 \times 2.0 = 8.0\text{ m/s}^2$ (safe limit is $1.5\text{ m/s}^2$).
* Because the robot's physical motors could not steer at $4.0\text{ rad/s}$, the MPC understeered, drifted wide by $10\text{–}15\text{ cm}$, and clipped the corner wall.

#### Fix:
Integrated kinematic yaw rate and centripetal acceleration limits directly into TOPP-RA's velocity ceiling:
1. **Yaw Rate Bound:** $\dot{s} \le \frac{\omega_{\text{max}}}{\kappa}$
2. **Lateral Acceleration Bound:** $\dot{s} \le \sqrt{\frac{a_{\text{lat\_max}}}{\kappa}}$

$$\dot{s}_{\text{max}}(s) = \min \left( \frac{v_{\text{wheel\_max}}}{1 + \frac{W}{2}\kappa(s)}, \; \frac{\omega_{\text{max}}}{\kappa(s)}, \; \sqrt{\frac{a_{\text{lat\_max}}}{\kappa(s)}}, \; v_{\text{max}} \right)$$

* **Result:** In sharp corners (kappa = 2.0), TOPP-RA now automatically calculates a safe entry speed (~0.75 m/s), pre-decelerating smoothly before the corner and accelerating back to full speed on straightaways, completely preventing corner clipping collisions.

---

## 8. Bug 10 / Fix 10 (2026-08-29): Balanced Motion Recovery & Dead-End Acceptance (4 Regression Fixes)

An interactive visualizer has been built to demonstrate all 4 scenarios in real-time:
👉 **Interactive Visualizer Tool:** [`behavior_visualizer.html`](file:///home/masuk/barn-2027-prep/behavior_visualizer.html)

### Summary of the 4 Regressions & Balanced Solutions

| Regression Issue | Root Cause in Safety Logic | Balanced Fix Implemented | Files Changed |
| :--- | :--- | :--- | :--- |
| **1. Clearance Gating Loop** | Attempting in-place turn inside 38cm corridor failed `< 0.40m` gate, causing endless reverse/re-enter loops. | Calibrated `obstacle_margin: 0.10` so forward path clearance is satisfied; robot smoothly creeps forward through narrow corridors. | `classical_mpc.yaml` |
| **2. Tunnel Reverse Exhaustion** | Exit condition required `clearance >= 0.40m`, forcing the robot to back up the entire 3m tunnel length. | Bounded reverse exit: allows replanning when `reversed >= 0.40m` AND front veto is clear (`!ctx.veto_active`), saving tunnel progress. | `recovery.cpp:L197` |
| **3. Anti-U-Turn Deadlock** | Rejection of 180° candidate paths trapped the robot fighting the planner when encountering dead ends. | **Dead-End Escape Acceptance**: Accepts the 180° escape route, then uses breadcrumbs to reverse out into open space before turning. | `classical_mpc_node.cpp:L692` |
| **4. Sensitive Stall Trigger** | Watchdog checked only `v < 0.08 m/s`, falsely interrupting slow, deliberate cornering maneuvers. | **Dual-Motion Progress Watchdog**: Resets stall timer if robot is commanding translation OR actively steering (`\|w\| > 0.15 rad/s`). | `classical_mpc_node.cpp:L1037`, `classical_mpc.yaml:L384` |

---

### Detailed Breakdown of the 4 Fixes

#### 1. Clearance-Gated Corridor Progression (Narrow Corridor Drive-Through)
* **Problem:** In narrow corridors (38 cm width), wall clearance is ~35 cm. When previous rules required 40 cm clearance to execute maneuvers, the robot assumed it was trapped, reversed out, and re-entered in an infinite loop.
* **Fix:** The demanded MPC clearance was calibrated to 0.3559 m (`obstacle_margin: 0.10`). As long as the corridor is unblocked, the robot does not trigger emergency turns or reverse recovery; it creeps forward at controlled speeds (0.15–0.25 m/s) with tight lateral bounds straight through to the goal.

#### 2. Bounded Reverse Recovery in Long Tunnels
* **Problem:** `kReverseToClearance` previously demanded both `reversed >= 0.40m` AND `ctx.clearance >= 0.40m`. Inside a 3-meter tunnel, clearance is 35 cm throughout the entire length, so the robot backed up 3 meters all the way to the start, discarding all travel progress.
* **Fix (`recovery.cpp:L197`):**
  ```cpp
  // Enforce at least 0.40m reverse to avoid 5cm twitch loops, but allow exiting
  // to replan once the front is clear (!ctx.veto_active) to avoid exhausting long tunnels.
  if (reversed >= 0.40 && (ctx.clearance >= ctx.rotation_radius || !ctx.veto_active)) {
    if (rotate_after_reverse_ && ctx.clearance >= ctx.rotation_radius) {
      target_yaw_ = barn_core::wrap_angle(ctx.pose.yaw + widest_gap_heading(ctx.scan));
      state_ = RecoveryState::kRotateToGap;
    } else {
      state_ = boost_after_ ? RecoveryState::kRequestReplanClearance : RecoveryState::kRequestReplan;
    }
    state_elapsed_ = 0.0;
    return {0.0, 0.0};
  }
  ```
  * **Result:** Backs up 40 cm (clearing the front bumper), then safely re-evaluates the local corridor plan without exhausting breadcrumbs.

#### 3. Dead-End Escape Acceptance (No Planner Fighting)
* **Problem:** When reaching a dead end, A* computes a 180° U-turn route. The Anti-U-Turn guard previously rejected this path with `goto done`, resulting in a deadlock where the planner repeatedly sent the escape route and the guard repeatedly rejected it.
* **Fix (`classical_mpc_node.cpp:L692`):**
  ```cpp
  if (heading_err > (M_PI / 2.0) && (!std::isfinite(cur_clearance) || cur_clearance < rotation_clearance_m_)) {
    request_reverse_recovery_ = true;
    planner_status_ = "escape_uturn_reversing";
    // Adopt the valid escape path so the system knows where to go after backing out
    global_path_ = std::move(candidate);
    path_to_publish = global_path_;
    replan_completed_ = true;
    last_path_swap_time_ = now();
    goto done;
  }
  ```
  * **Result:** The new escape route is accepted into memory. The robot backs out along known-safe breadcrumbs to open space, where it then cleanly turns around and follows the new route to the goal.

#### 4. Dual-Motion Progress Watchdog (Translation + Active Cornering)
* **Problem:** During sharp 90° corners, the robot creeps slowly (`v = 0.03 m/s`) while steering hard (`w = 0.45 rad/s`). The watchdog timer only looked at forward speed and triggered an emergency reverse after 1.2s, interrupting valid cornering.
* **Fix (`classical_mpc_node.cpp:L1037` & `classical_mpc.yaml:L384`):**
  * Handled true translation displacement (`hypot > 0.08m`) across curved arcs while ensuring in-place stalls trigger recovery.

---

## 9. Bug 11 / Fix 11 (2026-08-29): Elimination of In-Place Steering Twitch Masking & Guaranteed Pocket Reverse

### Problem:
* In testing (such as World 11 at $y \approx 7.2\text{m}$), the robot approached an obstacle cluster, and the forward route into the right wall was blocked.
* The robot came to a complete forward stop ($v = 0.0\text{ m/s}$), but the MPC controller rapidly oscillated steering left and right ($\omega \approx \pm 0.25\text{ rad/s}$) trying to find an opening.
* The robot sat permanently in place wiggling left and right without triggering Breadcrumb Reverse to back out.

### Root Causes Discovered:
1. **Commanded vs. Actual Velocity Mismatch:** In `classical_mpc_node.cpp`, the MPC solver was outputting a desired forward speed $v = 0.20\text{ m/s}$, which caused `command.v > 0.08` to evaluate to `true` and continuously reset `last_progress_time_`, even though the safety shield was **vetoing the command to $0.0\text{ m/s}$ at the wheels**!
2. **Premature Reverse Abort (`blocked_timeout`):** In `recovery.cpp:L164`, `blocked_elapsed_` was checking `ctx.veto_active` during `kReverseToClearance`. Because the robot was touching a front obstacle when recovery started, `veto_active` was `true`, causing `blocked_timeout` (0.7s) to **abort the reverse maneuver after only 0.7 seconds** and force a replan on the spot.
3. **Attempt 1 Replan In-Place:** In `Recovery::begin_episode`, when local clearance in a pocket was $\ge 0.40\text{m}$, Attempt 1 skipped reversing entirely and just requested a replan on the spot (`kRequestReplan`). Planning from the exact same trapped pocket repeatedly generated the same blocked path.

### How It Was Fixed:
1. **Veto-Aware Physical Motion Watchdog (`classical_mpc_node.cpp:L1040`):**
   * Progress requires **unvetoed forward motion** (`command.v > 0.08 && !veto_active && std::abs(state.v) > 0.04`) or physical spatial displacement (`hypot > 0.08m`).
   * When the robot is stopped by a front veto or wall, the timer does not reset and reliably fires after $1.0\text{ second}$.
2. **Uninterrupted Breadcrumb Reverse (`recovery.cpp:L164`):**
   * Restricted `blocked_timeout` aborts to rotation states (`kRotateToGap`) only. `kReverseToClearance` is **never aborted by front obstacle vetoes**, allowing it to execute its full reverse trajectory along breadcrumbs back into open space.
3. **Guaranteed Pocket Reverse (`recovery.cpp:L112`):**
   * Whenever recovery triggers and breadcrumbs exist, the robot **always executes `kReverseToClearance` first**, backing up at least $40\text{ cm}$ along its collision-free history into open space before replanning.

### What Was Changed to What:

#### In `classical_mpc_node.cpp`:
```cpp
// OLD CODE (Reset timer on desired command even when vetoed to 0 m/s):
} else if (command.v > 0.08) {
  last_progress_time_ = stamp;
}

// NEW CODE (Only resets if actively moving forward without veto):
} else if (command.v > 0.08 && !veto_active && std::abs(state.v) > 0.04) {
  last_progress_time_ = stamp;
} else if ((stamp - last_progress_time_).seconds() > no_progress_timeout_s_) {
  recovery_.trigger(recovery_context(state.pose, field, scan, veto_active));
  status = "no_progress_recovery";
}
```

#### In `recovery.cpp`:
```cpp
// OLD CODE (Aborted reverse after 0.7s because front veto was active):
const bool motion_state = state_ == RecoveryState::kReverseToClearance || state_ == RecoveryState::kRotateToGap;
if (motion_state && ctx.veto_active) {
  blocked_elapsed_ += std::max(0.0, dt);
  if (blocked_elapsed_ >= params_.blocked_timeout) {
    state_ = RecoveryState::kRequestReplanClearance;
    ...
  }
}

// NEW CODE (Reverse escape is never aborted by front vetoes):
const bool motion_state = state_ == RecoveryState::kRotateToGap;
if (motion_state && ctx.veto_active) { ... }
```

* **Result:** When the robot approaches a dead-end pocket and stops, the stall timer fires within $1.0\text{s}$, the robot reverses $40\text{–}50\text{ cm}$ back into the open space without being interrupted, and from the open space A* immediately finds the wide open left corridor and navigates smoothly to the goal!

---

## 10. Bug 12 / Fix 12 (2026-08-29): Pivot-First Alignment in Reverse Pure Pursuit

### The Core Problem:
* When the robot drove toward a blocked corridor, the obstacle wall ahead on the right pushed the robot's front to the left (via distance-field gradient repulsion in MPC).
* As the front swung left, the **rear bumper swung right**, pointing directly toward an obstacle island behind its rear-right wheel.
* When reverse recovery was commanded, the reverse controller commanded both backward velocity (v = -0.35 m/s) and steering (yaw rate w) at the exact same time.
* The rear bumper began translating diagonally backward toward the obstacle before the wheels could turn.
* The downstream **Swept-Footprint Safety Shield** projected this diagonal backward trajectory into the obstacle and vetoed the command (v = 0.0 m/s), leaving the robot trapped and unable to reverse.

### Root Cause:
* In `recovery.cpp:L88`, `reverse_command()` always issued full linear reverse speed (v = -0.35 m/s) regardless of the heading error between the robot's virtual rear heading and the target breadcrumb.
* When the angle was large (30 to 60 degrees), diagonal linear translation caused rear corner collisions with side obstacles.

---

### Deep Dive: WHY and HOW the Robot Rotates First Then Reverses

#### 1. WHY It Needs to Rotate First:
* **Differential Drive Physics:** On a differential-drive chassis, turning the nose left swings the rear bumper right.
* **The Danger of Immediate Backward Drive:** If the robot commands backward motion while sitting at an angle, the rear translates diagonally across the ground directly into obstacles behind it. The safety shield sees this collision vector and immediately slams on the emergency brakes (vetoing speed to 0), trapping the robot.
* **The In-Place Rotation Advantage:** By **spinning on its own center in place** (keeping its x, y location completely stationary), the rear bumper swings away from the obstacle and aligns directly with the safe blue breadcrumb trail without moving towards any side walls.

#### 2. HOW It Works Step-by-Step:

* **Phase 1: Check Alignment Angle (Every 50ms):**
  The reverse pure pursuit tracker looks at the breadcrumb point about 50 cm behind the robot and calculates the angle (alpha) between the robot's rear heading and the target crumb.
  * If angle > 26 degrees (0.45 radians): The robot is misaligned.
  * If angle <= 26 degrees: The rear is safely pointed along the breadcrumb trail.

* **Phase 2: In-Place Pivot (Rotate First, v = 0.0 m/s):**
  When the angle is greater than 26 degrees:
  * Forward / Backward speed is locked to **EXACTLY ZERO (v = 0.0 m/s)**.
  * Angular velocity is commanded to pivot on the spot: `w = clamp(2.0 * sin(alpha), -rotate_speed, rotate_speed)`.
  * The robot pivots around its center point, smoothly swinging its rear bumper in line with the breadcrumbs without translating backward into obstacles.

* **Phase 3: Smooth Reverse Drive (Drive Backward along Breadcrumbs):**
  Once the angle drops below 26 degrees:
  * Linear reverse speed smoothly ramps up using cosine alignment: `v = -0.35 * cos(alpha) m/s`.
  * When perfectly aligned, the robot reverses at full speed (-0.35 m/s) along the exact historical path it drove in on.
  * Because the rear is already aligned, the swept footprint box stays inside the safe corridor and never triggers the safety shield veto.

```
[ Robot Stuck at Wall at an Angle ]
                |
                v
[ Phase 1: Pure In-Place Pivot (v = 0.0 m/s, w != 0) ]
  -> Swings rear bumper in place to line up with breadcrumbs
                |
                v
[ Phase 2: Rear Aligned (Angle <= 26 degrees) ]
                |
                v
[ Phase 3: Smooth Reverse Drive (v = -0.35 * cos(angle) m/s) ]
  -> Backs up cleanly along breadcrumbs into open space
                |
                v
[ Phase 4: A* Global Replan from Open Space ]
  -> Drives forward through the open corridor to the goal!
```

---

### Code Implementation (`recovery.cpp:L79`):

```cpp
// NEW CODE (Pivot-First Alignment in recovery.cpp):
const double virtual_heading = barn_core::wrap_angle(ctx.pose.yaw + M_PI);
const double bearing = std::atan2(goal.y - ctx.pose.y, goal.x - ctx.pose.x);
const double alpha = barn_core::wrap_angle(bearing - virtual_heading);
const double lookahead = std::max(0.2, std::hypot(goal.x - ctx.pose.x, goal.y - ctx.pose.y));

double v_cmd = -params_.reverse_speed;
double w_cmd = 0.0;

if (std::abs(alpha) > 0.45) {
  // Pure in-place pivot to align rear with trail before translating
  v_cmd = 0.0;
  w_cmd = std::clamp(2.0 * std::sin(alpha), -params_.rotate_speed, params_.rotate_speed);
} else {
  // Smooth blended reverse pure pursuit with cosine scaling
  v_cmd = -params_.reverse_speed * std::cos(alpha);
  w_cmd = 2.0 * params_.reverse_speed * std::sin(alpha) / lookahead;
  w_cmd = std::clamp(w_cmd, -params_.rotate_speed, params_.rotate_speed);
}

return {v_cmd, w_cmd};
```

* **Result:** The robot rotates its rear bumper in place safely without translating into adjacent obstacles, aligns with the collision-free breadcrumb path, and then reverses smoothly into open space!

---

## 11. Bug 13 / Fix 13 (2026-08-30): Centerline Cell Validation for Global Plans (Eliminating Replan Rejection)

### Problem:
* In testing (such as World 11 at $y \approx 7.0\text{m}$), the robot approached an obstacle wall. The green path ahead was clearly blocked by obstacles.
* Although A* found a valid alternative escape path leading through the open left corridor, the robot **never adopted the new plan** and remained frozen in place holding the old blocked path into the wall.

### Root Cause:
* In `classical_mpc_node.cpp:L619`, every candidate path returned by A* was evaluated with `path_validator_.is_path_clear(candidate, *grid, false)`.
* In `path_validator.cpp`, `is_path_clear()` performed a **rigid whole-body rectangular footprint check** ($0.51\text{ m}$ wide box) across every single path segment.
* In BARN environments, corridors are narrow ($0.38\text{–}0.50\text{ m}$). The rigid rectangular footprint box collided with the corridor walls on the candidate escape path, causing `path_validator_` to return `accepted = false`.
* Because `accepted` evaluated to `false`, `classical_mpc_node` **discarded the valid new escape plan** and retained the old blocked path into the wall.

### How It Was Fixed (`path_validator.cpp:L8`):
Updated `PathValidator::is_path_clear` to perform dense centerline cell occupancy validation:
* Samples segments along the path at half-grid resolution ($0.025\text{–}0.05\text{ m}$).
* Verifies that the path centerline does not intersect occupied obstacle cells (`grid.classify(cell) == CellState::kOccupied`).
* Skips `path.front()` (the current robot position) so the robot is not falsely penalized when sitting snug against a wall.
* Allows fine spatial clearance and vehicle footprint steering to be handled dynamically by the local MPC controller and safety shield.

```cpp
// NEW CODE (Centerline cell validation in path_validator.cpp):
bool PathValidator::is_path_clear(
  const Path2D & path, const barn_core::OccupancyGrid2D & grid,
  bool unknown_is_obstacle) const
{
  if (path.empty()) {
    return false;
  }

  const double res = std::max(0.01, grid.resolution() * 0.5);
  for (std::size_t i = 1; i < path.size(); ++i) {
    const auto & p0 = path[i - 1];
    const auto & p1 = path[i];
    const double dist = std::hypot(p1.x - p0.x, p1.y - p0.y);
    const int steps = std::max(1, static_cast<int>(std::ceil(dist / res)));
    for (int step = 1; step <= steps; ++step) {
      const double frac = static_cast<double>(step) / steps;
      const double x = p0.x + frac * (p1.x - p0.x);
      const double y = p0.y + frac * (p1.y - p0.y);
      const auto cell = grid.world_to_cell(x, y);
      if (!grid.in_bounds(cell)) {
        return false;
      }
      const auto state = grid.classify(cell);
      if (state == barn_core::CellState::kOccupied ||
          (unknown_is_obstacle && state == barn_core::CellState::kUnknown))
      {
        return false;
      }
    }
  }
  return true;
}
```

* **Result:** Whenever the forward path is blocked, A*'s new escape path through the open left corridor is immediately validated and accepted into `global_path_`. The green trajectory snaps to the open corridor and the robot navigates out cleanly!

---

## 12. Bug 14 / Fix 14 (2026-09-04 22:14:10): Clearance-Aware Turning Velocity Profiling & Real-Time Yaw Governor

### The Problem:
* When the robot approached a corner or changed heading to follow a new path corridor, it maintained high linear velocity (1.2 to 1.5 m/s) while turning.
* In narrow corridors (less than 0.50 m wide), turning at high speed caused frequent collisions.
* The robot's outer corners swung outward into the corridor walls, or the swept stopping horizon intersected obstacles, triggering emergency safety shield vetoes and twitching stalls.
* However, in open spaces, slowing down for turns was unnecessary and reduced transit speed. The robot should only slow down when the space is narrow, and should move quickly through open curves.

### Root Cause:
1. **The 15% Clearance Slowdown Floor:** In `local_planner.cpp`, the clearance scale was hardcoded between 0.85 and 1.00. Even in the tightest corridor where the robot had only 3 cm of side room, speed was reduced by only 15%. In `local_plannerBspline.cpp`, clearance was not considered at all in the velocity profiler.
2. **Curvature Limits Treated Narrow and Open Spaces Identically:** The maximum lateral acceleration budget was fixed at 3.0 m/s^2 everywhere. A sharp turn allowed 1.41 m/s in an open field, and still allowed 1.20 m/s inside a narrow corridor.
3. **Heading Alignment Gate Allowed 30% Speed In Tight Turns:** When heading error exceeded 60 to 90 degrees, the old heading gate still commanded 30% forward speed (0.45 to 0.60 m/s) regardless of whether the robot was in open space or surrounded by tight walls. This forward creep pushed the front corner directly into the wall before the turn could complete.

### The Physics of Differential Drive Cornering:
For a rectangular robot (length 0.508 m, width 0.430 m):
* When driving straight, the robot occupies a corridor width of 0.430 m.
* When turning while moving forward, the diagonal corners swing outward along an expanded circle. The outer front corner sweeps a wider path than the wheels.
* If linear speed is high (over 1.0 m/s), tracking errors (5 to 10 cm) and the stopping distance box (0.30 to 0.60 m) cause immediate collisions with corridor walls.
* But when linear speed is reduced to a crawl (0.35 m/s), the stopping distance collapses to under 3 cm, corner swing is negligible, and the robot can navigate narrow 45 cm gaps with millimeter precision.
* In open space (clearance over 0.80 m), the outer corners have meters of free space, so the robot can safely take sweeping curves at 1.5 to 2.5 m/s.

### How It Was Fixed:
The solution is implemented across three coordinated layers:

#### 1. Clearance-Aware Heading Gate (Entering Turns)
When turning onto a new path segment, allowable forward speed depends directly on the available side clearance:
* Clearance ratio is calculated between minimum clearance (0.26 m) and open clearance (0.80 m):
  `clearance_ratio = clamp((clearance - min_clearance) / (open_clearance - min_clearance), 0.0, 1.0)`
* **In Narrow Space (clearance_ratio near 0):**
  If heading error is greater than 20 degrees (0.35 radians), forward speed drops to exactly 0.0 m/s. The robot stops translating and performs a pure in-place pivot until its nose points straight down the narrow corridor. It then drives forward without corner clipping.
* **In Open Space (clearance_ratio near 1):**
  If heading error is within 60 degrees (1.05 radians), the robot is allowed to drive forward in a wide, high-speed arc (1.5 to 2.0 m/s) with a minimum speed floor of 40%, keeping smooth momentum.

```
                    Corridor Clearance Check
                               |
            +------------------+------------------+
            |                                     |
            v                                     v
[ Narrow Space: Clearance < 0.35 m ]   [ Open Space: Clearance > 0.80 m ]
  - Heading Error > 20 degrees:          - Heading Error > 20 degrees:
      Linear Speed = 0.0 m/s                 Linear Speed = 1.5 - 2.0 m/s
      (Pure In-Place Pivot)                  (Smooth High-Speed Arc)
  - Aligns nose before moving            - Wide clearance absorbs corner swing
  - Zero wall clipping!                  - Maximum transit performance!
```

#### 2. Clearance-Scaled Lateral Acceleration Budget (Continuous Curves)
Instead of a fixed 3.0 m/s^2 budget everywhere, allowable lateral acceleration scales dynamically with clearance:
`lateral_budget = tight_lateral_accel + clearance_ratio * (open_lateral_accel - tight_lateral_accel)`
* **In Tight Corridors (clearance < 0.35 m):** Lateral acceleration is limited to 0.50 m/s^2.
  For a sharp curve with curvature 1.5, turning speed is capped at 0.57 m/s (or throttled down to the 0.35 m/s crawl floor). The robot glides smoothly with minimal stopping distance.
* **In Open Space (clearance > 0.80 m):** Lateral acceleration is granted up to 3.50 m/s^2.
  For curvature 1.5, turning speed reaches 1.52 m/s, allowing the robot to charge through open bends at high speed.

#### 3. Real-Time Command Governor (`classical_mpc_node.cpp`)
As a final real-time safety layer before commands are sent to the robot:
* The node checks the commanded angular velocity against the distance field clearance from the laser scan.
* If the MPC commands a high yaw rate while the robot is near a wall, the governor automatically caps linear velocity:
  `safe_turn_speed = max(crawl_speed, lateral_accel(clearance) / (|yaw_rate| + 0.001))`
  `command.linear = min(command.linear, safe_turn_speed)`
* If the robot is in a narrow corridor and turns sharply, linear speed is throttled to 0.35 m/s.
* If the robot is in open space, the governor does not restrict the command, allowing full speed.

### Result:
* In narrow chicanes and tight corridor corners, the robot slows to 0.35 to 0.50 m/s or pivots in place, completely eliminating corner clipping and safety shield traps.
* In open clearings, the robot corners aggressively at 1.5 to 2.5 m/s, preserving fast lap times.

---

## 13. Bug 15 / Fix 15 (2026-09-04 22:25:00): Path Hysteresis & Commitment (Eliminating Path Switching & Flip-Flops)

### The Problem:
* While navigating, the robot would plan a path forward, and then shortly after replan an alternate path, repeatedly switching back and forth between two routes.
* This caused the robot to wiggle or oscillate its steering between two corridor choices instead of committing to one and driving through.

### Root Cause:
* In `classical_mpc_node.cpp`, the replanning loop ran periodically.
* Path swaps were previously protected only by a temporary timer `path_cooldown_s_` (2.0 seconds).
* Once the 2 seconds expired, any valid candidate path produced by A* would unconditionally overwrite `global_path_`, even if:
  1. The current path ahead was 100% clear.
  2. The new path was essentially identical or only millimeters shorter.
  3. The new path led through an alternate fork, causing the robot to reverse its steering intention.
* As soon as the robot adjusted toward Path B, the next replan would find Path A slightly shorter again and swap back, causing a continuous switching cycle.

### How It Was Fixed (`classical_mpc_node.cpp:L637` & `classical_mpc.yaml:L116`):
Implemented permanent **Path Hysteresis & Commitment**:
1. **Clearance Check Ahead:** The planner inspects the robot's current path waypoints ahead against the occupancy grid.
2. **If Current Path Is Blocked Ahead:** The robot immediately accepts the new candidate path or escape route. Safety and obstacle clearance always take top priority!
3. **If Current Path Is Clear Ahead:** The robot stays committed to its existing path! It refuses to switch to any alternate candidate path unless the candidate path is at least **15% to 20% shorter/better** (`path_improvement_ratio = 0.85`):
   `is_much_better = candidate_length < 0.85 * retained_length`
   * If `is_much_better` is false, the candidate is discarded with status `"retained_hysteresis"`, and the robot continues driving down its current path smoothly.
   * If `is_much_better` is true (a genuine shortcut is discovered), the swap is permitted.

```
                    Replanner Candidate Arrives
                                 |
              Is current path blocked ahead by obstacle?
                                 |
                 +---------------+---------------+
                 | YES                           | NO
                 v                               v
       [ Switch to New Path ]        Is candidate >= 15% shorter?
       (Immediate Obstacle Escape)               |
                                     +-----------+-----------+
                                     | YES                   | NO
                                     v                       v
                           [ Accept Shortcut ]      [ Retain Current Path ]
                           (Much Better Route)      (Commitment Hysteresis)
                                                    Zero Path Switching!
```

### Result:
* Eliminates path flip-flops and decision jitter at forks and corridors.
* The robot commits firmly to its chosen corridor and drives through cleanly without hesitating or switching routes.

---

## 14. Bug 16 / Fix 16 (2026-09-04 22:30:00): Fast Turning in Open Space & Strict Narrow-Turn Slowdown Gating

### The Problem:
* The robot slowed down excessively whenever turning, even in wide-open clearings or moderate corridors where plenty of side clearance existed (as demonstrated in user testing imagery).
* In open space, the robot should comfortably execute fast sweeping turns at 1.5 to 2.5 m/s or higher. Slowing down should ONLY occur when the turn is very sharp AND the space is genuinely narrow.

### Root Cause Analysis:
1. **Distant Curvature Window Lookahead:**
   * The speed profile previously looked ahead 3.0 meters (`curvature_lookahead_m = 3.0`).
   * In open areas, if the path entered any bend or corridor 2.5 to 3.0 meters ahead, the planner set `limiting_clearance` and `lookahead_curvature` based on that distant point. This prematurely slammed the brakes while the robot was still out in wide open space.
2. **Unconditional Clearance Scaling:**
   * The local planner and B-spline TOPP-RA previously multiplied linear reference velocity by an unconditional `clearance_scale` whenever obstacle clearance was below `desired_clearance` (0.60m).
   * This throttled speed towards crawl speed (0.35 m/s) even along straight paths or gentle wide-radius curves.
3. **Over-Sensitive Command Governor:**
   * The real-time command governor throttled forward velocity whenever angular rate exceeded 0.15 rad/s (approx. 8.5 degrees/s), which is triggered by routine tracking corrections.
4. **Over-Damped Entry Heading Gate:**
   * Entering turns previously clamped forward speed down across a long 1.0-meter distance, forcing unnecessary crawling during turn initiation in open areas.

### How It Was Fixed:
Strict **Conjunctive Condition (AND Logic)** was implemented across both planners and the real-time controller:

1. **Strict Slowdown Condition (Both Must Be True):**
   * **Condition A (Space is Really Narrow):** `clearance < 0.38 m` (in local planner) or `c < 0.36 m` (in command governor).
   * **Condition B (Turn is Very Sharp):** `effective_curvature > 1.2` (curve radius < 0.83 m), heading error > 25 degrees (0.45 rad), or command angular rate `|w| > 0.80 rad/s`.
   * **Result:** If either condition is false (e.g., turning in open space, or driving straight in a narrow hallway), the robot maintains full speed!
2. **High-Speed Open Space Turn Budget:**
   * In open or moderate spaces, the lateral acceleration limit is set to 4.0 m/s^2 (`max_lateral_accel = 4.0`), allowing rapid, fluid cornering.
   * Only when BOTH conditions are met does the lateral acceleration budget scale down to `tight_lateral_accel` (0.50 to 1.20 m/s^2) and speed cap to safe crawl (0.35 to 0.50 m/s).
3. **Responsive Entry Heading Gate:**
   * In open space or gentle turns: keeps 85% to 100% forward speed (`heading_scale = max(0.85, 1.0 - heading_error / 3.0)`).
   * Only in really narrow space (< 0.38m) AND very sharp turns (> 25 deg) does `heading_scale` drop to 0.0 m/s to perform a safe in-place pivot.
   * Fade distance reduced from 1.0 m to 0.35 m (`heading_align_distance = 0.35`), allowing immediate acceleration once nose angle aligns.
4. **Shortened Curvature Lookahead:**
   * `curvature_lookahead_m` tuned to 1.2 m, preventing premature braking from distant corners across open ground.
5. **Real-Time Command Governor Update (`classical_mpc_node.cpp`):**
   * Angular rate threshold raised from 0.15 rad/s to 0.80 rad/s, and strictly combined with narrow clearance (`c < 0.36 m`).

```
                    Robot Negotiating a Path / Turn
                                   |
            Is space really narrow? (clearance < 0.38 m)
                                   |
                  +----------------+----------------+
                  | YES                             | NO (Open / Moderate Space)
                  v                                 v
        Is turn very sharp?                 [ FAST CORNERING ]
        (kappa > 1.2 or error > 25°)        Max speed up to 2.5-4.0 m/s
                  |                         Lateral accel up to 4.0 m/s²
          +-------+-------+                 85% to 100% entry speed
          | YES           | NO
          v               v
   [ SAFE CRAWL ]   [ FAST DRIVE ]
   0.35 - 0.50 m/s  Full speed
   or pivot first   Smooth transit
```

### Result:
* In open clearings and moderate spaces (like the user's test scenario), the robot corners at full speed with agile lateral dynamics.
* Crawling and pivoting are reserved exclusively for genuine tight corridor pinches where collisions would otherwise occur.

---

## 15. Bug 17 / Fix 17 (2026-09-04 22:50:00): Clearance-Aware Path Invalidation, Permissive Start-Pose & Dead-End Recovery

### The Problem:
* When approaching newly sensed obstacles, the robot stopped in front of a narrow gap where the existing global path passed through with insufficient physical clearance (the gap was narrower than the robot chassis width of 0.43m).
* The robot got stuck, but the global planner did not replan a new path, retaining the blocked path that pointed straight into the impassable gap.

### Root Cause Analysis:
1. **Centerline-Only Collision Checking:**
   * Both `PathValidator::is_path_clear` and the Path Hysteresis `ahead_blocked` check previously tested only if the 1-pixel infinitesimal centerline of the path intersected an occupied obstacle cell (`classify(cell) == kOccupied`).
   * Because the path passed between the two obstacles without touching either obstacle on the exact centerline, the system considered the path 100% "clear and valid".
   * As a result, the map update callback never marked the path as invalid, and hysteresis believed the corridor was completely open.
2. **Path Hysteresis Rejected Detour Paths:**
   * Because `ahead_blocked` evaluated to `false`, Path Hysteresis (Fix 15) enforced its commitment rule: any new path had to be at least 15% shorter than the current path.
   * Any valid detour around the obstacle block was naturally longer than the straight line through the gap. Hysteresis rejected the detour path with status `retained_hysteresis` and permanently kept the blocked path.
3. **Start-Pose Footprint Abortion in A*:**
   * In `GlobalPlannerAStar::plan`, a hard check `!footprint_is_clear(grid, start, params_.footprint, false)` returned an empty plan if any obstacle cell touched the robot's footprint bounding box (`half_width 0.2159m + 0.04m margin = 0.256m`).
   * Because the robot had pulled up close to the obstacles, A* failed on step 0 without even starting the search.
4. **Swept Segment Step 0 Trap:**
   * In `swept_segment_is_clear`, sampling started at step 0 (`from`), meaning that if the start pose had any slight obstacle encroachment, every search branch was rejected at `i = 0`.

### How It Was Fixed:
1. **Physical Clearance Validation Along Path:**
   * Enhanced `PathValidator::is_path_clear` and `ahead_blocked` in `classical_mpc_node.cpp` to verify distance field clearance at every waypoint along the path against the minimum physical clearance needed for the robot chassis (`kMinPassableClearance = 0.22 m`, matching the 0.2159m half-width).
   * If any waypoint ahead has clearance below 0.22m, the path is immediately recognized as blocked (`ahead_blocked = true`).
2. **Immediate Detour Acceptance (Hysteresis Bypass):**
   * When `ahead_blocked = true`, Path Hysteresis is bypassed immediately! The planner accepts any valid alternative path or detour around the obstacles regardless of length.
   * In the map update callback (`map_callback`), if the current path's clearance drops below 0.22m, `needs_replan = true` is triggered immediately on the very cycle the obstacle is mapped.
3. **Permissive Start-Pose in A*:**
   * In `GlobalPlannerAStar::plan`, replaced the strict footprint check at `start` with a cell occupancy check (`grid.classify(start_cell) == kOccupied`). Since the robot is already physically sitting at `start`, search is permitted as long as its center is free, allowing the search lattice to find an escape path leading away from the obstacles.
   * In `swept_segment_is_clear`, sample loop starts at step 1 (`i = 1`), preventing the start pose from rejecting outward-bound search branches while strictly verifying all subsequent poses along the motion.
   * Reduced `astar.footprint.margin` to 0.01m, allowing A* to find valid paths through 0.40m corridors while soft clearance penalties keep the path centered.
4. **Dead-End Reverse Recovery Trigger:**
   * If the current path ahead has no clearance (`ahead_blocked = true`) and A* cannot find any forward path from the current pose (a true dead-end pocket), the node immediately sets `request_reverse_recovery_ = true`.
   * The robot promptly reverses along its known-clear breadcrumb trail into open ground, allowing the next replan cycle to rotate and take an alternative route.

```
                  New Map / Replan Check
                             |
             Does current path ahead have clearance?
             (clearance >= 0.22m along entire line)
                             |
             +---------------+---------------+
             | YES                           | NO (Blocked or Pinch < 0.22m)
             v                               v
   [ Enforce Hysteresis ]          [ Bypass Hysteresis! ]
   Commit to clear route           Can A* find a detour around?
   Reject small wiggles                      |
                                 +-----------+-----------+
                                 | YES                   | NO (Dead-End)
                                 v                       v
                         [ Accept Detour Path ]  [ Trigger Breadcrumb Reverse ]
                         Bypasses length check   Backs out to open space!
```

### Result:
* Obstacles narrowing a corridor below the robot's physical width immediately invalidate the path.
* The planner accepts detours around newly discovered obstacles without being blocked by hysteresis.
* If no forward path exists, the robot immediately reverses out of the pocket instead of remaining stuck indefinitely.

---

## 16. Bug 18 / Fix 18 (2026-09-04 23:05:00): Continuous Clearance Scaling & Extended Curvature Lookahead

### The Problem:
* In Fix 16, a strict boolean condition was introduced: the robot only slowed down or pivoted when BOTH the space was really narrow (clearance < 0.38 m) AND the turn was very sharp (curvature > 1.2 rad/m, or yaw rate |w| > 0.80 rad/s).
* In user testing inside a moderate corridor (wall clearance approximately 0.45 m and curvature around 0.8 to 1.0 rad/m), both conditions were false.
* Consequently, the robot treated this corridor bend as if it were a wide-open field. It entered the bend hot at 2.0 m/s with a full 4.0 m/s^2 lateral acceleration budget.
* The resulting centripetal inertia caused the robot to drift wide toward the outside wall. As wall clearance plummeted below 0.35 m, the breadcrumb reverse recovery was triggered.
* Changing the boolean condition to an OR check (is_really_narrow || is_very_sharp) was considered, but OR introduces severe false slowdowns:
  1. In open fields, any sharp turn would falsely drop speed to 0.35 m/s even with meters of empty space on all sides.
  2. In narrow corridors, driving completely straight along a hallway would falsely crawl at 0.35 m/s even with zero curvature.

### How It Works:
Instead of binary on/off boolean thresholds, the system continuously scales allowable lateral acceleration and turning speed based on real-time distance-field clearance.

1. **Continuous Clearance Ratio:**
   The planner calculates the robot's local clearance ratio between the physical vehicle envelope (min_clearance = 0.256 m, matching half-width 0.2159 m plus margin) and fully open space (open_clearance = 0.80 m):
   ```
   clearance_ratio = clamp((limiting_clearance - min_clearance) / (open_clearance - min_clearance), 0.0, 1.0)
   ```
   * Open fields (clearance >= 0.80 m): clearance_ratio = 1.0.
   * Moderate corridors (clearance ~ 0.45 m): clearance_ratio ≈ 0.35 to 0.40.
   * Tight pinches (clearance <= 0.26 m): clearance_ratio = 0.0.

2. **Continuous Lateral Acceleration Budget:**
   The allowable lateral acceleration dynamically scales between tight_lateral_accel (0.60 m/s^2) and max_lateral_accel (3.50 m/s^2):
   ```
   lateral_budget = tight_lateral_accel + clearance_ratio * (max_lateral_accel - tight_lateral_accel)
   ```
   * In open space: lateral_budget = 3.50 m/s^2 (fast sweeping arcs up to 2.5 m/s).
   * In moderate corridor bends: lateral_budget ≈ 1.20 to 1.60 m/s^2 (controlled 1.0 to 1.2 m/s without lateral drift).
   * In tight pinches: lateral_budget = 0.60 m/s^2 (safe crawling at 0.35 to 0.50 m/s).

3. **Curvature-Limited Speed Profiling:**
   The allowable forward speed along the path is constrained by physics:
   ```
   curvature_speed = sqrt(lateral_budget / effective_curvature)
   ```
   Because lateral_budget scales continuously with clearance, the robot automatically selects the exact maximum speed that will not cause chassis drift into the surrounding walls.

4. **Continuous Tight-Pinch Speed Ceiling:**
   When clearance_ratio is below 0.35 and effective curvature is sharp (> 0.80 rad/m), speed is capped by a continuous clearance-scaled ceiling:
   ```
   tight_v_cap = crawl_speed + (clearance_ratio / 0.35) * (0.60 - crawl_speed)
   ```
   This scales smoothly from crawl_speed (0.35 m/s) up to 0.60 m/s without step-function discontinuities.

5. **Clearance-Aware Heading Gate (Entering Turns):**
   * Open space (clearance_ratio = 1.0): minimum forward speed floor is 0.85 (85% speed), allowing high-speed sweeping arcs even while steering.
   * Tight corridors (clearance_ratio < 0.25) with heading error > 20 degrees (0.35 rad): forward speed drops to 0.0 m/s, pivoting on the spot to align with the gap before translating forward, eliminating corner-clipping collisions.
   * Intermediate clearances: the speed floor scales smoothly: `min_floor = 0.30 + 0.55 * clearance_ratio`.

6. **Real-Time Command Governor (`classical_mpc_node.cpp`):**
   The final control command governor enforces continuous clearance scaling in real time on cmd_vel:
   ```
   if (clearance < open_clearance && |w| > 0.25 rad/s) {
     v_safe = max(crawl_speed, lateral_budget(clearance) / |w|)
     if (command.v > v_safe) {
       command.v = v_safe;
     }
   }
   ```
   This prevents high-speed chassis swinging into nearby obstacles regardless of what the trajectory profile originally requested.

7. **Extended Curvature Lookahead (2.0 m):**
   `curvature_lookahead_m` was extended from 1.2 m to 2.0 m in `classical_mpc.yaml`. At 2.0 m/s, a 1.2 m window provided only 0.6 seconds of preview, causing the robot to react too late to corridor bends. A 2.0 m window provides a full 1.0 second preview, allowing the robot to start smooth deceleration along the approach straightaway before entering the bend.

### Why It Helps:
* **Eliminates the Boolean Cliff:** Replaces fragile binary checks with smooth, continuous physics-based limits. Moderate corridors are no longer misclassified as wide-open fields.
* **Prevents Chassis Drift and Wall Collisions:** By scaling allowable lateral acceleration from 3.5 m/s^2 down to 1.2 - 1.6 m/s^2 in corridors, the robot enters bends at controlled speeds (1.0 - 1.2 m/s), eliminating outward drift and preventing breadcrumb recovery activations.
* **Preserves Top Speed in Open Space:** Open fields maintain the full 3.50 m/s^2 lateral acceleration budget, allowing high-speed sweeping turns (1.5 to 2.5+ m/s) with zero false slowdowns.
* **Smooth Control Signal:** Because all scaling functions are continuous, the MPC receives smooth reference speeds rather than abrupt step-function commands, improving tracking accuracy and eliminating motor jerking.

---

## 17. Bug 19 / Fix 19 (2026-09-04 23:55:00): Global Path Shortcutting, Loop Pruning, Corner Rotation Gating, and Clearance Recalibration

### The Problem:
In complex environments like World 282, the robot exhibited severe confusion and refusal to enter open, physically traversable corridors:
1. **Dubins 360-Degree Keyhole / Teardrop Loops:**
   Because forward steering motion primitives (radius 0.76m) were expanded without line-of-sight shortcutting, A* drew large 360-degree circular loops to align its heading with the corridor opening rather than driving straight in.
2. **Corner-Pinning Against Obstacle Faces:**
   A* permitted in-place rotations right up against obstacle boundaries (clearance < 0.35m). When the robot arrived with its nose 10cm from a corner wall, the path commanded a 90-degree in-place turn, which the swept footprint safety shield immediately vetoed. After repeated vetoes, the robot fell into a reverse recovery loop.
3. **False Path Invalidation on 10cm Grid:**
   The path validator used a continuous minimum clearance threshold of 0.22m. On a discrete 10cm grid, any narrow corridor of width 0.40m has distance-field values of 0.10m to 0.15m at cell centers. The validator falsely marked unblocked, drivable corridors as impassable, setting ahead_blocked to true and triggering continuous replanning.
4. **Astronomical Clearance Penalty in Configuration:**
   The YAML parameter clearance_weight was set to 1.2 while distance_weight was 0.3. For a narrow gap with 0.15m clearance, the clearance penalty was 7.15 per meter versus 0.30 per meter in open space (a 25x penalty). Traversing a 3-meter corridor was priced identically to a 75-meter detour, causing the planner to prefer massive detours and loops over entering the corridor.

### The Solution:
1. **Loop Elimination (`global_planner_astar.cpp`):**
   Scans backwards from the goal to detect when the path doubles back on itself (distance < 0.35m between waypoints separated by more than two steps). If swept line-of-sight is clear between the waypoints, the intermediate loop is spliced out entirely.
2. **Greedy Line-of-Sight Shortcutting & Densification (`global_planner_astar.cpp`):**
   Iteratively casts swept-footprint rays between non-adjacent waypoints to collapse jagged lattice steps and Dubins arcs into direct straight-line corridors. Interpolates waypoints at regular 0.15m spacing with continuous tangent orientations (atan2(dy, dx)), followed by a 5-point moving average filter.
3. **Corner In-Place Rotation Gating (`global_planner_astar.cpp`):**
   In-place rotation primitives are strictly gated on physical clearance:
   `if (std::isfinite(c) && c < 0.35) continue;`
   This prevents A* from generating in-place turns with the robot's nose against a wall, forcing the path to turn in open space before approaching the corridor.
4. **Recalibrated 10cm Grid Clearance Threshold (`classical_mpc_node.cpp`):**
   Changed minimum passable clearance from 0.22m to 0.09m for 10cm discrete grid cells. Unified candidate path validation to evaluate against the planning grid consistently with the distance field and replanning watchdog.
5. **Clearance Weight Normalization (`classical_mpc.yaml`):**
   Reduced clearance_weight from 1.2 to 0.15 and added global_planner_footprint_margin: -0.02. This provides a soft, natural corridor-centering bias (roughly 4x cost over open ground) without pricing narrow BARN corridors out of reach.

### Why It Helps:
* **No More Circles or Keyhole Loops:** Line-of-sight shortcutting and loop pruning collapse 70-waypoint looping trajectories into clean, direct 5-waypoint lines straight through the gap.
* **Corridors Are Readily Traversed:** Normalizing clearance cost ensures the planner eagerly chooses the direct corridor rather than spinning or searching for phantom detours.
* **No False Replan Thrashing:** The 0.09m threshold on the 10cm grid correctly identifies clear 0.40m corridors as passable.
* **Elimination of Corner-Pinning Freezes:** By banning rotations in tight corners, the robot approaches corridors already aligned with the passage heading.

---

## 18. Bug 20 / Fix 20 (2026-09-08 14:20:00): Restoring Physical Footprint Margin, Wide-Corridor Preference, and Clearance Validation

### The Problem:
In Fix 19, in an attempt to prevent the robot from hesitating at corridor entrances, `clearance_weight` was dropped from 1.2 to 0.15, `global_planner_footprint_margin` was set to -0.02m, and `kMinPassableClearance` was lowered from 0.22m to 0.09m. This caused a critical regression:
1. **Loss of Corridor Discrimination:**
   With `clearance_weight` reduced to 0.15, A* treated narrow pinches and wide corridors with virtually identical clearance cost. Because a razor-thin slit happened to be slightly shorter in Euclidean distance to the goal, A* greedily chose the slit instead of the wide, clear corridor available right next to it.
2. **Artificial Footprint Shrinkage:**
   A negative footprint margin (-0.02m) shrunk the robot's virtual half-width from 0.2159m to 0.1959m (total width 0.39m). A* concluded that the robot could easily pass through a 0.40m slit, whereas the physical Jackal robot has a rigid 0.432m chassis width.
3. **Sub-Chassis Clearance Acceptance:**
   Setting `kMinPassableClearance` to 0.09m allowed the path validator to accept paths where obstacles were only 9cm from the centerline (12.6 cm inside the robot's body).

### The Solution:
1. **Restored Clearance Weight (`clearance_weight: 1.2` in `classical_mpc.yaml` and `classical_mpc_node.cpp`):**
   Restores the proper penalty for wall proximity. When both a narrow slit and a wide corridor exist, A* evaluates the total cost (distance + clearance penalty) and strongly chooses the wide, unobstructed corridor.
2. **Restored Physical Footprint Margin (`global_planner_footprint_margin: 0.00`):**
   Ensures the A* lattice collision checker uses the exact physical dimensions of the Jackal chassis (half-width 0.2159m, half-length 0.254m), preventing paths through slits narrower than the vehicle.
3. **Restored Physical Clearance Validation (`kMinPassableClearance: 0.21` in `classical_mpc_node.cpp`):**
   Ensures `is_path_clear` and `ahead_blocked` reject any path segment where distance from centerline to obstacles is less than the robot's physical half-width (0.2159m).
4. **Retained Fix 19 Shortcutting and Loop Pruning:**
   Retains line-of-sight shortcutting, loop elimination, and obstacle-face in-place rotation gating in `global_planner_astar.cpp` to eliminate Dubins circles without compromising corridor safety.

### Why It Helps:
* **Chooses Wide Corridors:** When presented with choices between narrow choke points and open passages, A* reliably picks the wide route.
* **Guarantees Physical Passage:** Virtual planning footprint aligns with the physical robot chassis, preventing the planner from driving into unpassable gaps.
* **No Teardrop Loops:** Line-of-sight shortcutting and loop pruning remain active, ensuring direct, clean trajectories without circular loops.

---

## 19. Bug 21 / Fix 21 (2026-09-15 18:38:00): Horizon-Bounded Clearance Validation & Anti-Thrashing Hysteresis Lock

### The Problem:
When navigating near corridor splits or narrow choke points, the rover exhibited severe path flip-flopping, alternating between a straight forward path and a backward 120-degree detour. This caused the vehicle to endlessly spin left and right on the spot, eventually clipping nearby obstacles.

### Root Cause:
1. **Global vs. Local Horizon Mismatch:**
   In `classical_mpc_node.cpp`, both `map_callback` (at 15 Hz) and the `ahead_blocked` loop checked distance-field clearance (`c < 0.21m`) along the entire length of the global path (10 to 15 meters away).
2. **Short-Circuiting Path Hysteresis on Distant Noise:**
   Because lidar returns at 7 to 8 meters are sparse and noisy, distant corridor pinches frequently fluctuated slightly below 0.21m. Even though the path was wide open 1 to 3 meters in front of the bumper, `ahead_blocked` immediately flagged `true`. This bypassed the 15% hysteresis rule, causing the planner to discard the straight path and grab the wide backward detour.
3. **The Pivot-In-Place Cycle:**
   The detour required turning around to go behind the robot. The controller cut linear speed and pivoted in place. Mid-turn, the distant pinch cleared, the straight path was re-selected, and the rover reversed its spin direction, trapping it in an alternating rotation cycle.

### The Solution:
1. **Extended `PathValidator::is_path_clear` with Horizon Bounding (`path_validator.hpp`, `path_validator.cpp`):**
   Added `max_clearance_check_distance` (defaulting to infinity for backward compatibility).
   * **Hard Occupied Cells (`kOccupied`):** Strictly checked along the entire path all the way to the goal. Any real wall intersecting the path immediately invalidates it.
   * **Distance Field Clearance (`c < min_clearance`):** Evaluated only while accumulated distance along the path is less than or equal to `max_clearance_check_distance` (2.5m).
2. **Aligned `map_callback` and `planner_loop` on 2.5m Local Clearance Horizon (`classical_mpc_node.cpp`):**
   * Configured `kLocalClearanceHorizon = 2.5m` in `map_callback`, `planner_loop` candidate validation, and the `ahead_blocked` check.
   * The rover maintains path commitment and drives forward as long as the immediate 2.5 meters are clear. Distant pinches are evaluated as the vehicle approaches with high sensor fidelity.
   * Completely eliminates 15 Hz background replan thrashing.

### Why It Helps:
* **Eliminates Path Flip-Flopping:** Path hysteresis remains solidly locked because distant noise spikes 7 meters ahead cannot falsely declare the immediate path blocked.
* **Stops In-Place Spinning:** Rover drives straight toward the corridor rather than pivoting back and forth between conflicting candidate routes.
* **Full Collision Safety Preserved:** Hard obstacles are still checked across the entire world, and physical clearance is rigorously enforced within the local driving horizon.

---

## 20. Bug 22 / Fix 22 (2026-09-15 19:28:00): Passable Clearance Recalibration & Forward Heading Continuity Hysteresis

### The Problem:
Even with horizon-bounded clearance checks, when the rover approached narrow corridors (such as the straight passage in World 282), it continued to shuffle and spin in place, alternating between the straight corridor and a 15-meter wide detour around an obstacle behind it.

### Root Cause:
1. **False Blockage in Narrow Passages:**
   On a discrete 10cm grid, cell distance values in a 0.40m to 0.44m corridor hover around 0.18m to 0.20m. Setting `kMinPassableClearance = 0.21m` caused `ahead_blocked` to flag `true` as soon as the vehicle was within 2.5m of the narrow gap, falsely treating a physically traversable corridor as an impenetrable wall and forcing an emergency detour.
2. **Blind Geometric Length Comparison without Heading Continuity:**
   In `planner_loop`, the hysteresis check accepted any candidate path that was 15% shorter than the current path (`candidate_len < 0.85 * retained_len`), completely ignoring the heading of the first waypoint. When the rover was following the 15m detour, the 10m straight path was 33% shorter. The hysteresis rule blindly accepted the straight path even though it required an immediate 130-degree turnaround, causing the rover to reverse its rotation direction and oscillate.

### The Solution (`classical_mpc_node.cpp`):
1. **Passable Clearance Recalibration (`kMinPassableClearance = 0.16m`):**
   Calibrated `kMinPassableClearance` from 0.21m to 0.16m in `map_callback`, candidate validation, and `ahead_blocked`. This ensures valid 0.40m+ corridors are recognized as open, while genuine collisions (<0.16m) and tapering funnels trigger replanning.
2. **Forward Heading Continuity in Hysteresis Acceptance:**
   Enforced that while the current path ahead is clear, an alternative path is only accepted if it is both significantly shorter AND continues along the forward direction:
   ```cpp
   double cand_yaw = pose.yaw;
   if (candidate.size() >= 2) {
     cand_yaw = std::atan2(candidate[1].y - candidate[0].y, candidate[1].x - candidate[0].x);
   }
   const double heading_err = std::abs(barn_core::wrap_angle(cand_yaw - pose.yaw));
   const bool much_better = (retained_len > 0.1) &&
     (candidate_len < path_improvement_ratio_ * retained_len) &&
     (heading_err < (M_PI / 2.0));
   ```
   A candidate path demanding an immediate 90-degree+ backward turn will never be accepted unless the current path is physically blocked by an obstacle wall.

### Why It Helps:
* **Committed Forward Driving:** The rover drives straight through the narrow corridor without hesitating or falsely abandoning it.
* **Elimination of U-Turn Shuffles:** The rover will never swap to a backward-pointing route mid-run while moving forward along a clear path.
* **Zero Disruption to Recovery:** If the path ahead ever encounters a real obstacle, `ahead_blocked` triggers, bypassing this restriction and activating reverse recovery.

---

## 21. Empirical Performance Evolution & Benchmark Analysis

This section summarizes the chronological evaluation results on the development benchmark suite (Worlds 228, 246, 276, and 282), tracking system performance from the original pre-fix baseline through iterative safety fixes to the latest integrated build.

### Chronological Progression Table (All 6 Stages)

| Stage | Created Timestamp | Test Run Description | Trials | Success Rate | Collisions | Timeouts | Avg Success Time | BARN 2026 Score |
| :--- | :--- | :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **Stage 1 (Oldest)** | Aug 28, 2026 10:47 | Baseline (Before any fixes) | 30 | **40.0%** (12/30) | 6.7% (2) | 53.3% (16) | **17.06 s** | **0.1471** |
| **Stage 2** | Aug 28, 2026 03:43 | After Bug Fix 1 | 38 | **44.7%** (17/38) | 5.3% (2) | 50.0% (19) | 30.23 s | **0.1461** |
| **Stage 3** | Aug 28, 2026 14:01 | Intermediate Tuning | 28 | **28.6%** (8/28) | 21.4% (6) | 50.0% (14) | 32.41 s | **0.0826** |
| **Stage 4** | Aug 28, 2026 17:36 | Zero-Collision (Over-Gated) | 28 | **25.0%** (7/28) | **0.0%** (0) | **75.0%** (21) | 58.82 s | **0.0390** |
| **Stage 5** | Aug 29, 2026 15:50 | Dead-End & Bounded Reverse | 29 | **27.6%** (8/29) | 20.7% (6) | 51.7% (15) | 33.43 s | **0.0842** |
| **Stage 6 (Latest)** | **Aug 31, 2026 06:22** | **Latest Build (All Fixes)** | 30 | **73.3%** (22/30) | **13.3%** (4) | **13.3%** (4) | **17.92 s** | **0.2730** |

---

### Shared Hard Worlds Comparison (Worlds 228, 276, and 282)

```
Stage 1 (Oldest Baseline):   45.0% Success (9/20)  |  0.0% Coll | 55.0% Timeout | Score: 0.1573 | Avg Time: 18.43 s
Stage 2 (Bug Fix 1):         31.0% Success (9/29)  |  6.9% Coll | 62.1% Timeout | Score: 0.0662 | Avg Time: 46.39 s
Stage 3 (Intermediate):      28.6% Success (8/28)  | 21.4% Coll | 50.0% Timeout | Score: 0.0826 | Avg Time: 32.41 s
Stage 4 (Over-Gated):        25.0% Success (7/28)  |  0.0% Coll | 75.0% Timeout | Score: 0.0390 | Avg Time: 58.82 s  (Severe Freeze)
Stage 5 (Dead-End Escape):   27.6% Success (8/29)  | 20.7% Coll | 51.7% Timeout | Score: 0.0842 | Avg Time: 33.43 s
Stage 6 (Latest Integrated): 73.3% Success (22/30) | 13.3% Coll | 13.3% Timeout | Score: 0.2730 | Avg Time: 17.92 s  (Major Breakthrough)
```

---

### Per-World Breakthroughs in the Latest Build

* **World 228 (Narrow Maze / Tight Corridor)**:
  * **Success Rate:** Jumped to **70.0% (7/10)** (was 10.0% in Stage 4, 40.0% in Stage 1).
  * **Timeouts:** Reduced from 90.0% (Stage 4) down to **10.0% (1/10)**.
  * **Score:** Reached **0.2373** with average time **20.96 s**.
* **World 276 (Long Corridor with Sharp Cornering)**:
  * **Success Rate:** Jumped to **70.0% (7/10)** (was 22.2% in Stage 5, 40.0% in Stage 2).
  * **Timeouts:** Reduced to **10.0% (1/10)** with average traversal time of **16.83 s**.
  * **Score:** Reached **0.2686**.
* **World 282 (Tight Chicane / Low-Clearance Turns)**:
  * **Success Rate:** Jumped to **80.0% (8/10)** with **0.0% Collisions (0/10)** (was 20.0% in Stage 4, 10.0% in Stage 5).
  * **Score:** Reached **0.3131** with average traversal time of **16.20 s**.

---

### Key Architectural Takeaways

1. **Why Stage 4 Regressed (Score 0.0390, 75% Timeouts):**
   * Over-constrained clearance gating (`ctx.clearance < 0.40m`) prevented recovery from exiting reverse mode, forcing 3-meter retreat loops in narrow tunnels.
   * Rejecting 180-degree candidate paths caused permanent planner-rejection deadlocks when trapped in dead ends.
2. **Why Stage 6 Succeeded (Score 0.2730, 73.3% Success):**
   * **Pivot-First Alignment (`recovery.cpp`):** Prevents diagonal rear bumper clipping when initiating reverse maneuvers.
   * **Dead-End Escape Acceptance (`classical_mpc_node.cpp`):** Accepts 180-degree turnaround plans and uses breadcrumbs to safely reverse into open space.
   * **Centerline Path Validation (`path_validator.cpp`):** Validates narrow corridor global plans without rigid box rejection.
   * **Dual-Motion Watchdog (`classical_mpc_node.cpp`):** Prevents false stall triggers during slow, intentional cornering maneuvers.






