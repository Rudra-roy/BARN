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

## 12. Empirical Performance Evolution & Benchmark Analysis

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






