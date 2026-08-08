# Classical MPC Configuration Guide

This guide details the parameters available in `barn_bringup/config/classical_mpc.yaml`, what they control, and how to tune them to balance speed, safety, and robustness.

## 1. Safety Node (`safety_node`)
The safety node is the absolute final authority on robot movement. It uses a raw, swept-footprint check against the LiDAR scan to prevent collisions regardless of what the planner commands.

- **`v_max` (4.5)** / **`w_max` (3.0)**: The absolute maximum linear and angular velocities the robot is physically allowed to execute. *Tuning:* Set slightly higher than the MPC max speeds to allow the MPC to operate fully without artificial clipping.
- **`max_lin_accel` (5.0)** / **`max_ang_accel` (6.0)**: The maximum acceleration. *Tuning:* Higher values allow the robot to respond instantly to MPC commands, but setting this too high in simulation can cause slip or wheel lift.

### Shield geometry
The shield sweeps a box of `half_extent + emergency_margin` along the command's braking arc and returns the largest scale of the command that stays clear.

- **`emergency_margin` (0.02)**: Half-extent added to the physical body for the **hard veto** box. *Tuning:* **Do not raise this casually.** The scale search scales `v` and `w` together, so as the scale → 0 the swept envelope collapses onto the box at the robot's *current* pose: an obstacle already inside that box vetoes **every** command including an in-place rotation, and the robot freezes until the trial times out. `safety_node` discards returns inside `half + 1 cm` as self-hits, so this parameter sets the width of the shell in which that trap can occur — 4 cm at the old `0.05`, 1 cm at `0.02`. For reference, the tightest reference-path clearance in the scored worlds is 0.225 m against a box half-width of 0.2159 m. Anticipatory clearance does **not** come from this term; it comes from the sweep. Watch for collisions if you lower it further.
- **`shield_horizon_s` (0.0)** / **`shield_latency_s` (0.05)** / **`braking_decel` (2.5)**: The sweep length is `max(shield_horizon_s, |v|/braking_decel + shield_latency_s)`. *Tuning:* raise `shield_horizon_s` to force a minimum lookahead regardless of speed; it sweeps through a whole corridor if set too high.
- **`cmd_timeout_s` (0.4)** / **`scan_timeout_s` (0.5)**: Staleness watchdog. Publishes zero and resets the limiter when commands or scans stop arriving.

### Shield escape search (default OFF)
Scaling a command changes its magnitude but never its **direction**, so a blocked heading blocks every multiple of it — even when a free rotation is available. When every scale fails, the shield can optionally search a small menu of slow escape motions and take one that strictly increases clearance. Shipped disabled: on world 216 (12 trials/arm) it cost time (median AT 16.2 → 18.3 s) for no measurable reliability gain, which 12 trials cannot resolve either way.

- **`shield_escape_enable` (false)**: Master switch. With it off the shield is a pure filter that can only subtract motion.
- **`shield_escape_after_s` (1.5)**: Seconds of *sustained* dead stop before the escape may fire. *Tuning:* do not lower toward zero — firing on transient vetoes turns the shield into an actuator, and it pushed the robot off its start pose during the evaluator's reset, adding ~30 s to every trial.
- **`shield_escape_speed` (0.15)** / **`shield_escape_yaw_rate` (0.5)** / **`shield_escape_horizon_s` (0.5)**: The magnitude and rollout length of the candidate escape motions.

**Log lines to grep:** `SHIELD TRAPPED` (obstacle inside the box, no escape found) and `shield escape` (escape engaged). Plain `emergency_veto` is deliberately *not* logged — it is the routine stop in front of an obstacle and floods a tight corridor.

## 2. Classical MPC (`classical_mpc_node`)

### Core Kinematics
- **`max_speed` (4.0)**: The target cruise speed.
- **`max_yaw_rate` (3.0)**: The maximum rotational speed during turns.
- **`max_accel` (5.0)** / **`max_yaw_accel` (6.0)**: Acceleration limits passed to the QP solver. *Tuning:* Increasing these allows the MPC to brake harder when approaching tight corners.

### Global Planner (A*)
The global planner searches a grid map to find a collision-free path.
- **`heuristic_weight` (2.0)**: How aggressively A* pulls toward the goal. *Tuning:* Higher (e.g., 3.0) makes it find the shortest path but it will greedily hug obstacles. Lower (1.0-1.5) makes it explore more of the map to find wider gaps.
- **`distance_weight` (0.5)**: Penalty for path length.
- **`clearance_weight` (1.5)**: Penalty for driving close to obstacles. *Tuning:* Increase to force the global planner to take wide left/right detours around obstacle clusters rather than squeezing through tight middle gaps.
- **`unknown_cost_multiplier` (1.0)**: How much to penalize exploring unseen areas of the map.

### Local Planner & Prediction Horizon
The local planner extracts a short segment of the global path and computes a velocity profile for the MPC to track.
- **`local_horizon_m` (6.0)**: How many meters of the global path are handed to the MPC at once. *Tuning:* Must be larger than the `mpc_horizon` lookahead distance, otherwise the MPC will artificially brake because it thinks the path ends.
- **`mpc_horizon` (10)**: The number of discrete steps the MPC predicts into the future.
- **`mpc_dt` (0.1)**: The time step between predictions (in seconds).
  > **Tuning the Lookahead Distance:** The physical distance the MPC checks ahead is `mpc_horizon * mpc_dt * current_speed`. At `mpc_horizon: 10` and `mpc_dt: 0.1`, it looks 1.0 second ahead. At 4.0 m/s, it checks 4.0 meters ahead. If the robot reacts to obstacles too early or brakes for turns too soon, decrease the horizon (e.g., `mpc_horizon: 5`).

### Obstacle Avoidance (Distance Field Constraints)
- **`obstacle_margin` (0.20)**: The repulsive "buffer" around the robot. If the predicted footprint comes within this distance of an obstacle, the MPC applies a repulsive force to bend the trajectory away. *Tuning:* Increase to make the robot steer clear of walls earlier. Decrease to let it squeeze through tighter gaps without oscillating.
- **`max_obstacle_slack` (1.20)**: Allows the solver to temporarily violate the `obstacle_margin` to avoid mathematical infeasibility when forced through narrow doors. *Tuning:* If the MPC frequently reports "infeasible", increase this slack.

### Recovery Behaviours
- **`no_progress_timeout_s` (3.0)**: If the robot travels extremely slowly (or is stopped) for this many seconds, it triggers the Recovery state machine.
- **`startup_creep_delay_s` (1.0)** / **`startup_creep_speed` (0.25)**: If the robot spawns without a valid map or path, it slowly creeps forward at 0.25 m/s to clear sensor blind spots.

## 3. Mapping (`barn_mapping_node`)
*(Found in the launch files, usually hardcoded)*
- **`log_odds_hit` / `log_odds_miss`**: How quickly cells are marked as occupied or free.
- **`decay_rate`**: The rate at which the map forgets old obstacles. *Tuning:* Crucial for preventing LiDAR noise or dynamic artifacts from permanently blocking the path. Higher decay makes the robot trust its immediate scan more than past memory.
