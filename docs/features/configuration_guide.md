# Classical MPC Configuration Guide

This guide details the parameters available in `barn_bringup/config/classical_mpc.yaml`, what they control, and how to tune them to balance speed, safety, and robustness.

## 1. Safety Node (`safety_node`)
The safety node is the absolute final authority on robot movement. It uses a raw, swept-footprint check against the LiDAR scan to prevent collisions regardless of what the planner commands.

- **`v_max` (4.5)** / **`w_max` (3.0)**: The absolute maximum linear and angular velocities the robot is physically allowed to execute. *Tuning:* Set slightly higher than the MPC max speeds to allow the MPC to operate fully without artificial clipping.
- **`max_lin_accel` (5.0)** / **`max_ang_accel` (6.0)**: The maximum acceleration. *Tuning:* Higher values allow the robot to respond instantly to MPC commands, but setting this too high in simulation can cause slip or wheel lift.

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
