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
