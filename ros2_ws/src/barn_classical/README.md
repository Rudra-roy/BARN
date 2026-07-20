# barn_classical — Track A (Priority 1)

Classical navigation. The **full online-map planning pipeline is implemented**
(M0–M10 done) and is the primary classical producer; the original reactive
vertical slice remains as a legacy/smoke entrypoint.

## What runs now (the full stack, M4–M10)
`classical_mpc_node` (`classical_mpc_node.cpp`) — the complete pipeline:
```
scan+pose -> barn_mapping (log-odds grid + distance field) -> A* global planner
          -> path validate/replan -> local passage planner -> MPC controller (OSQP)
          -> recovery FSM -> /barn/cmd_desired
```
Launched by `barn_bringup/launch/classical_mpc.launch.py`
(`algo_type:=classical_mpc`). All algorithm modules are real, `TODO`-free
implementations linked into `barn_classical_algos`:

| Module | File | Does |
|--------|------|------|
| `global_planner_astar` | `global_planner_astar.cpp` | A\* over FREE/UNKNOWN cells with clearance-shaped costs |
| `path_validator` | `path_validator.cpp` | invalidate + immediate replan |
| `local_planner` | `local_planner.cpp` | local passage planner (gap selection/shaping) |
| `controller` | `controller.cpp` | sequentially-linearized MPC (OSQP), distance-field constraints |
| `collision_checker` | `collision_checker.cpp` | swept-footprint feasibility |
| `recovery` | `recovery.cpp` | 6-phase deadlock / local-minimum recovery FSM |

The map comes from `barn_mapping`; **never** load the test world's ground-truth
map (see the benchmark contract).

## Legacy vertical slice
`goal_seeker_node` (`goal_seeker.hpp`) — the original M0–M3 reactive control law
(turn toward goal, creep while misaligned, ramp speed with front clearance, stop
near obstacle/goal). It commands motion the moment it has a pose and goal (no wait
for a map/plan). Kept as a minimal end-to-end smoke path; superseded by
`classical_mpc_node` for real runs.

## Dynamic obstacles (🚧 WIP)
`controller.cpp` also carries the spatiotemporal moving-obstacle keep-out used by
the DynaBARN test bed (fed by `/barn/tracks`); see
[`docs/features/dynabarn_dynamic_obstacles.md`](../../../docs/features/dynabarn_dynamic_obstacles.md).

## Tests
`test/test_goal_seeker_math.cpp` exercises the reactive control law; additional
tests cover the MPC controller (incl. dynamic-obstacle constraints).
