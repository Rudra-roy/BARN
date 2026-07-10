# barn_classical — Track A (Priority 1)

Classical navigation. Today it ships the **runnable vertical slice**; it grows
into the full online-map planning pipeline.

## What runs now (the slice, M0–M3)
`goal_seeker_node` — a pure reactive control law (`goal_seeker.hpp`):
turn toward the goal, creep while poorly aligned, ramp forward speed with front
clearance, stop at the goal or when an obstacle is close. It commands motion the
moment it has a pose and a goal (no wait for a map/plan), which trips the
evaluator's >0.1 m motion clock quickly. Publishes `/barn/cmd_desired`.

## What is stubbed (Track A pipeline, M5–M10)
Compilable interfaces with `TODO`s, linked into `barn_classical_algos` and not
used by the slice:
| Header | Milestone | Becomes |
|--------|-----------|---------|
| `global_planner_astar.hpp` | M5 | A\* over FREE/UNKNOWN cells of the online grid |
| `path_validator.hpp` | M6 | invalidate + immediate replan |
| `local_planner.hpp` | M7 | local passage planner (DWA/lattice) |
| `controller.hpp` | M8 | curvature/clearance-aware tracking |
| `recovery.hpp` | M10 | deadlock / local-minimum recovery FSM |

## Target pipeline
```
scan+pose -> barn_mapping (grid) -> A* -> validate/replan -> local planner -> controller -> /barn/cmd_desired
```
The map comes from `barn_mapping`; **never** load the test world's ground-truth
map (see the benchmark contract).

## Tests
`test/test_goal_seeker_math.cpp` exercises the pure control law.
