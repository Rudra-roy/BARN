# ADR-0002: Evaluator checkout with a minimal algorithm dispatcher

> Purpose: keep the upstream evaluator in the same colcon workspace while making planner selection explicit and preserving its official baseline.

## Decision

`tools/setup_barn_eval.sh` clones the evaluator into its hardcoded source path,
`ros2_ws/src/The-Barn-Challenge-Ros2`, and runs one small idempotent
configurator. The configurator adds the `algo_type` launch argument and a
dispatcher inside the upstream-documented `launch_navigation_stack()` hook.

The evaluator package name remains `jackal_helper`. No Gazebo, robot spawning,
goal delivery, collision checking, timeout, result logging, or metric behavior
is changed.

The implemented branch is:

```bash
ros2 launch jackal_helper BARN_runner.launch.py \
  algo_type:=builtin world_idx:=0
```

`builtin` executes the evaluator's original Nav2/MPPI launch code. Any other
name is currently a fail-fast placeholder. When a planner is implemented, its
name receives an explicit dispatcher branch rather than silently replacing the
official baseline.

## Consequences

- The evaluator and our packages remain in one colcon workspace.
- The hardcoded checkout and `jackal_helper` names are preserved.
- The official baseline is always explicitly selectable as `builtin`.
- Unimplemented planner names terminate clearly instead of hanging a trial.
- The only evaluator integration surface remains `launch_navigation_stack()`.
