# tests/

Cross-package tests that don't belong to a single ROS package. (Per-package unit
tests live in each package's `test/` directory and run under `colcon test`.)

```
integration/test_slice_smoke.py   launch_testing: the classical slice commands /cmd_vel
```

## Running the slice smoke test
Standalone — no Gazebo or evaluator needed. Build and source the overlay, then:
```bash
launch_test tests/integration/test_slice_smoke.py
```
It launches `goal_seeker_node` + `safety_node` + `robot_adapter_node`, feeds a
latched goal, a pose, and a clear scan, and asserts a positive forward command
appears on `/cmd_vel`. This is the M3 regression guard: if it fails, the slice
no longer drives the robot.

> Not run by `colcon test` (it is outside the package tree) to keep CI free of
> launch-timing flakiness; run it manually or in a dedicated integration job.
