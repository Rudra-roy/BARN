# barn_goal_adapter

ROS-specific goal interface: `NavigateToPose` → internal `Goal2D`. Nothing more.

## Responsibilities
- Host a `nav2_msgs/action/NavigateToPose` action server on `/navigate_to_pose`
  (the topic the BARN evaluator sends the goal to).
- Transform the goal into the planning frame (`odom`) via tf2 when needed.
- Publish it **once, latched** (`transient_local` QoS) on `/barn/goal` so any
  navigation core — including one that starts late — receives it.
- Provide correct `ACCEPT → EXECUTING → SUCCEEDED/CANCELED` semantics with
  feedback (distance remaining), reading `/barn/pose` for progress.

## Boundary
The BARN evaluator scores **physical goal distance**, not this action's result.
The node implements proper action semantics for portability, but the navigation
cores must never wait on it to decide whether to move.

## Key parameters
`planning_frame` (odom), `goal_topic` (/barn/goal), `pose_topic` (/barn/pose),
`success_distance` (1.0 m), `feedback_period_s` (0.1).

## Tests
`test/test_goal_extract.cpp` unit-tests the pure `to_goal2d()` conversion.
