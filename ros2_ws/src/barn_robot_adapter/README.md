# barn_robot_adapter

The single ROS ↔ robot boundary shared by all three navigation tracks. Convert
ROS messages into `barn_core` types on the way in, and `barn_core` commands into
ROS messages on the way out — nothing else.

## Data flow
```
Ingress   /front/scan             --> /barn/scan   (relay)
          platform/odom/filtered  --> /barn/pose   (base_link in odom, via TF, odom fallback)
Egress    /barn/cmd_safe          --> /cmd_vel     (Twist or TwistStamped, see below)
```

## The `cmd_vel_type` parameter (important)
The BARN 2026 baseline runs commands through a velocity smoother whose input may
be `geometry_msgs/TwistStamped` **or** `geometry_msgs/Twist`. This node is the
**one place** that truth is confirmed. Default is `twist_stamped`. On first
bring-up:
```bash
ros2 topic info /cmd_vel --verbose
```
and flip `cmd_vel_type` to `twist` in `config/robot_adapter.yaml` if needed — no
recompile. Rationale: [`docs/decisions/0003-configurable-cmd-vel-type.md`](../../../docs/decisions/0003-configurable-cmd-vel-type.md).

## Reusable library
`barn_robot_conversions` (`conversions.hpp`) holds the pure message-boundary
functions and is unit-tested in `test/test_conversions.cpp` without a ROS graph.
