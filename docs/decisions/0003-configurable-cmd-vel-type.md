# ADR-0003: Configurable `/cmd_vel` message type

> Purpose: record why the final `/cmd_vel` command message type is selectable at runtime via a parameter rather than fixed at compile time.

Status: Accepted

## Context

The final velocity command our stack emits is published on `/cmd_vel`. In the BARN 2026 baseline, `/cmd_vel` is consumed by a **velocity smoother** before reaching the robot. The message type the smoother expects on its input is **not guaranteed**:

- ROS 2 stacks variously use `geometry_msgs/TwistStamped` (stamped, the modern default in much of Jazzy-era Nav2) or the older unstamped `geometry_msgs/Twist`.
- Which one the 2026 smoother actually subscribes to can only be determined by inspecting the **live graph** with `ros2 topic info /cmd_vel --verbose`, not by reading source or assuming a default.

If we hard-coded one type and it were wrong, the smoother would silently receive nothing usable and the robot would not move — a failure that only surfaces at run time under the evaluator, wasting a world attempt.

## Decision

**Expose a `cmd_vel_type` parameter that selects the `/cmd_vel` message type at runtime:**

| `cmd_vel_type` | Message type |
| --- | --- |
| `twist_stamped` (default) | `geometry_msgs/TwistStamped` |
| `twist` | `geometry_msgs/Twist` |

- The default is `twist_stamped`, matching the modern stamped convention.
- Switching to the unstamped type requires **no recompile** — pass the parameter through launch:

  ```bash
  ros2 launch barn_bringup barn_navigation.launch.py \
    mode:=classical use_sim_time:=true cmd_vel_type:=twist
  ```

- **The robot adapter (`barn_robot_adapter`) is the single place the wire type is confirmed and produced.** The true type must be verified on the live graph before an evaluated run:

  ```bash
  ros2 topic info /cmd_vel --verbose
  ```

  If the smoother expects unstamped `Twist`, flip `cmd_vel_type` to `twist`.

## Consequences

- **No recompile to switch wire formats.** The stamped/unstamped choice is a launch-time parameter, so adapting to the evaluator's smoother is a one-line change, not a rebuild.
- **A single confirmation point.** Because the robot adapter is the only node that finalizes the `/cmd_vel` type, there is exactly one place to reason about and verify the wire contract — no scattered publishers to keep in sync.
- **Verification is a required run-time step.** The correct value cannot be assumed; confirming `/cmd_vel` on the live graph is part of the standalone smoke test (Step 5 of the setup guide) and of every gotcha checklist.
- **Steady-rate publishing still required.** Independently of type, the smoother expects a regular command stream, so `/cmd_vel` must be published at a fixed rate (see gotcha (f) in the setup guide).

Related: setup guide [`docs/setup/barn_2026_jazzy_distrobox.md`](../setup/barn_2026_jazzy_distrobox.md), robot interface contract [`docs/robot_interface.md`](../robot_interface.md), and ADR-0001 (the robot adapter is a C++ real-time node).
