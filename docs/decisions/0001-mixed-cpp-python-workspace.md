# ADR-0001: Mixed C++/Python single colcon workspace

> Purpose: record why real-time nodes are written in C++ and RL/arbitration/training in Python, all within a single colcon workspace using only standard ROS 2 messages.

Status: Accepted

## Context

`barn-2027-prep` must run a full navigation stack — mapping, planning, control, safety, and adapters — on an **i3-class desktop CPU** while producing commands fast enough for the BARN 2026 evaluator's 100 s-per-world budget. The stack also includes learned components (an RL runtime and a hybrid arbiter) plus offline training under `learning/`, where iteration speed and access to the Python ML ecosystem matter more than per-cycle latency.

Two language ecosystems are therefore in play:

- **C++ (`rclcpp`)** — deterministic, low-latency real-time loops.
- **Python (`rclpy` + ML tooling)** — fast-iterating, higher-level logic and training.

We also had to decide whether to introduce a custom `rosidl` interface package for internal topics. Custom interfaces introduce message-generation build ordering into the workspace: every consumer must wait for the interface package to generate its headers/stubs, which is a common source of flaky mixed-language colcon builds.

The rate and latency requirements per component are:

| Component | Loop rate |
| --- | --- |
| Mapping | sensor rate |
| Global planner (A\*) | 2–5 Hz |
| Controller | 20–50 Hz |
| Safety | 50+ Hz |

## Decision

**Use a single colcon workspace at `ros2_ws/` that mixes `ament_cmake` (C++) and `ament_python` packages, split by responsibility:**

- **C++ for the real-time loops** — where the node is the final authority and must hit the i3-class CPU target:

  | Package | Role |
  | --- | --- |
  | `barn_core` | pure types / math (no ROS runtime dependency) |
  | `barn_goal_adapter` | goal action adapter |
  | `barn_robot_adapter` | robot / odom / TF adapter |
  | `barn_mapping` | mapping at sensor rate |
  | `barn_classical` | classical planner + controller |
  | `barn_dynamic_tracking` | dynamic obstacle tracking |
  | `barn_safety` | final command authority, 50+ Hz |
  | `barn_bringup` | `ament_cmake` launch / config aggregator |

- **Python (`ament_python`) where iteration speed wins and the node is not the final authority:**

  | Package | Role |
  | --- | --- |
  | `barn_rl_runtime` | RL policy runtime (`rclpy`) |
  | `barn_hybrid` | hybrid arbiter |

  plus offline training in `learning/`.

- **No custom `rosidl` interface package.** Internal topics use only standard messages — `geometry_msgs/PoseStamped`, `sensor_msgs/LaserScan`, `geometry_msgs/TwistStamped` (and the other `std_msgs`/`nav_msgs`/`geometry_msgs` types the contract already requires). This eliminates message-generation build-ordering hazards.

- **`barn_safety` (C++) is always the final command authority** on `/cmd_vel`. Learned Python nodes propose; the C++ safety layer disposes.

## Consequences

- **Two toolchains, one workspace.** The workspace carries both `ament_cmake` and `ament_python` build machinery. This is intentional and supported by colcon, but contributors must respect the conventions of each — notably the `ament_python` hygiene noted below.
- **A single build command works for everything:** `colcon build --symlink-install`. The `--symlink-install` flag means Python, launch, and YAML edits do not require a rebuild.
- **`ament_python` package hygiene is mandatory.** Each Python package needs a `resource/<pkg>` ament index marker and a `setup.cfg` with `install_scripts`, or its entry points will not be discoverable by `ros2 run` / `ros2 launch`.
- **No message-generation build ordering.** Because there is no custom interface package, there is no interface→consumer build dependency chain to sequence or to break, removing a whole class of flaky-build failures.
- **Latency guarantees stay in C++.** Any component that must meet a hard loop rate or act as final authority belongs in a C++ package. Python is reserved for advisory / learned / offline roles.
- **Safety is centralized.** Because `barn_safety` is the single final authority, safety reasoning is not duplicated across the learned and classical paths.

Related: ADR-0003 (configurable `/cmd_vel` type — the robot adapter is the one place the wire type is confirmed) and the setup guide [`docs/setup/barn_2026_jazzy_distrobox.md`](../setup/barn_2026_jazzy_distrobox.md).
