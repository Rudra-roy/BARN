# Robot Interface Contract

> Purpose: the single source of truth for the topics, frames, message types, QoS, goal action, and success rules that every track in `barn-2027-prep` must obey.

Related documents:
[Architecture Overview](./architecture/overview.md) ·
[Track A — Classical](./architecture/classical.md) ·
[Track B — End-to-end RL](./architecture/e2e_rl.md) ·
[Track C — Hybrid](./architecture/hybrid.md)

This document is authoritative. If code and this file disagree, treat it as a
bug against one of them. Every track shares this contract so the three are
benchmarkable head-to-head and so any of them transfers to the physical Jackal
by swapping only `barn_robot_adapter`.

---

## 1. Interface diagram

```
        EVALUATOR (owns Gazebo, Jackal, timeout/collision/goal monitors)
   ─────────────────────────────────────────────────────────────────────────
        │  /front/scan            │ platform/odom/filtered   │ TF odom->base_link
        │  (LaserScan)            │ (Odometry)               │
        ▼                         ▼                          ▼
   ┌───────────────────────────────────────────────────────────────────────┐
   │  barn_robot_adapter (ingress)                                         │
   │     ─▶ /barn/scan (LaserScan)    ─▶ /barn/pose (PoseStamped)          │
   └───────────────────────────────────────────────────────────────────────┘
        │                                                     ▲
   /navigate_to_pose ─▶ barn_goal_adapter ─▶ /barn/goal      │ /barn/cmd_safe
   (NavigateToPose)         (action server)   (PoseStamped)  │ (TwistStamped)
        │                                                     │
        ▼                                                     │
   ┌──────────────── command producers (per mode) ───────────┴───────────────┐
   │  classical:      goal_seeker      ─▶ /barn/cmd_desired  (legacy slice) │
   │  classical_mpc:  classical_mpc_node ─▶ /barn/cmd_desired (full pipeline)│
   │  e2e_rl:     rl_runtime  ─▶ /barn/cmd_desired                          │
   │  hybrid:     goal_seeker ─▶ /barn/cmd_classical ┐                      │
   │              rl_runtime  ─▶ /barn/cmd_rl        ├▶ hybrid ▶ /barn/cmd_desired │
   │              tracker     ─▶ /barn/tracks        ┘                      │
   └───────────────────────────────────────────────────────────────────────┘
        │ /barn/cmd_desired (TwistStamped)
        ▼
   ┌───────────────────────────────────────────────────────────────────────┐
   │  barn_safety (final authority: clamp / accel-limit / stale-gate)      │
   │     ─▶ /barn/cmd_safe (TwistStamped)                                  │
   └───────────────────────────────────────────────────────────────────────┘
        │
        ▼  barn_robot_adapter (egress) ──▶ /cmd_vel  (Twist | TwistStamped, per cmd_vel_type)
   ─────────────────────────────────────────────────────────────────────────
        EVALUATOR (velocity smoother / base)
```

---

## 2. External topics (evaluator boundary)

These are owned by the evaluator / robot side; our stack consumes or produces
them but does not define them.

| Topic                     | Dir | Message type                | Frame       | QoS notes                         |
|---------------------------|-----|-----------------------------|-------------|-----------------------------------|
| `/front/scan`             | in  | `sensor_msgs/LaserScan`     | LiDAR frame | **SensorData** (best-effort).     |
| `platform/odom/filtered`  | in  | `nav_msgs/Odometry`         | `odom`      | Filtered odometry.                |
| TF `odom -> base_link`    | in  | `tf2` transform             | —           | Global `odom`, robot `base_link`. |
| `/cmd_vel`                | out | `geometry_msgs/Twist` **or** `geometry_msgs/TwistStamped` | `base_link` | Type set by `cmd_vel_type`; **confirm live** (see §5). |

`in` = consumed by our stack; `out` = produced by our stack.

---

## 3. Goal action

| Item        | Value                                                            |
|-------------|------------------------------------------------------------------|
| Action name | `/navigate_to_pose`                                              |
| Action type | `nav2_msgs/action/NavigateToPose`                                |
| Sent by     | Evaluator                                                        |
| Goal frame  | `odom`                                                           |
| Goal pose   | `x = 10, y = 0` (10 m ahead of start)                            |
| Server      | `barn_goal_adapter/goal_adapter_node` -> internal `/barn/goal`   |

**Success is judged by physical distance, not the action result.** The evaluator
declares success when the robot is physically within **1 m** of the goal before
the timeout; whatever the action reports is irrelevant to scoring (see §6).

---

## 4. Internal topics (`/barn/*`, owned by us)

| Topic                | Dir     | Message type                      | Frame       | QoS / notes                                  |
|----------------------|---------|-----------------------------------|-------------|-----------------------------------------------|
| `/barn/goal`         | pub/sub | `geometry_msgs/PoseStamped`       | `odom`      | **Latched** (`transient_local`); goal persists for late subscribers. |
| `/barn/pose`         | pub/sub | `geometry_msgs/PoseStamped`       | `map` (classical_mpc) / `odom` | `base_link` pose from odom + TF, drift-corrected when mapping runs (`corrected_frame`). |
| `/barn/scan`         | pub/sub | `sensor_msgs/LaserScan`           | LiDAR frame | Relay of `/front/scan`; SensorData QoS.      |
| `/barn/cmd_desired`  | pub/sub | `geometry_msgs/TwistStamped`      | `base_link` | Producer output; input to `barn_safety`.     |
| `/barn/cmd_safe`     | pub/sub | `geometry_msgs/TwistStamped`      | `base_link` | Safety output; egress to `/cmd_vel`.         |
| `/barn/cmd_classical`| pub/sub | `geometry_msgs/TwistStamped`      | `base_link` | **hybrid mode only**: classical nominal.     |
| `/barn/cmd_rl`       | pub/sub | `geometry_msgs/TwistStamped`      | `base_link` | **hybrid mode only**: RL residual.           |
| `/barn/occupancy`    | pub/sub | `nav_msgs/OccupancyGrid`          | `map`       | Classical online map; built from LiDAR only. |
| `/barn/odom_correction` | pub/sub | `geometry_msgs/TransformStamped` | `map` <- `odom` | **Latched**. Scan-to-map drift correction from `barn_mapping`; `barn_robot_adapter` applies it to `/barn/pose` and `/barn/odom` so map, pose, and goal share one frame. Also broadcast on TF as `map -> odom` (REP-105); use `map` as the RViz fixed frame. |
| `/barn/tracks`       | pub/sub | `visualization_msgs/MarkerArray`  | `odom`      | **hybrid mode only**: dynamic-obstacle tracks. |

QoS conventions:

- **Scans** (`/front/scan`, `/barn/scan`) use the **SensorData** profile
  (best-effort, small depth) — losing a stale scan is better than blocking on it.
- **`/barn/goal`** is **latched** (`transient_local`, reliable): the goal is
  sent once, so any node that starts or resubscribes later still receives it.
- Command topics use reliable, low-depth QoS; they are refreshed every control
  tick, so only the freshest command matters.

---

## 5. `cmd_vel_type` and confirming the live type

The final `/cmd_vel` message type is **configurable** because the 2026 baseline
routes commands through a velocity smoother whose expected type can differ by
setup.

| `cmd_vel_type` value    | `/cmd_vel` message type       |
|-------------------------|-------------------------------|
| `twist_stamped` (default)| `geometry_msgs/TwistStamped`  |
| `twist`                 | `geometry_msgs/Twist`         |

The parameter is declared on `barn_navigation.launch.py` (default
`twist_stamped`) and consumed by `barn_robot_adapter/robot_adapter_node`
(`cmd_vel_type`). **Always confirm against the live graph on first bring-up**,
because a type mismatch means the base silently ignores every command:

```bash
ros2 topic info /cmd_vel --verbose
```

Inspect the subscriber's type. If the consumer wants `geometry_msgs/Twist`, flip
`cmd_vel_type:=twist` (or set it in `robot_adapter.yaml`) and relaunch.

---

## 6. Success, timing, and clock rules

These rules come directly from the evaluator and drive controller design.

| Rule                | Value / requirement                                                        |
|---------------------|-----------------------------------------------------------------------------|
| Success criterion   | Robot physically **within 1 m** of the goal, judged by distance not action result. |
| Timeout             | Must reach the goal before **100 s**.                                       |
| Clock start         | The evaluation clock starts **only after the robot moves `> 0.1 m`**.       |
| `use_sim_time`      | **Must be `true`** — the whole stack runs on the Gazebo `/clock`.           |

**Controller consequence:** because the clock starts only after `> 0.1 m` of
motion, the command producer must **command motion promptly at `t0`** and must
**not** wait for a map or a plan to converge. Idle time before first motion is
free, but any planner that stalls the first movement risks the 100 s budget once
the robot does start. The classical slice satisfies this by using a reactive law
that emits a command on its first control tick; see
[Track A — Classical](./architecture/classical.md).

---

## 7. Contract invariants

- **One egress.** Only `barn_robot_adapter` publishes `/cmd_vel`; it publishes
  only `/barn/cmd_safe`. No producer reaches `/cmd_vel` directly.
- **Safety is non-bypassable.** Every track's command passes
  `producer -> /barn/cmd_desired -> barn_safety -> /barn/cmd_safe`. RL must not
  bypass it. See [safety authority](./architecture/overview.md#2-design-shape-one-spine-three-tracks).
- **Frames are fixed.** Global `odom`, robot `base_link`, TF `odom -> base_link`;
  the goal and internal pose/map/tracks live in `odom`.
- **No privileged inputs.** Per the
  [competition-faithful rule](./architecture/overview.md#6-the-competition-faithful-rule),
  the only sanctioned inputs are `/front/scan`, `platform/odom/filtered`, and TF.
