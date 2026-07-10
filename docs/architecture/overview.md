# System Architecture Overview

> Purpose: describe the shared navigation spine and the three interchangeable tracks that make up `barn-2027-prep`, and show how a single command routes sensors to `/cmd_vel` for the ICRA BARN Challenge 2027.

Related documents:
[Track A — Classical](./classical.md) ·
[Track B — End-to-end RL](./e2e_rl.md) ·
[Track C — Hybrid](./hybrid.md) ·
[Robot Interface Contract](../robot_interface.md) ·
[Evaluation & Benchmarks](../../evaluation/)

---

## 1. The challenge in one paragraph

The BARN Challenge asks a [Clearpath Jackal](https://clearpathrobotics.com/jackal-small-unmanned-ground-vehicle/)
(differential drive, 2-D LiDAR, `v <= 2 m/s`) to drive from a start pose to a
goal **10 m ahead** through a dense field of **static** obstacles, with **no
collision**, **as fast as possible**, on **hidden** worlds the team never sees
before scoring. Our design priority order is fixed and never reordered:

```
reliability  >  consistency  >  speed
```

A stack that finishes every world slowly beats a stack that is fast on half of
them and crashes on the rest. Every design decision below is justified against
that order.

---

## 2. Design shape: one spine, three tracks

`barn-2027-prep` is deliberately structured as **one shared spine** plus **three
navigation tracks** that all satisfy an **identical navigation contract**
(same input topics, same output topic, same goal action, same safety authority).
Because the tracks are contract-compatible, they are selected at launch time with
a single argument and can be benchmarked head-to-head on the same worlds.

```
                         ┌───────────────────────────────┐
                         │        SHARED SPINE           │
                         │  barn_core   (types / math)   │
                         │  barn_goal_adapter            │
                         │  barn_robot_adapter           │
                         │  barn_safety  (final authority)│
                         │  barn_bringup (launch/config) │
                         └───────────────┬───────────────┘
                                         │  mode:=
              ┌──────────────────────────┼──────────────────────────┐
              ▼                          ▼                          ▼
      ┌───────────────┐          ┌───────────────┐          ┌───────────────┐
      │   TRACK A     │          │   TRACK B     │          │   TRACK C     │
      │  Classical    │          │  End-to-end   │          │   Hybrid      │
      │  (C++)        │          │  RL (Python)  │          │  (Py + C++)   │
      └───────────────┘          └───────────────┘          └───────────────┘
```

All three feed the same `barn_safety` node, which owns the last word on every
command. No track may emit to `/cmd_vel` directly.

---

## 3. End-to-end data flow (mode = classical vertical slice)

The diagram below traces one physical scan through to a wheel command. This is
the runnable slice (`mode:=classical`); the RL and hybrid tracks swap the
command producer but keep the same ingress and egress.

```
  EVALUATOR SIDE            │   OUR NAVIGATION STACK (barn_bringup)
                           │
  /navigate_to_pose  ──────┼──▶ goal_adapter_node ──▶ /barn/goal
   (nav2_msgs, odom x=10)  │        (action server)     (PoseStamped, latched)
                           │
  /front/scan  ────────────┼──┐
   (LaserScan, SensorData) │  │
  platform/odom/filtered ──┼──┼──▶ robot_adapter_node ──▶ /barn/scan  (LaserScan)
   (Odometry)              │  │        (sensors in)    └─▶ /barn/pose  (PoseStamped)
  TF odom->base_link  ─────┼──┘
                           │
                           │        /barn/goal ┐
                           │        /barn/pose ├─▶ goal_seeker_node ─▶ /barn/cmd_desired
                           │        /barn/scan ┘        (reactive law)     (TwistStamped)
                           │
                           │        /barn/cmd_desired ┐
                           │        /barn/scan        ├─▶ safety_node ─▶ /barn/cmd_safe
                           │                          ┘  (clamp/accel/stale)  (TwistStamped)
                           │
                           │        /barn/cmd_safe ──▶ robot_adapter_node (egress)
  /cmd_vel  ◀──────────────┼──────────────────────────────────┘
   (Twist / TwistStamped)  │        message type set by cmd_vel_type param
                           │
```

Key properties visible in this flow:

- **Ingress and egress are the only ROS-hardware-specific code.** Everything
  between `/barn/*` topics is world-agnostic and would run unchanged on the
  physical Jackal after swapping the adapter.
- **`barn_safety` sits on the command path, not beside it.** There is no route
  from a producer to `/cmd_vel` that skips it.
- **Motion starts at `t0`.** The evaluator clock begins only after the robot
  moves `> 0.1 m`, so the command producer must command motion immediately and
  must not wait for a map or plan. See [Robot Interface](../robot_interface.md).

---

## 4. Mode selection

There is exactly one entrypoint. The BARN evaluator's patched
`launch_navigation_stack()` includes it with a `mode:=` argument.

```bash
ros2 launch barn_bringup barn_navigation.launch.py mode:=classical
ros2 launch barn_bringup barn_navigation.launch.py mode:=e2e_rl
ros2 launch barn_bringup barn_navigation.launch.py mode:=hybrid
```

`barn_navigation.launch.py` declares three arguments and conditionally includes
exactly one per-mode launch file:

| Argument        | Default          | Meaning                                                        |
|-----------------|------------------|----------------------------------------------------------------|
| `mode`          | `classical`      | Track selector: `classical` \| `e2e_rl` \| `hybrid`.           |
| `use_sim_time`  | `true`           | Use the Gazebo `/clock`. **Must be true** under the evaluator. |
| `cmd_vel_type`  | `twist_stamped`  | Final `/cmd_vel` message type: `twist_stamped` \| `twist`.     |

Per-mode command flow:

| Mode        | Command producer(s)                                            | Path to safety                                        |
|-------------|---------------------------------------------------------------|-------------------------------------------------------|
| `classical` | `goal_seeker_node`                                            | `-> /barn/cmd_desired -> safety_node`                 |
| `e2e_rl`    | `rl_runtime_node` (zero motion until a model is trained)      | `-> /barn/cmd_desired -> safety_node`                 |
| `hybrid`    | `goal_seeker_node` (`/barn/cmd_classical`) + `rl_runtime_node` (`/barn/cmd_rl`) | `-> hybrid_node -> /barn/cmd_desired -> safety_node` |

In `hybrid` mode `tracker_node` additionally publishes `/barn/tracks` for the
dynamic-risk gate. See [Track C](./hybrid.md).

---

## 5. Package inventory

| Package                 | Role                                                                 | Language | Status |
|-------------------------|---------------------------------------------------------------------|----------|--------|
| `barn_core`             | Pure types & math (geometry, grids, control primitives). No ROS deps.| C++      | Foundation |
| `barn_goal_adapter`     | `NavigateToPose` action server -> internal `/barn/goal`.            | C++      | Runnable |
| `barn_robot_adapter`    | Sensors in (`/front/scan`, odom, TF) -> `/barn/*`; `/barn/cmd_safe` out to `/cmd_vel`. | C++ | Runnable |
| `barn_safety`           | **Final command authority**: clamp, accel-limit, stale-gate. All tracks pass through it. | C++ | Runnable |
| `barn_bringup`          | Launch files + per-mode config. Single `mode:=` entrypoint.         | Python (launch) | Runnable |
| `barn_classical`        | Track A: goal-seeker (runnable) + target pipeline (map->A*->plan->control->recovery, stubbed). | C++ | Slice runnable / pipeline stubbed |
| `barn_mapping`          | Track A online log-odds occupancy map from LiDAR.                    | C++      | Stubbed |
| `barn_rl_runtime`       | Track B CPU inference: observation -> ONNX policy -> action.        | Python   | Stub policy (no motion) |
| `barn_hybrid`           | Track C arbiter: classical nominal + gated RL residual.             | Python   | Runnable arbiter |
| `barn_dynamic_tracking` | Track C tracker: cluster -> associate -> Kalman -> TTC.             | C++      | Stubbed |

Track summaries:

- **Track A — Classical (Priority 1).** C++. Runnable today via a reactive
  vertical-slice goal-seeker; the full pipeline (online map -> A* ->
  validate/replan -> local passage planner -> controller -> recovery) is
  scaffolded and stubbed. See [classical.md](./classical.md).
- **Track B — End-to-end RL.** Python runtime (`barn_rl_runtime`, CPU
  inference) plus offline training in `learning/`. The stub policy commands
  zero motion until a model is trained. See [e2e_rl.md](./e2e_rl.md).
- **Track C — Hybrid.** Python arbiter (`barn_hybrid`) plus C++ tracking
  (`barn_dynamic_tracking`). Classical nominal command plus a dynamic-risk-gated
  RL residual; in static worlds the gate `alpha = 0`, so it reduces exactly to
  classical. See [hybrid.md](./hybrid.md).

---

## 6. The competition-faithful rule

Every component in this repository is held to a single litmus test, and it is
stated here because it governs the whole architecture:

> **"Would the algorithm still work on the physical Jackal after replacing only
> the ROS hardware adapter?"**

Concretely, in a **scored run** no component may:

- read Gazebo ground truth (true poses, contact sensors, model states);
- parse the current `.world` file or otherwise infer the obstacle layout; or
- load the test world's map or reference path.

The only sanctioned inputs are the robot's own sensors: `/front/scan`,
`platform/odom/filtered`, and the TF tree. This is why the spine isolates all
hardware coupling into `barn_robot_adapter`: swap that one node for a physical
driver and the identical navigation code runs on real hardware. Any privileged
signal that helps in simulation but is unavailable on the Jackal is prohibited,
because reliability on hidden worlds — not simulator score — is the goal.

---

## 7. Where to go next

- Producer designs: [Classical](./classical.md), [RL](./e2e_rl.md), [Hybrid](./hybrid.md).
- The exact topic / frame / message-type contract: [Robot Interface](../robot_interface.md).
- Scoring rules and world suites: [Evaluation & Benchmarks](../../evaluation/).
