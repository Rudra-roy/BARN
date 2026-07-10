# BARN 2026 Setup: ROS 2 Jazzy in an Ubuntu 24.04 Distrobox

> Purpose: end-to-end setup and first-run walkthrough for building `barn-2027-prep` inside the Ubuntu 24.04 + ROS 2 Jazzy distrobox and validating one navigation slice against the completed BARN 2026 ROS 2 evaluator.

This document is the canonical entry point for a new contributor. It covers prerequisites, the canonical first-run sequence, how to verify the live ROS interface, the standalone-slice smoke test, an under-evaluator run, and the ROS 2 Jazzy gotchas that repeatedly bite this stack.

Related reading:

- [`docs/robot_interface.md`](../robot_interface.md) — the topics, frames, action, and `/cmd_vel` contract summarized here.
- [`docs/benchmark_contract.md`](../benchmark_contract.md) — what the evaluator measures and what counts as success.
- ADR [`0001-mixed-cpp-python-workspace.md`](../decisions/0001-mixed-cpp-python-workspace.md) — why C++ and Python share one colcon workspace.
- ADR [`0002-evaluator-not-vendored.md`](../decisions/0002-evaluator-not-vendored.md) — why the evaluator is cloned + pinned + patched rather than committed.
- ADR [`0003-configurable-cmd-vel-type.md`](../decisions/0003-configurable-cmd-vel-type.md) — why `/cmd_vel` message type is a runtime parameter.

---

## 1. Prerequisites

The build and every run happen **inside** the distrobox container. The repo is never built on the host — the host only provides Docker/Podman and the distrobox CLI.

| Requirement | Value |
| --- | --- |
| Container OS | Ubuntu 24.04 (Noble) |
| ROS distribution | ROS 2 Jazzy Jalisco |
| Simulator | Gazebo (owned by the evaluator, not this repo) |
| Robot | Clearpath Jackal (spawned by the evaluator) |
| Workspace | single colcon workspace at `ros2_ws/` |
| Build command | `colcon build --symlink-install` |
| CPU target | i3-class desktop (drives the C++/Python split — see ADR-0001) |

This guide assumes the Ubuntu 24.04 + Jazzy distrobox **already exists** on your machine. If you are provisioning a fresh box, use the helper scripts under [`infra/distrobox`](../../infra/distrobox) to create and enter the container, and the environment overlays under [`infra/env`](../../infra/env) for shell configuration. Once inside a working container, return here.

Enter the container before doing anything else:

```bash
# On the host
distrobox enter barn        # or the box name defined in infra/distrobox
```

Everything below runs at the repo root, `/home/mt-labpc/BARN`, from **inside** the box.

---

## 2. Canonical first-run sequence

Run these steps in order the first time you set up the workspace. Steps 1–4 are one-time-per-shell or one-time-per-checkout; steps 5–7 are how you actually exercise the stack.

### Step 1 — Source the ROS 2 Jazzy environment

```bash
source /opt/ros/jazzy/setup.bash
```

Do this in **every** new shell before touching ROS tooling. Consider adding it to the container's shell profile via `infra/env`.

### Step 2 — Clone, pin, and patch the evaluator

```bash
bash tools/setup_barn_eval.sh
```

This script clones `The-Barn-Challenge-Ros2` (branch `master`), checks it out at the commit recorded in [`patches/pinned_commit.txt`](../../patches/pinned_commit.txt), and applies [`patches/barn_eval_launch_navigation_stack.patch`](../../patches/barn_eval_launch_navigation_stack.patch) into `ros2_ws/src/barn_eval`. That directory is **git-ignored**: only the pinned commit and the patch are tracked in this repo (see ADR-0002).

The patch rewrites the evaluator's `launch_navigation_stack()` so it launches our slice:

```
barn_bringup/launch/barn_navigation.launch.py  with  mode:=$BARN_MODE
```

> The evaluator owns Gazebo, the Jackal spawn, the LiDAR remap (`/sensors/lidar2d_0/scan` → `/front/scan`), and collision / goal / timeout monitoring and result logging. **Do not modify those.** The single integration point is the launch hook above.

Because upstream has no tagged release, the patch is a **template**. If upstream `master` has moved and the patch no longer applies cleanly, regenerate it against the new pinned commit:

```bash
# inside ros2_ws/src/barn_eval, after hand-editing launch_navigation_stack()
git diff > ../../../patches/barn_eval_launch_navigation_stack.patch
git restore .
```

Then update `patches/pinned_commit.txt` to the commit you built against.

### Step 3 — Resolve dependencies and build

```bash
bash tools/setup_workspace.sh
```

This runs `rosdep install` over `ros2_ws/src` and then `colcon build --symlink-install`. The workspace mixes `ament_cmake` (C++) and `ament_python` packages in one build (see the package table in §4 of ADR-0001). `--symlink-install` is required — see gotcha (g).

### Step 4 — Source the workspace overlay

```bash
source ros2_ws/install/setup.bash
```

Source this **after** `/opt/ros/jazzy/setup.bash` in every working shell so your packages overlay the base distro.

### Step 5 — Standalone-slice smoke test

Before involving the evaluator, prove the slice launches and produces commands on its own.

Terminal A (launch the slice):

```bash
ros2 launch barn_bringup barn_navigation.launch.py mode:=classical use_sim_time:=true
```

Terminal B (a second shell, both `setup.bash` files sourced) — watch the final command and send a goal:

```bash
# Confirm the true wire type first (see gotcha b)
ros2 topic info /cmd_vel --verbose

# Watch commands
ros2 topic echo /cmd_vel

# Send the canonical goal: odom x=10, y=0
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'odom'}, pose: {position: {x: 10.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

Confirm `/cmd_vel` is being published and that its type matches what the downstream smoother expects. **If the smoother expects unstamped `geometry_msgs/Twist`, flip the parameter** (see gotcha b and ADR-0003):

```bash
ros2 launch barn_bringup barn_navigation.launch.py \
  mode:=classical use_sim_time:=true cmd_vel_type:=twist
```

### Step 6 — One evaluated world

Run a single world under the real evaluator to confirm the launch hook wiring:

```bash
BARN_MODE=classical bash evaluation/scripts/run_single_world.sh 0 classical 1
```

Arguments are: world index (`0`), mode (`classical`), and repetitions (`1`). `BARN_MODE` is the value the patched `launch_navigation_stack()` threads into `mode:=`.

### Step 7 — Dev sweep, then the public suite

```bash
evaluation/scripts/run_dev_suite.sh classical
evaluation/scripts/run_barn2026_public_suite.sh classical
```

The dev suite is a fast subset for iteration; the BARN 2026 public suite is the benchmark against which the completed 2026 evaluator scores us. Results land under [`results/`](../../results); metrics, suites, and schemas live under [`evaluation/`](../../evaluation).

---

## 3. Verifying the ROS interface

Once a slice is running (Step 5) or a world is up (Step 6), confirm the live graph matches the contract in [`docs/robot_interface.md`](../robot_interface.md). `use_sim_time` must be `true` everywhere.

| What to check | Command | Expected |
| --- | --- | --- |
| LiDAR scan | `ros2 topic echo --once /front/scan` | `sensor_msgs/LaserScan` flowing (remapped by the evaluator from `/sensors/lidar2d_0/scan`) |
| Filtered odom | `ros2 topic echo --once platform/odom/filtered` | `nav_msgs/Odometry` flowing |
| TF chain | `ros2 run tf2_ros tf2_echo odom base_link` | a valid `odom` → `base_link` transform |
| Goal action | `ros2 action list \| grep navigate_to_pose` | `/navigate_to_pose` present (`nav2_msgs/action/NavigateToPose`) |
| Final command type | `ros2 topic info /cmd_vel --verbose` | matches `cmd_vel_type` (default `geometry_msgs/TwistStamped`) |
| Sim clock | `ros2 param get <node> use_sim_time` | `true` |

Interface facts to keep in mind:

- **Goal**: `NavigateToPose` in the `odom` frame, target `x=10, y=0`. The evaluator judges success by **physical goal distance (1 m)** reached before a **100 s timeout** — *not* by the action result. Its clock starts only after the robot has moved **> 0.1 m**.
- **`/cmd_vel`**: the message type is configurable (`cmd_vel_type`, default `twist_stamped` = `geometry_msgs/TwistStamped`; alternative `twist` = `geometry_msgs/Twist`) because the 2026 baseline runs a velocity smoother whose expected input is not guaranteed. Always confirm on the live graph.

---

## 4. ROS 2 Jazzy gotchas

These are the recurring hazards for this stack. Each maps to a design decision or a concrete mitigation already in the code.

**(a) Pin the evaluator to the distro you actually build in.** The Jackal simulation world, the robot description, and the Gazebo plugin ABI differ between ROS distributions. Since we build on Jazzy, `patches/pinned_commit.txt` must point at a commit that is valid for Jazzy. Do not bump the pin to a commit tested on a different distro without re-verifying the sim.

**(b) Verify the true `/cmd_vel` type on the live graph.** Never assume from source. Run `ros2 topic info /cmd_vel --verbose` against a running graph. If the 2026 velocity smoother expects unstamped `Twist`, relaunch with `cmd_vel_type:=twist`. See ADR-0003.

**(c) The goal-adapter action server is threaded.** `barn_goal_adapter` runs `execute()` on a detached thread under a `MultiThreadedExecutor` so that incoming goal and cancel requests stay responsive while a goal is being executed. Do not collapse it to a single-threaded executor.

**(d) TF `odom` → `base_link` races at startup.** The lookup can fail in the first moments before TF is populated. The robot adapter **catches the lookup exception and falls back to the odom pose**, so motion can begin at t0 instead of stalling. Preserve this fallback — the evaluator's clock and the 100 s timeout start ticking from first motion.

**(e) Thread `use_sim_time` and latched QoS everywhere.** `use_sim_time:=true` must be set on every node and passed through every launch file. The goal topic `/barn/goal` uses **`transient_local` (latched) QoS** so a late-joining subscriber still receives the last goal — publishers and subscribers must both declare `transient_local`, or the goal will silently not be delivered.

**(f) Publish `/cmd_vel` at a steady rate.** The downstream velocity smoother expects a regular command stream. Publish at a fixed rate rather than only on state change, or the smoother may decelerate the robot between sparse commands.

**(g) Use `--symlink-install`.** With `colcon build --symlink-install`, edits to Python, launch, and YAML files take effect **without a rebuild** — only C++ changes require recompiling. This is why the canonical build command always carries the flag.

**(h) `ament_python` package hygiene.** Each Python package needs a `resource/<pkg>` ament index marker and a `setup.cfg` with `install_scripts` (and matching `develop_scripts`) so console entry points install to the expected location. A missing marker makes the package invisible to `ros2 run` / `ros2 launch`.

---

## 5. Quick reference

```bash
# One-time per checkout
source /opt/ros/jazzy/setup.bash
bash tools/setup_barn_eval.sh
bash tools/setup_workspace.sh
source ros2_ws/install/setup.bash

# Smoke a slice standalone
ros2 launch barn_bringup barn_navigation.launch.py mode:=classical use_sim_time:=true
ros2 topic info /cmd_vel --verbose        # confirm the wire type

# One evaluated world, then the suites
BARN_MODE=classical bash evaluation/scripts/run_single_world.sh 0 classical 1
evaluation/scripts/run_dev_suite.sh classical
evaluation/scripts/run_barn2026_public_suite.sh classical
```
