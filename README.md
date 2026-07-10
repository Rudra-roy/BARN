<div align="center">

# barn-2027-prep

**Preparation programme for the ICRA BARN Challenge 2027**
*benchmarked against the completed BARN 2026 ROS 2 evaluation pipeline*

[![ci](https://img.shields.io/badge/ci-colcon%20build%20%2B%20test-blue)](.github/workflows/ci.yml)
[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314E)](https://docs.ros.org/en/jazzy/)
[![license](https://img.shields.io/badge/license-MIT-green)](LICENSE)

</div>

---

## What is this?

[BARN](https://people.cs.gmu.edu/~xiao/Research/BARN_Challenge/BARN_Challenge26.html) (Benchmark
Autonomous Robot Navigation) tests **fast, collision-free autonomous navigation of a Clearpath
Jackal through dense, highly constrained static obstacle fields** using only a 2-D LiDAR, at up to
2 m/s, on **hidden** environments the robot has never seen.

This repository is a **clean-room preparation programme** for the 2027 edition. It preserves the
finished **BARN 2026** ROS 2 evaluation environment as the benchmark oracle and replaces the
navigation stack with **three independently developed approaches** under one identical contract:

| Track | Package(s) | Language | Status |
|-------|-----------|----------|--------|
| **A — Classical** *(Priority 1)* | `barn_classical`, `barn_mapping` | C++ | runnable slice + algorithm stubs |
| **B — End-to-end RL** | `barn_rl_runtime`, `learning/` | Python | runtime + training stubs |
| **C — Hybrid** | `barn_hybrid`, `barn_dynamic_tracking` | Python + C++ | arbiter + tracker stubs |

All three share the same adapters (`barn_goal_adapter`, `barn_robot_adapter`), the same final
authority (`barn_safety`), the same bring-up (`barn_bringup`), and the same evaluator.

> **We do not reproduce Nav2.** The evaluator (Gazebo, Jackal spawn, collision/goal/timeout
> monitoring, result logging) is kept intact; only the navigation stack it launches is ours.

---

## The one rule that shapes everything

> **Would the algorithm still work on the physical Jackal after replacing only the ROS
> hardware adapter?**

If a component reads Gazebo ground truth, parses the current `.world` file, or loads the test
world's pre-computed map/path, the answer is *no* and it is **not allowed** in a scored run. The
full allowed/disallowed input contract lives in
[`docs/benchmark/barn_2026_contract.md`](docs/benchmark/barn_2026_contract.md).

---

## Repository map

```
barn-2027-prep/
├── docs/            # architecture, benchmark contract, metric notes, setup, ADRs
├── infra/           # distrobox creation + Jazzy provisioning scripts
├── ros2_ws/src/     # the ROS 2 navigation packages (see table below)
├── learning/        # offline RL training (NOT a ROS package)
├── models/          # model cards + normalization stats (weights are git-ignored)
├── evaluation/      # run scripts, dual metric reports, world suites, schemas
├── patches/         # the single launch-hook patch applied to the evaluator
├── tools/           # setup_barn_eval.sh, setup_workspace.sh, format.sh
├── results/         # raw per-trial results + manifests (source of truth)
└── tests/           # cross-package integration (launch_testing)
```

### ROS 2 packages (`ros2_ws/src/`)

| Package | Build type | Role |
|---------|-----------|------|
| `barn_core` | ament_cmake (C++) | pure types + math — **no** ROS/Gazebo deps |
| `barn_goal_adapter` | ament_cmake (C++) | `NavigateToPose` action server → internal `Goal2D` |
| `barn_robot_adapter` | ament_cmake (C++) | sensors in / velocity command out (message type configurable) |
| `barn_mapping` | ament_cmake (C++) | online log-odds occupancy grid *(stub)* |
| `barn_classical` | ament_cmake (C++) | A\*/local/controller/recovery + the runnable goal-seeker |
| `barn_dynamic_tracking` | ament_cmake (C++) | cluster / associate / Kalman / TTC *(stub)* |
| `barn_safety` | ament_cmake (C++) | **final command authority** — clamp, accel-limit, stale gate |
| `barn_rl_runtime` | ament_python | CPU policy inference *(stub)* |
| `barn_hybrid` | ament_python | risk gate + command fusion *(stub)* |
| `barn_bringup` | ament_cmake | launch + config; single `mode:=` entrypoint |

---

## Quick start (inside your ROS 2 Jazzy distrobox)

> Full walkthrough: [`docs/setup/barn_2026_jazzy_distrobox.md`](docs/setup/barn_2026_jazzy_distrobox.md).
> This repo builds **nothing on the host** — it is cloned into the container and built there.

```bash
# 0. Clone this repo into the container and enter it
git clone <your-fork-url> barn-2027-prep && cd barn-2027-prep

# 1. Fetch + pin + patch the BARN 2026 evaluator (writes to ros2_ws/src/barn_eval, git-ignored)
bash tools/setup_barn_eval.sh

# 2. Resolve dependencies and build the whole workspace
bash tools/setup_workspace.sh          # rosdep install + colcon build --symlink-install

# 3. Source the overlay
source ros2_ws/install/setup.bash

# 4. Smoke-test the navigation slice on its own (no evaluator)
ros2 launch barn_bringup barn_navigation.launch.py mode:=classical use_sim_time:=true
#    …then, in another sourced shell:
ros2 topic echo /cmd_vel                       # expect non-zero motion commands
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: odom}, pose: {position: {x: 10.0, y: 0.0}}}}"

# 5. Run one evaluated world under the BARN 2026 runner
BARN_MODE=classical bash evaluation/scripts/run_single_world.sh 0 classical 1

# 6. Development suite, then the full 500-trial public suite
bash evaluation/scripts/run_dev_suite.sh classical
bash evaluation/scripts/run_barn2026_public_suite.sh classical
```

---

## How you are scored

Per environment *i*, the **published BARN 2026** metric is

```
s_i = success_i * OT_i / clip(AT_i, 2*OT_i, 8*OT_i)      OT_i = reference_path_length_i / 2.0 m/s
```

A collision or a failure to reach the goal (within 1 m, before the 100 s timeout) scores **0**,
regardless of speed. The optimization priority is therefore **reliability → consistency → speed**,
in that order.

> ⚠️ The upstream `report_test.py` computes `clip(AT, 4*OT, 8*OT)`, which **disagrees** with the
> published rule. This repo always emits **two** reports so numbers are never silently confused —
> see [`docs/benchmark/metric_notes.md`](docs/benchmark/metric_notes.md).

---

## Development discipline

- **Public worlds (0–299)** are development/validation data. The **50 evenly spaced worlds**
  (`0, 6, …, 294`) form the fixed validation campaign; no per-world tuning.
- Every campaign captures a **manifest** (repo + evaluator commit, ROS/apt versions, params) and
  keeps the **raw** evaluator output as the source of truth (`evaluation/scripts/capture_manifest.sh`).
- Failures get a **taxonomy code** (`S01–S13` / `D01–D12`) — see
  [`docs/benchmark/failure_taxonomy.md`](docs/benchmark/failure_taxonomy.md).
- The full milestone roadmap (M0–M21) is in [`docs/roadmap.md`](docs/roadmap.md).

---

## Documentation

| Topic | Document |
|-------|----------|
| System architecture & data flow | [`docs/architecture/overview.md`](docs/architecture/overview.md) |
| Per-track designs | [`docs/architecture/{classical,e2e_rl,hybrid}.md`](docs/architecture/) |
| Robot interface contract (topics/frames/types) | [`docs/robot_interface.md`](docs/robot_interface.md) |
| Competition-faithful input policy | [`docs/benchmark/barn_2026_contract.md`](docs/benchmark/barn_2026_contract.md) |
| Metric definitions & the clip discrepancy | [`docs/benchmark/metric_notes.md`](docs/benchmark/metric_notes.md) |
| Failure taxonomy | [`docs/benchmark/failure_taxonomy.md`](docs/benchmark/failure_taxonomy.md) |
| Distrobox + Jazzy setup | [`docs/setup/barn_2026_jazzy_distrobox.md`](docs/setup/barn_2026_jazzy_distrobox.md) |
| Roadmap (M0–M21) | [`docs/roadmap.md`](docs/roadmap.md) |
| Architecture decisions | [`docs/decisions/`](docs/decisions/) |

---

## References

- ICRA 2026 BARN Challenge — <https://people.cs.gmu.edu/~xiao/Research/BARN_Challenge/BARN_Challenge26.html>
- *Lessons Learned from the Fifth BARN Challenge* — <https://people.cs.gmu.edu/~xiao/papers/barn26_report.pdf>
- BARN ROS 2 evaluation pipeline — <https://github.com/Saadmaghani/The-Barn-Challenge-Ros2>
- Original BARN infrastructure — <https://github.com/Daffan/the-barn-challenge>

## License

[MIT](LICENSE). *(Switch to BSD-3-Clause if you prefer to match the Clearpath/Jackal ecosystem.)*
