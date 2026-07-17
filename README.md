<div align="center">

# barn-2027-prep

**Preparation programme for the ICRA BARN Challenge 2027**
*benchmarked against the completed BARN 2026 ROS 2 evaluation pipeline*

[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314E)](https://docs.ros.org/en/jazzy/)
[![license](https://img.shields.io/badge/license-MIT-green)](LICENSE)

</div>

---

## 📸 Media & Highlights

*(Add your screenshots, GIFs, and videos of the robot navigating the BARN environments here)*

- **[Insert Video/Screenshot 1]** - *Caption describing the run*
- **[Insert Video/Screenshot 2]** - *Caption describing the run*

---

## What is this?

[BARN](https://people.cs.gmu.edu/~xiao/Research/BARN_Challenge/BARN_Challenge26.html) (Benchmark Autonomous Robot Navigation) tests **fast, collision-free autonomous navigation of a Clearpath Jackal through dense, highly constrained static obstacle fields** using only a 2-D LiDAR, at up to 2 m/s, on **hidden** environments the robot has never seen.

This repository is a **clean-room preparation programme** for the 2027 edition. We replace the default navigation stack with **three independently developed approaches**:

| Track | Package(s) | Language | Status |
|-------|-----------|----------|--------|
| **A — Classical** | `barn_classical`, `barn_mapping` | C++ | footprint-aware lattice + MPC stack |
| **B — End-to-end RL** | `barn_rl_runtime`, `learning/` | Python | runtime + training stubs |
| **C — Hybrid** | `barn_hybrid`, `barn_dynamic_tracking` | Python + C++ | arbiter + tracker stubs |

> **Note:** The core rule of this repo is that the algorithm must be deployable to the physical Jackal without modification. Reading Gazebo ground truth or pre-loading map structures is strictly forbidden.

---

## Quick start (inside your ROS 2 Jazzy distrobox)

> Full walkthrough: [`docs/setup/barn_2026_jazzy_distrobox.md`](docs/setup/barn_2026_jazzy_distrobox.md).

```bash
# 0. Clone this repo into the container and enter it
git clone <your-fork-url> barn-2027-prep && cd barn-2027-prep

# 1. Clone the evaluator and add the minimal algo_type dispatcher
bash tools/setup_barn_eval.sh

# 2. Resolve dependencies and build the whole workspace
bash tools/setup_workspace.sh          # rosdep install + colcon build --symlink-install

# 3. Source the overlay
source ros2_ws/install/setup.bash

# 4. Run the footprint-aware Classical MPC stack
ros2 launch jackal_helper BARN_runner.launch.py \
  algo_type:=classical_mpc world_idx:=0 gui:=true planner_rviz:=true
```

---

## Documentation & Details

For in-depth details on how the system is built, evaluated, and structured, please refer to the documentation links below:

### 🌟 New Features & Stack Updates
- **[Classical MPC & Recovery Feature Updates](docs/features/classical_mpc_updates.md)** - Details on the new 6-phase robust recovery, global planner stability, high-speed local planner tuning, and advanced map decay.
- **[Classical MPC Configuration Guide](docs/features/configuration_guide.md)** - A guide explaining the tunable parameters in the MPC, A* global planner, and safety node, including how to tune them for speed and tight spaces.

### Architecture & Guidelines
- **[System Architecture & Data Flow](docs/architecture/overview.md)**
- **[Robot Interface Contract (Topics/Frames)](docs/robot_interface.md)**
- **[Roadmap (M0–M21)](docs/roadmap.md)**
- **[Architecture Decisions (ADRs)](docs/decisions/)**

### Benchmark & Scoring
- **[Competition-Faithful Input Policy](docs/benchmark/barn_2026_contract.md)**
- **[Metric Definitions & Scoring](docs/benchmark/metric_notes.md)**
- **[Failure Taxonomy](docs/benchmark/failure_taxonomy.md)**

---

## References

- ICRA 2026 BARN Challenge — <https://people.cs.gmu.edu/~xiao/Research/BARN_Challenge/BARN_Challenge26.html>
- BARN ROS 2 evaluation pipeline — <https://github.com/Saadmaghani/The-Barn-Challenge-Ros2>

## License

[MIT](LICENSE). *(Switch to BSD-3-Clause if you prefer to match the Clearpath/Jackal ecosystem.)*
