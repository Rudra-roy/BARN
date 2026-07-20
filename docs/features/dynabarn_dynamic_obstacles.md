# DynaBARN Dynamic-Obstacle Support — 🚧 Work In Progress

> **Status: In progress — not on the competition path.** BARN 2026 scores **static**
> worlds only, so this work lives behind a **separate launch file** and a separate
> evaluator `algo_type`. The stable static/competition stack
> (`classical_mpc.launch.py`) is untouched. This note documents what has been built,
> what is verified, and what remains.

This is a preparatory test bed in case ICRA **2027** makes moving obstacles
mandatory. It reproduces the [DynaBARN](https://github.com/aninair1905/DynaBARN)
dynamic-obstacle setup so the Classical **MPC** can be developed and stress-tested
against moving cylinders — end to end, under the real ROS 2 evaluator. It maps to
roadmap milestones **M17** (dynamic-obstacle test infrastructure) and **M18**
(LiDAR dynamic tracker); see [`docs/roadmap.md`](../roadmap.md).

Related: [Track C — Hybrid architecture](../architecture/hybrid.md) ·
[`barn_dynamic_tracking`](../../ros2_ws/src/barn_dynamic_tracking/README.md) ·
[`barn_dynamic_obstacle`](../../simulation/barn_dynamic_obstacle/README.md)

---

## 1. What has been built

| Piece | Where | What it does |
|-------|-------|--------------|
| **Obstacle-track messages** | `barn_msgs` | `ObstacleTrack` (id, position, velocity, radius, confidence) + `ObstacleTrackArray` — the shared contract between the C++ tracker and the C++ MPC. |
| **Gazebo mover plugin** | `simulation/barn_dynamic_obstacle/` | A standalone `gz::sim::System` (`barn::sim::DynamicObstacle`) that drives a model along a Catmull-Rom cubic path at constant speed via `LinearVelocityCmd`. Mirrors DynaBARN's cubic-trajectory motion model. |
| **LiDAR dynamic tracker** | `barn_dynamic_tracking` | `tracker_node`: cluster `/barn/scan` → associate → per-track constant-velocity Kalman filter → publish `/barn/tracks` (`ObstacleTrackArray`) + `/barn/track_markers` (RViz). Built from **allowed LiDAR history only** — never Gazebo ground truth. |
| **MPC moving-obstacle constraints** | `barn_classical` (`controller.cpp`) | Family-1 **spatiotemporal soft keep-out**: each track is predicted forward at constant velocity and, per horizon step, a linearized half-plane keeps the robot's footprint outside `obstacle_radius + robot_radius + margin`. Uses a **separate** dynamic-slack block so it cannot loosen the static distance-field constraints. |
| **Launch split** | `barn_bringup` | `classical_mpc.launch.py` restored byte-stable (no tracker) = the static/competition entrypoint. New `classical_mpc_dynamic.launch.py` includes it verbatim and adds `tracker_node` on top. |
| **DynaBARN worlds + generator** | `worlds/DynaBARN/`, `tools/generate_dynabarn_worlds.py` | Three tiers (`world_0/1/2`, easy/medium/hard) of small red moving cylinders scattered ahead of the robot in a blue-walled arena, built for the DynaBARN drive axis (spawn `(11,0)` → goal `(-9,0)` along y=0). |
| **Evaluator glue** | `tools/setup_dynabarn.sh` | Installs the worlds into the evaluator, checks the mover plugin `.so` exists, and idempotently injects `GZ_SIM_SYSTEM_PLUGIN_PATH` into the evaluator launch so the cylinders actually move. |

---

## 2. Data flow (dynamic worlds)

```
  gz-sim world (barn_dynamic_obstacle plugin moves the cylinders)
        │  simulated LiDAR
        ▼
  /barn/scan ─┐
              ├─▶ tracker_node ─▶ /barn/tracks (ObstacleTrackArray) ─▶ classical_mpc_node
  /barn/pose ─┘   (cluster→assoc→KF)                                    │ spatiotemporal
   (map frame, drift-corrected)                                        │ soft keep-out
                                                                       ▼
                                                    MPC → /barn/cmd_desired → safety_node → /cmd_vel
```

Key frame fact: `robot_adapter` publishes `/barn/pose` **drift-corrected in the
`map` frame**, so track coordinates are already in `map`. The tracker's
`output_frame` is forced to `map` to match the MPC's planning frame — otherwise
the MPC warns and misplaces the keep-out.

Obstacle discrimination (which tracks the MPC will constrain) uses two
**drift-proof** gates: speed ≥ `dynamic_min_speed` **and** radius ≤
`dynamic_max_radius`. Static walls cluster large; moving cylinders are compact —
so walls are the distance field's job and never get a dynamic keep-out.

---

## 3. How to run it

```bash
# 1. Build the pieces (inside the barn-jazzy distrobox)
colcon build --symlink-install \
  --packages-select barn_msgs barn_dynamic_tracking barn_classical barn_bringup

# 2. Build the standalone mover plugin (out-of-tree, plain CMake)
cmake -S simulation/barn_dynamic_obstacle -B simulation/barn_dynamic_obstacle/build \
  && cmake --build simulation/barn_dynamic_obstacle/build -j

# 3. Generate + install the DynaBARN worlds into the evaluator
python3 tools/generate_dynabarn_worlds.py
bash tools/setup_dynabarn.sh

# 4. Run the DYNAMIC stack on a DynaBARN world (world_idx 300/301/302 = easy/med/hard)
source ros2_ws/install/setup.bash
ros2 launch jackal_helper BARN_runner.launch.py \
  algo_type:=classical_mpc_dynamic world_idx:=300 gui:=true planner_rviz:=true
```

The plain static stack is unaffected: `algo_type:=classical_mpc world_idx:=0` still
runs the competition path with no tracker.

**Isolation switch:** set `enable_dynamic_obstacles: false` in
`ros2_ws/src/barn_bringup/config/classical_mpc.yaml` to keep the tracker running
but have the MPC ignore all tracks — useful for separating a tracker/gate problem
from an MPC-constraint problem.

---

## 4. What is verified

- Mover plugin loads and the cylinders move along their cubic paths.
- Static stack navigates a DynaBARN world (world 300) cleanly — the world geometry
  is on the correct drive axis.
- Tracker publishes `/barn/tracks`; the MPC subscribes and adds/removes dynamic
  keep-outs; both nodes stay up.
- 56 unit tests pass (Controller dynamic-obstacle tests + clustering/KF/TTC tests).

## 5. Open items / not done

- **Full dynamic dodging run** still under manual verification — that the MPC bows
  smoothly around a crossing cylinder with **no** recovery spam on open stretches.
  The recent feasibility guard in `controller.cpp` (never add a keep-out closer
  than the dynamic slack can absorb) targets exactly the "recovery fires on an
  empty path" failure.
- **Hybrid arbiter does not consume tracks yet.** `barn_hybrid/hybrid_node.py`
  still hard-codes `min_ttc = ∞` and never subscribes `/barn/tracks`; the tracker
  is currently wired into the **classical** dynamic launch, not the hybrid arbiter
  (M19).
- **60-world DynaBARN generator** — only three demo tiers exist; the full
  `world_idx 300–359` set is not generated.
- **Evaluator spawn-pose branch** for worlds 300–359 relies on `barn_runner.py`'s
  existing INIT convention; a dedicated branch is not yet added upstream.

---

## 6. Files at a glance

- `ros2_ws/src/barn_msgs/msg/ObstacleTrack.msg`, `ObstacleTrackArray.msg`
- `simulation/barn_dynamic_obstacle/` (plugin + demo world, standalone CMake)
- `ros2_ws/src/barn_dynamic_tracking/` (tracker, `config/tracking.yaml`)
- `ros2_ws/src/barn_classical/src/controller.cpp` (dynamic keep-out block)
- `ros2_ws/src/barn_bringup/launch/classical_mpc_dynamic.launch.py`
- `ros2_ws/src/barn_bringup/config/classical_mpc.yaml` (dynamic-obstacle params)
- `worlds/DynaBARN/world_{0,1,2}.world`, `tools/generate_dynabarn_worlds.py`
- `tools/setup_dynabarn.sh`, `tools/configure_barn_eval_dispatch.py`
</content>
</invoke>
