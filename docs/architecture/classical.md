# Track A — Classical Navigation

> Purpose: document the classical C++ track — the runnable reactive goal-seeker that ships today, and the target map -> plan -> control -> recover pipeline it grows into.

Related documents:
[Architecture Overview](./overview.md) ·
[Track B — End-to-end RL](./e2e_rl.md) ·
[Track C — Hybrid](./hybrid.md) ·
[Robot Interface Contract](../robot_interface.md)

Track A is **Priority 1**: the most reliable, most explainable, and most
directly transferable-to-hardware of the three tracks. It is written entirely in
C++ across `barn_classical` (planning/control) and `barn_mapping` (perception),
both built on `barn_core` (pure types & math, no ROS deps).

---

## 1. Two layers, one contract

The track ships in two layers that satisfy the **same** navigation contract
(`/barn/goal` + `/barn/pose` + `/barn/scan` in, `/barn/cmd_desired` out):

| Layer                | State     | What it does                                                        |
|----------------------|-----------|---------------------------------------------------------------------|
| Vertical-slice goal-seeker | **Runnable** | Reactive control law; commands motion at `t0`; proves the end-to-end pipeline. |
| Target pipeline      | **Stubbed** | Online occupancy map -> A* -> path validate/replan -> local passage planner -> curvature/clearance controller -> recovery. |

The slice exists so the whole spine (adapters, safety, bringup, evaluator
handshake) can be validated **before** the planner is written. It is drop-in
replaced by the pipeline once that pipeline outperforms it on the benchmark.

---

## 2. The runnable goal-seeker (`goal_seeker_node`)

The slice node is a memoryless reactive controller. It holds no map and runs no
search; it converts the current goal bearing and the current forward clearance
directly into a body-frame twist at a fixed rate. Because it needs no map, it
**commands motion immediately at `t0`**, which starts the evaluation clock (the
clock begins only after the robot moves `> 0.1 m`).

### 2.1 Control law

Let `theta_err` be the bearing to the goal in the robot's base frame and let
`d_front` be the minimum LiDAR range within a forward sector.

```
  1. TURN TOWARD GOAL
       w = clamp( k_ang * theta_err , -w_max , +w_max )

  2. CREEP WHILE MISALIGNED
       if |theta_err| > heading_tol:
           v = creep_fraction * v_nominal     # rotate in place-ish, keep nudging
       else:
           v = v_nominal

  3. RAMP SPEED WITH FORWARD CLEARANCE
       if d_front < slow_distance:
           v *= (d_front - stop_distance) / (slow_distance - stop_distance)
       v = clamp(v, 0, v_max)

  4. STOP NEAR OBSTACLE OR GOAL
       if d_front < stop_distance:      v = 0      # keep w to rotate away
       if range_to_goal < goal_tolerance: v = 0; w = 0
```

### 2.2 Parameters (`barn_classical/config/goal_seeker.yaml`)

Conservative "prove the pipeline" values — deliberately slow, **not** tuned for
score.

| Parameter          | Value  | Role                                                             |
|--------------------|--------|------------------------------------------------------------------|
| `control_rate_hz`  | 20.0   | Fixed command rate.                                              |
| `front_sector_deg` | 60.0   | Angular width of the forward clearance sector.                  |
| `v_nominal`        | 0.5    | Cruise speed (slow for the slice).                              |
| `v_max`            | 2.0    | Hard speed cap (matches BARN maximum).                          |
| `w_max`            | 1.2    | Angular rate cap.                                              |
| `k_ang`            | 1.5    | Heading P-gain.                                                |
| `heading_tol`      | 0.35   | Above this bearing error (rad) the robot creeps and realigns.  |
| `stop_distance`    | 0.45   | Forward clearance (m) below which linear speed goes to zero.   |
| `slow_distance`    | 1.2    | Forward clearance (m) below which speed ramps down.            |
| `goal_tolerance`   | 0.8    | Internal stop radius (m); `< ` the evaluator's 1 m success.    |
| `creep_fraction`   | 0.15   | Fraction of `v_nominal` used while misaligned.                 |

Note `goal_tolerance = 0.8 m < 1.0 m`: the robot parks safely inside the
evaluator's success radius rather than racing across it.

---

## 3. The target pipeline

The full classical stack replaces the reactive law with a plan-based one while
keeping the same input/output contract. Data flows perception -> planning ->
control -> (recovery), each stage at its own rate.

```
   /barn/scan ──▶ [ MAPPING ]────────────────▶ /barn/occupancy  (OccupancyGrid)
   /barn/pose ──▶  online log-odds map              │
                                                    ▼
                                        [ GLOBAL PLANNER: A* ] ──▶ candidate path
                                                    │                (odom frame)
                                                    ▼
                                    [ PATH VALIDATOR / REPLAN TRIGGER ]
                                          valid? ──yes──▶ keep path
                                          invalid ──────▶ request A* replan (immediate)
                                                    │
                                                    ▼
                                   [ LOCAL PASSAGE PLANNER ] ──▶ local reference
                                          (pick/shape the gap)      + target curvature
                                                    │
                                                    ▼
                            [ CURVATURE / CLEARANCE CONTROLLER ] ──▶ /barn/cmd_desired
                                                    │
                                          stuck? ──▶ [ RECOVERY ] (rotate / back-off / re-seed)
```

Source scaffolding lives in `barn_classical/src/`
(`global_planner_astar.cpp`, `path_validator.cpp`, `local_planner.cpp`,
`controller.cpp`, `recovery.cpp`) and `barn_mapping/src/`.

### 3.1 Online occupancy map (`barn_mapping`)

The map is built **online from LiDAR only**. It is never seeded from a
ground-truth world map, the `.world` file, or a reference path — doing so would
violate the [competition-faithful rule](./overview.md#6-the-competition-faithful-rule)
and would not transfer to the physical Jackal.

Recommended settings:

| Setting        | Recommended value | Rationale                                            |
|----------------|-------------------|------------------------------------------------------|
| Frame          | `odom`            | Matches the planning/goal frame; no global map frame.|
| Resolution     | `0.05 m`          | Fine enough to resolve BARN passages.               |
| Extent         | `20 x 12 m`       | Covers the 10 m corridor plus lateral maneuvering.  |
| Representation | log-odds          | Numerically stable probabilistic fusion of hits/misses.|

Each cell is classified `FREE`, `OCCUPIED`, or `UNKNOWN` from its log-odds value.

### 3.2 Global planner — A*

A* searches the occupancy grid from the robot cell to the goal cell. It plans
over `FREE` **and** `UNKNOWN` cells (optimism into unmapped space), never
`OCCUPIED`, with an obstacle-inflation cost so paths keep clearance. Output is a
cell path lifted to a metric path in the `odom` frame.

### 3.3 Path validation & replanning

Every control cycle the current path is checked against the latest map. If a
segment now crosses an `OCCUPIED` (or under-clearance) cell, the path is
invalidated and an **immediate** A* replan is requested; otherwise the periodic
replan cadence carries it.

### 3.4 Local passage planner & controller

The local passage planner selects and shapes the specific gap the robot drives
through next, emitting a short local reference and target curvature. The
curvature/clearance controller turns that into a `(v, w)` twist that tracks the
reference while trading speed against clearance — slowing in tight passages,
opening up in clear ones. Its output is `/barn/cmd_desired`; it never writes
`/cmd_vel` and is always downstream-bounded by `barn_safety`.

### 3.5 Recovery

When progress stalls (no map advance, oscillation, or trapped clearance), the
recovery behavior intervenes: rotate in place to re-perceive, back off, and
re-seed the planner. Recovery protects **reliability**, the first priority.

---

## 4. Recommended loop rates

Rates are decoupled per stage so cheap safety runs fast while expensive search
runs slow.

| Stage                    | Recommended rate                          | Why                                               |
|--------------------------|-------------------------------------------|---------------------------------------------------|
| Occupancy map update     | sensor rate (per scan)                    | Fuse every measurement; no reason to drop scans.  |
| Path validation          | 10–20 Hz                                  | Catch newly-blocked paths quickly.               |
| A* replan                | 2–5 Hz **+ immediate on invalidation**    | Search is costly; run it lazily but react instantly when blocked. |
| Controller               | 20–50 Hz                                  | Smooth tracking and responsiveness.              |
| Safety (`barn_safety`)   | 50+ Hz                                    | Final authority must always be the fastest loop. |

---

## 5. Invariants

- **Map from allowed sensors only.** The occupancy grid is built exclusively
  from `/front/scan` + `/barn/pose`; never the ground-truth world map.
- **Motion at `t0`.** The runnable slice commands motion immediately; the
  pipeline must also begin moving without waiting for a converged map/plan.
- **Safety is downstream.** Every command exits through `/barn/cmd_desired ->
  barn_safety`; see the [Robot Interface Contract](../robot_interface.md).
- **Transfers to hardware.** Replacing only `barn_robot_adapter` must leave the
  classical algorithm running unchanged on the physical Jackal.
