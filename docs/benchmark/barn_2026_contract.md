# BARN 2026 Benchmark Contract

> Purpose: the authoritative specification of the task, robot, scoring, information policy, and testing discipline that every component in `barn-2027-prep` must satisfy. If an experiment violates this contract, its numbers are not comparable to the benchmark.

Related documents:

- [Scoring metric notes](./metric_notes.md) — formula, clip semantics, and the two-report policy.
- [Failure taxonomy](./failure_taxonomy.md) — static/dynamic failure codes and labeling format.
- [Robot interface](./robot_interface.md) — action/adapter contract between the stack and the evaluator.
- [Programme roadmap](../roadmap.md) — milestone plan M0–M21.

---

## 1. Overview

The ICRA BARN (Benchmark for Autonomous Robot Navigation) Challenge evaluates a full
navigation stack on a Clearpath **Jackal** differential-drive robot driving through
dense, cluttered static obstacle fields. This programme (`barn-2027-prep`) prepares for
the **BARN Challenge 2027** while benchmarking against the completed **BARN 2026 ROS 2
evaluator**, which is treated as the frozen ground truth for scoring and runner behavior.

The core principle of this repository: **the evaluator decides success from physical
state, not from what your stack claims.** Your job is to build an algorithm that would
survive being moved onto a physical Jackal with only a hardware-adapter change.

```
        goal pose
           |
           v
   +-------------------+     /cmd_vel     +--------------------+
   |  YOUR NAV STACK   | --------------->  |  ROS 2 EVALUATOR   |
   |  (perception,     |                   |  (Gazebo + runner) |
   |   planning,       | <---------------  |                    |
   |   control)        |  /front/scan,     |  judges collision, |
   +-------------------+  TF, odom, vel    |  goal, timeout via |
                                           |  SIM GROUND TRUTH  |
                                           +--------------------+
   Allowed inputs only  <----- hard boundary ----->  May use ground truth
```

---

## 2. Task Definition

| Aspect | Specification |
|---|---|
| Robot | Clearpath Jackal, differential drive, commanded via velocity (`/cmd_vel`) |
| Objective | Drive from a fixed start to a fixed goal through a static obstacle course |
| Goal tolerance | Reach within **1.0 m** of the goal (effective success radius) |
| Timeout | **100 s** wall/sim time per trial (default) |
| Collision | Any contact with an obstacle terminates the trial as a failure |
| Clock start | The evaluation clock starts only after the robot has moved **> 0.1 m** |
| Worlds | 300 public worlds (0–299); scored campaign uses 50 evenly spaced worlds |
| Trials | 10 trials per world → **500 trials** per full campaign |

### Runner success condition

The ROS 2 runner continues the trial while **all** of the following hold:

```
distance_to_goal > 1.0 m   AND   not collided   AND   elapsed < timeout(100 s)
```

The instant `distance_to_goal <= 1.0 m` with no collision, the trial is a **success**.
A collision or reaching the timeout is a **failure**.

> Do **not** rely on your `NavigateToPose` action server returning `SUCCEEDED`. The runner
> ignores your action result and determines success purely from the robot's physical state
> in simulation. A stack that reports success while parked short of the goal still fails.

---

## 3. Scoring Summary

Per-trial score:

```
s_i = success_i * OT_i / clip( AT_i , 2*OT_i , 8*OT_i )
```

- `success_i` = 1 if the goal is reached (within 1 m) without collision before timeout, else 0.
- `AT_i` = actual traversal time.
- `OT_i` = optimal traversal time = `reference_path_length_i / 2.0` (2 m/s max speed).
- The **clip** term bounds `AT` into `[2*OT, 8*OT]` so neither very fast nor very slow
  runs dominate the mean.

A **collision or timeout scores 0** regardless of how fast the robot was moving. The
overall score is the mean over all **500 trials** (50 worlds × 10 trials).

> There is a known discrepancy between the published clip lower bound (`2*OT`) and the
> upstream evaluator source (`4*OT`). This repository always emits **two reports** and
> never mixes them. See [metric_notes.md](./metric_notes.md) for the full explanation,
> the two scoring scripts, and a worked example.

---

## 4. Competition-Faithful Information Policy

A result is only valid if the navigation stack consumed **allowed inputs only**. The
evaluator is permitted to use simulator ground truth internally to judge collision and
goal; **your stack must not** touch any of it as perception.

### 4.1 Allowed navigation inputs

| Input | Notes |
|---|---|
| `/front/scan` | 2-D LiDAR — the primary exteroceptive sensor |
| TF | Coordinate frames / transforms |
| Odometry | Wheel/base odometry |
| Filtered odometry & velocity | e.g. `platform/odom/filtered` |
| Goal pose | The target the runner assigns |
| Previous commands | Your own command history |
| Sensor history | Buffered past sensor readings |
| Online occupancy grid | Built **at runtime from allowed sensor data only** |
| Estimated obstacle tracks / velocity | Internally estimated from allowed sensors |

### 4.2 NOT allowed in scored runs

| Forbidden input | Why it breaks the contract |
|---|---|
| Gazebo world geometry | Simulator-only ground truth |
| Parsing the current `.world` file | Reading the map you are being tested on |
| Ground-truth obstacle positions / model-state topics | Perception cheating |
| Current world index as an RL observation | Leaks which map you are on |
| Preloaded occupancy map of the current test world | Memorizes the test map |
| The reference Dijkstra path used directly by the controller | The reference path is for scoring `OT`, not for driving |
| Hard-coded per-world paths | Overfits to specific worlds |
| World-specific parameter lookup tables | Overfits to specific worlds |

### 4.3 The "physical Jackal" test

> **Could I replace the simulated Jackal with a physical Jackal and keep the algorithm
> intact after changing only the ROS hardware adapter?**

If the answer is no — because the stack reads world files, model-state topics, the world
index, or a preloaded map — the stack is non-compliant. Every perception and planning
component must pass this test. Only the hardware/simulation adapter layer is allowed to
differ between sim and real.

---

## 5. Generalization and Development-Data Discipline

The competition qualifier uses **50 HIDDEN worlds × 10 trials**. You never see those
maps. Therefore the whole programme is designed to measure and defend **generalization**,
not per-world performance.

Data partitions and how each may be used:

| Partition | Contents | Permitted use |
|---|---|---|
| Public worlds | Worlds 0–299 | Development and validation data |
| Scored campaign | 50 evenly spaced worlds `[0, 6, 12, …, 294]` | Reported benchmark numbers |
| Sealed set | A held-out set fixed in advance | **Must not** be used for per-world tuning |
| Hidden qualifier | 50 hidden worlds (competition only) | Never available to us |

Rules:

- Tune on public/dev worlds; never tune parameters against the sealed set.
- No per-world tuning of any kind — parameters must be world-agnostic (see §4.2).
- Treat the 50-world campaign as a proxy for the hidden set, not as something to overfit.

The list of the 50 campaign worlds lives in `evaluation/suites/barn2026_public_50.txt`;
the development pool lives in `evaluation/suites/dev_worlds.txt`.

---

## 6. Three Levels of Testing

The programme uses three escalating test levels. Cheaper levels gate expensive ones.

```
  L1 SMOKE          L2 DEV SWEEP            L3 500-TRIAL CAMPAIGN
  (seconds)         (minutes)              (hours)
  1 world           handful of dev worlds  50 worlds x 10 trials = 500
  1 trial           few trials each        the reportable number
  "does it run?"    "does it generalize    "the benchmark result"
                     a little?"
     |                    |                       |
     +---- must pass ---->+---- must pass ------->+
```

| Level | Scope | Purpose | Typical driver |
|---|---|---|---|
| **L1 Smoke** | 1 world, 1 trial | Confirm the stack launches and drives under the evaluator | manual / single invocation |
| **L2 Dev sweep** | A few dev worlds, a few trials | Catch regressions and gross generalization gaps quickly | `evaluation/scripts/` sweep helpers over `dev_worlds.txt` |
| **L3 500-trial campaign** | 50 campaign worlds × 10 trials | Produce the reportable benchmark score | `evaluation/scripts/run_barn2026_public_suite.sh` |

> Use `evaluation/scripts/run_barn2026_public_suite.sh` for the full campaign — it
> correctly iterates the 50 evenly spaced worlds. **Do not** use the upstream 2026
> `test.sh` for a fresh full campaign; it has a world-selection bug documented in
> [metric_notes.md](./metric_notes.md).

Every campaign captures a manifest via `evaluation/scripts/capture_manifest.sh`, and the
raw evaluator output under `results/` is the source of truth. Results tables must always
state which metric definition produced them.

---

## 7. Contract Compliance Checklist

Before reporting any number, confirm:

- [ ] Stack consumed only §4.1 allowed inputs; none from §4.2.
- [ ] Passes the "physical Jackal" test (§4.3).
- [ ] No per-world tuning; sealed set untouched (§5).
- [ ] Success determined by the runner's physical-state check, not the action result (§2).
- [ ] Both metric reports emitted; table states which one is shown (§3, [metric_notes.md](./metric_notes.md)).
- [ ] Full campaign run with `run_barn2026_public_suite.sh`, manifest captured (§6).
- [ ] Every failed trial has a code in `failure_labels.csv` ([failure_taxonomy.md](./failure_taxonomy.md)).
