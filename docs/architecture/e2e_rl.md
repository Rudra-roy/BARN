# Track B — End-to-end Reinforcement Learning

> Purpose: document the end-to-end RL track — the observation/action contract, the CPU-only ONNX runtime pipeline, and the strict training/runtime split that keeps inference deployable and honest.

Related documents:
[Architecture Overview](./overview.md) ·
[Track A — Classical](./classical.md) ·
[Track C — Hybrid](./hybrid.md) ·
[Robot Interface Contract](../robot_interface.md)

Track B learns a single policy that maps raw-ish sensor observations directly to
motion commands. Training happens offline in `learning/`; inference happens in
the ROS package `barn_rl_runtime` (Python). The shipped stub policy commands
**zero motion** until a model is trained — the plumbing is complete and safe,
and only the weights are missing.

---

## 1. Observation contract

The policy observes **only** what a physical Jackal could observe. This is a hard
contract, enforced to satisfy the
[competition-faithful rule](./overview.md#6-the-competition-faithful-rule).

**Allowed observation vector:**

| # | Component                | Notes                                              |
|---|--------------------------|----------------------------------------------------|
| 1 | Downsampled LiDAR        | Fixed-length beam vector from `/barn/scan`.        |
| 2 | Goal distance            | Range from robot to goal.                          |
| 3 | `sin(bearing)`           | Goal bearing encoded as sine…                      |
| 4 | `cos(bearing)`           | …and cosine (continuous, wrap-free heading).       |
| 5 | Linear velocity          | Current body-frame `v`.                            |
| 6 | Angular velocity         | Current body-frame `w`.                            |
| 7 | Previous action          | Last `[v, w]` command (temporal continuity).       |

**Prohibited inputs (never observed):**

- world index / world identifier;
- Gazebo poses or any simulator ground truth;
- global occupancy grid or prebuilt map;
- reference path;
- ground-truth obstacle class or count.

The bearing is split into `sin` + `cos` so the network sees a continuous signal
with no `±pi` wraparound discontinuity. Goal distance and bearing derive from
`/barn/goal` and `/barn/pose`, both of which are robot-derived, not privileged.

---

## 2. Action space

The policy outputs a normalized 2-vector, later scaled to physical limits:

```
   network output:   a = [a_v, a_w] ,   a_v, a_w in [-1, 1]
   scaled command:   v = a_v * v_max
                     w = a_w * w_max
```

Normalizing to `[-1, 1]` keeps the output range consistent across training and
deployment and independent of the physical limits, which are applied only at the
scaling step. Scaling uses the runtime's own `v_max` / `w_max`
(`barn_rl_runtime/config/rl_runtime.yaml`), and the scaled command is still
subject to the final clamp in `barn_safety`.

---

## 3. Runtime pipeline (`rl_runtime_node`)

Inference is a fixed-rate loop (`control_rate_hz = 20.0`). Each tick:

```
   /barn/scan ┐
   /barn/pose ├─▶ [ BUILD OBSERVATION ] ──▶ [ NORMALIZE ] ──▶ [ ONNX MODEL (CPU) ]
   /barn/goal ┘        observation.py          normalization.py     model_loader.py
                                                                          │
                                                                          ▼
                                                                  [ ACTION SCALE ]
                                                                   action_scale.py
                                                                          │
                                                                          ▼
                                               /barn/cmd_desired ──▶ safety_node ──▶ /cmd_vel
                                                  (TwistStamped)      (final authority)
```

Node module map (`barn_rl_runtime/barn_rl_runtime/`):

| Module              | Responsibility                                                    |
|---------------------|-------------------------------------------------------------------|
| `observation.py`    | Assemble the exact observation vector from `/barn/*`.             |
| `normalization.py`  | Apply saved input normalization (mean/variance) from training.   |
| `model_loader.py`   | Load the ONNX policy; empty `model_path` -> stub (no motion).     |
| `action_scale.py`   | Map `[-1,1]` action to `(v, w)` via `v_max` / `w_max`.           |
| `rl_runtime_node.py`| Wire the loop; publish `/barn/cmd_desired`.                       |

The stub path is the safe default: with `model_path: ""`, the node publishes zero
motion rather than random commands, so an untrained deployment simply does not
move — it never behaves unsafely.

---

## 4. Training / runtime split

The two halves are strictly separated so nothing simulator-specific leaks into
the deployed node.

```
   OFFLINE (learning/)                        ONLINE (barn_rl_runtime)
   ┌──────────────────────────┐               ┌──────────────────────────┐
   │ barn_rl_train            │   exports     │ rl_runtime_node          │
   │  • env + reward          │  ─────────▶   │  • load ONNX (CPU)       │
   │  • PPO/SAC training       │   .onnx  +    │  • same obs contract     │
   │  • normalization stats    │  norm stats   │  • scale + publish       │
   │  • configs/               │               │  • NO training deps      │
   └──────────────────────────┘               └──────────────────────────┘
        heavy, GPU-friendly                        light, CPU-only
```

- **Training** (`learning/barn_rl_train/`, with `configs/`) may use whatever
  compute and simulator access it wants — but the observation it trains on must
  be exactly the deployable contract in §1, or the learned policy will not
  transfer.
- **Runtime** carries only what inference needs: the exported `.onnx` model
  (`models/e2e_rl/`) and the frozen normalization statistics
  (`models/normalization/`). It has no training-framework dependency.
- The **exported artifact is the interface** between the halves: an ONNX graph
  plus normalization stats. Nothing else crosses the boundary.

---

## 5. CPU-only inference & latency budget

Scored inference must run on a modest CPU (**i3-class target, no GPU**). The
model is exported to ONNX and executed on CPU; this constrains network size and
observation dimensionality.

Metrics to record every time a policy changes:

| Metric                | Why it matters                                                   |
|-----------------------|-------------------------------------------------------------------|
| Mean inference latency| Must fit comfortably inside the `20 Hz` (50 ms) control period.   |
| p95 inference latency | Tail latency, not the average, is what causes missed control ticks.|

If p95 latency approaches the control period, shrink the network or the
downsampled LiDAR width rather than lowering the control rate — reliability first.

---

## 6. Prohibition on privileged inputs

Track B is the easiest track to accidentally cheat, so the rule is repeated: the
policy may consume **only** the observation vector in §1. Feeding it a world
index, Gazebo poses, a global map, a reference path, or ground-truth obstacle
labels — even "just for training" while planning to remove it later — produces a
policy that cannot transfer to the physical Jackal and is disqualified under the
[competition-faithful rule](./overview.md#6-the-competition-faithful-rule).

**Safety is not part of the policy.** The RL output is a *desired* command; it
passes through `barn_safety` exactly like every other track and can never bypass
the clamp / accel-limit / stale-gate. See the
[Robot Interface Contract](../robot_interface.md).
