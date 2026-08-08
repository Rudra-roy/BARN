# BARN 2026 Failure Taxonomy

> Purpose: a fixed vocabulary of failure modes so that every score-0 trial is explained by
> a concrete cause, and the labeling format (`failure_labels.csv`) that records them.

Related documents:

- [Benchmark contract](./barn_2026_contract.md) — success/failure definition and testing levels.
- [Scoring metric notes](./metric_notes.md) — why a failure scores 0 regardless of speed.
- [Programme roadmap](../roadmap.md) — where dynamic-obstacle and hybrid milestones introduce D-codes.

---

## 1. Why a Taxonomy

The metric collapses every failure to a score of **0**. That number tells you *that* a
trial failed, never *why*. When a Jackal clips the corner of a box, a reward curve or a
mean score will not tell you it was corner-cutting versus a late replan versus an
oscillation that drifted into the wall.

> **Reward curves and aggregate scores do not explain why a Jackal hit a box — failure
> labels do.** Label every failure, then attack the labels.

**Development priority: fix failures before optimizing speed.** A collision or timeout is
a hard zero; shaving a second off `AT` on already-successful runs is worth far less than
converting a zero into a success. Reliability first, then efficiency.

---

## 2. Static Failure Codes (S01–S13)

Static-world failures — dense obstacle fields with no moving obstacles.

| Code | Name | Description |
|---|---|---|
| S01 | Straight collision | Robot drives straight into an obstacle directly ahead |
| S02 | Corner cutting | Path clips the corner of an obstacle while turning |
| S03 | Narrow-gap side collision | Robot contacts a side wall while threading a narrow passage |
| S04 | Oscillation | Robot oscillates side-to-side without making progress |
| S05 | Local minimum | Robot gets trapped in a dead-end / concave pocket |
| S06 | Failed replan | Planner fails to produce a new valid path when needed |
| S07 | Invalid map accumulation | Online occupancy grid accumulates errors and corrupts planning |
| S08 | Localization/odom inconsistency | Pose estimate drifts or disagrees with motion |
| S09 | Excessive speed | Robot moves too fast to react and collides |
| S10 | Recovery failure | Recovery behavior triggers but fails to free the robot |
| S11 | Startup failure | Robot never begins moving / never leaves the start cleanly |
| S12 | Command pipeline failure | Commands are malformed, dropped, or not delivered to the base |
| S13 | Timeout due to conservatism | Robot is too cautious and runs out the 100 s clock without colliding |

> **Note on S10 vs S13 — the shield freeze.** The classical stack's dominant failure was a robot
> frozen at 0 m/s until the clock expired, with recovery *active* and commanding motion the
> safety shield vetoed to zero. That is **S13**, not S10: recovery did not fail to trigger or
> fail to choose a maneuver, it was prevented from executing one. Label by which component was
> the binding constraint — `emergency_veto`/`emergency_trapped` in the log means the shield was.
> See [`docs/features/classical_mpc_updates.md`](../features/classical_mpc_updates.md) §6.

---

## 3. Dynamic Failure Codes (D01–D12)

Dynamic-world failures — introduced with the moving-obstacle infrastructure (see roadmap
M17–M21).

| Code | Name | Description |
|---|---|---|
| D01 | Frontal crossing obstacle | Collision with an obstacle crossing the path head-on |
| D02 | Lateral crossing obstacle | Collision with an obstacle crossing from the side |
| D03 | Oncoming obstacle | Collision with an obstacle approaching directly toward the robot |
| D04 | Following interaction | Failure while following/being followed by a moving obstacle |
| D05 | Multiple dynamic obstacles | Failure amid several simultaneous moving obstacles |
| D06 | Tracker association failure | Tracker mis-associates detections across frames |
| D07 | Velocity estimation failure | Estimated obstacle velocity is wrong, causing a bad avoidance |
| D08 | Risk gate activated late | Safety/risk gate engages too late to prevent contact |
| D09 | Risk gate chattering | Risk gate toggles rapidly, producing unstable motion |
| D10 | RL residual unsafe | The learned residual policy commands an unsafe action |
| D11 | Safety layer rejected RL repeatedly | Safety layer keeps vetoing the RL policy, causing stall/failure |
| D12 | Static/dynamic misclassification | A static obstacle is treated as dynamic (or vice versa) |

---

## 3.5 Not a Failure Code: Invalidated Trials

Some zero-score trials are not the stack's fault at all — the *harness* was broken. These must
be **discarded and re-run**, never labeled. Labeling them pollutes the frequency ranking in §5
and sends you optimizing a planner that was never at fault.

The dominant case is **state leaking between trials**. `ros2 launch` does not reap its children,
so a trial killed any way other than a clean exit re-parents them to `init` — still on your
`ROS_DOMAIN_ID`, still talking to the *next* trial. The worst survivor is the `/clock` bridge
(`parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock`): it keeps publishing
simulation time from a world that no longer exists, so the evaluator's clock starts during the
reset teleport and roughly **30 s is added to every subsequent trial**, uniformly and silently.

| Symptom | What it looks like | Real cause |
|---|---|---|
| Robot logged at `(2.00, 2.00)` at `t = 0.01` | should be the BARN start pose `(-2.25, 3.1x)` | leaked `/clock` bridge; clock started during the teleport |
| Every trial ~30 s slower than the same config yesterday | reads exactly like a planner regression | same |
| `Test on world_N not finished` in the metric report | trial abandoned with no result line | 300 s wall-clock cap in `run_single_world.sh` — Gazebo never came up |

**Check before you label:** open the trial log and confirm the startup pose. Then confirm no
leaked processes survive; `tools/preflight_barn_campaign.sh` refuses to start a campaign while a
stale `/clock` bridge or Gazebo is alive, and `evaluation/scripts/run_single_world.sh` reaps by
PID after *every* trial. A campaign started without preflight is not a campaign.

Other invalidating conditions: a Gazebo startup segfault, a run whose `out_file` was not set
(results land in the evaluator's default file and get mixed with someone else's), and any batch
whose config snapshot does not match the config you intended to test.

---

## 4. Labeling Format: `failure_labels.csv`

Every failed trial gets exactly one code. Store labels in `failure_labels.csv` with these
columns:

```
experiment_id,world_idx,trial,status,failure_code,notes
```

| Column | Meaning |
|---|---|
| `experiment_id` | Identifier of the experiment/campaign that produced the trial |
| `world_idx` | World index (e.g. one of `[0, 6, …, 294]`) |
| `trial` | Trial number within the world (0–9) |
| `status` | Trial outcome (e.g. `collision`, `timeout`, `failure`) |
| `failure_code` | One code from S01–S13 or D01–D12 |
| `notes` | Free-text detail for triage |

### Example rows

```csv
experiment_id,world_idx,trial,status,failure_code,notes
classical_slice_v1,42,3,collision,S02,clipped box corner while turning left into the gap
hybrid_dyn_v0,120,7,collision,D03,failed to yield to oncoming obstacle; braked too late
```

Guidance:

- Label **every** failed trial — an unlabeled zero is a lost debugging opportunity.
- Successful trials do not need a code (leave them out of the failure CSV).
- Use `notes` to capture the specifics the code cannot (which gap, which obstacle, timing).
- Aggregate by `failure_code` to see which mode dominates, then prioritize that fix before
  touching speed.

---

## 5. Triage Workflow

```
   500-trial campaign
          |
          v
   discard INVALIDATED trials (§3.5) and re-run them  -- do NOT label these
          |
          v
   for each remaining score-0 trial  ---->  assign S## / D## + notes  ---->  failure_labels.csv
          |
          v
   group by failure_code  ---->  rank modes by frequency
          |
          v
   fix the top failure mode (reliability)  BEFORE  optimizing AT (speed)
```
