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
   for each score-0 trial  ---->  assign S## / D## + notes  ---->  failure_labels.csv
          |
          v
   group by failure_code  ---->  rank modes by frequency
          |
          v
   fix the top failure mode (reliability)  BEFORE  optimizing AT (speed)
```
