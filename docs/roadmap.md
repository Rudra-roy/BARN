# barn-2027-prep Programme Roadmap (M0–M21)

> Purpose: the ordered milestone plan across the classical, RL, and hybrid tracks, with the
> current repo state marked and the development-priority and static-regression rules that
> gate progress.

Related documents:

- [Benchmark contract](./benchmark/barn_2026_contract.md) — the task, robot, and information policy every milestone must respect.
- [Scoring metric notes](./benchmark/metric_notes.md) — the 500-trial benchmark and two-report policy.
- [Failure taxonomy](./benchmark/failure_taxonomy.md) — how each milestone's failures are labeled.

---

## 1. Development Priority

Every milestone is ordered by the same principle:

```
   RELIABILITY   -->   CONSISTENCY   -->   SPEED
   (don't fail)       (fail the same,     (then optimize AT)
                       rarely, and
                       recover)
```

- **Reliability** — convert score-0 trials (collisions, timeouts) into successes first.
  A hard zero costs far more than a slow success.
- **Consistency** — reduce variance across the 10 trials per world and across worlds;
  generalize to unseen maps (the hidden qualifier is 50 unseen worlds).
- **Speed** — only once reliable and consistent, shrink `AT` toward `OT` to raise the
  clipped score.

Fix failures before optimizing speed (see [failure_taxonomy.md](./benchmark/failure_taxonomy.md)).

---

## 2. Current Repo State

```
   Track A (shared / classical vertical slice)
   M0 --- M1 --- M2 --- M3 === M4 ... M10 === M11      M0-M10 DONE, M11 (benchmark) pending
   [DONE][DONE][DONE][DONE]  [DONE]...[DONE]  ^
                                             Full stack runnable
```

- **M0–M10 are DONE**. The complete Classical MPC stack is fully implemented, tuned, and robust.
- **M11 is the current focus**: Running the formal 500-trial baseline benchmark.
- **M12–M21 are scaffolded stubs** — directories and interfaces exist, implementations pending.

---

## 3. Milestone Table

Tracks: **A** = shared/classical, **B** = RL, **C** = hybrid.

| ID | Track | Milestone | Status |
|---|---|---|---|
| M0 | A | 2026 BARN baseline runs unchanged | **DONE** |
| M1 | A | Custom `NavigateToPose` adapter | **DONE** |
| M2 | A | Custom robot adapter publishes valid commands | **DONE** |
| M3 | A | Trivial goal seeker runs under the evaluator | **DONE** |
| M4 | A | Online LiDAR occupancy map | **DONE** |
| M5 | A | A* through free + unknown space | **DONE** |
| M6 | A | Path invalidation and on-the-fly replanning | **DONE** |
| M7 | A | Local passage planner | **DONE** |
| M8 | A | Curvature / clearance-aware control | **DONE** |
| M9 | A | Safety shield | **DONE** |
| M10 | A | Recovery behavior | **DONE** |
| M11 | A | Classical 500-trial benchmark | Pending (Current focus) |
| M12 | B | Fast 2-D RL environment | Pending / stub |
| M13 | B | E2E PPO baseline | Pending / stub |
| M14 | B | E2E SAC candidate | Pending / stub |
| M15 | B | Gazebo transfer and CPU inference | Pending / stub |
| M16 | B | E2E 500-trial benchmark | Pending / stub |
| M17 | C | Dynamic-obstacle ROS 2 test infrastructure | Pending / stub |
| M18 | C | LiDAR dynamic tracker | Pending / stub |
| M19 | C | Hybrid residual RL | Pending / stub |
| M20 | C | Hybrid static regression test | Pending / stub |
| M21 | C | Hybrid dynamic benchmark | Pending / stub |

---

## 4. Track Summaries

### Track A — Shared / Classical (M0–M11)

Builds the compliant classical stack end to end: adapters (M1–M2), a trivial seeker under
the evaluator (M3), online mapping and planning (M4–M7), control and safety (M8–M10), and
the first reportable **classical 500-trial benchmark** (M11). This track also produces the
shared infrastructure the other tracks reuse.

### Track B — RL (M12–M16)

A fast 2-D environment (M12) trains end-to-end policies — a PPO baseline (M13) and a SAC
candidate (M14) — then transfers to Gazebo with CPU inference (M15) and runs the **E2E
500-trial benchmark** (M16) for comparison against the classical baseline.

### Track C — Hybrid (M17–M21)

Adds dynamic obstacles: test infrastructure (M17), a LiDAR dynamic tracker (M18), and a
**hybrid residual RL** controller (M19) layered on the classical stack. It is validated by
a **static regression test** (M20) and a **hybrid dynamic benchmark** (M21).

---

## 5. Track C Static-Regression Rule

Adding the hybrid residual RL layer must **not** degrade static-world performance. The
gate:

> **Hybrid static success >= classical static success − ~1 percentage point.**

Worked example:

| Configuration | Static success |
|---|---|
| Classical baseline | 97.4% |
| Hybrid (with residual RL) | 96.4% |

Here the hybrid is `97.4% − 96.4% = 1.0 pp` below the classical baseline — right at the
tolerance. Anything worse than roughly 1 pp of static regression is a **failure of M20**
and blocks progress to the hybrid dynamic benchmark (M21). The dynamic capability must be
purchased without meaningfully sacrificing the static reliability already achieved.

---

## 6. Milestone Completion Criteria (general)

For each milestone to count as done:

- Runs under the real ROS 2 evaluator, obeying the [information policy](./benchmark/barn_2026_contract.md#4-competition-faithful-information-policy).
- Benchmark milestones (M11, M16, M21) run the full 50-world × 10-trial campaign via
  `evaluation/scripts/run_barn2026_public_suite.sh`, emit **both** metric reports, and
  capture a manifest.
- Every failed trial is labeled per [failure_taxonomy.md](./benchmark/failure_taxonomy.md).
- Reliability and consistency targets met before speed is optimized (§1).
