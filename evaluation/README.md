# evaluation/

Everything for running BARN campaigns and scoring them reproducibly.

```
scripts/   run_single_world.sh · run_dev_suite.sh · run_barn2026_public_suite.sh · capture_manifest.sh
metrics/   barn2026_metric.py (published 2·OT) · upstream_compat_metric.py (4·OT) · _common.py
suites/    dev_worlds.txt · barn2026_public_50.txt (0,6,…,294)
schemas/   experiment.schema.yaml · result_record.schema.json
tuning/    single-world parameter batches: run_tuning_batch.sh · in_box.sh · RESULTS.md
```

## Three levels of testing
1. **Smoke** — `run_single_world.sh <idx> builtin` on one world (GUI optional).
2. **Dev sweep** — `run_dev_suite.sh builtin` over `suites/dev_worlds.txt`.
3. **Scored campaign** — `run_barn2026_public_suite.sh builtin`: 50 worlds × 10
   trials = 500 trials, with a captured manifest and both metric reports.

## Trial isolation — read this before trusting any numbers
`ros2 launch` does not reap its children. A trial killed any way other than a
clean exit leaks them to `init`, still on your `ROS_DOMAIN_ID`. The worst is the
`/clock` bridge: it keeps publishing **simulation time from a dead world**, so
the evaluator's clock starts during the reset teleport and ~30 s is added to
every later trial — silently, looking exactly like a planner regression. One
survivor poisoned two full 12-trial batches.

- `run_single_world.sh` reaps **after every trial**, not only on the timeout
  path, matching by PID via `/proc/<pid>/cmdline` (never a bare `pkill` on a
  node name — that ignores `ROS_DOMAIN_ID`).
- `tools/preflight_barn_campaign.sh` refuses to start while a stale `/clock`
  bridge or Gazebo is alive. It runs automatically from `run_single_world.sh`.
- Tell-tale of a poisoned trial: the log shows the robot at `(2.00, 2.00)` at
  `t=0.01` instead of the start pose `(-2.25, 3.1x)`. Discard, don't label —
  see [failure_taxonomy.md §3.5](../docs/benchmark/failure_taxonomy.md).

## Parameter tuning batches (`tuning/`)
One world, N trials, headless, with the exact config that produced the results
snapshotted next to them:

```bash
distrobox enter barn-jazzy -- bash \
  /run/host/home/mt-labpc/BARN/evaluation/tuning/in_box.sh \
  run_tuning_batch.sh <world> <trials> <tag>
```

Each batch writes `tuning/world_<idx>/<tag>/` containing `raw_results.txt`, the
exact `classical_mpc.yaml` that produced it, per-trial launch logs, and a
`summary.txt` — a tuning number without its parameters is worthless three
iterations later. `tuning/baseline_config/` holds the pristine pre-tuning YAMLs.

`in_box.sh` exists because `distrobox enter -- bash -c` is non-interactive and
skips the box's `.bashrc`, so the host's ROS Humble leaks in; it replicates the
scrub deterministically. Use `distrobox enter`, **not** `docker exec` — the
latter starts with an empty `LD_LIBRARY_PATH` and `gz sim` segfaults with no
GL context. Results, accepted and rejected changes: [`tuning/RESULTS.md`](./tuning/RESULTS.md).

## Two metric reports — never mixed
| Script | Clip | Use |
|--------|------|-----|
| `barn2026_metric.py` | `[2·OT, 8·OT]` | research numbers (published rule) |
| `upstream_compat_metric.py` | `[4·OT, 8·OT]` | evaluator-compatibility debugging |

Both accept `--out_path <evaluator out_file>` and `--selftest`. Every table
states which metric produced it. Why two:
[`docs/benchmark/metric_notes.md`](../docs/benchmark/metric_notes.md).

## Note on the out_file format
`_common.py:COLUMN_ORDER` assumes `world_idx success collided timeout actual_time
optimal_time`. Confirm this against your evaluator's output on the first
run and adjust if the columns differ.
