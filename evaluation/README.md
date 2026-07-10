# evaluation/

Everything for running BARN campaigns and scoring them reproducibly.

```
scripts/   run_single_world.sh · run_dev_suite.sh · run_barn2026_public_suite.sh · capture_manifest.sh
metrics/   barn2026_metric.py (published 2·OT) · upstream_compat_metric.py (4·OT) · _common.py
suites/    dev_worlds.txt · barn2026_public_50.txt (0,6,…,294)
schemas/   experiment.schema.yaml · result_record.schema.json
```

## Three levels of testing
1. **Smoke** — `run_single_world.sh <idx> classical` on one world (GUI optional).
2. **Dev sweep** — `run_dev_suite.sh classical` over `suites/dev_worlds.txt`.
3. **Scored campaign** — `run_barn2026_public_suite.sh classical`: 50 worlds × 10
   trials = 500 trials, with a captured manifest and both metric reports.

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
optimal_time`. Confirm this against your pinned evaluator's output on the first
run and adjust if the columns differ.
