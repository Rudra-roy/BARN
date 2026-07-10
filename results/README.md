# results/

Benchmark output. **The raw evaluator output is the source of truth** — derived
metrics are recomputed from it, never the other way around.

## Layout
```
results/<track>/<experiment_id>/
├── raw_results.txt              # evaluator's per-trial output (source of truth)
├── manifest.json                # repo+evaluator commit, ROS/host info (capture_manifest.sh)
├── experiment.yaml              # experiment manifest (see evaluation/experiment.example.yaml)
├── report_barn2026_rules.txt    # published metric, clip=[2*OT, 8*OT]
├── report_upstream_master.txt   # upstream metric, clip=[4*OT, 8*OT]
└── failure_labels.csv           # one row per failed trial (taxonomy code)
```

## What is committed
Heavy/raw artifacts (`*.txt` results, bags, csv) are git-ignored. Manifests,
`experiment.yaml`, `report_*.txt`, and `failure_labels.csv` are kept so a
campaign's context and headline numbers are traceable. Store large raw outputs
outside git (release artifact / archive) and reference them from the manifest.

## Rules
- Never overwrite the only raw result file for a campaign.
- Every table states its metric definition (see
  [`docs/benchmark/metric_notes.md`](../docs/benchmark/metric_notes.md)).
- No per-world tuning during a scored campaign.
