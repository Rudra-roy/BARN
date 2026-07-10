# models/

Trained-policy artifacts and their provenance.

```
e2e_rl/          end-to-end policies (Track B)
hybrid/          residual policies (Track C)
normalization/   observation mean/std used at train AND inference time
```

## Policy: not the binary
Model **weights** (`*.onnx`, `*.pt`, `*.zip`) are **git-ignored** — they are
large and machine-generated. What is tracked is:
- a **model card** per model (architecture, training config, dataset/curriculum,
  metrics, CPU latency),
- the **normalization statistics** (`normalization/obs_stats.json`),
- a placeholder `*.REMOVED_MODEL` marker where a weight file belongs.

Store the actual weights via a release artifact or Git LFS, and record the
sha256 + producing commit in the model card so a benchmark is reproducible.

## CPU latency is a first-class metric
Every model card must report single-step and p95 inference latency on the
CPU-only target. A policy that scores well but misses the control-rate budget on
an i3-class CPU is not deployable.
