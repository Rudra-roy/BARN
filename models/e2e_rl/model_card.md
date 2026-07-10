# Model card: e2e_rl/policy.onnx (template)

> Fill this in when a policy is exported. Weights are git-ignored; keep this card
> and the `policy.onnx.REMOVED_MODEL` marker tracked.

| Field | Value |
|-------|-------|
| Model id | `e2e_rl_exp_000` |
| Architecture | tanh MLP, hidden (256, 256), action ∈ [-1, 1] |
| Algorithm | PPO (see `learning/.../configs/ppo_barn.yaml`) |
| Observation version | `v1` (must match `barn_rl_runtime/observation.py`) |
| Training commit | `REPLACE_WITH_GIT_HASH` |
| Weights sha256 | `REPLACE` |
| Normalization | `models/normalization/obs_stats.json` |

## Runtime (CPU-only target, required)
| Metric | Value |
|--------|-------|
| Runtime | ONNX Runtime CPU |
| Mean latency | `REPLACE` ms |
| p95 latency | `REPLACE` ms |
| Policy rate | `REPLACE` Hz |

## Benchmark
| Suite | Metric | Success | Score |
|-------|--------|---------|-------|
| public_50 × 10 | BARN 2026 published | `REPLACE` | `REPLACE` |
