# learning/

Offline RL training for the end-to-end (Track B) and hybrid-residual (Track C)
policies. **ROS is not required** to run the fast 2-D trainer.

```
barn_rl_train/
  barn_rl_train/env.py          Gymnasium-compatible 2-D BARN env (stub)
  barn_rl_train/policy.py       policy network definition (stub)
  barn_rl_train/train.py        training entrypoint (stub)
  barn_rl_train/export_onnx.py  ONNX export + normalization stats (stub)
  barn_rl_train/config.py       YAML config loader
  configs/ppo_barn.yaml         example PPO config
requirements.txt                training/export deps (GPU allowed)
```

## Workflow
```bash
python -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
python -m barn_rl_train.train --config barn_rl_train/configs/ppo_barn.yaml
python -m barn_rl_train.export_onnx --checkpoint <ckpt> --output ../models/e2e_rl/policy.onnx
```
Then point `barn_rl_runtime`'s `model_path` at the exported `.onnx`.

## Hard rules
- **Train with GPU, benchmark inference on CPU** (i3-class target). Measure
  mean/p95 latency before claiming a policy is deployable.
- The observation must match `barn_rl_runtime/observation.py` **exactly** and
  contain no privileged simulator information — see
  [`docs/architecture/e2e_rl.md`](../docs/architecture/e2e_rl.md) and
  [`docs/benchmark/barn_2026_contract.md`](../docs/benchmark/barn_2026_contract.md).
- Export the normalization stats used in training to
  `models/normalization/obs_stats.json`.
