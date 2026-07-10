# barn_rl_runtime — Track B (runtime only)

CPU policy inference for the end-to-end RL track. **Runtime only** — training
lives in [`learning/`](../../../learning/), never in a ROS package.

## Data flow
```
/barn/goal, /barn/pose, /barn/scan
   -> observation.build_observation
   -> normalization.Normalizer.apply
   -> model_loader.PolicyModel.infer   (ONNX, CPU, lazy import)
   -> action_scale.scale_action
   -> /barn/cmd_desired (TwistStamped) -> barn_safety
```

## Status
Stub. With no `model_path` set, `PolicyModel.infer` returns a neutral action, so
the node publishes **zero motion** — expected until a policy is trained and
exported. `action_scale` and `normalization` are real and testable.

## Contract (hard rules)
- Observation = downsampled LiDAR + goal distance + `sin`/`cos` bearing +
  velocity + previous action. **Never** world index, Gazebo poses, or a
  reference path. See [`docs/architecture/e2e_rl.md`](../../../docs/architecture/e2e_rl.md).
- Inference must be **CPU-capable**; measure mean/p95 latency (i3-class target).
- RL output still passes through `barn_safety`; it never reaches `/cmd_vel` directly.

## Tests
`test/test_action_scale.py` plus ament flake8/pep257 lint.
