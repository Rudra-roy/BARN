"""Training entrypoint (STUB).

Wire a learner (PPO/SAC/TD3 via Stable-Baselines3 or a custom trainer) over
env.BarnEnv2D using the config in configs/. Export the result with export_onnx.py.
"""

import argparse

from barn_rl_train.config import load_config


def main(argv=None):
    """Parse args, load the config, and (eventually) train."""
    parser = argparse.ArgumentParser(description='Train a BARN navigation policy (stub).')
    parser.add_argument('--config', required=True, help='path to a training YAML config')
    args = parser.parse_args(argv)

    cfg = load_config(args.config)
    print(f"[train] loaded config: algorithm={cfg.get('algorithm', '?')} "
          f"total_steps={cfg.get('total_steps', '?')}")
    print('[train] STUB: implement the training loop here, then run export_onnx.py.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
