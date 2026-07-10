"""Export a trained policy to ONNX for CPU inference (STUB).

The exported graph is loaded by the ROS package barn_rl_runtime. Also emit the
observation normalization statistics (mean/std) to
models/normalization/obs_stats.json — the SAME statistics must be used at
inference time.
"""

import argparse


def main(argv=None):
    """Parse args and export (stub)."""
    parser = argparse.ArgumentParser(description='Export a policy to ONNX (stub).')
    parser.add_argument('--checkpoint', required=True, help='trained policy checkpoint')
    parser.add_argument('--output', required=True, help='destination .onnx path')
    args = parser.parse_args(argv)
    print(f"[export_onnx] STUB: convert {args.checkpoint} -> {args.output}")
    print('[export_onnx] remember to also write models/normalization/obs_stats.json')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
