#!/usr/bin/env python3
"""BARN 2026 PUBLISHED-RULE metric report.

Per-trial score:  s = success * OT / clip(AT, 2*OT, 8*OT)

This is the metric to use for research comparisons. For evaluator-compatibility
debugging use upstream_compat_metric.py instead, and NEVER mix the two numbers
in one table (see docs/benchmark/metric_notes.md).

Usage:
    python3 barn2026_metric.py --out_path results/.../raw_results.txt
    python3 barn2026_metric.py --selftest
"""

import argparse
import sys

import _common as C

LOWER_MULT = 2
UPPER_MULT = 8
LABEL = 'BARN 2026 published rule  clip=[2*OT, 8*OT]'


def report(out_path):
    """Print the published-rule summary for an evaluator out_file."""
    records = C.parse_out_file(out_path)
    summary = C.summarize(records, LOWER_MULT, UPPER_MULT)
    _print(summary)
    return 0


def _print(summary):
    print('=' * 64)
    print('Metric definition:')
    print('  ' + LABEL)
    print('-' * 64)
    print(f"  trials         : {summary['trials']}")
    print(f"  score          : {summary['score']:.4f}")
    print(f"  success_rate   : {summary['success_rate']:.3f}")
    print(f"  collision_rate : {summary['collision_rate']:.3f}")
    print(f"  timeout_rate   : {summary['timeout_rate']:.3f}")
    print('=' * 64)


def selftest():
    """Verify the published clip bound on the synthetic trial set."""
    records = C.synthetic_records()
    summary = C.summarize(records, LOWER_MULT, UPPER_MULT)
    # Fast success (AT=6, OT=5): published clips AT UP to the 2*OT=10 lower
    # bound -> 5/10 = 0.5 (the maximum per-trial score under the published rule).
    fast = C.trial_score(True, 6.0, 5.0, LOWER_MULT, UPPER_MULT)
    assert abs(fast - 0.5) < 1e-9, fast
    # A mid-speed success (AT=15) sits above the 2*OT lower bound, so it is
    # unclipped: 5/15 = 0.3333 (this is where the upstream 4*OT rule diverges).
    mid = C.trial_score(True, 15.0, 5.0, LOWER_MULT, UPPER_MULT)
    assert abs(mid - (5.0 / 15.0)) < 1e-9, mid
    # Failures score 0; 2 of 5 trials fail -> success_rate 0.6.
    assert abs(summary['success_rate'] - 0.6) < 1e-9, summary
    print('barn2026_metric selftest OK')
    _print(summary)
    return 0


def main(argv=None):
    """CLI entrypoint."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out_path', help='evaluator raw out_file to score')
    parser.add_argument('--selftest', action='store_true', help='run self-check and exit')
    args = parser.parse_args(argv)
    if args.selftest:
        return selftest()
    if not args.out_path:
        parser.error('provide --out_path or --selftest')
    return report(args.out_path)


if __name__ == '__main__':
    sys.exit(main())
