#!/usr/bin/env python3
"""UPSTREAM-COMPATIBLE metric report.

Reproduces what the ROS 2 evaluator's own report_test.py computes, whose
source clips actual time as:

    np.clip(actual_time, optimal_time * 4, optimal_time * 8)

i.e. s = success * OT / clip(AT, 4*OT, 8*OT). This DISAGREES with the published
BARN 2026 rule (lower bound 2*OT). Keep this report only for evaluator-
compatibility debugging; use barn2026_metric.py for research numbers. This
script prints both numbers and their delta so the discrepancy is never silent.

Usage:
    python3 upstream_compat_metric.py --out_path results/.../raw_results.txt
    python3 upstream_compat_metric.py --selftest
"""

import argparse
import sys

import _common as C

LOWER_MULT = 4
UPPER_MULT = 8
LABEL = 'upstream ROS2 master report_test.py  clip=[4*OT, 8*OT]'


def report(out_path):
    """Print the upstream summary alongside the published-rule number."""
    records = C.parse_out_file(out_path)
    upstream = C.summarize(records, LOWER_MULT, UPPER_MULT)
    published = C.summarize(records, 2, UPPER_MULT)
    _print(upstream, published)
    return 0


def _print(upstream, published):
    delta = upstream['score'] - published['score']
    print('=' * 64)
    print('Metric definition:')
    print('  ' + LABEL)
    print('-' * 64)
    print(f"  trials              : {upstream['trials']}")
    print(f"  score (upstream 4OT): {upstream['score']:.4f}")
    print(f"  score (published 2OT): {published['score']:.4f}")
    print(f"  delta (upstream-pub): {delta:+.4f}")
    print(f"  success_rate        : {upstream['success_rate']:.3f}")
    print(f"  collision_rate      : {upstream['collision_rate']:.3f}")
    print(f"  timeout_rate        : {upstream['timeout_rate']:.3f}")
    print('=' * 64)


def selftest():
    """Verify the upstream clip bound and that it diverges from the published one."""
    # Fast success (AT=6, OT=5): upstream clips AT UP to 4*OT=20 -> 5/20 = 0.25,
    # while the published 2*OT rule clips to 10 -> 0.5. Same trial, different score.
    fast_upstream = C.trial_score(True, 6.0, 5.0, LOWER_MULT, UPPER_MULT)
    fast_published = C.trial_score(True, 6.0, 5.0, 2, UPPER_MULT)
    assert abs(fast_upstream - 0.25) < 1e-9, fast_upstream
    assert abs(fast_published - 0.5) < 1e-9, fast_published
    assert fast_upstream < fast_published, 'reports must diverge on a fast success'
    records = C.synthetic_records()
    _print(C.summarize(records, LOWER_MULT, UPPER_MULT), C.summarize(records, 2, UPPER_MULT))
    print('upstream_compat_metric selftest OK')
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
