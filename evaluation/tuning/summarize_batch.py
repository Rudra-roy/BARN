#!/usr/bin/env python3
"""Summarize one tuning batch: score, timing, and why the slow trials were slow.

The score alone cannot direct a tuning decision -- two configs can share a mean
and differ entirely in whether the robot drove smoothly or stalled and
recovered. So this also counts the recovery episodes in each trial's log, which
is the quantity the batch is actually trying to move.
"""

import os
import re
import sys

import numpy as np

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
PATH_DIR = os.path.join(
    REPO, 'ros2_ws/src/The-Barn-Challenge-Ros2/jackal_helper/worlds/BARN/path_files')
INIT = (-2.25, 3.0)
GOAL = (-2.0, 13.0)
RADIUS = 0.075

# Log lines that mark the controller giving up on forward progress. Matched
# case-insensitively against the whole launch log of one trial.
RECOVERY_PATTERNS = [
    ('no_progress', r'no progress|no_progress'),
    ('recovery', r'recovery|recovering'),
    ('reverse', r'revers'),
    ('replan_fail', r'no path|plan failed|planning failed|unreachable'),
    ('mpc_deadline', r'deadline|solver (failed|timeout)'),
]


def optimal_time(world_idx):
    """OT for a world, using the evaluator's own reference path and conversion."""
    path = np.load(os.path.join(PATH_DIR, f'path_{world_idx}.npy'))
    pts = [INIT]
    for x, y in path:
        pts.append((x * (RADIUS * 2) - RADIUS - 30 * RADIUS * 2,
                    y * (RADIUS * 2) + RADIUS + 5))
    pts.append(GOAL)
    length = sum(float(np.hypot(b[0] - a[0], b[1] - a[1]))
                 for a, b in zip(pts[:-1], pts[1:]))
    return length / 2.0


def read_results(outdir):
    """Parse the evaluator's result lines for this batch."""
    path = os.path.join(outdir, 'raw_results.txt')
    rows = []
    if os.path.exists(path):
        for line in open(path):
            f = line.split()
            if len(f) == 6:
                rows.append((int(f[0]), int(f[1]), int(f[2]), int(f[3]),
                             float(f[4]), float(f[5])))
    return rows


def log_events(outdir):
    """Count recovery-ish log lines per trial, in trial order."""
    logdir = os.path.join(outdir, 'logs')
    if not os.path.isdir(logdir):
        return []
    out = []
    names = sorted(os.listdir(logdir),
                   key=lambda n: int(re.search(r'(\d+)', n).group(1)))
    for name in names:
        text = open(os.path.join(logdir, name), errors='replace').read().lower()
        counts = {k: len(re.findall(p, text)) for k, p in RECOVERY_PATTERNS}
        out.append((name, counts))
    return out


def main():
    outdir = os.path.abspath(sys.argv[1])
    rows = read_results(outdir)
    tag = os.path.basename(outdir)
    world = os.path.basename(os.path.dirname(outdir)).replace('world_', '')

    print(f'=== tuning batch: world {world}, tag "{tag}" ===')
    if not rows:
        print('NO RESULTS -- every trial failed to produce a result line.')
        for name, counts in log_events(outdir):
            print(f'  {name}: {counts}')
        return

    ot = optimal_time(int(world))
    n = len(rows)
    succ = sum(r[1] for r in rows)
    coll = sum(r[2] for r in rows)
    tout = sum(r[3] for r in rows)
    ats = [r[4] for r in rows if r[1] == 1]
    scores = [r[5] for r in rows]

    print(f'trials {n}   success {succ} ({100 * succ / n:.0f}%)   '
          f'collision {coll}   timeout {tout}')
    print(f'OT {ot:.2f} s   2*OT clip {2 * ot:.2f} s')
    if ats:
        print(f'AT  min {min(ats):.1f}  median {np.median(ats):.1f}  '
              f'max {max(ats):.1f}  sd {np.std(ats):.1f}')
        print(f'AT/OT median {np.median(ats) / ot:.2f}   '
              f'trials above the 2*OT clip: {sum(a > 2 * ot for a in ats)}/{len(ats)}')
    print(f'MEAN SCORE {np.mean(scores):.4f}   '
          f'(ceiling for this success rate: {0.5 * succ / n:.4f})')
    print('per-trial: ' + ' '.join(
        f'{r[4]:.0f}' if r[1] else ('C' if r[2] else 'T') for r in rows))

    events = log_events(outdir)
    if events:
        print('\nlog events per trial (recovery is the quantity being tuned):')
        keys = [k for k, _ in RECOVERY_PATTERNS]
        print('  trial       ' + ' '.join(f'{k:>13}' for k in keys))
        for name, counts in events:
            print(f'  {name:<11} ' + ' '.join(f'{counts[k]:>13}' for k in keys))


if __name__ == '__main__':
    main()
