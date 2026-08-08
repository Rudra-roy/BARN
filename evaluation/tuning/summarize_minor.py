#!/usr/bin/env python3
"""Compare the minor-world sweep against the same worlds in the 430-trial campaign.

Both sides are scored with the evaluator's own published rule,
s = success * OT / clip(AT, 2*OT, 8*OT), using OT derived from BARN's reference
paths -- so the two columns differ only in the config and the run conditions,
not in how they are measured.
"""

import collections
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..'))
EVAL = os.path.join(REPO, 'ros2_ws/src/The-Barn-Challenge-Ros2')
PATH_DIR = os.path.join(EVAL, 'jackal_helper/worlds/BARN/path_files')
CAMPAIGN = os.path.join(EVAL, 'res/out.txt')

WORLDS = [78, 114, 120, 126, 150, 162, 168, 174, 180, 186, 198, 216, 222, 234, 288]
INIT, GOAL, RADIUS = (-2.25, 3.0), (-2.0, 13.0), 0.075


def optimal_time(world):
    path = np.load(os.path.join(PATH_DIR, f'path_{world}.npy'))
    pts = [INIT]
    for x, y in path:
        pts.append((x * (RADIUS * 2) - RADIUS - 30 * RADIUS * 2,
                    y * (RADIUS * 2) + RADIUS + 5))
    pts.append(GOAL)
    length = sum(float(np.hypot(b[0] - a[0], b[1] - a[1]))
                 for a, b in zip(pts[:-1], pts[1:]))
    return length / 2.0


def read(path):
    rows = []
    if os.path.exists(path):
        for line in open(path):
            f = line.split()
            if len(f) == 6:
                rows.append((int(f[0]), int(f[1]), int(f[2]), int(f[3]),
                             float(f[4]), float(f[5])))
    return rows


def stats(rows, ot):
    if not rows:
        return None
    ats = [r[4] for r in rows if r[1] == 1]
    return dict(
        n=len(rows),
        succ=sum(r[1] for r in rows),
        coll=sum(r[2] for r in rows),
        tout=sum(r[3] for r in rows),
        med=float(np.median(ats)) if ats else float('nan'),
        score=float(np.mean([r[1] * ot / np.clip(r[4], 2 * ot, 8 * ot) for r in rows])),
    )


def main():
    tag = sys.argv[1] if len(sys.argv) > 1 else 'minor_10'

    # The campaign is the FIRST 10 rows per world. res/out.txt is append-only and
    # the evaluator writes there whenever a trial runs without an explicit
    # out_file, so ad-hoc runs land in the same file as the scored campaign --
    # it has picked up extra rows on worlds 0, 114 and 288 since the campaign
    # finished. Taking the first 10 per world recovers the campaign exactly and
    # keeps the baseline column honest.
    campaign = collections.defaultdict(list)
    for r in read(CAMPAIGN):
        if len(campaign[r[0]]) < 10:
            campaign[r[0]].append(r)

    print(f'=== Tier-2 "minor" worlds: campaign vs tuned (tag "{tag}") ===\n')
    head = (f'{"world":>5} | {"campaign":>22} | {"tuned (headless)":>22} | {"delta":>7}')
    print(head)
    print(f'{"":>5} | {"succ  medAT   score":>22} | {"succ  medAT   score":>22} |')
    print('-' * len(head))

    tot_c, tot_t = [], []
    for w in WORLDS:
        ot = optimal_time(w)
        c = stats(campaign.get(w, []), ot)
        t = stats(read(os.path.join(HERE, f'world_{w}', tag, 'raw_results.txt')), ot)
        if t is None:
            print(f'{w:>5} | {"(missing)":>22} | {"BATCH MISSING":>22} |')
            continue
        tot_c.append(c)
        tot_t.append(t)
        cs = f'{c["succ"]}/{c["n"]}  {c["med"]:5.1f}  {c["score"]:.4f}' if c else '(none)'
        ts = f'{t["succ"]}/{t["n"]}  {t["med"]:5.1f}  {t["score"]:.4f}'
        d = f'{t["score"] - c["score"]:+.4f}' if c else '    n/a'
        flag = '  <-- COLLISION' if t['coll'] else ''
        print(f'{w:>5} | {cs:>22} | {ts:>22} | {d:>7}{flag}')

    print('-' * len(head))
    if tot_t:
        def agg(rows, key):
            return sum(r[key] for r in rows)
        cs = agg(tot_c, 'succ') / max(1, agg(tot_c, 'n'))
        ts = agg(tot_t, 'succ') / max(1, agg(tot_t, 'n'))
        cm = float(np.mean([r['score'] for r in tot_c]))
        tm = float(np.mean([r['score'] for r in tot_t]))
        print(f'{"MEAN":>5} | {100 * cs:20.0f}%  {cm:.4f} | {100 * ts:20.0f}%  {tm:.4f} '
              f'| {tm - cm:+.4f}')
        print(f'\ncampaign: {agg(tot_c, "succ")}/{agg(tot_c, "n")} success, '
              f'{agg(tot_c, "coll")} collisions, {agg(tot_c, "tout")} timeouts')
        print(f'tuned:    {agg(tot_t, "succ")}/{agg(tot_t, "n")} success, '
              f'{agg(tot_t, "coll")} collisions, {agg(tot_t, "tout")} timeouts')
        print('\nNote: headless vs the campaign\'s rviz:=true is a confound on AT '
              '(not uniform across\nworlds), so treat the success/collision columns as the '
              'solid comparison. See RESULTS.md.')


if __name__ == '__main__':
    main()
