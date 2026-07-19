#!/usr/bin/env python3
# Copyright 2026 barn-2027-prep contributors. MIT License.
"""Offline analysis of a recovery trace JSONL (pure Python, NO ROS needed).

Reads a ``recovery_trace.jsonl`` produced by ``tools/record_recovery_trace.py``,
reconstructs each recovery episode, and reports the failure modes that make
recovery misbehave: over-rotation, veto-escape overrun, flapping (immediate
re-trigger), permanent lockout (kFailed), and no-progress.

Usage:
    python3 tools/analyze_recovery_trace.py recovery_trace.jsonl
    python3 tools/analyze_recovery_trace.py run12.jsonl --episodes   # per-episode detail
"""

import argparse
import json
import math
import sys

# Mirror barn_classical::RecoveryState (recovery.hpp). Keep in sync.
STATE_NAMES = {
    0: 'Inactive',
    1: 'ReverseToClearance',
    2: 'RotateToGap',
    3: 'RequestReplan',
    4: 'RequestReplanClearance',
    5: 'Failed',
}
# States that actively command motion (reverse or rotate) — used by the
# veto-escape-overrun detector.
MOTION_STATES = {1, 2}
ROTATE_STATES = MOTION_STATES
FAILED_STATE = 5


def sname(state):
    if state is None:
        return '?'
    return STATE_NAMES.get(int(state), f'State{int(state)}')


def load(path):
    rows = []
    with open(path) as handle:
        for line_no, line in enumerate(handle, 1):
            line = line.strip()
            if not line:
                continue
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError as exc:
                print(f'warning: skipping malformed line {line_no}: {exc}',
                      file=sys.stderr)
    return rows


def is_active(state):
    return state is not None and int(state) != 0


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def build_episodes(rows):
    """Group contiguous active-recovery samples into episodes."""
    episodes = []
    current = None
    for row in rows:
        state = row.get('recovery_state')
        if is_active(state):
            if current is None:
                current = []
            current.append(row)
        else:
            if current:
                episodes.append(current)
                current = None
    if current:
        episodes.append(current)
    return episodes


def summarize_episode(ep, prev_end_row):
    first, last = ep[0], ep[-1]
    t0, t1 = first.get('t', 0.0), last.get('t', 0.0)
    duration = t1 - t0

    # State sequence (collapse consecutive duplicates).
    seq = []
    for row in ep:
        s = int(row['recovery_state'])
        if not seq or seq[-1][0] != s:
            seq.append((s, row.get('t')))

    # Yaw sweep vs net rotation — over-rotation detector.
    yaws = [r.get('pose_yaw') for r in ep if r.get('pose_yaw') is not None]
    swept = 0.0
    net_yaw = 0.0
    if len(yaws) >= 2:
        for a, b in zip(yaws, yaws[1:]):
            swept += abs(wrap(b - a))
        net_yaw = abs(wrap(yaws[-1] - yaws[0]))

    # Net translation.
    disp = None
    if all(k in first for k in ('pose_x', 'pose_y')) and \
            all(k in last for k in ('pose_x', 'pose_y')):
        disp = math.hypot(last['pose_x'] - first['pose_x'],
                          last['pose_y'] - first['pose_y'])

    # Goal progress across the episode (negative = got closer).
    goal_delta = None
    if first.get('goal_dist') is not None and last.get('goal_dist') is not None:
        goal_delta = last['goal_dist'] - first['goal_dist']

    # Veto behaviour: what fraction of the episode had the shield vetoing, and
    # when (if ever) it first cleared after being active.
    veto_vals = [(r.get('t'), r.get('veto')) for r in ep]
    veto_true = sum(1 for _, v in veto_vals if v is True)
    veto_frac = veto_true / len(ep) if ep else 0.0
    veto_clear_t = None
    seen_true = False
    for t, v in veto_vals:
        if v is True:
            seen_true = True
        elif v is False and seen_true and veto_clear_t is None:
            veto_clear_t = t

    triggers = {r.get('control_status') for r in ep if r.get('control_status')}
    ended_failed = any(int(r['recovery_state']) == FAILED_STATE for r in ep)

    # Gap since previous episode ended (flapping detector).
    gap = None
    moved_between = None
    if prev_end_row is not None:
        gap = t0 - prev_end_row.get('t', t0)
        if all(k in prev_end_row for k in ('pose_x', 'pose_y')) and \
                all(k in first for k in ('pose_x', 'pose_y')):
            moved_between = math.hypot(first['pose_x'] - prev_end_row['pose_x'],
                                       first['pose_y'] - prev_end_row['pose_y'])

    return {
        't0': t0, 't1': t1, 'duration': duration,
        'seq': seq, 'swept': swept, 'net_yaw': net_yaw, 'disp': disp,
        'goal_delta': goal_delta, 'veto_frac': veto_frac,
        'veto_clear_t': veto_clear_t, 'triggers': triggers,
        'ended_failed': ended_failed, 'gap': gap, 'moved_between': moved_between,
        'attempts': max((r.get('recovery_attempts') or 0) for r in ep),
    }


def find_findings(summaries, rows):
    findings = []  # (severity, code, message)

    run_span = (rows[-1].get('t', 0.0) - rows[0].get('t', 0.0)) if rows else 0.0
    total_recovery = sum(s['duration'] for s in summaries)
    if run_span > 0 and total_recovery / run_span > 0.35:
        findings.append((
            'WARN', 'HIGH_RECOVERY_FRACTION',
            f'{total_recovery:.1f}s of {run_span:.1f}s '
            f'({100*total_recovery/run_span:.0f}%) spent in recovery — planner is '
            'losing the corridor, not just occasionally nudging.'))

    for i, s in enumerate(summaries):
        tag = f'ep#{i+1} @t={s["t0"]:.1f}s'

        # Over-rotation: swept far more than the net heading change needed.
        if s['swept'] > 1.15 * 2 * math.pi:
            findings.append((
                'HIGH', 'OVER_ROTATION',
                f'{tag}: swept {math.degrees(s["swept"]):.0f}° total (net only '
                f'{math.degrees(s["net_yaw"]):.0f}°) — spinning in place / '
                'over-rotating past a good heading.'))
        elif s['swept'] > 3.0 and s['net_yaw'] < 0.4:
            findings.append((
                'MED', 'ROTATE_NO_NET',
                f'{tag}: {math.degrees(s["swept"]):.0f}° swept but ~0 net turn — '
                'oscillating left/right without committing.'))

        # Veto-escape overrun: shield cleared mid-episode but recovery kept
        # running (rotating) well past that point. This is the exact bug the
        # veto_active early-exit is meant to prevent.
        if s['veto_clear_t'] is not None:
            overrun = s['t1'] - s['veto_clear_t']
            still_rotating = any(st in ROTATE_STATES for st, _ in s['seq'])
            if overrun > 0.6 and still_rotating:
                findings.append((
                    'HIGH', 'VETO_ESCAPE_OVERRUN',
                    f'{tag}: safety veto cleared at t={s["veto_clear_t"]:.1f}s but '
                    f'recovery ran {overrun:.1f}s longer — veto escape not '
                    'terminating when the shield releases.'))

        # Permanent lockout.
        if s['ended_failed']:
            findings.append((
                'HIGH', 'LOCKOUT_FAILED',
                f'{tag}: reached kFailed (attempts={s["attempts"]:.0f}) — robot '
                'stops commanding motion; verify it self-clears and retries.'))

        # No progress despite a long episode.
        if s['duration'] > 3.0 and (s['disp'] is None or s['disp'] < 0.15):
            disp_txt = '?m' if s['disp'] is None else f'{s["disp"]:.2f}m'
            findings.append((
                'MED', 'NO_PROGRESS',
                f'{tag}: {s["duration"]:.1f}s episode moved {disp_txt} — recovery '
                'not opening space.'))

        # Flapping: re-triggered almost immediately with little movement.
        if s['gap'] is not None and s['gap'] < 0.4 and \
                (s['moved_between'] is None or s['moved_between'] < 0.1):
            moved_txt = '?m' if s['moved_between'] is None else f'{s["moved_between"]:.2f}m'
            findings.append((
                'HIGH', 'FLAPPING',
                f'{tag}: re-triggered {s["gap"]:.2f}s after the previous episode '
                f'(moved {moved_txt}) — recovery⇄MPC flapping.'))

    # Cluster flapping: many episodes in a short window.
    if len(summaries) >= 3:
        for i in range(len(summaries) - 2):
            window = summaries[i + 2]['t1'] - summaries[i]['t0']
            if window < 12.0:
                findings.append((
                    'MED', 'EPISODE_CLUSTER',
                    f'3+ episodes within {window:.1f}s '
                    f'(around t={summaries[i]["t0"]:.1f}s) — persistent trigger '
                    'source, not a one-off.'))
                break

    return findings


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('trace', help='recovery_trace.jsonl path')
    parser.add_argument('--episodes', action='store_true',
                        help='print per-episode detail')
    args = parser.parse_args()

    rows = load(args.trace)
    if not rows:
        print('no samples in trace', file=sys.stderr)
        return 1

    run_span = rows[-1].get('t', 0.0) - rows[0].get('t', 0.0)
    episodes = build_episodes(rows)

    summaries = []
    prev_end = None
    for ep in episodes:
        summaries.append(summarize_episode(ep, prev_end))
        prev_end = ep[-1]

    print('=' * 72)
    print(f'Recovery trace: {args.trace}')
    print(f'  samples          : {len(rows)}  ({run_span:.1f}s span)')
    print(f'  recovery episodes: {len(episodes)}')
    print(f'  time in recovery : {sum(s["duration"] for s in summaries):.1f}s')
    goal_first = next((r.get('goal_dist') for r in rows if r.get('goal_dist') is not None), None)
    goal_last = next((r.get('goal_dist') for r in reversed(rows) if r.get('goal_dist') is not None), None)
    if goal_first is not None and goal_last is not None:
        print(f'  goal distance    : {goal_first:.2f}m -> {goal_last:.2f}m')
    print('=' * 72)

    if args.episodes:
        for i, s in enumerate(summaries):
            path = ' -> '.join(sname(st) for st, _ in s['seq'])
            print(f'\nEpisode {i+1}: t={s["t0"]:.1f}-{s["t1"]:.1f}s '
                  f'({s["duration"]:.1f}s)  attempts~{s["attempts"]:.0f}')
            print(f'  states   : {path}')
            print(f'  trigger  : {", ".join(sorted(s["triggers"])) or "?"}')
            print(f'  swept    : {math.degrees(s["swept"]):.0f}° '
                  f'(net {math.degrees(s["net_yaw"]):.0f}°)')
            if s['disp'] is not None:
                print(f'  moved    : {s["disp"]:.2f}m')
            if s['goal_delta'] is not None:
                print(f'  goalΔ    : {s["goal_delta"]:+.2f}m')
            print(f'  veto     : {100*s["veto_frac"]:.0f}% of episode'
                  + (f', cleared @t={s["veto_clear_t"]:.1f}s' if s['veto_clear_t'] else ''))

    findings = find_findings(summaries, rows)
    print('\n' + '-' * 72)
    if not findings:
        print('FINDINGS: none — recovery behaved within expected bounds.')
    else:
        order = {'HIGH': 0, 'WARN': 1, 'MED': 2}
        findings.sort(key=lambda f: order.get(f[0], 9))
        print(f'FINDINGS ({len(findings)}):')
        for sev, code, msg in findings:
            print(f'  [{sev:4}] {code}: {msg}')
    print('-' * 72)
    return 0


if __name__ == '__main__':
    sys.exit(main())
