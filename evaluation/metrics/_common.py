"""Shared parsing and scoring helpers for the BARN metric reports.

Two report scripts import this module:
  * ``barn2026_metric.py``       -> published rule, clip = [2*OT, 8*OT]
  * ``upstream_compat_metric.py``-> upstream report_test.py, clip = [4*OT, 8*OT]

Keeping the scoring math in one place guarantees the two reports differ ONLY by
the lower clip multiplier (see docs/benchmark/metric_notes.md). Neither script
reads Gazebo/world files; they only consume trial results.
"""

import re

V_MAX = 2.0  # m/s, BARN maximum robot speed (used to derive OT from path length)

# ---------------------------------------------------------------------------
# Result-file parsing
# ---------------------------------------------------------------------------
# The BARN evaluator writes one line per trial to its out_file. The exact column
# layout depends on the evaluator revision, so VERIFY this against your
# evaluator's output the first time and adjust COLUMN_ORDER if needed. The
# default matches the common BARN layout:
#     world_idx  success  collided  timeout  actual_time  optimal_time
COLUMN_ORDER = ('world_idx', 'success', 'collided', 'timeout', 'actual_time', 'optimal_time')


def parse_out_file(path):
    """Parse the evaluator's raw out_file into a list of trial dicts.

    Blank lines and lines beginning with '#' are ignored. Each remaining line is
    split on whitespace and mapped onto COLUMN_ORDER. Extra columns are ignored;
    missing trailing columns are left unset.
    """
    records = []
    with open(path, 'r', encoding='utf-8') as handle:
        for raw in handle:
            line = raw.strip()
            if not line or line.startswith('#'):
                continue
            fields = re.split(r'[\s,]+', line)
            record = {}
            for name, value in zip(COLUMN_ORDER, fields):
                record[name] = value
            records.append(_normalize(record))
    return records


def _normalize(record):
    """Coerce a raw string record into typed values."""
    out = {}
    out['world_idx'] = int(float(record.get('world_idx', -1)))
    out['success'] = _as_bool(record.get('success', 0))
    out['collided'] = _as_bool(record.get('collided', 0))
    out['timeout'] = _as_bool(record.get('timeout', 0))
    out['actual_time'] = float(record.get('actual_time', 0.0) or 0.0)
    out['optimal_time'] = float(record.get('optimal_time', 0.0) or 0.0)
    return out


def _as_bool(value):
    """Interpret common truthy encodings ('1', 'true', 'True') as bool."""
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    return text in ('1', 'true', 't', 'yes', 'y')


# ---------------------------------------------------------------------------
# Scoring
# ---------------------------------------------------------------------------
def optimal_time(path_length):
    """Optimal traversal time = reference path length / maximum speed."""
    return path_length / V_MAX


def clip(value, lo, hi):
    """Clamp value into [lo, hi]."""
    return max(lo, min(hi, value))


def trial_score(success, actual_time, opt_time, lower_mult, upper_mult):
    """BARN per-trial score.

        s = success * OT / clip(AT, lower_mult*OT, upper_mult*OT)

    A failed trial (collision or no-goal) scores 0 regardless of time.
    """
    if not success or opt_time <= 0.0:
        return 0.0
    at = clip(actual_time, lower_mult * opt_time, upper_mult * opt_time)
    return opt_time / at


def summarize(records, lower_mult, upper_mult):
    """Aggregate trial records into a summary dict for one clip definition."""
    n = len(records)
    if n == 0:
        return {'trials': 0, 'score': 0.0, 'success_rate': 0.0,
                'collision_rate': 0.0, 'timeout_rate': 0.0}
    scores = []
    successes = collisions = timeouts = 0
    for r in records:
        successes += 1 if r['success'] else 0
        collisions += 1 if r['collided'] else 0
        timeouts += 1 if r['timeout'] else 0
        scores.append(trial_score(
            r['success'], r['actual_time'], r['optimal_time'], lower_mult, upper_mult))
    return {
        'trials': n,
        'score': sum(scores) / n,
        'success_rate': successes / n,
        'collision_rate': collisions / n,
        'timeout_rate': timeouts / n,
    }


def synthetic_records():
    """Deterministic trial set used by the --selftest self-checks.

    Includes a fast success (AT < 2*OT), a mid success, a slow success
    (2*OT < AT < 4*OT so the two clip rules diverge), a collision, and a
    timeout. OT = 5 s throughout (path_length 10 m at 2 m/s).
    """
    ot = 5.0
    return [
        {'world_idx': 0, 'success': True, 'collided': False, 'timeout': False,
         'actual_time': 6.0, 'optimal_time': ot},   # AT < 2*OT -> clipped up
        {'world_idx': 6, 'success': True, 'collided': False, 'timeout': False,
         'actual_time': 15.0, 'optimal_time': ot},  # 2*OT < AT < 4*OT -> differs
        {'world_idx': 12, 'success': True, 'collided': False, 'timeout': False,
         'actual_time': 30.0, 'optimal_time': ot},  # AT = 6*OT
        {'world_idx': 18, 'success': False, 'collided': True, 'timeout': False,
         'actual_time': 4.0, 'optimal_time': ot},   # collision -> 0
        {'world_idx': 24, 'success': False, 'collided': False, 'timeout': True,
         'actual_time': 100.0, 'optimal_time': ot},  # timeout -> 0
    ]
