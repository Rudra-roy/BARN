#!/usr/bin/env python3
"""Detect the MPC-freeze state from navigation telemetry, offline.

THE STATE BEING DETECTED
------------------------
The MPC's obstacle row requires `distance_field >= obstacle_margin` at footprint
boundary points already inflated by footprint.margin, i.e. a demanded centre
clearance of 0.2159 + 0.04 + 0.20 = 0.4559 m. BARN's reference paths thread gaps
at 0.225 m. Where the corridor is narrower than the demand, the constraint is
violated AT THE CURRENT POSE, no control action satisfies it, every candidate
pays the same heavy slack, and the cheapest one is v = 0. The robot stops in a
corridor it could drive through and stays stopped until something else changes
the geometry.

Traced on world 66: 6.8 s of a 21 s trial stationary at 0.40 m clearance with
0.69 m free ahead, the shield reporting `clear` at scale 1.0 and the MPC
reporting `solved`. That conjunction is the signature -- nothing is blocking the
robot and nothing reports an error, yet it does not move.

WHY A DETECTOR AND NOT A GLOBAL PARAMETER
-----------------------------------------
Lowering obstacle_margin to 0.10 fixes world 66 (+0.064) but costs world 288
(-0.016), because with less margin the planner routes nearer walls on worlds
that were never clearance-starved. A detector pays nothing on worlds that never
freeze, which is most of them: the minor tier logged 14 recovery episodes across
150 trials.

ONLINE-IMPLEMENTABLE BY CONSTRUCTION
------------------------------------
`FreezeDetector.update()` is a streaming state machine: one sample in, a verdict
out, no lookahead and no whole-trace statistics. What runs here offline is the
same code that would run in the node, which is the point of testing it this way
-- a detector validated with hindsight is not a detector.

Usage:
    freeze_detector.py <trace.jsonl> [...]     # per-trace episodes
    freeze_detector.py --sweep <trace.jsonl> [...]   # dwell-time sensitivity
"""

import argparse
import gzip
import json
import math
import sys
from collections import deque


def open_trace(path):
    """Traces are committed gzipped: 11 MB of JSONL compresses to ~1 MB, and they
    are the evidence the detector's numbers rest on, so they are worth keeping in
    the tree rather than regenerating."""
    if str(path).endswith('.gz'):
        return gzip.open(path, 'rt')
    return open(path)

# Shield reasons that mean "the shield is NOT what is stopping the robot".
# emergency_veto / emergency_trapped are excluded on purpose: those are the
# shield doing its job, a different failure with its own escape logic.
SHIELD_PASSIVE = {"clear", "clear_no_returns", "braking_distance_clamp", "stationary"}

# Control statuses where the MPC believes it produced a valid answer. If the
# planner is in recovery or waiting for inputs, the stop is explained and this
# detector must stay quiet -- that subsystem is already acting.
CONTROL_HEALTHY = {"solved", "solved inaccurate"}


class FreezeDetector:
    """Streaming detector for 'commanded to stop by nothing in particular'.

    Fires when, continuously for `dwell_s`:
      * the commanded speed is ~0,
      * the robot has not actually moved,
      * the shield is not vetoing,
      * the MPC reports a solve,
      * and there is still a goal to drive to.
    """

    def __init__(self, dwell_s=1.0, v_eps=0.05, move_eps_m=0.03,
                 progress_window_s=1.0, goal_tol_m=1.0):
        self.dwell_s = dwell_s
        self.v_eps = v_eps
        self.move_eps_m = move_eps_m
        self.progress_window_s = progress_window_s
        self.goal_tol_m = goal_tol_m
        self._poses = deque()          # (t, x, y) inside the progress window
        self._candidate_since = None
        self._latched = False

    def update(self, s):
        """One telemetry sample -> (is_frozen_now, just_fired)."""
        t = s.get("t")
        if t is None:
            return False, False

        x, y = s.get("pose_x"), s.get("pose_y")
        if x is not None and y is not None:
            self._poses.append((t, x, y))
            while self._poses and t - self._poses[0][0] > self.progress_window_s:
                self._poses.popleft()

        # Displacement across the window. Pose publishes slower than the
        # sampler, so per-sample deltas are mostly zero and useless; a window
        # is the only honest way to ask "has it moved".
        moved = 0.0
        if len(self._poses) >= 2:
            _, x0, y0 = self._poses[0]
            moved = math.hypot(x - x0, y - y0)
        window_full = (len(self._poses) >= 2 and
                       self._poses[-1][0] - self._poses[0][0] >= self.progress_window_s * 0.8)

        desired_v = s.get("desired_v")
        goal_dist = s.get("goal_dist")
        conditions = (
            desired_v is not None and abs(desired_v) < self.v_eps,
            goal_dist is not None and goal_dist > self.goal_tol_m,
            s.get("safety_reason") in SHIELD_PASSIVE,
            s.get("control_status") in CONTROL_HEALTHY,
            window_full and moved < self.move_eps_m,
        )
        frozen_now = all(conditions)

        just_fired = False
        if frozen_now:
            if self._candidate_since is None:
                self._candidate_since = t
            elif not self._latched and (t - self._candidate_since) >= self.dwell_s:
                self._latched = True
                just_fired = True
        else:
            self._candidate_since = None
            self._latched = False
        return frozen_now, just_fired


def true_stalls(rows, min_s=1.5, move_eps=0.05):
    """Ground truth: contiguous spans where the robot did not move, any cause.

    Deliberately cause-agnostic and computed with hindsight -- it is the thing
    the detector is scored against, so it must not share the detector's logic.
    """
    pts = [r for r in rows if r.get("pose_x") is not None and (r.get("goal_dist") or 0) > 1.0]
    spans, start = [], None
    for i in range(1, len(pts)):
        d = math.hypot(pts[i]["pose_x"] - pts[i - 1]["pose_x"],
                       pts[i]["pose_y"] - pts[i - 1]["pose_y"])
        window = [p for p in pts[max(0, i - 20):i + 1]]
        disp = math.hypot(window[-1]["pose_x"] - window[0]["pose_x"],
                          window[-1]["pose_y"] - window[0]["pose_y"]) if len(window) > 1 else 0.0
        stalled = disp < move_eps
        if stalled and start is None:
            start = pts[i]["t"]
        elif not stalled and start is not None:
            if pts[i]["t"] - start >= min_s:
                spans.append((start, pts[i]["t"]))
            start = None
    if start is not None and pts and pts[-1]["t"] - start >= min_s:
        spans.append((start, pts[-1]["t"]))
    return spans


def analyse(path, dwell_s):
    rows = [json.loads(l) for l in open_trace(path) if l.strip()]
    det = FreezeDetector(dwell_s=dwell_s)
    episodes, cur = [], None
    for s in rows:
        frozen, fired = det.update(s)
        if fired:
            cur = [det._candidate_since, s["t"]]
        elif cur is not None:
            if frozen:
                cur[1] = s["t"]
            else:
                episodes.append(tuple(cur))
                cur = None
    if cur is not None:
        episodes.append(tuple(cur))
    return rows, episodes, true_stalls(rows)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("traces", nargs="+")
    ap.add_argument("--dwell", type=float, default=1.0)
    ap.add_argument("--sweep", action="store_true")
    args = ap.parse_args()

    dwells = [0.5, 0.75, 1.0, 1.5, 2.0] if args.sweep else [args.dwell]
    for dwell in dwells:
        if args.sweep:
            print(f"\n===== dwell {dwell:.2f} s =====")
        tot_stall = tot_caught = tot_fp = 0.0
        for path in args.traces:
            rows, eps, stalls = analyse(path, dwell)
            name = "/".join(path.split("/")[-4:-2]) + "/" + path.split("/")[-1]
            stall_s = sum(b - a for a, b in stalls)
            det_s = sum(b - a for a, b in eps)
            # Detected time overlapping a real stall vs. not.
            overlap = 0.0
            for a, b in eps:
                for c, d in stalls:
                    overlap += max(0.0, min(b, d) - max(a, c))
            fp = det_s - overlap
            tot_stall += stall_s
            tot_caught += overlap
            tot_fp += fp
            lead = ""
            if eps and stalls:
                first_lead = min((a - c for a, b in eps for c, d in stalls
                                  if a >= c and a <= d + 1.0), default=None)
                if first_lead is not None:
                    lead = f"  first fire {first_lead:.2f}s into the stall"
            print(f"  {name:<34} stalled {stall_s:5.1f}s | detected {det_s:5.1f}s "
                  f"({len(eps)} ep) | true {overlap:5.1f}s | false {fp:4.1f}s{lead}")
        rate = 100 * tot_caught / tot_stall if tot_stall else 0.0
        print(f"  {'TOTAL':<34} stalled {tot_stall:5.1f}s | caught {tot_caught:5.1f}s "
              f"({rate:.0f}%) | false-positive {tot_fp:.1f}s")


if __name__ == "__main__":
    sys.exit(main())
