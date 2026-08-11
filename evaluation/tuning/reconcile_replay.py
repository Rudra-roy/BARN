#!/usr/bin/env python3
"""INVALIDATED -- kept for the method, not the numbers. See the warning below.

Replay traces to test whether a map-vs-scan reconciliation rule could pick a
safe obstacle_margin online.

!! THE PREMISE WAS WRONG !!
This script compares the MPC's clearance_m (distance field AT THE CURRENT POSE)
against the shield's safety_clearance. But the shield's minimum_clearance is the
minimum over its entire SWEPT STOPPING ENVELOPE, not a value at the current pose
(swept_footprint_shield.cpp:44-55). Comparing a point value against a
forward-swept minimum measures geometry, not map-vs-scan disagreement, and it
manufactured a "map is optimistic in 85% of samples" result that does not hold.

Re-measured at the SAME pose, map distance field vs the lateral scan minimum
(min(left_m, right_m)) over 4453 samples:
    map median 0.500 m, scan median 0.557 m
    map minus scan: p10 -0.429, p50 -0.016, p90 +0.101
    map optimistic in only 42.6% of samples -- i.e. roughly balanced, and
    slightly PESSIMISTIC at the median.

So the map does NOT systematically over-report clearance. Any conclusion in this
file that depends on the optimism bias -- including the "% of run inside the veto
box" table -- is void.

The corrected picture points the other way and is consistent with the
max-pooled 10 cm planning grid + non-interpolated distance-field lookup: the
MPC is fed a clearance that is if anything too SMALL, which is why it freezes in
corridors that are actually drivable, and why lowering obstacle_margin appeared
to help (it was cancelling a phantom pad) while causing shield traps (it
overshot into space the scan-reading shield genuinely disputes).

THE PROBLEM IT TARGETS
----------------------
The MPC plans against the distance field (a smoothed occupancy map); the shield
vetoes against raw scan returns. Measured over 7,574 samples, the MPC's clearance
is MORE OPTIMISTIC than the shield's in 85% of samples, by more than 0.10 m in
43%. So the two authorities are not measuring the same quantity, and a margin
that looks safe to the MPC can put the robot inside the shield's veto box.

That is what sank obstacle_margin 0.10. On paper it demands 0.2559 + 0.10 =
0.3559 m of centre clearance, comfortably above the shield's 0.2359 m veto box.
In practice worlds 282 and 276 logged 12 and 8 SHIELD TRAPPED events, because the
0.3559 m was measured on the optimistic map.

THE RULE UNDER TEST
-------------------
Estimate the bias online from the two clearances the stack already publishes, and
require the margin to keep the SHIELD-EXPERIENCED clearance above the veto box:

    experienced = (half_extent + footprint_margin + margin) - bias
    require       experienced >= veto_box + buffer
    =>  margin   >= veto_box + buffer + bias - (half_extent + footprint_margin)

THE QUESTION THIS ANSWERS
-------------------------
At the moments the robot actually froze, what minimum margin would the rule have
demanded? If it demands MORE than 0.10, the rule would have refused the
relaxation that produced the traps -- correct, but it also means the freeze
cannot be fixed by relaxing the margin at all. If it permits 0.10 at freeze
moments while forbidding it elsewhere, the rule discriminates and is worth
building.

Usage:  reconcile_replay.py <trace.jsonl[.gz]> ...
"""

import glob
import gzip
import json
import statistics as st
import sys

HALF_WIDTH = 0.2159
FOOTPRINT_MARGIN = 0.04          # hardcoded in collision_checker.hpp
EMERGENCY_MARGIN = 0.02          # barn_safety veto box
VETO_BOX = HALF_WIDTH + EMERGENCY_MARGIN          # 0.2359 m, centre-equivalent
MPC_BASE = HALF_WIDTH + FOOTPRINT_MARGIN          # 0.2559 m before obstacle_margin
BUFFER = 0.05                    # headroom demanded above the veto box

# Shield reasons meaning the shield is not itself the thing stopping the robot.
SHIELD_PASSIVE = {"clear", "clear_no_returns", "braking_distance_clamp", "stationary"}
CONTROL_HEALTHY = {"solved", "solved inaccurate"}


def open_trace(path):
    return gzip.open(path, "rt") if str(path).endswith(".gz") else open(path)


def bias_of(row):
    """MPC map clearance minus shield scan clearance, both centre-equivalent."""
    mpc = row.get("clearance_m")
    shd = row.get("safety_clearance")
    if mpc is None or shd is None:
        return None
    centre_equiv = shd + VETO_BOX
    if not (0.0 < centre_equiv < 50.0) or not (0.0 < mpc < 50.0):
        return None                      # no returns in range, or a sentinel
    return mpc - centre_equiv


def required_margin(bias):
    """Smallest obstacle_margin that keeps the SHIELD-experienced clearance above
    the veto box plus buffer, given the map is optimistic by `bias`."""
    return VETO_BOX + BUFFER + bias - MPC_BASE


def main(paths):
    all_bias, freeze_bias = [], []
    for path in paths:
        rows = [json.loads(l) for l in open_trace(path) if l.strip()]
        # Mark freeze samples the same way the detector does: stopped, nothing
        # blocking, still far from the goal.
        for r in rows:
            b = bias_of(r)
            if b is None or (r.get("goal_dist") or 0) <= 1.0:
                continue
            all_bias.append(b)
            frozen = (
                (r.get("desired_v") is not None and abs(r["desired_v"]) < 0.05) and
                r.get("safety_reason") in SHIELD_PASSIVE and
                r.get("control_status") in CONTROL_HEALTHY
            )
            if frozen:
                freeze_bias.append(b)

    def report(name, xs):
        if not xs:
            print(f"  {name}: no samples")
            return None
        xs = sorted(xs)
        med = st.median(xs)
        p90 = xs[int(0.9 * len(xs)) - 1]
        print(f"  {name:<22} n={len(xs):>6}  bias median {med:+.3f}  p90 {p90:+.3f}"
              f"   -> required margin {required_margin(med):.3f} (p90 {required_margin(p90):.3f})")
        return med

    print("map-optimism bias and the margin it implies\n")
    med_all = report("all navigating", all_bias)
    med_frz = report("while FROZEN", freeze_bias)

    print(f"\n  veto box {VETO_BOX:.4f} m | MPC base {MPC_BASE:.4f} m | buffer {BUFFER:.2f} m")
    if med_frz is not None and med_all is not None:
        need_frz = required_margin(med_frz)
        print(f"\nVERDICT")
        print(f"  at freeze moments the rule would demand margin >= {need_frz:.3f}")
        if need_frz > 0.10:
            print(f"  -> it REFUSES the 0.10 that produced 12 and 8 shield traps. Correct,")
            print(f"     but it also means the freeze is NOT fixable by relaxing this margin:")
            print(f"     any margin low enough to clear the freeze is low enough to trap.")
        else:
            print(f"  -> it PERMITS 0.10 while frozen; the rule discriminates and is worth building.")
        print(f"  nominal 0.20 gives experienced clearance "
              f"{MPC_BASE + 0.20 - med_frz:.3f} m vs veto box {VETO_BOX:.3f} m "
              f"(headroom {MPC_BASE + 0.20 - med_frz - VETO_BOX:+.3f} m)")
        print(f"  relaxed 0.10 gives experienced clearance "
              f"{MPC_BASE + 0.10 - med_frz:.3f} m vs veto box {VETO_BOX:.3f} m "
              f"(headroom {MPC_BASE + 0.10 - med_frz - VETO_BOX:+.3f} m)")


if __name__ == "__main__":
    args = sys.argv[1:]
    paths = [p for a in args for p in glob.glob(a)] or glob.glob(
        "world_*/*instrumented*/traces/*.jsonl.gz")
    if not paths:
        print("no traces found", file=sys.stderr)
        sys.exit(1)
    main(paths)
