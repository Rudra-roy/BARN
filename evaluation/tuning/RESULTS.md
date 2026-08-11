# Classical MPC parameter tuning — session results

Single-world tuning runs against `barn_bringup/config/classical_mpc.yaml`.
Harness: `run_tuning_batch.sh <world> <trials> <tag>`, driven through
`in_box.sh` inside the `barn-jazzy` box, headless. Every batch snapshots the
exact YAML that produced it and keeps each trial's full launch log.

Pristine pre-tuning config: `baseline_config/` (verified byte-identical to
git HEAD at the start of the session).

---

## Accepted change (1)

### `safety_node.emergency_margin: 0.05 -> 0.02`

This fixes a **correctness defect**, not a tuning preference.

`SweptFootprintShield::safe_at_scale` hard-vetoes a command when an obstacle
falls inside a box of `half_extent + emergency_margin`, and the scale search
scales `v` and `w` together. As the scale approaches zero the stopping time
collapses to `shield_latency_s` and the swept envelope shrinks to the box at the
robot's *current* pose. So a swept intersection can always be scaled away — but
an obstacle already inside the pose-0 box makes **every** scale unsafe,
including an in-place rotation. The robot is then frozen until the trial times
out.

At `0.05` that box has half-width `0.215 + 0.05 = 0.265 m`, while the tightest
reference-path clearance in the scored worlds is `0.225 m`: the intended route
passes through gaps the shield treats as already-collided. `safety_node`
discards returns inside `half + 1 cm` as self-hits, so `0.05` leaves a 4 cm
shell in which the robot is stuck; `0.02` leaves 1 cm.

Observed directly on world 114: `"Safety shield has vetoed 6 consecutive
commands"` -> `[Recovery] Triggered due to: safety_veto` -> silence to 100 s.
After the change the same world's recovery trigger becomes `no_progress`, which
the replan resolves.

Anticipatory clearance is unaffected — it comes from sweeping the box along the
braking envelope, not from this term.

| world | campaign (10 trials, RViz on) | tuned (12 trials, headless) |
|---|---|---|
| 114 | 0.3589 — 8/10, 2 timeouts | **0.4843 — 12/12** |
| 288 | 0.3547 — 8/10, 2 timeouts | **0.4469 — 12/12** |
| 216 | 0.2983 — 9/10, 1 timeout  | **0.3154 — 11/12** |

**80 tuning trials, 0 collisions.**

---

## Rejected changes (3)

All measured on world 288 or 216, 12 trials per config, against the accepted
config above. Recorded here and inline in the YAML so they are not retried blind.

| change | result | why it failed |
|---|---|---|
| `obstacle_margin 0.20 -> 0.12` | 0.4469 -> **0.4077**; sd 1.6 -> 5.0; recovery trials 1 -> 2 | Lets the MPC plan into the band where `barn_safety`'s shield vetoes commands. Buys stalls, not speed. The MPC's margin should stay at or above what the shield allows. |
| `clearance_penalty_radius 1.4 -> 0.8` | 0.4469 -> **0.3044**; success 12/12 -> 9/12 | Restores a gradient near walls but stops biasing the route toward corridor centres, so A* hugs walls into unrecoverable pinches. The wide band is doing reliability work, not just centering. |
| `max_lateral_accel 3.0 -> 5.0` | mean AT 16.55 -> 15.83 s (4.4%, sd 2.0 -> 3.1) | Inside the noise at n=12. Batch score rose only because one timeout flipped. Not shown harmful — reverted to avoid confounding the next campaign. |

---

## Caveats on these numbers

- **Headless != the campaign.** The 500-trial campaign ran with `rviz:=true`;
  these batches run headless. The offset is not uniform — world 114 was *faster*
  headless (10.8 vs 13.0 s median) but 288 was slightly *slower* (13.9 vs 13.0).
  Speed comparisons against the campaign are therefore unreliable. Reliability
  comparisons (8/10 -> 12/12) are the solid part.
- **World selection inflates the apparent gain.** These three worlds were chosen
  *because* they had timeouts. Campaign-wide, failures cost only 0.0244 of score
  versus 0.0997 lost to slowness, so eliminating the freeze is worth roughly
  **+0.01 to +0.03** overall, not the ~+0.10 the per-world table suggests. Its
  real value is converting hard zeros.
- **12 trials cannot resolve a 1-in-10 failure rate.** Treat single-trial
  differences between batches as noise.

---

## Recommended next steps

1. **Re-run the full 500-trial campaign** with `emergency_margin: 0.02`. That is
   the only change, and it needs campaign-scale validation — in particular that
   0 collisions in 80 trials holds up against the baseline's 1 in 430.
2. **Finish the campaign's missing worlds** — `0, 6, 12, 18, 24, 30, 36` have no
   trials (the run covered indices 7..49), so M11 is not formally complete.
3. **If chasing speed**, test `max_lateral_accel` properly: ~30 trials per config
   on a speed-bound world (216, or 66 at AT/OT 5.24). Speed is where the score
   actually is — 63% of successful campaign trials finish above the `2*OT` clip,
   costing 4x more than every failure combined.
4. **Do not** retry `obstacle_margin` or `clearance_penalty_radius` downward
   without a new mechanism; both regressed measurably.

---

# Session 2 — shield freeze investigation

## The finding that matters most: leaked processes corrupt later trials

A trial killed any way other than the wall-clock timeout leaks its `ros2 launch`
children, which get re-parented to init and keep running. The worst is the
**`/clock` bridge** (`parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock`):
it keeps publishing SIMULATION TIME from a world that no longer exists, so every
node in every later trial reads corrupted time.

Observed directly. One survivor at 140% CPU for 34 minutes poisoned two full
12-trial batches before it was spotted:

| | startup pose at t=0.01 | median AT |
|---|---|---|
| clean machine | `(-2.25, 3.1x)` in 10/10 | 13-18 s |
| with the leaked clock bridge | `(2.00, 2.00)` in 8/12 (the Gazebo spawn origin) | ~40 s |

The evaluator's timer starts during the reset teleport, adding ~30 s to every
trial in a way that reads exactly like "the navigation stack got slower". It is
NOT repetition itself: the campaign's own world 216 started correctly in all 10
trials, and world 288 genuinely timed out twice in the campaign.

**Fixed:** `run_single_world.sh` now reaps unconditionally (previously only on
the timeout path) and matches the `/clock` bridge explicitly;
`preflight_barn_campaign.sh` refuses to start when one is alive.

## The shield cannot change direction (root cause of the freezes)

`SweptFootprintShield::apply` only ever SCALES the desired command. If the
desired heading is blocked, every multiple of it is blocked too -- even when
rotating in place is completely free. Two variants:

* an obstacle INSIDE the veto box (`half + emergency_margin`) makes every scale
  unsafe including in-place rotation;
* an obstacle just OUTSIDE it, dead ahead, blocks every forward scale.

Measured on world 216: the robot sat stationary at `(-2.51, 8.56)` for **89 s**
with `signed clearance 0.001 m` -- positive, so it was the second variant. The
planner's own reverse-to-clearance recovery was running and producing no motion,
because the shield vetoed the reverse too.

`emergency_margin: 0.05 -> 0.02` (session 1) shrank the trap shell 4x (4 cm ->
1 cm) and reduced the frequency, but did NOT fix the mechanism. Session 1
reported it as fixed; that was wrong.

## Escape search — implemented, NOT validated, default OFF

`find_escape()` searches eight slow motions and accepts one only if it strictly
increases signed clearance and never decreases it along the path. Gated behind
`shield_escape_after_s` (1.5 s of sustained dead stop) because firing it on
transient vetoes turned the shield from a filter into an actuator and shoved the
robot off its start pose during the evaluator's reset.

World 216, 12 trials each, clean machine:

| config | success | median AT | score |
|---|---|---|---|
| no escape (`00_with_em0.02`) | 11/12 | 16.2 s | **0.3154** |
| escape, gated | 11/12 | 18.3 s | 0.3039 |

28 escapes fired and 5 `emergency_trapped` events remained. It costs time for no
measurable reliability gain, and 12 trials cannot resolve the difference.
**`shield_escape_enable` defaults to false.** Enable it for a properly powered
comparison (30+ trials per arm) before adopting.

## Observability (kept, enabled)

A shield-frozen robot used to produce ZERO log output, which is why the 89 s
freeze took so long to find. Now:

* `emergency_trapped` -- obstacle inside the box, no escape found (throttled WARN)
* `emergency_escape` -- escape engaged (throttled WARN)
* `publish_zero` -- stale scan / stale command stops (throttled WARN)

Plain `emergency_veto` is deliberately NOT logged: it is the routine stop in
front of an obstacle and floods a tight corridor.

## Still open

* One timeout per ~12 trials on world 216 survives everything above.
* No global stall watchdog: `no_progress` only ticks in the MPC branch, so it is
  frozen while recovery runs. Recovery's own states are individually bounded, so
  there is no evidence this is reachable -- but nothing backstops it.
* `emergency_margin` is still 0.02. With the escape off, that reduction is again
  the only thing shrinking the trap, so it should stay until the escape is
  properly evaluated.

---

# Session 3 — the major tier: where the time actually goes

The 11 worlds scoring below 0.29 (192, 282, 138, 294, 276, 66, 228, 204, 210,
264, 246). Unlike the minor tier this is a SPEED problem: three of them are
10/10 successes and still score 0.20-0.26. Converting every failure in the tier
is worth ~+0.008 campaign-wide; halving AT/OT is worth ~+0.06.

## The tier is not one problem

| world | recovery | AT/OT | diagnosis |
|---|---|---|---|
| 66 | 9 episodes / 5 trials | 5.23 | **freeze** — MPC margin infeasible at the current pose |
| 294 | 0 / 10 trials | 5.36 | **uniformly slow** — never stalls, never goes fast |

Anything measured on one of these does not transfer to the other.

## Instrumentation is what made this tractable

`instrument_trial.sh` + `tools/record_recovery_trace.py` (20 Hz: cmd_desired,
cmd_safe, pose, planner/control status, shield reason). One trial eliminated an
entire branch of the search space:

* shield reason `clear` 97% of samples, cut the command in 3.2% by a median 14%,
  **never zeroed it** — no safety parameter affects these worlds' speed;
* MPC solve 1.13 ms median against a 35 ms deadline, zero misses.

## World 66: the MPC freezes on its own margin

The obstacle row requires `distance_field >= obstacle_margin` at footprint
boundary points already inflated by `footprint.margin` (0.04, HARDCODED, not a
parameter), so the demanded centre clearance is `0.2159 + 0.04 + 0.20 = 0.4559 m`.
BARN's own reference paths thread gaps at 0.225 m.

Caught live: 6.8 s of a 21 s trial stopped dead at 0.40 m clearance with 0.69 m
free ahead, shield `clear` at scale 1.0, MPC `solved`, `goal_dist` frozen. The
constraint is violated at the CURRENT pose, so no action satisfies it and the
QP's cheapest option is not to move. Same shape as the old shield hard veto.

| world | obstacle_margin 0.20 | 0.10 |
|---|---|---|
| 66 | 0.2263, AT max 49.9 s, recovery 9 | **0.2901, AT max 20.4 s, recovery 0** |
| 294 | 0.1953 | 0.1871 (noise, sd ~4) |
| 288 | 0.4286, median 11.9 s | 0.4128, median 13.6 s |

It fixes worlds that have the freeze, does nothing where there is none, and
costs a little on already-fast worlds (less margin -> routes nearer walls).
**Not adopted**: campaign value depends on freeze prevalence, measured on only
3 worlds. The minor tier is nearly freeze-free (14 recovery episodes / 150
trials), so the upside there is small.

## Rejected: fixing the freeze in code (2 variants, world 66, 10 trials each)

Clamping the demanded margin to clearance the robot can actually reach:

| variant | result | why |
|---|---|---|
| at the current pose | 10/10 but **0.2522**, 4 trials still stalled | fires too late — the robot halts JUST BEFORE the tight gap, where its own clearance is still fine |
| min over the whole horizon | **8/10, 2 timeouts, 0.2164** (below baseline) | fires too eagerly — relaxing on the tightest point ahead commits the MPC into pinches it should decline |

| per-step, from `reference[k].clearance` | **9/10, 1 timeout, 0.2372**, recovery 9 / 6 trials | under-engages — see below |

The third was the best-motivated and still failed. It clamped on the clearance at
the PLANNED path point, but `clearance_weight` routes the plan down corridor
CENTRES, so the plan's clearance is systematically more optimistic than the
robot's own while it tracks off-centre. Measured on world 66: the robot's
clearance is below the 0.4559 m engage threshold 31% of the time, with a median
of 0.728 m — so the clamp rarely fired where the robot was actually stuck, and
the batch came out as baseline plus a timeout.

**Conclusion after three attempts: stop feeding the constraint a different
clearance number.** All three variants did that and the mechanism is not
sensitive to it. `obstacle_margin: 0.10` works precisely because it lowers the
demand UNCONDITIONALLY, including when the robot is off-plan — which is the
situation that traps it. If the QP route is revisited, the question to attack is
why standing still is the cheapest response to a violated soft constraint at all
(the slack penalty structure), not which margin the constraint is handed.

All three reverted. Do not retry any of them without a new mechanism.

## World 294: the curvature lookahead sets cruise speed

The speed profile takes the MAX curvature over the next `curvature_lookahead_m`
and applies it to the current point. An offline model over the evaluator's own
reference paths (reconstructs OT to within 1% on 5 worlds) predicted mean v_ref
1.25 m/s for world 294; **measured mean desired_v was 1.137 m/s**. On stall-free
worlds this term, not the shield and not the MPC, is the cap.

`kLookaheadDistance` was a hardcoded constexpr; it is now the parameter
`curvature_lookahead_m` (default 3.0, unchanged behaviour).

**Rejected: 3.0 -> 1.5** (world 294, 10 trials). Successes got faster — median
AT 31.6 -> 28.4 s (-10%), sd 3.8 -> 2.9 — but 10/10 -> 9/10 with a timeout, score
0.1953 -> 0.1890. Exactly the trade the 1.5 -> 3.0 widening was made to avoid.
On this metric a hard zero costs far more than 10% of cruise speed.

**The offline model predicted NO CHANGE for this test and was wrong.** It reads
the reference path; the robot's actual A* path is 1.44x longer with a different
curvature profile. It predicts cruise speed well and window changes badly.

## Measurement corrections made this session

* "294 regressed under obstacle_margin 0.10" — WRONG, that compared headless
  against the RViz campaign. The proper headless baseline says neutral.
* "4x commanded-vs-actual tracking gap" — WRONG. `/barn/pose` publishes at ~11 Hz
  against a 20 Hz sampler, so 44.7% of steps show zero movement and the median
  was meaningless. Actual speed reaches 2.04-2.22 m/s, matching peak commands:
  there is no execution ceiling.

Always baseline headless-vs-headless, and check publish rate before differencing
a pose series.

## Still open

* 8 of 11 major worlds unmeasured (192, 282, 138, 276, 228, 204, 210, 264, 246).
* Which of them have the freeze decides whether obstacle_margin 0.10 is worth
  adopting — recovery-episode count per world is the cheap discriminator.
* World 294 remains 5.36x OT with everything healthy. Its remaining losses are a
  1.44x path-length inefficiency and acceleration dynamics, neither investigated.
* `footprint.margin` (0.04) silently enters the clearance sum with no parameter
  binding — anyone tuning `obstacle_margin` is really setting `0.2559 + margin`.

---

# Session 4 — the method change that found it

Six mechanisms were hypothesised, implemented and rejected on measurement before
anything worked. What finally located the defect was abandoning hypothesis-first
tuning for a direct diff of good runs against bad ones, using traces already on
disk.

## The diff that reframed everything

World 66 trials split cleanly into ~20 s and ~45-85 s with nothing between.
Comparing three of each:

| | distance | path/reference | median v | stopped | recovery |
|---|---|---|---|---|---|
| FAST (14.6-21.2 s) | 10.6 m | 0.99 | 1.10 | 9% | 0% |
| SLOW (44.1-84.6 s) | 11.0 m | 1.00 | 0.12 | **50%** | 12% |

Slow trials drive the SAME distance along the SAME-length route. Route selection
is not the problem -- which is why the A* drivability barrier made things worse
(0.2263 -> 0.1460), and it also retires the earlier "paths are 1.44x the
reference" claim (that was world 294 including pre-clock wandering).

The whole difference is time spent stopped. Breaking down 112 s of stopped time
across the three slow trials:

    29.1%  32.5s  MPC solved, planner OK -- commanding zero anyway
    27.6%  30.9s  planner in recovery
    20.3%  22.8s  MPC solved, path retained (cooldown)
    20.0%  22.4s  startup creep
     2.8%   3.2s  MPC "solved inaccurate"

**The shield accounts for none of it.** Six interventions had been aimed at the
shield, the MPC margin and the distance field -- i.e. at 0% of the measured cost.

## The defect

Instrumenting the speed profile per-term and catching a 7.6 s stall:

| term | during stall | while moving |
|---|---|---|
| vref_curvature | **11.678 rad/m** | 1.367 |
| vref_curv_speed | 0.259 m/s | 1.481 |
| vref_clear_scale | 0.936 | 1.000 |
| vref_head_scale | 0.518 | 0.615 |
| vref | 0.132 | 0.911 |

kappa = 11.678 is an 8.6 cm turn radius on a 51 cm robot. The clearance scale and
heading gate are ruled out -- both are unchanged between stalled and moving.

Cause: `kappa = |dyaw| / ds` guarded only by `ds > 1e-4` (0.1 mm). The elastic
band displaces path points without resampling, so neighbours can end up nearly
coincident, and a real yaw change over a sub-millimetre baseline explodes. The
profile takes the MAX over a 3 m lookahead, so one such sample pins the speed for
the entire approach. Arithmetic checks out:
min(sqrt(3.0/11.678), 3.0/11.678) = 0.257 vs 0.259 measured.

This also explains the bimodality that survived every other change: whether a
trial is 20 s or 80 s depends on whether the band happens to produce a degenerate
point pair on that run's path. Same route, same length, same config.

Fix: require a 5 cm baseline before trusting a curvature estimate, and clamp
kappa at 1/0.30 m (a differential-drive robot cannot usefully arc tighter than
its own rotation radius; anything sharper is a place to rotate in place).

Unlike every rejected change this is a numerical defect, not a tuning trade: no
margin is reduced, no clearance is given up, the shield's view is unchanged.

## Method note

The diff cost minutes and used data already on disk. The six batches that
preceded it cost hours. When a metric is bimodal, diff the modes before
hypothesising a mechanism.

## Session 4 result — three bug fixes, measured

| world | baseline | +curvature | +recovery/replan |
|---|---|---|---|
| 66  | 0.2263, 10/10 | 0.3262 | **0.4196** (+85%) |
| 282 | 0.2101, **8/10** | 0.2309, 8/10 | **0.3205, 10/10** (+53%) |
| 294 | 0.1953, 10/10 | **0.2336** | 0.2280 (flat, sd 3.2) |

30 trials, ZERO collisions, no world regressed. World 282's two timeouts -- which
survived every earlier intervention -- cleared with the recovery refund, and its
sd fell 7.8 -> 2.4. World 66's best trials now run 11.1 s against a 10.66 s
(2*OT) clip, i.e. close to saturating the metric there.

The three fixes, none of which is a tuning trade:

1. **Curvature from a degenerate baseline.** `kappa = |dyaw|/ds` guarded at
   `ds > 1e-4` (0.1 mm); the elastic band displaces points without resampling, so
   near-coincident neighbours produced kappa = 11.678 rad/m (8.6 cm radius on a
   51 cm robot), which the max-over-3 m lookahead then propagated across the whole
   approach. Fixed with a 5 cm baseline requirement and a clamp at 1/0.30 m.
2. **Recovery attempt budget zeroed after 0.18 m.** The robot covers that in
   ~0.2 s after every episode, so every episode restarted at attempt 1 and the
   escalation ladder (rotate >=2, clearance boost >=3, fail 5) was unreachable.
   A* is deterministic, so it returned the same path into the same pinch
   indefinitely -- the loop observed live in RViz. Now decrements one attempt per
   metre of real progress; the clearance boost holds for 2 m instead of 0.18 m.
3. **Global replan fired only on an empty path.** The driven route was the one
   computed against a nearly-blank map (unknown_cost_multiplier 2.0) and was never
   re-optimised as the corridor was observed -- visible as the world-294 U-turn.
   Now re-plans at 2 Hz (13.9 ms against a 500 ms budget), with a >=15%
   improvement escape from the 2 s swap cooldown.

Also fixed, no measurable effect alone but they make measurements trustworthy:
planner_status assigned on rejected plans; recovery integrating on measured dt
rather than a hardcoded 0.033; the dead 600k-cell EDT rebuilt at 15 Hz in
mapping_node; the A* heuristic cutoff comparing a cost against a distance; and the
shield escape latch that cancelled itself after one cycle.

REVERTED after measurement: scan-matcher xy budget (broke registration -- the
translation debit starved the SHARED yaw budget), MPC distance field at 5 cm with
interpolation (freezes 1.8 -> 2.4/trial), A* drivability barrier (0.2263 ->
0.1460; the barrier saturates because nearly every BARN route is below the
threshold).
