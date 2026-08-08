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
