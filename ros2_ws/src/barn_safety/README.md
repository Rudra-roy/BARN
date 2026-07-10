# barn_safety

Final command authority. **Every** track's command passes through here before
reaching the robot adapter:

```
Track A (classical) ─┐
Track B (RL)         ─┼──▶ barn_safety ──▶ /barn/cmd_safe ──▶ robot adapter ──▶ /cmd_vel
Track C (hybrid)     ─┘
```

## Responsibilities (implemented)
- Magnitude clamp of linear/angular velocity to configured limits.
- Per-axis **acceleration** rate-limiting (smooth, smoother-friendly output).
- **Stale-sensor / stale-command** kill: output zero when inputs are old.
- E-stop **watchdog**: publishes a hard zero if the desired command lapses.

## Responsibilities (stubbed — `limiter.hpp`)
- `apply_forward_corridor()`: clearance-aware forward slowdown. Currently a
  pass-through; when implemented it may only *reduce* speed, never increase it.

## Design rule
RL and any experimental controller **must not** bypass this node. The limiter
logic is pure and unit-tested (`test/test_limiter.cpp`); the node adds only ROS
plumbing and timing.
