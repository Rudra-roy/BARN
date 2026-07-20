# barn_safety

Final command authority. **Every** track's command passes through here before
reaching the robot adapter:

```
Track A (classical) ─┐
Track B (RL)         ─┼──▶ barn_safety ──▶ /barn/cmd_safe ──▶ robot adapter ──▶ /cmd_vel
Track C (hybrid)     ─┘
```

## Responsibilities (implemented)
- Magnitude clamp of linear/angular velocity to configured limits (`limiter.hpp`).
- Per-axis **acceleration** rate-limiting (smooth, smoother-friendly output).
- **Stale-sensor / stale-command** kill: output zero when inputs are old.
- E-stop **watchdog**: publishes a hard zero if the desired command lapses.
- **Swept-footprint clearance shield** (`swept_footprint_shield.{hpp,cpp}`): an
  anticipatory veto that sweeps the robot footprint along the commanded motion
  against the latest scan and scales the command down to the largest collision-free
  version. It may only *reduce* speed, never increase it.

## Design rule
RL and any experimental controller **must not** bypass this node. The limiter and
shield logic are pure and unit-tested (`test/test_limiter.cpp`,
`test/test_swept_footprint_shield.cpp`); the node adds only ROS plumbing and
timing.
