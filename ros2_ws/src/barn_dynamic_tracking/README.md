# barn_dynamic_tracking

Shared dynamic-obstacle perception for **Track C (hybrid)**. Track B's initial
end-to-end RL baseline does not need it.

## Pipeline
```
/barn/scan -> clustering -> association -> per-track KF -> relative velocity / TTC -> /barn/tracks
```

## Status
| Component | File | Status |
|-----------|------|--------|
| Constant-velocity Kalman filter | `kalman.hpp` | **implemented + tested** |
| Time-to-collision | `ttc.hpp` | **implemented + tested** |
| Clustering | `clustering.hpp` | stub (M18) |
| Association | `association.hpp` | stub (M18) |
| Tracker node | `tracker_node.hpp` | stub — publishes empty `/barn/tracks` |

The tracker builds tracks from **allowed LiDAR history only** — never from
Gazebo model states or ground-truth obstacle poses.

## Tests
`test/test_kalman.cpp` verifies the KF converges to a constant target velocity
and that the TTC estimate is correct for head-on / separating cases.
