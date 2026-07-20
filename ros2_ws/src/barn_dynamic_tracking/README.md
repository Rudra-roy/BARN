# barn_dynamic_tracking

Shared dynamic-obstacle perception. First used by the **DynaBARN** moving-obstacle
test bed feeding the classical MPC (see
[`docs/features/dynabarn_dynamic_obstacles.md`](../../../docs/features/dynabarn_dynamic_obstacles.md));
Track C (hybrid) will later consume the same `/barn/tracks`. Track B's initial
end-to-end RL baseline does not need it.

## Pipeline
```
/barn/scan + /barn/pose -> clustering -> association -> per-track KF -> /barn/tracks (ObstacleTrackArray)
                                                                     -> /barn/track_markers (RViz)
```

## Status — 🚧 in progress (M17/M18)
| Component | File | Status |
|-----------|------|--------|
| Constant-velocity Kalman filter | `kalman.{hpp,cpp}` | **implemented + tested** |
| Time-to-collision | `ttc.{hpp,cpp}` | **implemented + tested** |
| Clustering (adaptive Euclidean) | `clustering.{hpp,cpp}` | **implemented + tested** |
| Association (greedy NN gating) | `association.{hpp,cpp}` | **implemented + tested** |
| Tracker node | `tracker_node.{hpp,cpp}` | **implemented** — predict→cluster→transform→associate→KF→spawn/prune, publishes `/barn/tracks` |

The tracker builds tracks from **allowed LiDAR history only** — never from
Gazebo model states or ground-truth obstacle poses. Parameters (cluster/gate
distances, KF noise, `min_hits`/`max_misses`, `output_frame`) live in
`config/tracking.yaml`. `output_frame` must be `map` to match the MPC planning
frame (tracks are computed from the drift-corrected `/barn/pose`).

## Tests
`test/test_kalman.cpp` verifies the KF converges to a constant target velocity
and that the TTC estimate is correct for head-on / separating cases; clustering
tests cover the adaptive Euclidean grouping.
