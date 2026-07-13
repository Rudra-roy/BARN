# barn_mapping

Online LiDAR occupancy mapping for Track A. Turns `/barn/scan` + `/barn/pose`
into a log-odds `OccupancyGrid2D` (odom frame), published as
`nav_msgs/OccupancyGrid` for the planner and RViz.

The implementation transforms each scan into `odom`, marks free cells along
each ray, accumulates occupied endpoints with bounded log odds, and publishes a
latched occupancy grid for planning and RViz.

## Hard rule
Build the map **only** from allowed sensor data (`/barn/scan`, pose/TF). Never
load the test world's ground-truth `.npy`/`.pgm` map. Public maps may be used
offline for unit-testing A\* and visualizing inflation — never in a scored run.
See [`docs/benchmark/barn_2026_contract.md`](../../../docs/benchmark/barn_2026_contract.md).

## Tests
`test/test_logodds.cpp` exercises the real `barn_core` log-odds + grid
primitives this node builds on.
