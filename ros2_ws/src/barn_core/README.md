# barn_core

Pure algorithm types and mathematics shared by every navigation track. This is
the foundation of the workspace and the one hard boundary in the design.

## Responsibilities
- Value types: `Pose2D`, `Goal2D`, `VelocityCommand`, `Limits` (`types.hpp`).
- Planar geometry: `wrap_angle`, `clamp`, `dist2d`, `heading_to`, `yaw_from_quat` (`geometry.hpp`).
- LiDAR queries over a non-owning `ScanView`: `min_range_in_sector`, `nearest_obstacle` (`scan.hpp`).
- Online mapping primitives: `OccupancyGrid2D` + log-odds helpers (`occupancy.hpp`, `logodds.hpp`).

## Hard boundaries (do not cross)
- **No** `rclcpp`, **no** ROS message types, **no** `tf2`, **no** Gazebo.
- Nothing here touches the wire. Adapters convert ROS ↔ `barn_core` types.

This is what makes the "would it run on the physical Jackal?" test pass: the
algorithms depend only on plain data, so swapping `barn_robot_adapter` is the
only change needed to move from simulation to hardware.

## Tests
`test/test_geometry.cpp`, `test/test_scan.cpp` (gtest). Run with
`colcon test --packages-select barn_core`.
