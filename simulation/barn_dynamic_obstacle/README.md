# barn_dynamic_obstacle

A self-contained Gazebo Sim (Harmonic / gz-sim8) **system plugin** that moves a
model along a smooth cubic waypoint path at a constant speed. It is the
moving-obstacle test bed for an MPC dodging demo and mirrors the DynaBARN
dynamic-obstacle motion model.

This package is intentionally **standalone** — it lives outside `ros2_ws/` so it
builds with plain CMake and never races the ament/colcon build.

## What it is

- `src/DynamicObstacle.cc` — a `gz::sim::System` (`ISystemConfigure` +
  `ISystemPreUpdate`), plugin name string `barn::sim::DynamicObstacle`.
- `worlds/moving_cylinder_demo.sdf` — a minimal world (sun, ground plane, one
  cylinder obstacle) that loads the plugin and traces an S-curve.
- `CMakeLists.txt` — standalone build producing `libbarn_dynamic_obstacle.so`.

### SDF parameters (on the `<plugin>` element)

| Element            | Default | Meaning                                                        |
|--------------------|---------|----------------------------------------------------------------|
| `<speed>`          | `1.0`   | Constant travel speed along the path (m/s).                    |
| `<loop>`           | `true`  | `true` = ping-pong back and forth; `false` = stop at the end.  |
| `<waypoint>x y</waypoint>` | — | World-XY control point. At least 2 required; order matters.    |
| `<gain>`           | `2.0`   | Proportional gain on the position-correction term.             |
| `<max_correction>` | `2.0`   | Clamp (m/s) on the position-correction velocity magnitude.    |

### How it works

1. **Path**: a Catmull-Rom / cubic-Hermite spline is fit through the waypoints,
   yielding degree-3 polynomial segments (matching DynaBARN's cubic paths).
   Endpoint neighbours are clamped so the curve passes through the first and last
   control points.
2. **Arc-length parameterization**: the spline is densely sampled and tagged with
   cumulative arc length, so `distance = speed * simTime` maps to a constant-speed
   target point and unit tangent.
3. **Motion**: each `PreUpdate`, the plugin sets the model's
   `LinearVelocityCmd` component (planar only — no Z, no angular) to
   `speed * unit_tangent + clamp(gain * (path_point - current_pos))`. The
   feedforward keeps the constant speed; the clamped proportional term servos the
   model back onto the path. Work is skipped while paused or when `dt == 0`.

## Build

Inside the `barn-jazzy` distrobox (the gz-sim8 vendor packages come from ROS
Jazzy and are only found after sourcing it):

```bash
distrobox enter barn-jazzy -- bash -lc '
  source /opt/ros/jazzy/setup.bash
  cd /run/host/home/mt-labpc/BARN/simulation/barn_dynamic_obstacle
  cmake -S . -B build && cmake --build build -j
'
```

This produces `build/libbarn_dynamic_obstacle.so`.

## Run (headless)

```bash
distrobox enter barn-jazzy -- bash -lc '
  source /opt/ros/jazzy/setup.bash
  cd /run/host/home/mt-labpc/BARN/simulation/barn_dynamic_obstacle
  export GZ_SIM_SYSTEM_PLUGIN_PATH=$PWD/build
  gz sim -s -r --iterations 500 worlds/moving_cylinder_demo.sdf
'
```

`gz` resolves to `/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz` (Gazebo Sim 8.11.0).

### Verify it moves

With the server running (drop `--iterations` and use `-r` for real time), sample
the obstacle pose:

```bash
gz topic -e -t /world/moving_cylinder_demo/dynamic_pose/info -n 1 | grep -A6 obstacle_0
```

Repeated samples show the cylinder's XY changing as it follows the S-curve.

## How it maps to DynaBARN

DynaBARN populates BARN worlds with dynamic obstacles that follow smooth
cubic-polynomial trajectories through randomly sampled control points. This
plugin reproduces that motion model: cubic (Catmull-Rom) paths through
user-supplied waypoints, traversed at a fixed speed, with ping-pong looping — a
controllable, repeatable moving obstacle for developing and stress-testing an MPC
dodging controller.
