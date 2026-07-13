# BARN 2026 setup: Jazzy Distrobox and official evaluator baseline

This is the canonical first-run guide. It creates the Ubuntu 24.04 / ROS 2
Jazzy environment, clones the official evaluator into the same colcon workspace
as our packages, adds the minimal `algo_type` dispatcher, builds everything
together, and runs the evaluator's stock Nav2/MPPI branch.

No custom algorithm is selected during setup. The evaluator's hardcoded
`jackal_helper` package name is preserved, and `algo_type:=builtin` retains its
original navigation behavior.

## 1. Create the Distrobox

On the host, from this repository:

```bash
cd /home/kage/BARN
bash infra/distrobox/create_barn_jazzy.sh
distrobox enter barn-jazzy
```

Use `--nvidia` only when host NVIDIA integration is needed for the Gazebo GUI:

```bash
bash infra/distrobox/create_barn_jazzy.sh --nvidia
```

## 2. Install ROS 2 Jazzy inside the box

Locate the host checkout from inside Distrobox and run its provisioning script:

```bash
if [[ -d /run/host/home/kage/BARN ]]; then
  export BARN_REPO=/run/host/home/kage/BARN
else
  export BARN_REPO=/home/kage/BARN
fi

cd "$BARN_REPO"
bash infra/distrobox/setup_jazzy.sh
source /opt/ros/jazzy/setup.bash
source infra/env/barn_jazzy.env
```

Verify the required distribution:

```bash
test "$ROS_DISTRO" = jazzy && echo "ROS 2 Jazzy ready"
```

## 3. Clone the evaluator and configure algorithm selection

From the repository root inside the box:

```bash
cd "$BARN_REPO"
bash tools/setup_barn_eval.sh
```

This clones the upstream repository into its expected source path,
`ros2_ws/src/The-Barn-Challenge-Ros2`. Upstream hardcodes both that checkout
directory and the `jackal_helper` ROS package name, so neither is renamed. It
then adds the idempotent `algo_type` dispatcher. Its `builtin` branch retains
the official Nav2 example, and its `movement_and_odom_test` branch launches the
small wiring-test package from this repository.

Confirm the only evaluator source change is the runner dispatcher:

```bash
git -C ros2_ws/src/The-Barn-Challenge-Ros2 status --short
```

Expected:

```text
M jackal_helper/launch/BARN_runner.launch.py
```

## 4. Resolve dependencies and build the shared workspace

```bash
cd "$BARN_REPO"
source /opt/ros/jazzy/setup.bash
bash tools/setup_workspace.sh
source ros2_ws/install/setup.bash
```

Both the official evaluator and our still-in-development packages are built by
this one colcon workspace. A second evaluator clone or workspace is not needed.

Verify the upstream package name:

```bash
ros2 pkg prefix jackal_helper
```

## 5. Run the official Nav2/MPPI example

Headless:

```bash
ros2 launch jackal_helper BARN_runner.launch.py \
  algo_type:=builtin world_idx:=0
```

With Gazebo GUI:

```bash
ros2 launch jackal_helper BARN_runner.launch.py \
  algo_type:=builtin world_idx:=0 gui:=true
```

With Gazebo and RViz:

```bash
ros2 launch jackal_helper BARN_runner.launch.py \
  algo_type:=builtin world_idx:=0 gui:=true rviz:=true
```

The evaluator should finish with `succeeded`, `collided`, or `timeout` and print
the elapsed time and navigation metric. All three outcomes demonstrate that the
official trial pipeline completed.

## 6. Test the official reporter

Run the reporter against the example results shipped by upstream:

```bash
cd "$BARN_REPO/ros2_ws/src/The-Barn-Challenge-Ros2"
python3 report_test.py --out_path res/mppi_out.txt
```

To record a fresh trial:

```bash
mkdir -p "$BARN_REPO/results/official"
ros2 launch jackal_helper BARN_runner.launch.py \
  algo_type:=builtin \
  world_idx:=0 \
  out_file:="$BARN_REPO/results/official/raw_results.txt"

cd "$BARN_REPO/ros2_ws/src/The-Barn-Challenge-Ros2"
python3 report_test.py \
  --out_path "$BARN_REPO/results/official/raw_results.txt"
```

Missing-world warnings are expected after a single trial.

## 7. Run the movement and odometry wiring test

This test is not an obstacle-avoiding planner. It only confirms the complete
goal -> odometry -> velocity-command path: it receives the evaluator's goal,
reads `/platform/odom/filtered`, rotates toward the goal, and drives straight
for the requested amount of simulation time.

```bash
ros2 launch jackal_helper BARN_runner.launch.py \
  algo_type:=movement_and_odom_test \
  world_idx:=0 gui:=true \
  movement_duration:=4.0 \
  forward_velocity:=0.25 \
  rotation_speed:=0.5
```

The units are SI: `forward_velocity` is m/s, `rotation_speed` is rad/s, and
`movement_duration` is simulation seconds. Thus `forward_velocity:=2.0` really
requests 2 m/s in Gazebo. Use a low value first because this test intentionally
does not check LiDAR or avoid collisions.

Future real algorithms use the same evaluator integration point:

```text
ros2_ws/src/The-Barn-Challenge-Ros2/jackal_helper/launch/BARN_runner.launch.py
  -> launch_navigation_stack()
```

Add another named `algo_type` branch inside that function when its planner is
ready. Do not rename `jackal_helper` or change Gazebo, spawning, goal
publication, collision detection, timeout, result logging, or metric behavior.

## Every new container shell

```bash
if [[ -d /run/host/home/kage/BARN ]]; then
  export BARN_REPO=/run/host/home/kage/BARN
else
  export BARN_REPO=/home/kage/BARN
fi

source /opt/ros/jazzy/setup.bash
source "$BARN_REPO/infra/env/barn_jazzy.env"
source "$BARN_REPO/ros2_ws/install/setup.bash"
```

Then the official baseline remains:

```bash
ros2 launch jackal_helper BARN_runner.launch.py \
  algo_type:=builtin world_idx:=0 gui:=true
```
