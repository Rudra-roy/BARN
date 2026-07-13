# movement_and_odom_test

This is a deliberately unsafe wiring-test planner, not a navigation algorithm.
It accepts the evaluator's `/navigate_to_pose` goal, reads
`/platform/odom/filtered`, rotates toward the goal, then publishes a constant
`geometry_msgs/TwistStamped` on `/cmd_vel` for a fixed amount of simulation
time. It does not read LiDAR or avoid obstacles.

Run it through the official evaluator:

```bash
ros2 launch jackal_helper BARN_runner.launch.py \
  world_idx:=0 gui:=true \
  algo_type:=movement_and_odom_test \
  movement_duration:=4.0 \
  forward_velocity:=0.25 \
  rotation_speed:=0.5
```

The velocity units are SI in Gazebo: `forward_velocity:=2.0` requests 2 m/s,
and `rotation_speed:=0.5` requests 0.5 rad/s. Start slowly because this planner
has no collision handling.
