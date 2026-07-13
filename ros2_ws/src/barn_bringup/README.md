# barn_bringup

Launch and configuration composition. One entrypoint selects the track; the BARN
evaluator does not need to know which one runs.

## Entrypoint
```bash
ros2 launch barn_bringup barn_navigation.launch.py mode:=classical use_sim_time:=true
ros2 launch barn_bringup barn_navigation.launch.py mode:=classical_mpc use_sim_time:=true
ros2 launch barn_bringup barn_navigation.launch.py mode:=e2e_rl
ros2 launch barn_bringup barn_navigation.launch.py mode:=hybrid
```
`barn_navigation.launch.py` includes exactly one selected navigation stack.
`classical_mpc` launches mapping, the footprint-aware planner/controller,
adapters, and the independent safety process.

## Arguments
| Arg | Default | Meaning |
|-----|---------|---------|
| `mode` | `classical` | `classical` \| `classical_mpc` \| `e2e_rl` \| `hybrid` |
| `use_sim_time` | `true` | must be true under the evaluator (Gazebo `/clock`) |
| `cmd_vel_type` | `twist_stamped` | final `/cmd_vel` type; `twist` \| `twist_stamped` |

## Config
`config/common.yaml` (shared `use_sim_time`) plus one file per mode
(`classical.yaml`, `e2e_rl.yaml`, `hybrid.yaml`). Each file has one section per
node name; every node loads `common.yaml` then its mode file. Override a single
parameter on the command line, e.g.:
```bash
ros2 launch barn_bringup barn_navigation.launch.py mode:=classical cmd_vel_type:=twist
```

## Official evaluator integration

The evaluator's `algo_type:=builtin` branch still runs its official
`jackal_helper` Nav2 baseline. Select this stack with `algo_type:=classical_mpc`;
the evaluator package name, goal action, simulation, collision detection,
timeout, and scoring logic remain unchanged.
