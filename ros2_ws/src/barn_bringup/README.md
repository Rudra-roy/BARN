# barn_bringup

Launch and configuration composition. One entrypoint selects the track; the BARN
evaluator does not need to know which one runs.

## Entrypoint
```bash
ros2 launch barn_bringup barn_navigation.launch.py mode:=classical use_sim_time:=true
ros2 launch barn_bringup barn_navigation.launch.py mode:=e2e_rl
ros2 launch barn_bringup barn_navigation.launch.py mode:=hybrid
```
`barn_navigation.launch.py` includes exactly one of `slice_classical.launch.py`,
`e2e_rl.launch.py`, or `hybrid.launch.py` based on `mode`.

## Arguments
| Arg | Default | Meaning |
|-----|---------|---------|
| `mode` | `classical` | `classical` \| `e2e_rl` \| `hybrid` |
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

## How the evaluator uses it
The evaluator's `launch_navigation_stack()` is patched to include this file with
`mode:=$BARN_MODE`. See [`patches/`](../../../patches/) and
[`tools/setup_barn_eval.sh`](../../../tools/setup_barn_eval.sh).
