# infra/

Reproducible environment setup. You already run an Ubuntu 24.04 + ROS 2 Jazzy
distrobox; these scripts document and reproduce that environment.

```
distrobox/create_barn_jazzy.sh   create a dedicated Ubuntu 24.04 box with its own HOME
distrobox/setup_jazzy.sh         install ROS 2 Jazzy inside the box (run INSIDE it)
env/barn_jazzy.env               shell env: ROS domain, RMW, CPU-only benchmark mode
```

Typical first-time flow on a fresh host:
```bash
bash infra/distrobox/create_barn_jazzy.sh      # host
distrobox enter barn-jazzy                      # host -> container
bash infra/distrobox/setup_jazzy.sh             # container
source infra/env/barn_jazzy.env
bash tools/setup_barn_eval.sh && bash tools/setup_workspace.sh
```

**Docker note:** we deliberately target distrobox, not Docker — it matches the
existing setup and integrates the host GUI/GPU cleanly. Add a Dockerfile later
only if CI or a teammate needs it.
