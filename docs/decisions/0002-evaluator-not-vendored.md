# ADR-0002: Evaluator not vendored (clone + pin + patch)

> Purpose: record why the BARN 2026 evaluator is cloned, pinned, and patched at setup time rather than committed into this repository.

Status: Accepted

## Context

The benchmark harness is the upstream project `The-Barn-Challenge-Ros2` (branch `master`). It owns everything about the evaluated environment:

- Gazebo and the Jackal spawn
- the LiDAR remap `/sensors/lidar2d_0/scan` → `/front/scan`
- collision, goal-distance, and timeout monitoring
- result logging

Our stack needs exactly **one** change to that harness: the evaluator's `launch_navigation_stack()` must be made to launch our navigation slice (`barn_bringup/launch/barn_navigation.launch.py` with `mode:=$BARN_MODE`) instead of the baseline stack. Everything else in the evaluator must remain untouched so that our scores are comparable to the reference BARN 2026 results.

We considered vendoring the evaluator directly — either as committed source or as a git submodule. Both have costs:

- **Committed source** bloats the repo with third-party code we do not own and blurs the line between our work and the harness.
- **Submodules** add operational friction (detached HEADs, `--recursive` clones, update dance) for what is fundamentally a build-time dependency.
- Upstream has **no tagged release**, so we cannot depend on a stable version reference; we must pin an explicit commit ourselves.

## Decision

**Do not vendor the evaluator. Clone, pin, and patch it at setup time.**

`tools/setup_barn_eval.sh`:

1. Clones `The-Barn-Challenge-Ros2` (`master`) into `ros2_ws/src/barn_eval`, which is **git-ignored**.
2. Checks it out at the exact commit in `patches/pinned_commit.txt`.
3. Applies `patches/barn_eval_launch_navigation_stack.patch`, which rewrites `launch_navigation_stack()` to launch `barn_bringup/launch/barn_navigation.launch.py` with `mode:=$BARN_MODE`.

**Only two artifacts are tracked in this repo:** the pinned commit (`patches/pinned_commit.txt`) and the one integration patch. The evaluator checkout itself is never committed.

Because upstream is untagged and moves, **the patch is a template.** If it stops applying, regenerate it against the new pinned commit and restore the tree:

```bash
# inside ros2_ws/src/barn_eval, after hand-editing launch_navigation_stack()
git diff > ../../../patches/barn_eval_launch_navigation_stack.patch
git restore .
```

Then update `patches/pinned_commit.txt` to the commit you built against.

## Consequences

- **One manual setup step.** A fresh checkout must run `bash tools/setup_barn_eval.sh` before building. This is documented as Step 2 of the canonical first-run sequence in [`docs/setup/barn_2026_jazzy_distrobox.md`](../setup/barn_2026_jazzy_distrobox.md).
- **The repo stays light.** No third-party harness source or Gazebo assets are committed; our tree contains only our code plus a commit reference and a small patch.
- **Exactly one integration change is recorded.** The patch is the complete, auditable record of how we hook into the evaluator — no hidden edits to Gazebo, the Jackal spawn, the LiDAR remap, or the monitoring/logging.
- **The patch is maintenance-sensitive.** Because upstream is untagged, the patch may need regenerating whenever the pin is bumped. This is an accepted, low-frequency cost and is scripted above.
- **Reproducibility rests on the pin.** `patches/pinned_commit.txt` is the single source of truth for which evaluator revision our results were produced against; it must be bumped deliberately and only for a commit verified on our build distro (see gotcha (a) in the setup guide).
- **Never modify evaluator internals.** Anything beyond the launch hook (Gazebo, spawn, remap, monitoring, logging) stays as upstream ships it, preserving comparability with the reference BARN 2026 scores.

Related: setup guide [`docs/setup/barn_2026_jazzy_distrobox.md`](../setup/barn_2026_jazzy_distrobox.md).
