# patches/

The BARN 2026 evaluator is **not vendored** — it is cloned and pinned locally by
[`tools/setup_barn_eval.sh`](../tools/setup_barn_eval.sh) into
`ros2_ws/src/barn_eval/` (git-ignored). Only two things are tracked here:

| File | Purpose |
|------|---------|
| `pinned_commit.txt` | the exact evaluator commit this repo is validated against |
| `barn_eval_launch_navigation_stack.patch` | the one change we make to the evaluator |

## The only change we make
The upstream README says to modify the navigation-stack launch hook and leave
everything else alone. The patch rewrites `launch_navigation_stack()` to
`IncludeLaunchDescription` our `barn_bringup/launch/barn_navigation.launch.py`
with `mode:=$BARN_MODE`. It does **not** touch collision detection, the goal
position, the timeout, result writing, world selection, or the metric.

## Why the patch is a template
Upstream has **no tagged release**, so the exact file path, function signature,
and surrounding lines can differ between revisions. The shipped `.patch` is
therefore a **template**: `setup_barn_eval.sh` tries to apply it and, if the
context does not match your pinned revision, prints instructions to apply the
change by hand instead of failing.

## Pin a commit (do this once)
```bash
bash tools/setup_barn_eval.sh                 # clones master if unpinned
git -C ros2_ws/src/barn_eval rev-parse HEAD   # copy this hash
# put the hash on the first non-comment line of patches/pinned_commit.txt
```

## Regenerate the patch against your pinned revision
```bash
cd ros2_ws/src/barn_eval
#  ... edit launch_navigation_stack() to include barn_navigation.launch.py ...
git diff > ../../../patches/barn_eval_launch_navigation_stack.patch
git restore .        # keep the evaluator pristine; the patch is the source of truth
```
Re-running `tools/setup_barn_eval.sh` will then apply it cleanly and
idempotently.
