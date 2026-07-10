# BARN 2026 Scoring Metric Notes

> Purpose: define the scoring metric precisely, explain the clip, document the `2*OT` vs
> `4*OT` discrepancy between the published rule and the upstream evaluator source, and
> specify the mandatory two-report policy plus the correct campaign suite.

Related documents:

- [Benchmark contract](./barn_2026_contract.md) — task, robot, and information policy.
- [Failure taxonomy](./failure_taxonomy.md) — how failed (score-0) trials are labeled.
- [Programme roadmap](../roadmap.md) — where the 500-trial benchmark milestones sit.

---

## 1. The Per-Trial Score

```
s_i = success_i * OT_i / clip( AT_i , L*OT_i , 8*OT_i )
```

| Symbol | Meaning |
|---|---|
| `success_i` | 1 if the robot reaches the goal (within 1 m) without collision before the 100 s timeout, else 0 |
| `AT_i` | Actual traversal time for trial *i* |
| `OT_i` | Optimal traversal time = `reference_path_length_i / 2.0` (2 m/s max speed) |
| `L` | Clip **lower-bound multiplier** — the entire discrepancy in this document is about whether `L = 2` or `L = 4` |
| `8` | Clip **upper-bound multiplier** (agreed by both sources) |

The overall score is the mean of `s_i` over all **500 trials** (50 worlds × 10 trials).

A collision, a timeout, or any failure sets `success_i = 0`, so `s_i = 0` regardless of
speed. **You cannot score points by being fast; you score by finishing cleanly, then
being efficient.**

---

## 2. What the Clip Does

The clip bounds the actual time into `[L*OT, 8*OT]` before it divides `OT`:

```
   clip(AT, L*OT, 8*OT):

   AT < L*OT  ->  treated as L*OT   (caps the reward for going implausibly fast)
   AT > 8*OT  ->  treated as 8*OT   (floors the reward for crawling)
   otherwise  ->  AT itself
```

- Without the **lower** bound, a run that momentarily exceeds the intended speed model
  would earn a score above 1 and dominate the mean.
- Without the **upper** bound, an extremely slow-but-successful crawl would contribute a
  near-zero score that overweights slow successes relative to outright failures.

The lower bound is exactly where the two metric definitions disagree.

---

## 3. The `2*OT` vs `4*OT` Discrepancy

### 3.1 Published rule (the research number)

The **published BARN 2026 rule** clips with lower bound `2*OT`:

```
s_i = success_i * OT_i / clip( AT_i , 2*OT_i , 8*OT_i )     # L = 2
```

### 3.2 Upstream evaluator source (evaluator-compat)

The upstream **ROS 2 master `report_test.py`** source actually computes:

```python
np.clip(actual_time, optimal_time * 4, optimal_time * 8)     # L = 4
```

i.e. lower bound `4*OT`. Worse, the file's **inline comment disagrees with its own
code** — the comment describes one bound while the code applies another. The code is what
ran, so the code is what the upstream evaluator scored with.

### 3.3 Why this matters

For any **successful trial faster than `4*OT`** (i.e. `AT < 4*OT`) the two definitions clip
at different points and therefore assign **different non-zero scores** to the same trial:

- `AT <= 2*OT` (very fast): published clips up to `2*OT` → score `0.5`; upstream clips up to
  `4*OT` → score `0.25`. **Maximum divergence.**
- `2*OT < AT < 4*OT`: published leaves `AT` unclipped → score `OT/AT`; upstream still clips
  up to `4*OT` → score `0.25`. **They diverge.**
- `4*OT <= AT <= 8*OT`: both use `AT` directly → **identical scores**.
- `AT > 8*OT`: both clip down to `8*OT` → identical (minimum non-zero) scores.

So the two reports agree only for slow-but-successful runs (`AT >= 4*OT`); they diverge for
every fast success.

---

## 4. Mandatory Two-Report Policy

This repository **always emits two reports** and **never mixes them**:

| Script | Clip bounds | Use for |
|---|---|---|
| `evaluation/metrics/barn2026_metric.py` | `[2*OT, 8*OT]` (published, `L=2`) | **Research numbers** — the rule we report against |
| `evaluation/metrics/upstream_compat_metric.py` | `[4*OT, 8*OT]` (upstream, `L=4`) | **Evaluator-compat debugging only** — reproducing what the upstream evaluator would have printed |

Rules:

1. Every results table **must state which metric definition produced it.** A table
   without this label is invalid.
2. Never average or combine the two reports into one figure.
3. Use `barn2026_metric.py` for anything reported as a research result; use
   `upstream_compat_metric.py` only when explaining or reproducing upstream behavior.
4. Both scripts support a `--selftest` flag that runs the built-in numeric checks
   (including the worked example below) so you can confirm each script applies the clip
   bound it claims before trusting a campaign report.

```
                raw evaluator output (results/)  <- source of truth
                          |
             +------------+------------+
             v                         v
   barn2026_metric.py         upstream_compat_metric.py
   clip[2*OT, 8*OT]            clip[4*OT, 8*OT]
   "PUBLISHED"                 "UPSTREAM-COMPAT"
   research numbers            debugging only
             |                         |
             v                         v
     report labeled              report labeled
     "published rule"           "upstream compat"     (never merged)
```

---

## 5. Worked Numeric Example

Take a **slow success**: `OT = 5 s`, `AT = 15 s`, `success = 1`.

| Report | Clip lower bound | Clipped AT | `s = OT / clipped_AT` |
|---|---|---|---|
| Published (`barn2026_metric.py`, `L=2`) | `2*OT = 10 s` | `clip(15, 10, 40) = 15` | `5 / 15 ≈ 0.333` |
| Upstream (`upstream_compat_metric.py`, `L=4`) | `4*OT = 20 s` | `clip(15, 20, 40) = 20` | `5 / 20 = 0.250` |

Wait — verify against the intended divergence:

- Published: `AT = 15` sits **above** `2*OT = 10`, so it is not clipped up; score
  `= 5/15 ≈ 0.333`.
- Upstream: `AT = 15` sits **below** `4*OT = 20`, so it **is** clipped up to `20`; score
  `= 5/20 = 0.250`.

So for this single trial the **published rule gives ≈ 0.333** and the **upstream variant
gives 0.250** — the same physical run yields two different non-zero scores. This is
exactly why the two reports must be kept separate and each table must name its metric.

Second example — a **fast success** (`OT = 5 s`, `AT = 6 s`), the maximum-divergence case:

| Report | Clip lower bound | Clipped AT | `s = OT / clipped_AT` |
|--------|------------------|------------|-----------------------|
| Published (`L=2`) | `2*OT = 10 s` | `clip(6, 10, 40) = 10` | `5 / 10 = 0.500` |
| Upstream (`L=4`) | `4*OT = 20 s` | `clip(6, 20, 40) = 20` | `5 / 20 = 0.250` |

Here `AT = 6` is below **both** lower bounds, so each report clips it up to its own bound:
the published rule caps the per-trial score at `0.5`, the upstream variant at `0.25`. These
are exactly the values asserted by each script's `--selftest`.

Upper-bound sanity check (`8*OT = 40 s`): a run with `AT = 100 s` (near timeout, but a
success) is clipped to `40` under both, scoring `5/40 = 0.125` in each — the reports agree
above `4*OT`.

---

## 6. Upstream `test.sh` World-Selection Bug

The upstream 2026 master `test.sh` loops with:

```bash
for i in {7..49}     # BUG
```

which starts at **world_42** and **skips worlds 0, 6, 12, 18, 24, 30, 36**, even though
its own comment claims it runs the full evenly spaced set `[0, 6, …, 294]`.

```
  intended:  0  6  12  18  24  30  36  42  48 ... 294   (50 worlds)
  actual:    x  x   x   x   x   x   x  42  48 ... 294   (43 worlds, first 7 skipped)
             ^^^^^^^^^^^^^^^^^^^^^^^^^  dropped by {7..49}
```

**Do not use the upstream `test.sh` for a fresh full campaign** — it silently benchmarks
only 43 of the 50 worlds.

### Correct suite

Use `evaluation/scripts/run_barn2026_public_suite.sh`, which correctly iterates:

```
for i in 0..49:  world = i * 6      # -> 0, 6, 12, ..., 294  (all 50)
                 run 10 trials each  # -> 500 trials
```

This is the only supported way to produce a reportable 500-trial campaign. Capture the
run manifest with `evaluation/scripts/capture_manifest.sh` and keep raw output under
`results/` as the source of truth.

---

## 7. Reporting Checklist

- [ ] Scored from raw `results/` output, not intermediate summaries.
- [ ] Both `barn2026_metric.py` and `upstream_compat_metric.py` reports produced.
- [ ] Each table labeled with its metric (published vs upstream-compat).
- [ ] Research conclusions cite the **published** (`2*OT`) report.
- [ ] `--selftest` passed on both scripts before trusting the campaign report.
- [ ] Campaign run via `run_barn2026_public_suite.sh` (all 50 worlds), manifest captured.
