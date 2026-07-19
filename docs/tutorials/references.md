# References & further reading

The BARN navigation tutorial cites foundational works by short key (e.g.
`[Hart 1968]`). Full entries are collected here, grouped by topic, with a one-line
note on *why it matters for this stack* and a pointer to where it shows up.

These are starting points, not a reading list you must finish. If you read three,
read [Thrun 2005], [LaValle 2006], and [Rawlings 2017] — between them they cover
most of what this repository does.

---

## The benchmark

- **[Perille 2020]** — Perille, D., Truong, A., Xiao, X., Stone, P. (2020).
  *Benchmarking Metric Ground Navigation.* IEEE Int. Symp. on Safety, Security,
  and Rescue Robotics (SSRR).
  → Defines the BARN environments and the "metric navigation" task this whole
  repo targets. See [Chapter 00](./00-the-barn-problem.md).
- **[Xiao 2022]** — Xiao, X., Xu, Z., Warnell, G., Stone, P., et al. (2022).
  *Autonomous Ground Navigation in Highly Constrained Spaces: Lessons Learned
  from the BARN Challenge.* ICRA / arXiv.
  → What actually works (and fails) on BARN across teams. Context for our design
  choices and the [failure taxonomy](../benchmark/failure_taxonomy.md).

## Search & global planning

- **[Dijkstra 1959]** — Dijkstra, E. W. (1959). *A note on two problems in
  connexion with graphs.* Numerische Mathematik, 1, 269–271.
  → The uniform-cost shortest path. Our A* heuristic is a Dijkstra cost-to-go
  field. See [Chapter 03](./03-global-planning-with-a-star.md).
- **[Hart 1968]** — Hart, P. E., Nilsson, N. J., Raphael, B. (1968). *A Formal
  Basis for the Heuristic Determination of Minimum Cost Paths.* IEEE Trans. on
  Systems Science and Cybernetics, 4(2), 100–107.
  → A* itself: optimal search with an admissible heuristic. [Chapter 03](./03-global-planning-with-a-star.md).
- **[Dolgov 2010]** — Dolgov, D., Thrun, S., Montemerlo, M., Diebel, J. (2010).
  *Path Planning for Autonomous Vehicles in Unknown Semi-structured
  Environments.* Int. Journal of Robotics Research, 29(5).
  → Hybrid A* / state lattices: searching over `(x, y, heading)` with a vehicle
  motion model, as our global planner does. [Chapter 03](./03-global-planning-with-a-star.md).
- **[LaValle 2006]** — LaValle, S. M. (2006). *Planning Algorithms.* Cambridge
  University Press. (Free online.)
  → The reference text for configuration space, search, and sampling-based
  planning.

## Mapping & perception

- **[Elfes 1989]** — Elfes, A. (1989). *Using Occupancy Grids for Mobile Robot
  Perception and Navigation.* Computer, 22(6), 46–57.
  → The occupancy grid idea. [Chapter 02](./02-mapping-occupancy-and-distance-fields.md).
- **[Thrun 2005]** — Thrun, S., Burgard, W., Fox, D. (2005). *Probabilistic
  Robotics.* MIT Press.
  → Log-odds occupancy mapping, sensor models, and the probabilistic view of
  everything a robot believes. [Chapter 02](./02-mapping-occupancy-and-distance-fields.md).
- **[Felzenszwalb 2012]** — Felzenszwalb, P. F., Huttenlocher, D. P. (2012).
  *Distance Transforms of Sampled Functions.* Theory of Computing, 8, 415–428.
  → The linear-time Euclidean distance transform our distance field uses.
  [Chapter 02](./02-mapping-occupancy-and-distance-fields.md).

## Local planning & control

- **[Quinlan 1993]** — Quinlan, S., Khatib, O. (1993). *Elastic Bands: Connecting
  Path Planning and Control.* IEEE ICRA.
  → Deforming a global path into a smooth, clearance-aware local path — exactly
  what our local planner does. [Chapter 04](./04-local-planning-and-mpc.md).
- **[Coulter 1992]** — Coulter, R. C. (1992). *Implementation of the Pure Pursuit
  Path Tracking Algorithm.* CMU-RI-TR-92-01.
  → Geometric path following; we use a reverse variant in recovery.
  [Chapter 06](./06-recovery-and-backtracking.md).
- **[Fox 1997]** — Fox, D., Burgard, W., Thrun, S. (1997). *The Dynamic Window
  Approach to Collision Avoidance.* IEEE Robotics & Automation Magazine, 4(1).
  → The classic reactive local planner; a useful contrast to our MPC.
  [Chapter 04](./04-local-planning-and-mpc.md).
- **[Rawlings 2017]** — Rawlings, J. B., Mayne, D. Q., Diehl, M. M. (2017).
  *Model Predictive Control: Theory, Computation, and Design* (2nd ed.). Nob Hill.
  → The MPC textbook: receding horizon, cost design, stability. [Chapter 04](./04-local-planning-and-mpc.md).
- **[Mayne 2000]** — Mayne, D. Q., Rawlings, J. B., Rao, C. V., Scokaert, P. O. M.
  (2000). *Constrained model predictive control: Stability and optimality.*
  Automatica, 36(6), 789–814.
  → Why constrained MPC is well-posed. [Chapter 04](./04-local-planning-and-mpc.md).

## Optimization

- **[Boyd 2004]** — Boyd, S., Vandenberghe, L. (2004). *Convex Optimization.*
  Cambridge University Press. (Free online.)
  → Quadratic programs, convexity, duality — the language the MPC is written in.
  [Chapter 04](./04-local-planning-and-mpc.md).
- **[Stellato 2020]** — Stellato, B., Banjac, G., Goulart, P., Bemporad, A.,
  Boyd, S. (2020). *OSQP: An Operator Splitting Solver for Quadratic Programs.*
  Mathematical Programming Computation, 12(4).
  → The exact QP solver our MPC calls each control tick. [Chapter 04](./04-local-planning-and-mpc.md).

## Safety & recovery

- **[Ames 2019]** — Ames, A. D., Coogan, S., Egerstedt, M., Notomista, G.,
  Sreenath, K., Tabuada, P. (2019). *Control Barrier Functions: Theory and
  Applications.* European Control Conference (ECC).
  → The "filter an unsafe command into the nearest safe one" idea behind our
  safety shield. [Chapter 05](./05-the-safety-shield.md).
- **[Macenski 2020]** — Macenski, S., Martín, F., White, R., Ginés Clavero, J.
  (2020). *The Marathon 2: A Navigation System.* IEEE/RSJ IROS.
  → Nav2, whose recovery-behavior primitives we contrast with our backtracking
  recovery. [Chapter 06](./06-recovery-and-backtracking.md).

## Robotics foundations

- **[Siegwart 2011]** — Siegwart, R., Nourbakhsh, I. R., Scaramuzza, D. (2011).
  *Introduction to Autonomous Mobile Robots* (2nd ed.). MIT Press.
  → Differential-drive kinematics, sensors, coordinate frames. Good companion to
  [Chapter 01](./01-the-robot-and-its-senses.md).

---

◀ [tutorial index](./README.md)
