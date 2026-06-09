# ADR 0007: Front-end order — 2D scan matching first, as point-to-line ICP

- **Status:** accepted
- **Date:** 2026-06-09
- **Deciders:** Séverin Lemaignan

## Context

M3 builds our own odometry: an RGB-D-inertial front-end, a 2D planar scan-matching
front-end, and their fusion into the backend (ADR 0002 fixes the *roles*: RGB-D+IMU make
the 3D map, the 2D lidars are the planar backbone and primary loop-closure sensor). Open
questions: which front-end to build first, and which scan-matching algorithm.

Facts on the ground:

- The loop-closure pipeline (M4, the top requirement) consumes **lidar geometry**; the
  scan front-end therefore de-risks the critical path, the visual front-end doesn't.
- OpenLORIS bags (the robot's twin) already carry `/scan` (`sensor_msgs/LaserScan`), and
  the IMU extraction machinery (`slam-datasets`, ROS1 wire decoding) extends naturally.
- A scan matcher is an order of magnitude less code than visual odometry, needs no new
  heavy dependency, and is testable against analytically known synthetic environments.

## Decision

1. **Build order: 2D scan matching → RGB-D-inertial → fusion.** Each lands as its own
   `SlamSystem` runnable by `slam-replay`, benchmarked the day it exists.
2. **Algorithm: point-to-line ICP (PLICP, Censi 2008), scan-to-keyframe.**
   - Point-to-line converges quadratically on the straight-wall geometry of indoor
     corridors (vs. point-to-point's linear rate) and is the standard planar matcher.
   - **Scan-to-keyframe**, not scan-to-scan: matching against a held keyframe (renewed
     after ~0.3 m / ~0.3 rad of motion) suppresses the per-pair random-walk drift.
   - Robustness: max-correspondence-distance gate + **trimmed** least squares (drop the
     worst residuals each iteration) — dynamic objects (people) shed as outliers, per the
     dynamics strategy; no semantic masking needed at this layer.
   - Nearest neighbours via a k-d tree (`kiddo`, per the ADR 0001 survey).
3. **The front-end estimates SE(2)**, embedded into SE(3) (z = 0, roll = pitch = 0) at
   the output boundary: the lidars are planar (ADR 0002); pretending otherwise would
   manufacture fake 3D information. The RGB-D front-end owns out-of-plane motion.
4. **Pure Rust, in `slam-frontend-scan`;** no C++ involved.

## Consequences

- **Easier:** the M4 loop-closure work starts from a proven, measured planar matcher; the
  harness gains its first real-data win condition ("beats IMU dead-reckoning on
  OpenLORIS").
- **Harder / deferred:** sequences without a lidar (EuRoC, TUM RGB-D) get nothing from
  this front-end — acceptable, they are the visual front-end's benchmark. Multi-lidar
  merging (the robot has two) is deferred until data with two scanners exists.
- **Risk accepted:** PLICP can converge to a wrong local minimum in degenerate geometry
  (long featureless corridor → unconstrained along-track direction). This is inherent to
  planar matching; the fusion layer (IMU + RGB-D) is the mitigation, and the matcher
  reports a health signal (matched fraction + residual) so downstream can de-weight it.

## Alternatives considered

- **Visual front-end first:** exercises the harder problem sooner, but delays the
  loop-closure-critical lidar path and needs camera intrinsics/extrinsics plumbing,
  feature extraction, and depth handling before the first number. Rejected for ordering.
- **Correlative / branch-and-bound matching (Cartographer-style):** robust to bad
  initial guesses, but heavier per scan and mainly valuable for loop *closure* search —
  which M4 covers via MapClosures-style detection. Sequential odometry has good initial
  guesses; PLICP is cheaper and more precise there.
- **NDT (normal distributions transform):** comparable accuracy, smoother cost surface,
  but more tuning (grid resolution) and no clear win on corridor geometry. PLICP's
  point-to-line metric matches walls directly.
- **Wrap an existing matcher (csm/cartographer):** against the project's purpose — the
  front-ends are exactly the "novel core we write ourselves" (CLAUDE.md); only
  hard-solved *solver/detector* layers are wrapped.
