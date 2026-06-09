# ADR 0002: Sensor roles — RGB-D + IMU build the 3D map; 2D lidars are the planar/loop-closure backbone

- **Status:** accepted
- **Date:** 2026-06-09
- **Deciders:** Séverin Lemaignan

## Context

The robot carries two 2D laser scanners (opposite corners), two RGB-D cameras
(front + rear), and an IMU. The requirement is a **fully 3D** map with **excellent loop
closure**, indoors, amid dynamics and repetitive corridors.

The instinct on a lidar-equipped robot is a lidar-centric LIO pipeline
(FAST-LIO2 / Point-LIO / GLIM). But those assume a **3D** lidar producing a full point
cloud per scan. **Two fixed 2D scanners sample only horizontal slices** and cannot build
a 3D map on their own. Conversely, 2D lidars are long-range, low-noise, and give clean,
distinctive, drift-resistant planar geometry — the best possible loop-closure signal in
corridors where RGB-D is noisy and visually aliased.

## Decision

Assign sensors to the roles they are actually good at:

- **3D map + odometry source: RGB-D (front + rear) + IMU.** This is where "fully 3D"
  output comes from. IMU is tightly coupled for de-skew and motion prediction (≈10 cm of
  motion per RGB-D frame at 2 m/s makes this non-optional).
- **2D lidars: the precise planar backbone and the primary loop-closure sensor.** They
  contribute planar registration factors and the most reliable place signatures. Their
  inter-lidar extrinsics and time-sync to the IMU are treated as make-or-break and
  calibrated/verified explicitly.
- **Loop closure:** MapClosures-style detection (CPU-only, prunes self-similar structure
  → resists corridor aliasing), **always** geometrically verified before acceptance,
  optionally augmented with learned visual VPR to disambiguate geometrically-identical
  corridors. Backend uses robust kernels so a stray false loop cannot corrupt the graph.
- **Dynamics:** detection + mask-propagation (YOLO-seg + optical-flow/depth) masks
  people/chairs/doors *before* they enter odometry and the map; the map uses occupancy
  decay to evict transient objects. Per-frame heavy segmentation (Mask R-CNN) is avoided.
- **All factors** (IMU, RGB-D, lidar planar, inter-lidar, loop) are fused in **one factor
  graph** ([ADR 0001](0001-language-and-optimizer.md)).

## Consequences

- **Easier:** plays to each sensor's strength; gives a credible path to robust corridor
  loop closure, the top requirement.
- **Harder:** RGB-D-inertial odometry (rather than turnkey LIO) is the front-end we must
  build; multi-sensor extrinsic calibration and time-sync become explicit subsystems.
- **Risk:** corridor perceptual aliasing (false loops). Mitigated by self-similarity
  pruning + mandatory geometric verification + robust back-end kernels + lidar-geometry
  gating of visually-proposed loops.
- **Revisit when:** the platform gains a 3D lidar, or the 2D scanners are remounted to a
  nodding/rotating rig — either would re-open a lidar-centric front-end.

## Alternatives considered

- **Lidar-centric LIO (FAST-LIO2/GLIM) on the 2D scans:** not applicable — those need 3D
  clouds. Feeding GLIM the RGB-D clouds is viable and is a candidate *reference baseline*
  ([ADR 0005](0005-evaluation-methodology.md)), but not our 2D-scan strategy.
- **RGB-D only, ignore the lidars:** discards the best corridor loop-closure cue and the
  most accurate planar geometry. Rejected.
