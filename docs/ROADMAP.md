# Roadmap

Milestones are ordered to **de-risk first**: get a measurable, reproducible pipeline
working before writing novel SLAM, then add capability one verifiable layer at a time.
Each milestone defines its own success metric so progress is never a matter of opinion.

Legend: ☐ todo · ◐ in progress · ☑ done

---

## M0 — Harness & trivial baselines (the vertical slice) — ✅ done
**Goal:** prove the whole dev/test loop works end-to-end on any CPU-only machine.

- ☑ Architecture docs + ADRs (0001–0005), `CLAUDE.md` project memory
- ☑ Rust workspace; `slam-types` (time, SE(3), IMU, trajectories, TUM I/O)
- ☑ Trivial baselines: stationary/identity, IMU dead-reckoning; `slam-replay` CLI
- ☑ CPU-only eval harness: synthetic generator, `evo` ATE/RPE, gated self-test
- ☑ CI (GitLab): fmt/clippy/test + synthetic end-to-end benchmark, all GPU-free

**Done:** `cargo test` green; the harness synthesises a trajectory, runs both baselines,
and gates on ATE/RPE with no GPU and no download. Measured: stationary ATE 2.23 m vs
dead-reckoning 0.028 m (RPE@1m 3.8 mm) — dead-reckoning beats the floor and stays bounded.
*(Compute metrics — latency/CPU/RAM/real-time-factor — deferred to M1 alongside real data.)*

## M1 — Real datasets & reference baseline — ◐ largely done
**Goal:** a "number to beat" on data that looks like the robot.

- ☑ Dataset adapters: **EuRoC** (real IMU+GT) and **OpenLORIS-Scene** via the in-house
  indexed ROS1 bag reader (ADR 0008, no ROS install): IMU, laser scans, depth+colour
  images, wheel odometry, `tf_static`. TUM RGB-D / Bonn Dynamic arrive with dynamics
  work (M5).
- ☑ Download + cache build steps (`make data-euroc|data-openloris|data-openloris-gt`).
- ☑ Compute metrics (latency p50/p95/p99, throughput, real-time factor, peak RSS).
- ☑ One-command benchmark report (accuracy + compute, mean±std → JSON + Markdown);
  emitted as a CI artifact.
- ☑ Reference-baseline **scaffolding**: external-trajectory scoring (`harness.score`),
  a Dockerised RTAB-Map runner skeleton, and an archive format under `eval/reference/`.
- ☐ **Actually run** RTAB-Map/GLIM on OpenLORIS and archive the numbers — an operator
  step on a ROS/GPU machine (not CI); blocked only on hardware/time, not code.

**Status:** the harness can score any system (ours or a reference) on real data and
produce a reproducible accuracy+compute report. Remaining: execute the reference system
on the robot/workstation and commit its baseline JSON. Full RGB-D/lidar dataset use
arrives with the front-ends (M3).

## M2 — IMU preintegration & the backend — ◐ largely done
**Goal:** the factor graph exists and is exercised.

- ☑ `slam-gtsam-sys` (cxx shim over vendored GTSAM 4.3a1, static + Boost-free — ADR 0006)
  + safe `slam-backend` wrapper (typed keys, pose priors/betweens, LM optimisation,
  per-solve instrumentation: initial/final error, iterations, wall time)
- ☑ IMU preintegration factors (`ImuPreintegrator` + `ImuFactor`); pose-graph optimisation path
- ☑ Backend unit/integration tests (square-loop pose graphs incl. loop closure on biased
  odometry; preintegration vs. analytic motion; IMU factor recovers known motion)
- ☑ **Wired into the live pipeline** (ADR 0010 stage 3b): pose graph over
  anchor-relative submaps, optimised on every verified loop closure; the `AnchorGraph`
  seam keeps the front-end C++-free.
- ☐ Green CI run with GTSAM built CPU-only on the GitLab runner (config landed; first
  pipeline pays the GTSAM compile, then it is cached)

**Done when:** a synthetic pose graph with loop constraints optimises to the known
ground truth within tolerance, in CI (GTSAM built CPU-only).

## M3 — Front-ends: 3D scan/depth registration against TSDF submaps — ◐ largely done
**Goal:** our own odometry, beating dead-reckoning on real data.

- ☑ 2D planar scan-matching front-end (the planar backbone): trimmed point-to-line ICP,
  scan-to-keyframe (`slam-frontend-scan`, ADR 0007). **Measured on OpenLORIS cafe1-1:
  ATE RMSE 0.090 m** (floor baselines: 33 m stationary, 6.4 km IMU dead-reckoning);
  archived as the **parity gate** (`eval/reference/baselines/m3-planar-frontend/`).
- ☑ Multi-sensor rig (ADR 0009): URDF / bag `tf_static`, frame-tagged measurements,
  multi-lidar fusion through per-sensor extrinsics; YAML run configs (ADR 0013).
- ☑ **Full-3D migration** (ADR 0010 stages 1–3b): IMU attitude → tilt-compensated 3D
  fans, scan-to-submap registration against narrow-band TSDFs with an ICP degeneracy
  guard, anchor-relative submaps + GTSAM pose graph. **Beats the planar parity gate:
  ATE 0.039/0.055 m on cafe1-1/-2 (gate: 0.090/0.066) at 53× real time, p99 0.9 ms.**
- ☑ RGB-D depth registration (the 3D-source path): range-adaptive sampled clouds
  register against a separate coarser 3D field; wheel-odometry motion prior
  (`/odom`, ADR 0012 — the robot ships IMU-less; measured no-IMU cost ≈ 4 cm on
  cafe1-1, `configs/no-imu.yaml`). Depth+odom tracks scan-less
  sequences (market1-1: ATE 4.38 m ≈ the paper's wheel-odom baseline; cafe1-1
  depth-only 0.456 m). Pose updates from depth stay **gated off behind dynamics
  masking** when scans are present (people dominate the error — measured).
- ☐ Hybrid per-point fan registration (ADR 0010 refinement): laser fans register
  against the 3D field where trilinear stencils are complete (camera-covered
  regions), 2D-field fallback elsewhere; the 2D field fades as RGB-D coverage grows.
- ☐ Odometry as a graph factor and as a standalone baseline in the harness.
- ☐ CameraInfo distortion is currently ignored (OpenLORIS aligned depth is rectified,
  so correct there) — handle D for raw/unrectified streams or the robot's cameras.
- ☐ Visual (feature/photometric) front-end — deliberately deferred; illumination
  variance is the hard part, and lifelong-SLAM evidence favours masking + geometry
  (and learned features over raw RGB) first.

**Done when:** the combined front-end beats the IMU baseline and approaches the reference
on OpenLORIS/TUM RPE.

## M4 — Loop closure (the top requirement) — ◐ in progress
**Goal:** globally consistent maps in repetitive corridors.

- ☑ Geometrically verified loop closure against frozen submaps (seed-grid
  re-registration, inlier-gated), per modality (laser fans → 2D field, depth clouds
  → 3D field); corrections distributed by the GTSAM pose graph (39 verified
  closures on cafe1-1). Proximity-gated for now. Known niche gap: loops during the
  submap hand-over overlap window still snap instead of optimising.
- ☐ MapClosures-style per-submap appearance signatures (replaces proximity gating;
  the corridor-aliasing defence) + **stage 4: re-localization service** — < 1 s
  cold-start / tracking-loss localization over frozen-submap signatures, scored
  with the OpenLORIS lifelong protocol (CR, CS-R; `eval/reference/sota/`)
- ☐ Depth loop-closure basin: loop seeds must land inside the 3D field's truncation
  (15 cm) — a coarse-to-fine seed pyramid (or scan-context-style pre-alignment)
  would decouple verification from the field's voxel size and let a finer field
  recover near-range accuracy (measured trade-off recorded in `scan_to_map.rs`).
- ☐ Optional learned visual VPR for corridor disambiguation; robust back-end kernels
- ☐ Loop-closure eval: ATE with/without; detector precision/recall on corridor sequences

**Done when:** loop closure cuts ATE sharply on revisits with **zero** map-corrupting false
positives on the corridor stress sequences.

## M5 — Dense map & dynamics
**Goal:** a usable 3D navigation map that ignores transient objects.

- ◐ Map substrate: `TsdfMap` trait + pure-Rust sparse narrow-band TSDF landed
  (`slam-map`, the CI/default backend); still ☐: **stage 5 — system-OpenVDB
  backend** (`libopenvdb-dev` 10.x, feature-gated `cxx` shim; ADR 0010) with a
  conformance suite vs `SparseTsdf` and in-process grid hand-over to reMap;
  GPU TSDF/ESDF
- ☐ Dynamic masking (YOLO-seg + flow/depth propagation; CPU EP fallback) + occupancy
  decay — **the top accuracy blocker**: three independent measurements say un-masked
  people dominate the error (depth-only odometry 2.8 m ATE on cafe1-1; depth→pose
  fusion 0.16→3.0; laser-band depth contribution 0.164→0.357). Unlocks the two gated
  bridges (`depth_updates_pose`, `reg_band_tolerance`).
- ☐ Clean-3D-map: depth outlier filtering, post-hoc model filtering, compaction →
  a *compact* map suitable for downstream tasks (semantic segmentation); plan
  drafted in [clean-map-plan.md](clean-map-plan.md)
- ☐ Voxel colour channel: coloured clouds + rerun a\*b\* display landed; remaining is
  per-voxel chroma accumulation — quantized CIELAB (a\*, b\*), 2 B/voxel, surface
  voxels only, config-gated so depth-only memory stays at 8 B/voxel (a\*b\* is
  perceptually uniform, so weighted averaging is well-behaved). Then colour
  `world/tsdf` cubes from the voxel channel instead of height. Implement together
  with dynamics masking (same colour-image decode path); useful for reMap.
- ☐ Half-float TSDF voxels (halves map memory; ADR 0010 budget headroom)
- ☐ Map-quality eval vs. ground-truth meshes; dynamic-vs-static ATE deltas

**Done when:** dynamic sequences (TUM `walking_*`, Bonn) show minimal ATE degradation and
moving-object contamination of the static map is below threshold.

## Beyond
ROS 2 node (rclrs) + zero-copy bindings (pyo3/cxx); on-robot integration; lifelong /
multi-session mapping; hard-real-time hardening (thread pinning, pre-allocation).

---

### Cross-cutting, always-on
- Every change keeps CI green and is measured by the harness ([ADR 0005](adr/0005-evaluation-methodology.md)).
- Every GPU path has a tested CPU fallback ([ADR 0003](adr/0003-gpu-optional-cpu-fallback.md)).
- Every architectural decision gets an ADR.

### Evaluation & test-data debt
- ☐ Synthetic depth-camera scenario (raycast 2.5D world → depth images/clouds):
  CI coverage for the depth path, clean + noisy variants.
- ☐ Python synthetic generator two-lidar mode (ADR 0009 noise-suite item; the Rust
  raycast harness already covers CI — optional completeness).
- ☐ First green GitLab CI with the vendored GTSAM build (M2 carry-over).
