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

- ☑ Dataset adapters: **EuRoC** (real IMU+GT, runnable today) and **OpenLORIS-Scene**
  IMU path via the Rust `rosbag` reader (`slam-datasets`/`slam-bag2imu`). TUM RGB-D +
  OpenLORIS RGB-D/lidar adapters are stubbed pending the visual/lidar front-ends (M3+).
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

## M2 — IMU preintegration & the backend — ◐ in progress
**Goal:** the factor graph exists and is exercised.

- ☑ `slam-gtsam-sys` (cxx shim over vendored GTSAM 4.3a1, static + Boost-free — ADR 0006)
  + safe `slam-backend` wrapper (typed keys, pose priors/betweens, LM optimisation,
  per-solve instrumentation: initial/final error, iterations, wall time)
- ☑ IMU preintegration factors (`ImuPreintegrator` + `ImuFactor`); pose-graph optimisation path
- ☑ Backend unit/integration tests (square-loop pose graphs incl. loop closure on biased
  odometry; preintegration vs. analytic motion; IMU factor recovers known motion)
- ☐ Green CI run with GTSAM built CPU-only on the GitLab runner (config landed; first
  pipeline pays the GTSAM compile, then it is cached)

**Done when:** a synthetic pose graph with loop constraints optimises to the known
ground truth within tolerance, in CI (GTSAM built CPU-only).

## M3 — Front-ends: RGB-D-inertial odometry + 2D planar scan-matching
**Goal:** our own odometry, beating dead-reckoning on real data.

- ☐ RGB-D-inertial odometry front-end (the 3D source)
- ☐ 2D planar scan-matching front-end (the planar backbone)
- ☐ Fusion of both + IMU into the backend

**Done when:** the combined front-end beats the IMU baseline and approaches the reference
on OpenLORIS/TUM RPE.

## M4 — Loop closure (the top requirement)
**Goal:** globally consistent maps in repetitive corridors.

- ☐ MapClosures-style detection + mandatory geometric verification
- ☐ Optional learned visual VPR for corridor disambiguation; robust back-end kernels
- ☐ Loop-closure eval: ATE with/without; detector precision/recall on corridor sequences

**Done when:** loop closure cuts ATE sharply on revisits with **zero** map-corrupting false
positives on the corridor stress sequences.

## M5 — Dense map & dynamics
**Goal:** a usable 3D navigation map that ignores transient objects.

- ☐ `Map` trait + CPU TSDF fallback + GPU TSDF/ESDF; OpenVDB layer for reMap
- ☐ Dynamic masking (YOLO-seg + flow/depth propagation; CPU EP fallback) + occupancy decay
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
