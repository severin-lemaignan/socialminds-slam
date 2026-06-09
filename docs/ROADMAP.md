# Roadmap

Milestones are ordered to **de-risk first**: get a measurable, reproducible pipeline
working before writing novel SLAM, then add capability one verifiable layer at a time.
Each milestone defines its own success metric so progress is never a matter of opinion.

Legend: ☐ todo · ◐ in progress · ☑ done

---

## M0 — Harness & trivial baselines (the vertical slice)
**Goal:** prove the whole dev/test loop works end-to-end on any CPU-only machine.

- ☑ Architecture docs + ADRs (0001–0005), `CLAUDE.md` project memory
- ◐ Rust workspace; `slam-types` (time, SE(3), IMU, trajectories, TUM I/O)
- ◐ Trivial baselines: stationary/identity, IMU dead-reckoning; `slam-replay` CLI
- ◐ CPU-only eval harness: synthetic generator, `evo` ATE/RPE, compute metrics, gates
- ☐ CI: fmt/clippy/test + synthetic end-to-end benchmark, all GPU-free

**Done when:** `cargo test` is green and the harness synthesises a trajectory, runs a
baseline, and reports ATE/RPE — in CI, no GPU, no download. IMU dead-reckoning visibly
beats stationary on a moving sequence; both are bounded on a static one.

## M1 — Real datasets & reference baseline
**Goal:** a "number to beat" on data that looks like the robot.

- ☐ Dataset adapters: OpenLORIS-Scene (primary), TUM RGB-D `fr3/walking_*`
- ☐ Reference baseline wired in (RTAB-Map or GLIM fed RGB-D) with archived metrics
- ☐ Benchmark report generation (accuracy + compute) reproducible from one command

**Done when:** we can report ATE/RPE + compute for a reference system on OpenLORIS and
TUM, reproducibly, and the trivial baselines sit below it as expected.

## M2 — IMU preintegration & the backend
**Goal:** the factor graph exists and is exercised.

- ☐ `slam-gtsam-sys` (cxx shim) + safe `slam-backend` wrapper
- ☐ IMU preintegration factors; a pose-graph optimisation path
- ☐ Backend unit/integration tests (synthetic graphs with known solutions)

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
