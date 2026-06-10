# Project memory — socialminds-slam

Repo-committed context so any machine / any session starts aligned. Keep this current
when decisions change. Authoritative detail lives in [`docs/`](docs/); this is the
fast briefing.

## What we're building
A from-scratch, heavily multi-threaded, **real-time fully-3D SLAM engine** for an indoor
mobile robot. Also an explicit **software-engineering showcase**: every architectural
decision is an ADR, every performance claim is a reproducible benchmark, extensive tests.
Top priority is **loop closure / global consistency**. Full requirements in
[`REQUIREMENTS.md`](REQUIREMENTS.md).

## The robot (drives everything)
- Omnidirectional base ~50×70 cm, ≤ 2 m/s, indoor.
- **2× 2D laser scanners** (opposite corners) — **planar, NOT 3D lidar**.
- **2× RGB-D cameras** (front + rear). **IMU** up to ~1 kHz.
- On-board: 24+ core CPU, **RTX 5060 8 GB shared** GPU.
- Environment: feature-rich but **repetitive corridors** (perceptual aliasing) and
  **dynamic** (people, doors, chairs; people occlude cameras).

## Locked decisions (see docs/adr/)
1. **Rust core + wrapped GTSAM** optimiser via a thin `cxx`/C-ABI shim. Rust is the
   zero-copy hub to Python (`pyo3`) and C++ (`cxx`). GTSAM is the *only* mandatory C++
   dep; isolated behind `slam-backend` so it can be swapped for pure-Rust later. — ADR 0001
   GTSAM is **vendored**: pinned shallow submodule `third_party/gtsam` (tag 4.3a1), built
   static + Boost-free by `slam-gtsam-sys/build.rs` (needs only C++17 + CMake; first build
   is slow, then cached; `SLAM_GTSAM_PREFIX` skips it). Clone with `--recursive`. — ADR 0006
2. **Sensor roles:** the **3D map comes from RGB-D + IMU**; the **2D lidars are the
   planar backbone + the primary loop-closure sensor** (clean corridor geometry). Do NOT
   try to run 3D-lidar LIO (FAST-LIO2/GLIM) on the 2D scans. — ADR 0002
3. **GPU is optional, feature-gated; CPU fallback is the default and CI is CPU-only.**
   Dev/test/benchmark must run on any GPU-less, ROS-less machine. Every GPU kernel has a
   tested CPU counterpart. — ADR 0003
4. **Map = a `Map` trait with multiple backends:** GPU TSDF/ESDF (+ CPU TSDF fallback)
   for the Nav2 navigation map, **and** an OpenVDB layer to interop with **reMap**
   (external VDB-based geometric world model). Both share optimised poses + occupancy
   decay. — ADR 0004
5. **Evaluation-first:** harness + trivial baselines (stationary, IMU dead-reckoning)
   before novel algorithms. TUM trajectory format; `evo` for ATE/RPE; run N× → mean±std;
   loop closure judged by ATE with/without + detector precision/recall. — ADR 0005
6. **Sensor rig from the robot's URDF** (read directly, no ROS runtime; `slam-rig`);
   intrinsics from CameraInfo; measurements self-identify via `header.frame_id`; ROS
   timestamps are the single time base. No bespoke calibration format. — ADR 0009
7. **Full-3D state + TSDF submap registration.** Base is a 3D body (scan plane tilts —
   IMU attitude is a front-end prerequisite); scans register as 3D fans against
   per-submap narrow-band TSDFs (~5 cm voxels, submaps re-posed by the pose graph, never
   voxel rewrites; 100×100×12 m target, 1–2 GB RAM budget). **Dual map backend** behind
   the `Map` trait: pure-Rust sparse grid (default, CI) + feature-gated **system**
   OpenVDB 10.x (`libopenvdb-dev`, NOT vendored) for the robot and in-process reMap
   sharing. **Re-localization < 1 s, verified** (per-submap MapClosures signatures).
   Multi-sensor synthetic tests come in clean + noisy variants. — ADR 0010

## Approach
**Write the novel core ourselves** (orchestration, fusion, front-ends, map); **reuse the
hard-solved bits** (GTSAM optimiser, ONNX/YOLO segmentation, the MapClosures algorithm).
Lock-free stage pipeline (crossbeam) + rayon; no stage blocks on a slower one (drop frames).

## Loop closure & dynamics (the two hard problems)
- **Corridor aliasing:** MapClosures-style detection (prunes self-similar structure),
  **always geometrically verified**, lidar-geometry-gated, robust back-end kernels. Never
  trust a descriptor alone.
- **Dynamics/people:** YOLO-seg + optical-flow/depth **mask propagation** (real-time,
  CPU-capable), mask classes *before* feature extraction; **occupancy decay** evicts
  transient objects from the map. Avoid per-frame Mask R-CNN (too slow).

## Datasets & tooling
- CI: zero-download **synthetic** generator.
- **OpenLORIS-Scene** (the robot's twin) is freely hosted on **Hugging Face** as **ROS1
  bags** (`shixuesong/openloris-scene`); read in Rust via the **`rosbag` crate** in
  `slam-datasets` (`slam-bag2imu`) — no ROS install. Ground truth ships as TUM already.
  **EuRoC** (the first runnable real-data IMU benchmark) is now hosted on the **ETH
  Research Collection**, downloaded as one zip **per collection** (e.g. `machine_hall.zip`
  bundles MH_01..05) by bitstream UUID. Then **TUM RGB-D `fr3/walking_*`**, **Bonn
  Dynamic** once the visual front-end exists.
- Datasets are downloaded+cached via `make data-euroc|data-openloris|data-openloris-gt`
  into `$SLAM_DATA_DIR` (default `./data`, git-ignored). Big bags never touch CI.
- Metrics via **`evo`**; replay/regression fixtures via **MCAP** bags on the ROS side.

## Repo layout & workflow
- `crates/` Rust workspace (middleware-independent core) · `docs/` (+ `docs/adr/`) ·
  `eval/` CPU-only Python harness.
- **Commit hygiene:** small, logically-grouped commits with clear messages.
- **Remotes:** `origin` → GitLab (`gitlab.iiia.csic.es`) is the real one **where CI runs**;
  `github` is a mirror that receives commits via the user's own tooling (auto-push is
  expected). The M0 bootstrap landed directly on `main`.
- **CI runs on GitLab** (`.gitlab-ci.yml`), CPU-only: fmt + clippy(-D warnings) + tests,
  then the gated `eval` self-test benchmark. There is no GitHub Actions.
- **Always:** keep CI green, measure changes with the harness, give every GPU path a CPU
  fallback, write an ADR for every architectural decision.

## Benchmarking entrypoints
- `make test` (Rust + pytest) · `make bench` (gated self-test) · `make help`.
- `python -m harness.benchmark` → `eval/results/{report.md,results.json}` (ATE/RPE +
  real-time factor / latency p99 / peak RSS, mean±std). `--euroc SEQ` / `--openloris SEQ`
  (repeatable) benchmark cached real data — locate-only, never download; `--synthetic`
  adds the synthetic sequence; `--init-pose-from-gt` seeds runs with the ground-truth
  initial pose. OpenLORIS split gyro/accel topics are merged by the adapter.
- `python -m harness.score …` → score an external reference (RTAB-Map/GLIM) trajectory.
- `slam-replay` (run a system over `--imu`/`--scan` CSVs or straight from a ROS1 bag via
  `--bag` + topic flags; `--urdf` resolves multi-lidar frames/extrinsics, ADR 0009;
  `--metrics`, `--init-pose-from-tum`); `slam-bag2imu` / `slam-bag2scan` / `slam-bag2csv`
  (ROS1 bag → CSV, `--list`). `benchmark --direct-bag` skips CSV materialisation.
- `python -m harness.viz --openloris cafe1-1` — interactive scan/trajectory debugger
  (scans through estimated poses + ground truth; `--save` for headless PNG).

## Status
**M0 done** (harness + trivial baselines). **M1 done**: EuRoC + OpenLORIS(IMU) adapters,
fetch/cache, compute metrics, one-command report (real-data flags: `--euroc`/`--openloris`),
reference scoring. **Ground-0 baseline archived** (`eval/reference/baselines/ground0/`):
trivial baselines on MH_01_easy + cafe1-1 — the floor to beat. Remaining M1 operator step:
run RTAB-Map/GLIM on the robot and archive the baseline. **M2 largely done**:
`slam-gtsam-sys` + `slam-backend` (pose graphs, IMU preintegration, instrumented LM solves,
synthetic-graph tests green locally); awaiting first green CI with the vendored GTSAM.
**M3 in progress**: 2D scan-matching front-end done (`slam-frontend-scan`, PLICP, ADR 0007)
— **ATE 0.090 m on cafe1-1, 0.066 m on cafe1-2** (vs 0.251 m best published camera-based,
`eval/reference/sota/`; caveat: OpenLORIS GT is itself laser-based). **Sensor rig landed**
(ADR 0009): `slam-rig` reads the robot's URDF, measurements are frame-tagged, the scan
front-end fuses **multiple lidars** through per-sensor extrinsics into one shared pose
(dual-lidar raycast harness + mock URDF in `slam-frontend-scan/tests/`); next
scan-to-local-map, RGB-D-inertial, then fusion. See [`docs/ROADMAP.md`](docs/ROADMAP.md).
