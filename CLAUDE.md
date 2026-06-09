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
  **EuRoC** is the first runnable real-data IMU benchmark (small CSVs). Then **TUM RGB-D
  `fr3/walking_*`**, **Bonn Dynamic** once the visual front-end exists.
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

## Status
Bootstrapping **M0** (harness + trivial baselines). See [`docs/ROADMAP.md`](docs/ROADMAP.md).
