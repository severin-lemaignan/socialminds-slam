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
- **2× RGB-D cameras** (front + rear), mounted **near floor level** → clean person
  recognition/masking cannot be assumed; no strategy may critically rely on robust
  dynamics masking (ADR 0014). **IMU** up to ~1 kHz.
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
   Multi-sensor synthetic tests come in clean + noisy variants. **Parity gate:** the 3D
   pipeline must match the planar front-end — accuracy *and* compute — on the archived
   baseline (`eval/reference/baselines/m3-planar-frontend/`); benchmark every migration
   stage against it and justify any regression explicitly. — ADR 0010

8. **Map decay = contradiction-driven free-space carving** (ADR 0014): a beam passing
   through a voxel proves it empty — multiplicative weight decay, eviction below
   weight 1, **every active field** (an uncarved registration field collapses after
   ~1 min in a crowd: 114 m ATE measured; carved + odom prior: 0.90 m). **No
   time-based decay** (erodes unobserved geometry — measured 15× worse); frozen
   submaps stay immutable (filter-at-freeze instead); **masking is an enhancer,
   never a foundation** (floor-level cameras). — ADR 0014
9. **Dynamics masking = yolo11s-seg ONNX at depth ingest** (ADR 0015, from the
   survey in `docs/REPORT_HUMAN_DETECTION.md`): `slam-dynamics` runs it via `ort`
   (CPU EP default; TensorRT on the robot), dynamic class set, conf 0.2, dilated
   mask; pixels are rejected **before back-projection** (`PixelMask`, stamp-gated
   like colour). Feature-gated (`slam-replay --features dynamics`; `masking:` YAML /
   `--mask-model`) — ONNX Runtime never becomes a second mandatory C++ dep. Model
   committed in `onnx/` (20 MB; ⚠ AGPL weights, revisit before redistribution).
   The square 640 export loses recall vs a rect export at the camera's shape —
   the input shape is read from the model, so a rect re-export drops in. — ADR 0015

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
  **Caveat:** the `market` scenes were recorded on a different robot (Scrubber 75) and
  their bags carry **no 2D laser topic** — RGB-D/VIO + wheel `/odom` only; the harness
  probes the bag index and scan systems skip those sequences cleanly.
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
  `--bag` + either a YAML `--config` (ADR 0013, see `configs/`) or per-topic flags:
  scans/IMU/`--depth-topic` (+`--color-topic` for the coloured map)/`--odom-topic`;
  `--urdf` resolves multi-lidar frames/extrinsics, ADR 0009; `--metrics`,
  `--init-pose-from-tum`; A/B knobs `--no-loops`/`--no-graph`/`--loop-min-inliers`);
  `slam-bag2imu` / `slam-bag2scan` / `slam-bag2csv` (ROS1 bag → CSV, `--list`).
  `benchmark --direct-bag` skips CSV materialisation.
- `python -m harness.viz --openloris cafe1-1` — interactive scan/trajectory debugger
  (scans through estimated poses + ground truth; `--save` for headless PNG).
- **Live/progressive 3D viz (ADR 0011):** build with `--features viz`, then
  `slam-replay --rerun spawn` (live during the run) or `--rerun save:run.rrd`
  (timeline-scrubbable replay of the map building); viewer via `pip install rerun-sdk`.
  `--map-out FILE` dumps raw TSDF voxels (STSD binary) headlessly. `--rig-from-bag`
  builds the rig from a bag's `/tf_static` (e.g. OpenLORIS: IMUs ride the cameras).

## Status
**M0 done** (harness + trivial baselines). **M1 done**: EuRoC + OpenLORIS(IMU) adapters,
fetch/cache, compute metrics, one-command report (real-data flags: `--euroc`/`--openloris`),
reference scoring. **Ground-0 baseline archived** (`eval/reference/baselines/ground0/`):
trivial baselines on MH_01_easy + cafe1-1 — the floor to beat. Remaining M1 operator step:
run RTAB-Map/GLIM on the robot and archive the baseline. **M2 done**:
`slam-gtsam-sys` + `slam-backend` (pose graphs, IMU preintegration, instrumented LM
solves); GitLab CI green with the vendored GTSAM (CPU-only, cached after the first run).
CI's synthetic self-test now also gates `odom_dead_reckoning` (wheel-odometry replay,
ADR 0012 floor) and `scan_matching_3d` on generated scans+odometry — the front-end runs
on every pipeline.
**M3 largely done**: planar PLICP front-end (ADR 0007, **ATE 0.090/0.066 m** on
cafe1-1/-2, archived as the **parity gate**) superseded by the full-3D pipeline
(ADR 0010 stages 1–3b): IMU attitude + tilt-compensated 3D fans + ICP degeneracy guard,
`slam-map` (TSDF trait + Rust sparse backend), scan-to-submap registration. **Beats the
parity gate: ATE 0.039/0.055 m, p99 0.9 ms, 53x real-time** (cafe1-1/-2). Multi-lidar
rig (ADR 0009: URDF/`tf_static`, frame-tagged measurements); **RGB-D depth registration**
(range-adaptive sampling, separate 5 cm 3D field — market1-1 tracks at ATE 4.38 m ≈
paper's wheel-odom baseline); **wheel-odometry motion prior** (ADR 0012 IMU-less
contract: depth-only cafe1-1 2.8 → 0.456 m); YAML run configs (ADR 0013). Depth→pose
stays gated behind dynamics masking when scans are present (people dominate — measured).
**M4 in progress**: geometrically verified, modality-aware loop closure against frozen
anchor-relative submaps + **GTSAM pose graph wired** (stage 3b; optimise on every
verified loop, anchors re-posed, voxels never rewritten). Rerun viz shows the coloured
3D map (CIELAB a*b*, illumination-invariant), true-size voxel cubes, per-submap TSDF
entities. Map ghosts from unmasked people are evicted by **free-space carving**
(ADR 0014; 98.7 % stale-ghost eviction measured, CI-gated, maskless). **Dynamics
masking integrated** (ADR 0015, 2026-06-11): `slam-dynamics` + ingest-side
`PixelMask` rejection + `--features dynamics` replay wiring + CI smoke test;
remaining: A/B-measure it on cached real data, then unlock the gated depth bridges
(`depth_updates_pose`, `reg_band_tolerance`). **Next (top blockers): measure
masking A/B on the depth path, per-submap appearance signatures (corridor
aliasing + re-localization), OpenVDB backend.**
Open work lives in [`docs/ROADMAP.md`](docs/ROADMAP.md) (per-milestone checklists —
the former TODO.md is folded in).
