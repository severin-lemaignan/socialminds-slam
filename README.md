# socialminds-slam

A from-scratch, real-time, fully-3D SLAM engine for an indoor mobile robot.

> **Status:** the 2D-lidar front-end is live — tilt-compensated 3D scan fans registered
> against TSDF submaps, multi-sensor rigs from URDF / `tf_static`, verified geometric
> loop closure — benchmarked against the OpenLORIS-Scene dataset and the published
> state of the art (see the [roadmap](docs/ROADMAP.md)). RGB-D-inertial is next.

## What this is

A heavily multi-threaded SLAM engine targeting an omnidirectional indoor robot
(≤ 2 m/s) equipped with **two 2D laser scanners**, **two RGB-D cameras**
(front + rear) and an **IMU**. The engine is middleware-independent at its core
but is designed for zero-copy integration into a ROS 2 stack.

Design priorities, in order:

1. **Loop closure / global consistency** — the single most important requirement.
2. **Real-time on the robot** — 24+ core CPU, optional GPU (RTX 5060, 8 GB).
3. **Robustness to dynamics** — people, doors, chairs; repetitive corridors.
4. **Reproducibility & test coverage** — this project is also a software-engineering
   showcase: every architectural decision is recorded as an
   [ADR](docs/adr/), and every performance claim is backed by a reproducible
   benchmark.

## Key design decisions (the short version)

| Decision | Choice | ADR |
|---|---|---|
| Core language | **Rust** | [0001](docs/adr/0001-language-and-optimizer.md) |
| Factor-graph optimizer | **Wrap GTSAM** (Rust core, thin C-ABI shim) | [0001](docs/adr/0001-language-and-optimizer.md) |
| Source of the 3D map | **RGB-D + IMU** (2D lidars = planar backbone + loop closure) | [0002](docs/adr/0002-sensor-roles-and-pipeline.md) |
| GPU | **Optional, feature-gated; CPU fallback is the default** | [0003](docs/adr/0003-gpu-optional-cpu-fallback.md) |
| Map representation | **`Map` trait, multiple backends** (GPU TSDF/ESDF + OpenVDB) | [0004](docs/adr/0004-map-representation.md) |
| First milestone | **Eval harness + trivial baselines before novel algorithms** | [0005](docs/adr/0005-evaluation-methodology.md) |
| Sensor geometry | **Rig from the robot's URDF / a bag's `tf_static`** (frame-tagged measurements) | [0009](docs/adr/0009-sensor-rig-model.md) |
| 3D state & registration | **SE(3) body + TSDF submap registration, dual Rust/OpenVDB backend** | [0010](docs/adr/0010-3d-state-vdb-submap-registration.md) |
| Visualization | **rerun for live/progressive 3D** (feature-gated); matplotlib for quick 2D | [0011](docs/adr/0011-visualization-stack.md) |

See [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) for the full picture and
[docs/ROADMAP.md](docs/ROADMAP.md) for the milestone plan.

## Repository layout

```
crates/                Rust workspace (the engine; middleware-independent core)
  slam-types/          Foundational types: time, SE(3), sensor frames, TUM I/O
  slam-baseline/       Trivial reference baselines (stationary, IMU dead-reckoning)
  slam-rig/            Sensor rig: frames + extrinsics from URDF / tf_static
  slam-map/            3D map substrate: narrow-band TSDF behind a batch-level trait
  slam-frontend-scan/  2D-lidar front-end: PLICP, attitude, scan-to-submap, loops
  slam-backend/        Factor-graph optimisation (wrapped GTSAM)
  slam-datasets/       ROS1 bag reader (no ROS install): IMU, scans, tf_static
  slam-replay/         CLI: run a system over a dataset → TUM trajectory (+ viz)
docs/                  Architecture, roadmap, and Architecture Decision Records
  adr/                 One file per decision
eval/                  CPU-only evaluation harness (Python): datasets, metrics, gates
```

## Quick start (CPU-only, no GPU/ROS required)

```bash
# Build & test the engine (Rust) + harness (Python)
make test

# End-to-end harness sanity check: synthesise a trajectory, run a baseline,
# score it with ATE/RPE + compute metrics — runs anywhere, no GPU, no download.
make bench           # gated self-test (what CI runs)

# Full benchmark report (accuracy + compute, mean ± std) → eval/results/
make setup           # one-time: create the Python venv
cd eval && . .venv/bin/activate && python -m harness.benchmark
```

Other entry points (`make help` for the full list):

| Command | What it does |
|---|---|
| `python -m harness.benchmark [--repeats N]` | run the (sequence × system) matrix → `eval/results/{report.md,results.json}` |
| `python -m harness.score --groundtruth … --estimate …` | score a reference system's trajectory into the report |
| `make data-euroc SEQ=MH_01_easy` | download + cache an EuRoC sequence (real IMU + ground truth) |
| `make data-openloris SCENE=office1` | download + cache an OpenLORIS scene (ROS1 bags; large) |
| `slam-bag2imu --bag … --list` | inspect / extract IMU from a ROS1 bag (no ROS install) |

The entire dev/test pipeline is **CPU-only and platform-independent** by design,
so it runs unchanged on a laptop or a GPU-less CI build farm. GPU acceleration is
an optional fast-path, never a requirement (see
[ADR 0003](docs/adr/0003-gpu-optional-cpu-fallback.md)).

## Visualization & debugging

Two complementary tools ([ADR 0011](docs/adr/0011-visualization-stack.md)):

**Live / progressive 3D — [rerun](https://rerun.io).** The engine logs directly into
the rerun viewer: the current scan sweep, the growing estimated trajectory, ground
truth, the map accumulating chunk by chunk, and the final TSDF surface (coloured by
height), all on a scrubbable `sensor_time` timeline. The rerun SDK is a heavy
dependency, so it is **feature-gated** — build once with `--features viz`:

```bash
# one-time: the viz-enabled engine + the viewer itself
cargo build --release -p slam-replay --features viz
pip install rerun-sdk            # provides the `rerun` viewer binary

# live: a viewer opens and the map builds in front of you while the engine runs
./target/release/slam-replay --baseline scan-matching-3d \
    --scan data/openloris/_materialized/cafe1-1/scan.csv \
    --init-pose-from-tum data/openloris/groundtruth/per-sequence/cafe1-1/groundtruth.txt \
    --rerun spawn

# record instead, then replay the progressive map build at your own pace
./target/release/slam-replay … --rerun save:cafe1-1.rrd
rerun cafe1-1.rrd                # scrub the timeline

# attach to an already-running viewer (e.g. on another machine)
./target/release/slam-replay … --rerun connect
```

Notes: a `--rerun` run is for debugging, not benchmarking (logging adds overhead,
though the estimated trajectory is bit-identical). Without the `viz` feature the flag
fails with a clear message; CI checks that the feature build keeps compiling. For
headless analysis, `--map-out map.stsd` dumps the raw TSDF voxels
(tiny versioned binary: voxel size + `(ix iy iz tsdf weight)` records).

**Quick 2D — matplotlib.** `python -m harness.viz --openloris cafe1-1` steps through
scans rendered at their estimated poses over both trajectories (slider / arrow keys /
autoplay; `--save` for a headless PNG). Dependency-light, instant, no build flags.

## License

Licensed under the [Apache License, Version 2.0](LICENSE).
