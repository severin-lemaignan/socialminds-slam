# socialminds-slam

A from-scratch, real-time, fully-3D SLAM engine for an indoor mobile robot.

> **Status:** bootstrapping. The architecture and evaluation harness are being laid
> down first; novel algorithms land against a working benchmark (see the
> [roadmap](docs/ROADMAP.md)).

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

See [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) for the full picture and
[docs/ROADMAP.md](docs/ROADMAP.md) for the milestone plan.

## Repository layout

```
crates/            Rust workspace (the engine; middleware-independent core)
  slam-types/      Foundational types: time, SE(3), IMU, trajectories, TUM I/O
  slam-baseline/   Trivial reference baselines (stationary, IMU dead-reckoning)
  slam-replay/     CLI: run a baseline/system over a dataset → TUM trajectory
docs/              Architecture, roadmap, and Architecture Decision Records
  adr/             One file per decision
eval/              CPU-only evaluation harness (Python): datasets, metrics, gates
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

## License

Licensed under the [Apache License, Version 2.0](LICENSE).
