# ADR 0005: Evaluation-first — build the benchmark harness and trivial baselines before novel algorithms

- **Status:** accepted
- **Date:** 2026-06-09
- **Deciders:** Séverin Lemaignan

## Context

This project is explicitly a software-engineering showcase: reproducible accuracy *and*
compute benchmarks, extensive tests. The risk in SLAM work is writing a lot of clever code
with no trustworthy way to tell whether it helps. We also have a near-perfect target
dataset (OpenLORIS-Scene matches the robot's sensor suite) and standard tooling (`evo`).

## Decision

**Stand up the evaluation harness and trivial baselines first**, so that every later change
is measured against a working, reproducible benchmark.

- **Trajectory format:** TUM (`timestamp tx ty tz qx qy qz qw`) is the engine's output.
- **Metrics:** ATE (global consistency; SE(3)-Umeyama aligned — scale is known) and RPE
  (local drift), via **`evo`**. Loop closure is evaluated by ATE **with vs. without** loop
  closure, and the detector by **precision/recall** (precision prioritised — one false loop
  wrecks the map). Map quality is accuracy/completeness vs. a ground-truth mesh where
  available.
- **Compute metrics:** per-frame latency (p50/p95/p99), CPU%, peak/steady RAM, GPU/VRAM when
  used, and **real-time factor** (must stay ≥ 1.0 for online use). CPU-only numbers from CI
  are the portable floor ([ADR 0003](0003-gpu-optional-cpu-fallback.md)).
- **Non-determinism:** multi-threaded SLAM is not bit-reproducible; each sequence is run N
  times and reported as **mean ± std**, gated on a threshold with tolerance, not equality.
- **Trivial baselines first:** a **stationary/identity** baseline and an **IMU
  dead-reckoning** baseline. They are not meant to be good — they exist to prove the dataset
  I/O, trajectory format, metrics, gating, and CI all work end-to-end, and to give a sanity
  floor every real system must beat.
- **Reference baseline:** an existing system (RTAB-Map or GLIM fed RGB-D) provides the
  "number to beat" once datasets are wired.
- **Datasets, prioritised:** (1) a zero-download **synthetic** generator for CI; (2)
  **OpenLORIS-Scene** (the robot's twin: 2D lidar + RealSense RGB-D + IMU + wheel odom,
  indoor/dynamic); (3) **TUM RGB-D** `fr3/walking_*` (cheap dynamic baseline); (4) **Bonn
  RGB-D Dynamic** (dynamics + map-quality). KITTI only for place-recognition baselining.
- **Replay:** datasets are driven deterministically; on the ROS side, recorded **MCAP** bags
  are the regression fixtures.

## Consequences

- **Easier:** every algorithmic claim is backed by a reproducible number from day one;
  regressions are caught in CI; new contributors have an immediate "does it beat the floor?"
  check.
- **Harder:** upfront harness investment before the "exciting" SLAM code. Accepted — it is
  the cheapest insurance against unmeasurable progress.
- **Revisit when:** metrics or dataset priorities change; add ADRs for new metric families
  (e.g. lifelong/multi-session) as they arrive.

## Alternatives considered

- **Algorithms first, benchmark later:** the usual trap — produces code whose value can't be
  judged and a harness retrofitted to flatter it. Rejected.
