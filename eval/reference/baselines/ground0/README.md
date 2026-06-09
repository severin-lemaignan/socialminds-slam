# Ground 0 — trivial baselines, pre-backend

The floor every future system must beat: the M0 trivial baselines (`stationary`,
`imu_dead_reckoning`) scored on the first real data, **before** any SLAM exists (no
factor-graph backend, no front-ends, no loop closure). Recorded so M2+ improvements are
measured against a fixed, honest starting point rather than a moving one.

- **Date:** 2026-06-09
- **Commit:** `6d69634` (+ uncommitted benchmark-CLI wiring; M2 backend in progress)
- **Command:** `python -m harness.benchmark --euroc MH_01_easy --openloris cafe1-1 --synthetic --repeats 3`
- **Machine:** Intel Core Ultra 9 185H (22 threads), Linux 6.17, Rust 1.94.1, Python 3.13.7
- **Data:** EuRoC MH_01_easy (ETH Research Collection); OpenLORIS cafe1-1 (d400 split
  gyro/accel merged, accel interpolated onto the 200 Hz gyro timeline)

Files: [`results.json`](results.json) (machine-readable), [`report.md`](report.md).

## Reading the numbers

- `imu_dead_reckoning` diverges on real data (ATE ~7×10⁴ m on MH_01, ~6×10³ m on cafe1-1):
  expected — double-integrating consumer-grade IMU noise without gravity alignment or any
  correction *is* the point of a floor baseline. On the noise-free synthetic it is
  near-exact (ATE 14 mm), which separates "the machinery works" from "the sensor drifts".
- `stationary` ATE is the trajectory's scale (how far the robot gets from its start),
  ~5.5 m for MH_01, ~33 m for cafe1-1.
- Compute columns are per-sample (IMU-rate) figures for trivial math; they set the
  measurement floor of the harness itself, not a meaningful speed target.
