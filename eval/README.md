# Evaluation harness

CPU-only, GPU-free, download-free evaluation for the SLAM engine. Methodology and
rationale live in [ADR 0005](../docs/adr/0005-evaluation-methodology.md); this is the
operator's guide.

## Why it exists

Every change to the engine is judged against reproducible **accuracy** (ATE/RPE) and, in
time, **compute** numbers — not opinion. The harness is deliberately platform-independent
so it runs identically on a laptop and a GPU-less CI runner
([ADR 0003](../docs/adr/0003-gpu-optional-cpu-fallback.md)).

## Setup

```bash
cd eval
python3 -m venv .venv && . .venv/bin/activate
pip install -r requirements.txt
```

## The end-to-end self-test (M0 acceptance check)

```bash
python -m harness.selftest          # synthesize → run baselines → score → gate
python -m harness.selftest --keep   # keep artifacts under eval/_run/ for inspection
```

It synthesises a known trajectory + a consistent IMU stream, runs the `stationary` and
`dead-reckoning` baselines through the Rust `slam-replay` binary, scores them with `evo`,
and asserts the expected ordering (dead-reckoning beats stationary; drift bounded). This
is what CI runs.

The `slam-replay` binary is located via `SLAM_REPLAY_BIN`, then `PATH`, then
`target/{release,debug}/`, and is built on demand if absent.

## Modules

| Module | Role |
|---|---|
| `harness.synthetic` | generate a ground-truth trajectory + exactly-consistent IMU (no downloads) |
| `harness.replay` | locate/build `slam-replay` and run a baseline → TUM trajectory |
| `harness.metrics` | ATE / RPE via `evo` (SE(3) alignment; scale known) |
| `harness.selftest` | wire the above into the gated end-to-end benchmark |

Generate a sequence standalone:

```bash
python -m harness.synthetic --out-dir /tmp/seq --duration 20 --rate 200
```

## Coming next (see [roadmap](../docs/ROADMAP.md))

- Adapters for real datasets: **OpenLORIS-Scene** (the robot's twin), **TUM RGB-D**.
- A **reference baseline** (RTAB-Map / GLIM) as the "number to beat".
- Compute metrics (latency p50/p95/p99, CPU%, RAM, real-time factor) and an N×-repeat
  mean±std report.
