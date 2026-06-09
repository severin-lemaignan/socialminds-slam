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

All commands run from `eval/` with the venv active. The Rust binaries (`slam-replay`,
`slam-bag2imu`) are located via `SLAM_REPLAY_BIN` / `SLAM_BAG2IMU_BIN`, then `PATH`, then
`target/{release,debug}/`, and are built on demand if absent.

## The end-to-end self-test (M0/M1 acceptance check)

```bash
python -m harness.selftest          # synthesize → run baselines → score → gate
python -m harness.selftest --keep   # keep artifacts under eval/_run/ for inspection
```

Synthesises a known trajectory + consistent IMU, runs the `stationary` and `dead-reckoning`
baselines through `slam-replay`, scores accuracy (`evo`) and compute, and gates on the
expected ordering (dead-reckoning beats stationary; drift bounded; runs in real time). This
is what CI runs.

## Benchmark report (accuracy + compute)

```bash
python -m harness.benchmark                 # synthetic matrix → eval/results/{report.md,results.json}
python -m harness.benchmark --repeats 5      # N repeats → mean ± std
```

Reports ATE/RPE, real-time factor, latency p99, and peak RSS per (sequence × system). CI
emits the report as an artifact.

## Datasets — download + cache

Datasets are cached under `$SLAM_DATA_DIR` (default `<repo>/data`, git-ignored). Use the
`make` targets (from the repo root) or the `fetch` module directly:

```bash
make data-euroc SEQ=MH_01_easy      # EuRoC IMU+GT (small)
make data-openloris-gt              # OpenLORIS ground truth (~11 MB)
make data-openloris SCENE=office1   # OpenLORIS scene rosbags (LARGE: 6–33 GB)
```

OpenLORIS bags are read with the Rust `rosbag` reader (no ROS install):

```bash
slam-bag2imu --bag data/openloris/office1-1.bag --list           # inspect topics
slam-bag2imu --bag data/openloris/office1-1.bag --out imu.csv     # extract IMU
```

## Reference baselines (the "number to beat")

Reference systems (RTAB-Map, GLIM) run **externally** (ROS/GPU, the dataset) — see
[`reference/`](reference/). Score their TUM output into the same report:

```bash
python -m harness.score --groundtruth gt.tum --estimate rtabmap.tum \
    --system rtabmap --sequence office1-1 --out reference/baselines/office1-1_rtabmap.json
```

## Modules

| Module | Role |
|---|---|
| `harness.synthetic` | generate a ground-truth trajectory + exactly-consistent IMU (no downloads) |
| `harness.datasets` | uniform `Sequence` interface + adapters (synthetic, EuRoC, OpenLORIS) |
| `harness.fetch` | download + cache datasets under `$SLAM_DATA_DIR` |
| `harness.replay` | locate/build the Rust binaries and run a baseline → TUM trajectory |
| `harness.metrics` | ATE / RPE via `evo` (SE(3) alignment; scale known) |
| `harness.compute` | compute metrics: latency / throughput / real-time factor / peak RSS |
| `harness.benchmark` | run the (sequence × system) matrix → mean±std JSON + Markdown report |
| `harness.score` | score an external reference trajectory into the report |
| `harness.selftest` | wire the above into the gated end-to-end benchmark |

Generate a sequence standalone:

```bash
python -m harness.synthetic --out-dir /tmp/seq --duration 20 --rate 200
```
