# Architecture

This document gives the high-level picture. Individual decisions are recorded as
[ADRs](adr/); this document links to them rather than repeating their rationale.

## 1. The robot, and what it implies

| Property | Value | Consequence for the design |
|---|---|---|
| Base | omnidirectional, ~50×70 cm | revisits corridors from arbitrary directions → loop closure must handle reverse/opposite-direction loops |
| Max linear velocity | 2 m/s | ~10 cm of motion per 50 ms RGB-D frame → tight time-sync and IMU de-skew are mandatory |
| Range sensors | **2× 2D laser scanners**, opposite corners | *planar*, not 3D. They cannot build the 3D map alone — see [ADR 0002](adr/0002-sensor-roles-and-pipeline.md) |
| Cameras | 2× RGB-D (front + rear) | the actual source of the 3D map; also the people/dynamics sensor |
| IMU | up to ~1 kHz | tight-coupled for de-skew, motion prediction, and as a standalone dead-reckoning baseline |
| Compute | 24+ core CPU; RTX 5060 8 GB (shared) | CPU is the workhorse; GPU is an optional, budgeted fast-path ([ADR 0003](adr/0003-gpu-optional-cpu-fallback.md)) |
| Environment | indoor, feature-rich, **repetitive corridors**, **dynamic** (people/doors/chairs) | perceptual aliasing and dynamic-object rejection are first-class concerns |

The single most consequential fact: **the 2D scanners are planar**. The mainstream
LiDAR-inertial-odometry literature assumes a 3D lidar and does not apply directly.
Our 3D output therefore comes from **RGB-D + IMU**, with the 2D scanners providing a
precise planar geometric backbone and our most reliable loop-closure cue.

## 2. Dataflow (target system)

```
                          ┌──────────────────────────────────────────────┐
  IMU  (~1 kHz) ─────────▶│  preintegration (IMU factors, de-skew)        │
                          │                                               │
  2D lidar ×2 (~?Hz) ────▶│  planar scan-matching ──┐                     │
                          │                          │   front-end         │
  RGB-D ×2 (~20 fps) ────▶│  dynamic masking ──▶ RGB-D-inertial odometry ─┼──┐
                          │  (YOLO-seg + flow,       │                     │  │
                          │   GPU-opt / CPU)         │                     │  │
                          └──────────────────────────┼─────────────────────┘  │
                                                     ▼                         │
                          ┌──────────────────────────────────────────────┐    │
  loop closure  ◀────────▶│  factor graph backend (wrapped GTSAM/iSAM2)   │◀───┘
  (MapClosures +          │  fuses: IMU, lidar, RGB-D, loop, inter-lidar  │
   geometric verify +     │  robust kernels → globally consistent poses   │
   visual VPR)            └──────────────────────────────────────────────┘
                                                     │  optimized poses
                                                     ▼
                          ┌──────────────────────────────────────────────┐
                          │  Map (trait, multiple backends)               │
                          │   • GPU TSDF/ESDF  → Nav2 costmap             │
                          │   • OpenVDB layer  → reMap geometric reasoning │
                          │   occupancy decay evicts dynamic objects      │
                          └──────────────────────────────────────────────┘
```

Everything inside the boxes is the **middleware-independent core** (Rust). ROS 2 is
a thin I/O shell around it ([ADR 0001](adr/0001-language-and-optimizer.md),
binding strategy).

## 3. Concurrency model

The engine is a set of stages connected by **lock-free queues** (crossbeam), each
sensor stream running at its own rate and the heavy per-point work parallelised with
rayon. The backend optimiser runs asynchronously and publishes corrections that the
map and front-end consume. Hard-real-time guarantees (thread pinning, pre-allocation,
`SCHED_FIFO`) are a deployment concern, not a language feature — see
[ADR 0001](adr/0001-language-and-optimizer.md).

Design rule: **no stage blocks on a slower stage.** Frames are dropped to keep input
and processing rates matched (a known failure mode in dynamic-SLAM systems).

## 4. Crate structure

The core is a Cargo workspace. Crates are introduced as the roadmap reaches them; the
intended shape:

| Crate | Responsibility | Status |
|---|---|---|
| `slam-types` | time, SE(3)/SO(3), sensor samples, trajectories, TUM I/O — the zero-copy data structures | **present** |
| `slam-baseline` | `SlamSystem` trait + trivial reference baselines (stationary, IMU dead-reckoning) | **present** |
| `slam-replay` | CLI: drive a `SlamSystem` over a dataset, emit a TUM trajectory | **present** |
| `slam-imu` | IMU preintegration | planned |
| `slam-frontend-lidar` | 2D planar scan-matching | planned |
| `slam-frontend-rgbd` | RGB-D-inertial odometry | planned |
| `slam-loop` | MapClosures-style detection + geometric/visual verification | planned |
| `slam-backend` | factor graph; wraps GTSAM via `slam-gtsam-sys` | planned |
| `slam-map` | `Map` trait + TSDF/ESDF and OpenVDB backends | planned |
| `slam-engine` | stage orchestration, threading, sensor bus | planned |
| `slam-ros2` | rclrs node; zero-copy bridge | planned |
| bindings | `pyo3` (Python) / `cxx` (C++) zero-copy hub | planned |

## 5. Evaluation as a first-class subsystem

Accuracy *and* compute are benchmarked reproducibly on every change. The harness is
CPU-only and dataset-driven; the trivial baselines exist precisely to prove the harness
works end-to-end before any real algorithm is written. See
[ADR 0005](adr/0005-evaluation-methodology.md) and [`eval/`](../eval/).

## 6. Open questions tracked for later ADRs

- RGB-D-inertial odometry: build on a filter (MSCKF-style) vs. sliding-window graph.
- Loop-closure descriptor stack: MapClosures alone vs. + learned visual VPR (NetVLAD/AnyLoc).
- Dynamic masking: which segmentation model, and the CPU-fallback path when no GPU.
- Map: confirm GPU-TSDF + OpenVDB split and the reMap interface ([ADR 0004](adr/0004-map-representation.md)).
