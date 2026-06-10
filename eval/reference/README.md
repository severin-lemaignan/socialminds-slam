# Reference baselines — the "number to beat"

Before trusting our own engine's numbers, we anchor them against an established SLAM system
on the same data (ADR 0005). Because the reference systems need ROS and/or a GPU and the
multi-GB datasets, **they are run externally — on the robot or a capable workstation — not
in CI.** Their scored results are archived under [`baselines/`](baselines/) and compared in
the benchmark report.

Published results from the literature live under [`sota/`](sota/) — currently the
OpenLORIS-Scene paper's numbers (ORB-SLAM2, VINS-Mono, DS-SLAM, …) on the exact sequences
we benchmark on.

## Candidates

| System | Why | Needs |
|---|---|---|
| **RTAB-Map** | Matches the robot's sensor suite (multi-RGB-D + 2D lidar + IMU/odom), CPU-only, BSD | ROS, the dataset |
| **GLIM** | Strong RGB-D + IMU factor-graph SLAM, GPU-accelerated | ROS, CUDA GPU, the dataset |

(See `docs/adr/0002` for why these fit; the survey flagged RTAB-Map as the closest match to
this platform.)

## Workflow

1. **Get the data** (cached, one-time):
   ```bash
   make data-openloris SCENE=office1     # or: make data-euroc SEQ=MH_01_easy
   make data-openloris-gt                # ground truth (~11 MB)
   ```
2. **Run the reference system** to produce a TUM trajectory. For RTAB-Map on an OpenLORIS
   bag, [`run_rtabmap.sh`](run_rtabmap.sh) is a Docker-based starting point (adjust topic
   remaps to the sequence). The output must be a TUM file: `timestamp tx ty tz qx qy qz qw`.
3. **Score it** against ground truth, into the archive:
   ```bash
   python -m harness.score \
       --groundtruth data/openloris/groundtruth/per-sequence/office1-1/groundtruth.txt \
       --estimate /tmp/rtabmap_office1-1.tum \
       --system rtabmap --sequence office1-1 \
       --out eval/reference/baselines/office1-1_rtabmap.json
   ```
4. **Commit** the small JSON under `baselines/`. It becomes the reference our engine is
   measured against in the report.

## Notes

- Use SE(3) alignment (the default; scale is known). Note any frame offset between the
  reference system's output frame and the ground-truth frame.
- Reference runs are *not reproducible in CI* and that is expected — record the machine and
  software versions in the archived JSON's surrounding notes when it matters.
- Our own engine is scored by `python -m harness.benchmark`; reference numbers slot into the
  same table via `harness.score`.
