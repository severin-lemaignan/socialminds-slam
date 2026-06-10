# ADR 0013: YAML run configuration — sensor-set selection, not calibration

- **Status:** accepted
- **Date:** 2026-06-10
- **Deciders:** Séverin Lemaignan

## Context

A run of the engine needs to know *which sensors to use* — which scan/depth/IMU topics,
with what ingest tuning (depth stride, range clip, frame decimation) — and that set
varies per robot generation (no IMU initially, ADR 0012), per dataset (OpenLORIS
`market` has no laser) and per experiment. Today this is a growing pile of CLI flags.
ADR 0009 deliberately rejected a bespoke *calibration* file: extrinsics live in the
URDF/`tf_static`, intrinsics in `CameraInfo`. This is a different artifact.

## Decision

A small **YAML run configuration** (`slam-replay --config run.yaml`), with a hard
boundary: it may name sensors and carry **operational** parameters that genuinely exist
nowhere else (topics to use, ingest tuning, rig *source* selection). It must never
carry extrinsics, intrinsics or distortion — those stay in URDF/`tf_static`/CameraInfo
(ADR 0009), and a config that tried would have no mechanism to win anyway.

```yaml
rig:
  source: bag            # bag (tf_static) | urdf | identity
  base_frame: base_link
sensors:
  scans:
    - topic: /scan
  imus:
    - gyro_topic: /d400/gyro/sample      # RealSense-style split pair…
      accel_topic: /d400/accel/sample
    # - topic: /imu                      # …or a single 6-axis topic
  depth:
    - topic: /d400/aligned_depth_to_color/image_raw
      # camera_info: defaults to the sibling …/camera_info
      stride: 4          # pixel stride (keep stride·z/fx ≤ voxel)
      min_range: 0.3     # m
      max_range: 6.0
      every_nth: 3       # frame decimation
```

Reference configs are committed under [`configs/`](../../configs/) (one per
robot/dataset variant); the individual topic CLI flags remain for one-off experiments
and are mutually exclusive with `--config`.

## Consequences

- **Easier:** a dataset/robot variant is one file, not a shell incantation; the no-IMU
  robot (ADR 0012) and the laser-less market bags are just configs that omit sections.
- **Harder:** one more file format in the repo — bounded by the calibration firewall
  above and by reusing `serde` (the config is a plain struct).
- **Revisit when:** the live ROS 2 node lands — the same struct should deserialize from
  ROS parameters so the file and the parameter server stay one schema.
