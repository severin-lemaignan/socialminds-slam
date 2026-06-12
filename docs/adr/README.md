# Architecture Decision Records

Each significant, hard-to-reverse decision is recorded here as one file. New decisions
get the next number; superseded ones are marked, not deleted. Use
[`0000-template.md`](0000-template.md) as the starting point.

| # | Decision | Status |
|---|---|---|
| [0001](0001-language-and-optimizer.md) | Rust core with a wrapped GTSAM optimizer | accepted |
| [0002](0002-sensor-roles-and-pipeline.md) | RGB-D + IMU build the 3D map; 2D lidars are the planar/loop-closure backbone | accepted |
| [0003](0003-gpu-optional-cpu-fallback.md) | GPU is optional/feature-gated; CPU fallback is the default | accepted |
| [0004](0004-map-representation.md) | `Map` trait with multiple backends (GPU TSDF/ESDF + OpenVDB) | accepted |
| [0005](0005-evaluation-methodology.md) | Evaluation-first; trivial baselines before novel algorithms | accepted |
| [0006](0006-vendored-gtsam-build.md) | Vendor GTSAM as a pinned submodule, built static and Boost-free by cargo | accepted |
| [0007](0007-frontend-order-and-scan-matching.md) | Front-end order: 2D scan matching first, as point-to-line ICP | accepted |
| [0008](0008-inhouse-bag-reader.md) | In-house indexed ROS1 bag reader (replacing the `rosbag` crate) | accepted |
| [0009](0009-sensor-rig-model.md) | Sensor rig from URDF + CameraInfo; frame-tagged measurements | accepted |
| [0010](0010-3d-state-vdb-submap-registration.md) | Full-3D state; TSDF submap registration (Rust + system OpenVDB backends); fast re-localization | accepted |
| [0011](0011-visualization-stack.md) | Visualization: rerun for live/progressive 3D (feature-gated); matplotlib 2D stays | accepted |
| [0012](0012-imu-optional.md) | The IMU is an optional accuracy enhancer, never a prerequisite | accepted |
| [0013](0013-run-configuration.md) | YAML run configuration: sensor-set selection, not calibration | accepted |
| [0014](0014-map-update-and-decay-policy.md) | Map decay = contradiction-driven carving; no time decay; masking never load-bearing | accepted |
| [0015](0015-dynamics-masking-yolo-onnx.md) | Dynamics masking: YOLO11-seg via ONNX Runtime at depth ingest (feature-gated) | accepted |
| [0016](0016-masking-altitude.md) | Masking altitude: semantics gate durable products (map/signatures/re-loc), never the pose estimate | accepted |
