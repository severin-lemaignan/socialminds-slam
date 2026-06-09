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
