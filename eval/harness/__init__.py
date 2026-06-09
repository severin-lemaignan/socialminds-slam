"""CPU-only SLAM evaluation harness.

See ADR 0005. The harness is deliberately platform-independent and GPU-free so the
whole accuracy pipeline runs unchanged on a laptop or a CI build farm:

- `synthetic`  — generate a known trajectory + a consistent IMU stream (no downloads)
- `datasets`   — uniform Sequence interface + adapters (synthetic, EuRoC, OpenLORIS)
- `fetch`      — download + cache datasets (EuRoC, OpenLORIS) under $SLAM_DATA_DIR
- `replay`     — drive the Rust engine (`slam-replay`) over recorded input
- `metrics`    — ATE / RPE via the `evo` toolkit
- `compute`    — capture compute metrics (latency / throughput / RTF / peak RSS)
- `selftest`   — wire the above into an end-to-end, self-checking benchmark
"""

__all__ = ["synthetic", "datasets", "fetch", "replay", "metrics", "compute"]
