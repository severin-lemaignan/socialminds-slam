"""Test compute-metrics capture (runs the engine, parses its metrics sidecar)."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

from harness import compute, datasets, replay


def _replay_or_skip() -> Path:
    try:
        return replay.find_replay_binary(build_if_missing=False)
    except FileNotFoundError:
        pytest.skip("slam-replay not built; run `cargo build -p slam-replay`")


def test_run_with_metrics_on_synthetic(tmp_path):
    binary = _replay_or_skip()
    seq = datasets.materialize_synthetic(tmp_path / "seq")
    stats = compute.run_with_metrics(binary, "dead-reckoning", seq.imu_csv, tmp_path / "dr.tum")

    assert stats.n_samples > 100
    assert stats.processing_wall_s > 0.0
    # The trivial baseline is far faster than real time.
    assert stats.real_time_factor > 1.0
    assert stats.is_real_time
    for key in ("p50", "p95", "p99", "max", "mean"):
        assert key in stats.latency_us

    # On Linux /proc is available, so peak RSS should be measured and positive.
    if sys.platform.startswith("linux"):
        assert stats.peak_rss_mb is not None and stats.peak_rss_mb > 0.0
