"""Tests for the benchmark matrix + report generator."""

from __future__ import annotations

from pathlib import Path

import pytest

from harness import benchmark, datasets, replay


def _replay_or_skip() -> Path:
    try:
        return replay.find_replay_binary(build_if_missing=False)
    except FileNotFoundError:
        pytest.skip("slam-replay not built; run `cargo build -p slam-replay`")


def test_mean_std():
    ms = benchmark.mean_std([1.0, 2.0, 3.0])
    assert ms.mean == pytest.approx(2.0)
    assert ms.std == pytest.approx(0.8164965, abs=1e-6)
    nan = benchmark.mean_std([])
    assert nan.mean != nan.mean  # NaN


def test_matrix_and_report(tmp_path):
    binary = _replay_or_skip()
    seq = datasets.materialize_synthetic(tmp_path / "seq")
    results = benchmark.run_matrix(
        [seq], benchmark.default_systems(), workdir=tmp_path, repeats=2, align=False
    )
    # one row per (sequence, system)
    by_system = {r.system: r for r in results}
    assert set(by_system) == {"stationary", "imu_dead_reckoning"}

    # Dead-reckoning must beat stationary on accuracy and run faster than real time.
    assert by_system["imu_dead_reckoning"].ate_rmse_m.mean < by_system["stationary"].ate_rmse_m.mean
    assert by_system["imu_dead_reckoning"].real_time_factor.mean > 1.0
    assert by_system["imu_dead_reckoning"].repeats == 2

    json_path, md_path = benchmark.write_report(results, tmp_path / "results")
    assert json_path.exists() and md_path.exists()
    assert "ATE RMSE" in md_path.read_text()


def test_score_external_trajectory(tmp_path):
    """An externally-produced trajectory scores via the same accuracy path."""
    from harness import compute

    binary = _replay_or_skip()
    seq = datasets.materialize_synthetic(tmp_path / "seq")
    est = tmp_path / "dr.tum"
    compute.run_with_metrics(binary, "dead-reckoning", seq.imu_csv, est)

    agg = benchmark.score_trajectory(
        seq.groundtruth_tum, est, system="ref", sequence="synthetic", align=False
    )
    assert agg.system == "ref"
    assert agg.source == "reference"
    assert agg.ate_rmse_m.mean == pytest.approx(0.0282, abs=2e-3)
    # Compute fields are not observed for external trajectories.
    assert agg.real_time_factor.mean != agg.real_time_factor.mean  # NaN
