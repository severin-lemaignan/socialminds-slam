"""Tests for dataset adapters (run with `pytest` from `eval/`)."""

from __future__ import annotations

from pathlib import Path

import pytest

from harness import datasets

FIXTURES = Path(__file__).parent / "fixtures"


def _read_data_lines(path: Path) -> list[str]:
    return [ln for ln in path.read_text().splitlines() if ln and not ln.startswith("#")]


def test_ns_to_seconds_str_is_exact():
    assert datasets._ns_to_seconds_str(1_403_636_579_758_555_392) == "1403636579.758555392"
    assert datasets._ns_to_seconds_str(0) == "0.000000000"
    assert datasets._ns_to_seconds_str(5_000_000_000) == "5.000000000"
    assert datasets._ns_to_seconds_str(1) == "0.000000001"


def test_convert_euroc_imu_stream(tmp_path):
    seq = datasets.convert_euroc(FIXTURES / "euroc_mini" / "mav0", tmp_path, name="mini")
    assert seq.source == "euroc"
    assert seq.has_gyro

    imu_lines = _read_data_lines(seq.imu_csv)
    assert len(imu_lines) == 3
    # First row: exact ns timestamp, then gyro xyz, accel xyz passed straight through.
    first = imu_lines[0].split()
    assert first[0] == "1403636579.758555392"
    assert [float(x) for x in first[1:4]] == [0.1, -0.02, 0.03]
    assert [float(x) for x in first[4:7]] == [8.1, -0.3, 4.5]


def test_convert_euroc_groundtruth_reorders_quaternion(tmp_path):
    seq = datasets.convert_euroc(FIXTURES / "euroc_mini" / "mav0", tmp_path, name="mini")
    gt_lines = _read_data_lines(seq.groundtruth_tum)
    assert len(gt_lines) == 2

    # EuRoC stores q as (w, x, y, z) = (0.9, 0.01, 0.02, 0.43); TUM wants (x, y, z, w).
    f = gt_lines[0].split()
    assert f[0] == "1403636579.758555392"
    assert [float(x) for x in f[1:4]] == [0.5, 1.2, -0.3]            # tx ty tz
    assert [float(x) for x in f[4:8]] == [0.01, 0.02, 0.43, 0.9]      # qx qy qz qw


def test_convert_euroc_duration(tmp_path):
    seq = datasets.convert_euroc(FIXTURES / "euroc_mini" / "mav0", tmp_path, name="mini")
    # (1403636579768555520 - 1403636579758555392) ns = 0.01 s
    assert seq.duration_s == pytest.approx(0.00999_9872, abs=1e-6)


def test_convert_euroc_missing_files(tmp_path):
    with pytest.raises(FileNotFoundError):
        datasets.convert_euroc(tmp_path, tmp_path / "out", name="missing")


def test_euroc_download_url_uses_research_collection():
    # MH_01_easy lives in the machine_hall collection → that collection's bitstream UUID.
    url = datasets.euroc_download_url("MH_01_easy")
    assert url == (
        "https://www.research-collection.ethz.ch/server/api/core/bitstreams/"
        "7b2419c1-62b5-4714-b7f8-485e5fe3e5fe/content"
    )
    # Sequences in the same collection share the URL.
    assert datasets.euroc_download_url("MH_05_difficult") == url
    assert datasets.euroc_collection("V2_03_difficult") == "vicon_room2"
    with pytest.raises(KeyError):
        datasets.euroc_download_url("does_not_exist")


def test_locate_euroc_mav0(tmp_path):
    # Collection layout: <collection>/<sequence>/mav0/...
    mav0 = tmp_path / "machine_hall" / "MH_03_medium" / "mav0"
    (mav0 / "imu0").mkdir(parents=True)
    found = datasets.locate_euroc_mav0(tmp_path, "MH_03_medium")
    assert found == mav0
    # Prefix match works even if the dir drops the difficulty suffix.
    alt = tmp_path / "vicon_room1" / "V1_01" / "mav0"
    alt.mkdir(parents=True)
    assert datasets.locate_euroc_mav0(tmp_path / "vicon_room1", "V1_01_easy") == alt


def test_materialize_synthetic(tmp_path):
    seq = datasets.materialize_synthetic(tmp_path)
    assert seq.source == "synthetic"
    assert seq.imu_csv.exists() and seq.groundtruth_tum.exists()
    assert len(_read_data_lines(seq.imu_csv)) > 100
