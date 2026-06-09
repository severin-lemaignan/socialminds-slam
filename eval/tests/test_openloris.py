"""Test the OpenLORIS IMU materialisation path (bag → IMU CSV via the Rust tool).

Uses the committed `mini.bag` fixture as a stand-in OpenLORIS bag. Skipped if the
`slam-bag2imu` binary hasn't been built (kept out of pytest's job to stay Python-fast).
"""

from __future__ import annotations

from pathlib import Path

import pytest

from harness import datasets, replay

REPO = Path(__file__).resolve().parents[2]
BAG = REPO / "crates" / "slam-datasets" / "tests" / "fixtures" / "mini.bag"
GROUNDTRUTH = Path(__file__).parent / "fixtures" / "openloris_mini" / "groundtruth.txt"


def _bag2imu_or_skip() -> Path:
    try:
        return replay.find_bag2imu_binary(build_if_missing=False)
    except FileNotFoundError:
        pytest.skip("slam-bag2imu not built; run `cargo build -p slam-datasets`")


def test_materialize_openloris(tmp_path):
    binary = _bag2imu_or_skip()
    seq = datasets.materialize_openloris(BAG, GROUNDTRUTH, tmp_path, name="mini", bag2imu_bin=binary)

    assert seq.source == "openloris"
    assert seq.has_gyro

    imu_lines = [ln for ln in seq.imu_csv.read_text().splitlines() if ln and not ln.startswith("#")]
    assert len(imu_lines) == 3  # the fixture bag has three IMU messages

    gt_lines = [ln for ln in seq.groundtruth_tum.read_text().splitlines() if ln and not ln.startswith("#")]
    assert len(gt_lines) == 3  # OpenLORIS groundtruth is copied through unchanged (already TUM)

    # IMU span 1560000083.920771360 .. .930771360 = 0.01 s.
    assert seq.duration_s == pytest.approx(0.01, abs=1e-4)


def test_materialize_openloris_split_imu(tmp_path):
    # Real OpenLORIS bags split gyro/accel into separate RealSense topics. The fixture
    # bag has a single full-IMU topic; using it as *both* streams exercises the
    # extract-twice-and-merge path, and self-merging must reproduce the original samples.
    binary = _bag2imu_or_skip()
    seq = datasets.materialize_openloris(
        BAG,
        GROUNDTRUTH,
        tmp_path,
        name="mini",
        gyro_topic="/d400/imu",
        accel_topic="/d400/imu",
        bag2imu_bin=binary,
    )
    merged = [ln.split() for ln in seq.imu_csv.read_text().splitlines() if not ln.startswith("#")]
    single = [
        ln.split()
        for ln in (tmp_path / "gyro.csv").read_text().splitlines()
        if not ln.startswith("#")
    ]
    assert len(merged) == 3
    # Timestamps and gyro columns pass through verbatim; accel interpolates exactly.
    assert [r[:4] for r in merged] == [r[:4] for r in single]
    for got, want in zip(merged, single):
        assert [float(v) for v in got[4:7]] == pytest.approx([float(v) for v in want[4:7]])


def test_materialize_openloris_topic_arg_validation(tmp_path):
    with pytest.raises(ValueError, match="both gyro_topic and accel_topic"):
        datasets.materialize_openloris(BAG, GROUNDTRUTH, tmp_path, gyro_topic="/d400/gyro/sample")
    with pytest.raises(ValueError, match="mutually exclusive"):
        datasets.materialize_openloris(
            BAG, GROUNDTRUTH, tmp_path,
            imu_topic="/d400/imu", gyro_topic="/a", accel_topic="/b",
        )


def test_merge_split_imu_interpolates(tmp_path):
    gyro = tmp_path / "gyro.csv"
    accel = tmp_path / "accel.csv"
    out = tmp_path / "imu.csv"
    # Gyro at 0.5 Hz offsets inside (and outside) the accel span [1.0, 3.0].
    gyro.write_text(
        "# header\n"
        "0.500000000 0.1 0.2 0.3 0 0 0\n"   # before accel span → dropped
        "1.000000000 0.1 0.2 0.3 0 0 0\n"
        "1.500000000 0.4 0.5 0.6 0 0 0\n"
        "3.000000000 0.7 0.8 0.9 0 0 0\n"
        "3.500000000 1.0 1.1 1.2 0 0 0\n"   # after accel span → dropped
    )
    accel.write_text(
        "1.000000000 0 0 0 2.0 4.0 8.0\n"
        "2.000000000 0 0 0 4.0 8.0 16.0\n"
        "3.000000000 0 0 0 6.0 12.0 24.0\n"
    )
    datasets.merge_split_imu(gyro, accel, out)

    rows = [ln.split() for ln in out.read_text().splitlines() if not ln.startswith("#")]
    assert [r[0] for r in rows] == ["1.000000000", "1.500000000", "3.000000000"]
    # Gyro columns verbatim; accel linearly interpolated at each gyro stamp.
    assert rows[1][1:4] == ["0.4", "0.5", "0.6"]
    assert [float(v) for v in rows[0][4:7]] == pytest.approx([2.0, 4.0, 8.0])
    assert [float(v) for v in rows[1][4:7]] == pytest.approx([3.0, 6.0, 12.0])
    assert [float(v) for v in rows[2][4:7]] == pytest.approx([6.0, 12.0, 24.0])


def test_merge_split_imu_empty_stream(tmp_path):
    gyro = tmp_path / "gyro.csv"
    accel = tmp_path / "accel.csv"
    gyro.write_text("1.0 0.1 0.2 0.3 0 0 0\n")
    accel.write_text("# only a header\n")
    with pytest.raises(ValueError, match="empty IMU stream"):
        datasets.merge_split_imu(gyro, accel, tmp_path / "imu.csv")
