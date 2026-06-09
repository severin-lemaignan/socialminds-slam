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
