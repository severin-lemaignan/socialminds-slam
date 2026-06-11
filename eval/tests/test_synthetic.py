"""Tests for the synthetic sensor-stream generators (run with `pytest` from `eval/`)."""

from __future__ import annotations

import math

from harness import datasets, synthetic


def _short_spec() -> synthetic.TrajectorySpec:
    return synthetic.TrajectorySpec(duration_s=2.0, rate_hz=100.0)


# ----------------------------------------------------------------- wheel odometry


def test_odometry_starts_exact_and_drifts():
    samples = synthetic.generate(synthetic.TrajectorySpec())
    rows = synthetic.derive_odometry(samples)
    t0, x0, y0, th0 = rows[0]
    assert (x0, y0, th0) == (0.0, 0.0, 0.0)

    # Drift must be visible at the end (the imperfection model is active) but bounded —
    # the same property the CI self-test gates on.
    gt_end = samples[-1]
    _, xe, ye, _ = rows[-1]
    err = math.hypot(xe - gt_end.px, ye - gt_end.py)
    assert 0.01 < err < 0.5


def test_perfect_odometry_replays_groundtruth():
    """With scale 1 and zero bias the integration must reproduce the trajectory."""
    samples = synthetic.generate(_short_spec())
    rows = synthetic.derive_odometry(
        samples, synthetic.OdomSpec(scale=1.0, yaw_rate_bias=0.0)
    )
    gt = {s.t: s for s in samples}
    for t, x, y, _ in rows:
        assert math.hypot(x - gt[t].px, y - gt[t].py) < 1e-9


def test_odometry_rate_subsamples():
    samples = synthetic.generate(_short_spec())
    rows = synthetic.derive_odometry(samples, synthetic.OdomSpec(rate_hz=10.0))
    dt = rows[1][0] - rows[0][0]
    assert abs(dt - 0.1) < 1e-9


def test_odom_tum_format(tmp_path):
    samples = synthetic.generate(_short_spec())
    path = tmp_path / "odom.tum"
    synthetic.write_odom_tum(synthetic.derive_odometry(samples), path)
    lines = [ln for ln in path.read_text().splitlines() if not ln.startswith("#")]
    fields = lines[0].split()
    assert len(fields) == 8  # t x y z qx qy qz qw
    # Planar: z = roll = pitch = 0, unit quaternion.
    assert float(fields[3]) == 0.0
    qx, qy, qz, qw = (float(v) for v in fields[4:8])
    assert (qx, qy) == (0.0, 0.0)
    assert abs(qz * qz + qw * qw - 1.0) < 1e-9


# ------------------------------------------------------------------- laser scans


def test_scan_raycast_hits_the_room_walls():
    # A stationary pose at the origin of a known box: beams along the axes must
    # measure the wall distances exactly (zero noise).
    spec = synthetic.ScanSpec(noise_m=0.0, room=(-2.0, 5.0, -2.0, 4.0))
    assert synthetic._raycast_box(0.0, 0.0, 0.0, spec.room) == 5.0
    assert synthetic._raycast_box(0.0, 0.0, math.pi / 2.0, spec.room) == 4.0
    assert abs(synthetic._raycast_box(0.0, 0.0, math.pi, spec.room) - 2.0) < 1e-12


def test_scans_are_consistent_with_groundtruth():
    """Every beam endpoint, placed at the ground-truth pose, must lie on a wall."""
    spec = synthetic.ScanSpec(noise_m=0.0)
    samples = synthetic.generate(_short_spec())
    scans = synthetic.generate_scans(samples, spec)
    gt = {s.t: s for s in samples}
    angle_min = -spec.fov_rad / 2.0
    inc = spec.fov_rad / (spec.n_beams - 1)
    x_min, x_max, y_min, y_max = spec.room
    for t, ranges in scans[:: max(1, len(scans) // 5)]:
        s = gt[t]
        yaw = 2.0 * math.atan2(s.qz, s.qw)
        for i in range(0, len(ranges), 90):
            a = yaw + angle_min + i * inc
            x = s.px + ranges[i] * math.cos(a)
            y = s.py + ranges[i] * math.sin(a)
            on_wall = (
                min(abs(x - x_min), abs(x - x_max), abs(y - y_min), abs(y - y_max))
                < 1e-9
            )
            assert on_wall, f"beam {i} at t={t} ends off-wall at ({x:.3f}, {y:.3f})"


def test_scan_csv_format(tmp_path):
    spec = synthetic.ScanSpec()
    samples = synthetic.generate(_short_spec())
    path = tmp_path / "scan.csv"
    synthetic.write_scan_csv(synthetic.generate_scans(samples, spec), path, spec)
    lines = [ln for ln in path.read_text().splitlines() if not ln.startswith("#")]
    fields = lines[0].split()
    # t angle_min angle_increment range_min range_max n r0..r(n-1)
    n = int(fields[5])
    assert n == spec.n_beams
    assert len(fields) == 6 + n


def test_materialize_synthetic_provides_all_streams(tmp_path):
    seq = datasets.materialize_synthetic(tmp_path, _short_spec())
    assert seq.imu_csv.exists()
    assert seq.groundtruth_tum.exists()
    assert seq.scan_csv is not None and seq.scan_csv.exists()
    assert seq.odom_csv is not None and seq.odom_csv.exists()
