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


# --------------------------------------------------------------- dynamic objects


def test_person_commutes_between_waypoints():
    p = synthetic.PersonSpec(start=(0.0, 0.0), end=(4.0, 2.0), period_s=8.0)
    assert p.center(0.0) == (0.0, 0.0)
    x, y = p.center(4.0)  # half a period: at the far waypoint
    assert abs(x - 4.0) < 1e-9 and abs(y - 2.0) < 1e-9
    x, y = p.center(8.0)  # full period: back home
    assert abs(x) < 1e-9 and abs(y) < 1e-9


def test_walker_occludes_the_wall():
    # A person standing 2 m in front of a stationary lidar: the central beam must
    # return the person's near edge, not the wall.
    spec = synthetic.ScanSpec(
        noise_m=0.0,
        people=(synthetic.PersonSpec(start=(2.0, 0.0), end=(2.0, 0.0), radius=0.2),),
    )
    sample = synthetic.Sample(
        t=0.0, px=0.0, py=0.0, pz=0.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0,
        gx=0.0, gy=0.0, gz=0.0, ax=0.0, ay=0.0, az=0.0,
    )
    (_, ranges), = synthetic.generate_scans([sample], spec)
    # The beam nearest bearing 0 (the fan has no exact 0 beam).
    beam = round((0.0 - (-spec.fov_rad / 2.0)) / (spec.fov_rad / (spec.n_beams - 1)))
    assert abs(ranges[beam] - 1.8) < 0.01  # 2.0 m to centre − 0.2 m radius


def test_follower_occludes_a_fixed_bearing():
    """The follower must shorten the same body-frame bearing in *every* scan."""
    spec = synthetic.ScanSpec(noise_m=0.0, follower=synthetic.FollowerSpec())
    samples = synthetic.generate(_short_spec())
    scans = synthetic.generate_scans(samples, spec)
    angle_min = -spec.fov_rad / 2.0
    inc = spec.fov_rad / (spec.n_beams - 1)
    f = spec.follower
    beam = round((f.bearing_rad - angle_min) / inc)
    expected = f.distance_m - f.radius
    for _, ranges in scans:
        assert abs(ranges[beam] - expected) < 0.01


def test_dynamic_scans_are_contaminated_but_mostly_walls():
    samples = synthetic.generate(_short_spec())
    clean = synthetic.generate_scans(samples)
    dyn = synthetic.generate_scans(
        samples,
        synthetic.ScanSpec(people=synthetic.DEFAULT_PEOPLE, follower=synthetic.FollowerSpec()),
    )
    total = hit = 0
    for (_, rc), (_, rd) in zip(clean, dyn):
        total += len(rc)
        hit += sum(1 for a, b in zip(rc, rd) if b < a - 0.05)
    # Real contamination (the dynamic variant means something) yet walls dominate
    # (the sequence stays registrable).
    assert 0.02 < hit / total < 0.5


def test_materialize_synthetic_dynamic_is_scan_only(tmp_path):
    seq = datasets.materialize_synthetic_dynamic(tmp_path, _short_spec())
    assert seq.name == "synthetic-dynamic"
    assert seq.imu_csv is None
    assert seq.scan_csv is not None and seq.scan_csv.exists()
    assert seq.groundtruth_tum.exists()
