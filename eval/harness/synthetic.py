"""Synthetic trajectory + IMU + wheel-odometry + 2D-laser-scan generator.

Produces a *ground-truth* trajectory and sensor streams that are exactly consistent with
it, with **no downloads and no GPU** — the zero-dependency dataset the CI benchmark runs
on (ADR 0005). Beyond the IMU it can derive:

- a **wheel-odometry** stream (TUM format) with a deterministic imperfection model
  (translation scale error + yaw-rate bias — the two dominant wheel-odometry failure
  modes), so `odom_dead_reckoning` shows bounded, reproducible drift;
- **2D laser scans** raycast against a rectangular room around the trajectory, with
  deterministic per-beam range noise, so the scan front-ends run end-to-end in CI.

Design constraints that make the IMU baselines reconstructable:

- The trajectory starts **at rest at the identity pose** (`p(0)=0`, `v(0)=0`, `R(0)=I`),
  matching `slam-replay`'s dead-reckoning initial state, so any tracking error is *drift*,
  not an initialisation mismatch.
- Position and yaw are smooth analytic functions, so the body-frame specific force and
  angular rate are computed in closed form (no numerical-differentiation noise).

The IMU convention matches `slam_types::sensor` / `slam-baseline`: the accelerometer
reports specific force ``f_b = Rᵀ (a_world − g_vec)`` with ``g_vec = (0, 0, −g)``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

STANDARD_GRAVITY = 9.80665


@dataclass(frozen=True)
class TrajectorySpec:
    """A smooth, start-at-rest motion: each axis follows ``a·(1 − cos(ω·t))``.

    The ``(1 − cos)`` shape guarantees zero initial position and velocity, and the yaw the
    same, so the body starts level and stationary at the origin.
    """

    duration_s: float = 20.0
    rate_hz: float = 200.0
    gravity: float = STANDARD_GRAVITY
    # Position amplitudes (m) and angular frequencies (rad/s) per axis.
    amp_x: float = 1.5
    amp_y: float = 1.0
    amp_z: float = 0.05
    w_x: float = 2.0 * math.pi / 10.0   # one X cycle / 10 s
    w_y: float = 2.0 * math.pi / 7.0
    w_z: float = 2.0 * math.pi / 5.0
    # Yaw motion: yaw(t) = yaw_amp·(1 − cos(w_yaw·t)).
    yaw_amp: float = 0.8
    w_yaw: float = 2.0 * math.pi / 13.0


@dataclass(frozen=True)
class Sample:
    t: float
    # Ground-truth pose.
    px: float
    py: float
    pz: float
    qx: float
    qy: float
    qz: float
    qw: float
    # IMU (body frame).
    gx: float
    gy: float
    gz: float
    ax: float
    ay: float
    az: float


def _axis(amp: float, w: float, t: float):
    """Return (position, acceleration) for one ``amp·(1−cos(w·t))`` axis."""
    pos = amp * (1.0 - math.cos(w * t))
    acc = amp * w * w * math.cos(w * t)  # second derivative
    return pos, acc


def generate(spec: TrajectorySpec = TrajectorySpec()) -> list[Sample]:
    """Generate the full list of samples for a spec."""
    n = int(round(spec.duration_s * spec.rate_hz)) + 1
    out: list[Sample] = []
    g = spec.gravity
    for i in range(n):
        t = i / spec.rate_hz

        px, ax_w = _axis(spec.amp_x, spec.w_x, t)
        py, ay_w = _axis(spec.amp_y, spec.w_y, t)
        pz, az_w = _axis(spec.amp_z, spec.w_z, t)

        yaw = spec.yaw_amp * (1.0 - math.cos(spec.w_yaw * t))
        yaw_rate = spec.yaw_amp * spec.w_yaw * math.sin(spec.w_yaw * t)

        # Orientation: rotation about world +Z by yaw.
        c, s = math.cos(yaw), math.sin(yaw)
        qz, qw = math.sin(yaw / 2.0), math.cos(yaw / 2.0)

        # Specific force f_b = Rᵀ (a_world − g_vec), g_vec = (0,0,−g) → a_world + (0,0,g).
        wx, wy, wz = ax_w, ay_w, az_w + g
        # Rᵀ for a +yaw rotation about Z is [[c, s, 0], [−s, c, 0], [0, 0, 1]].
        fbx = c * wx + s * wy
        fby = -s * wx + c * wy
        fbz = wz

        # Angular rate is purely about body Z (= world Z for a yaw-only rotation).
        out.append(
            Sample(
                t=t,
                px=px, py=py, pz=pz,
                qx=0.0, qy=0.0, qz=qz, qw=qw,
                gx=0.0, gy=0.0, gz=yaw_rate,
                ax=fbx, ay=fby, az=fbz,
            )
        )
    return out


def write_imu_csv(samples: list[Sample], path: Path) -> None:
    """Write the IMU stream in the `slam-replay` CSV format."""
    path = Path(path)
    with path.open("w") as f:
        f.write("# t gx gy gz ax ay az  (seconds, rad/s, m/s^2)\n")
        for s in samples:
            f.write(
                f"{s.t:.9f} {s.gx:.9f} {s.gy:.9f} {s.gz:.9f} "
                f"{s.ax:.9f} {s.ay:.9f} {s.az:.9f}\n"
            )


def write_groundtruth_tum(samples: list[Sample], path: Path) -> None:
    """Write the ground-truth trajectory in TUM format."""
    path = Path(path)
    with path.open("w") as f:
        f.write("# timestamp tx ty tz qx qy qz qw\n")
        for s in samples:
            f.write(
                f"{s.t:.9f} {s.px:.9f} {s.py:.9f} {s.pz:.9f} "
                f"{s.qx:.9f} {s.qy:.9f} {s.qz:.9f} {s.qw:.9f}\n"
            )


# ------------------------------------------------------------------- wheel odometry


@dataclass(frozen=True)
class OdomSpec:
    """Deterministic wheel-odometry imperfection.

    The two dominant failure modes of real wheel odometry, applied to the ground-truth
    relative motion: a translation **scale** error (wrong wheel radius / slip) and a
    **yaw-rate bias** (heading drift). Deterministic, so the resulting baseline error is
    stable enough to gate on.
    """

    rate_hz: float = 20.0
    scale: float = 1.02
    yaw_rate_bias: float = 0.004  # rad/s


def _yaw(s: Sample) -> float:
    """Yaw of a yaw-only quaternion (the generator's orientations are yaw-only)."""
    return 2.0 * math.atan2(s.qz, s.qw)


def derive_odometry(samples: list[Sample], spec: OdomSpec = OdomSpec()) -> list[tuple]:
    """Integrate spec-perturbed ground-truth relative motion → `(t, x, y, yaw)` rows.

    The stream is the platform's pose in its own odometry frame (planar, like a real
    wheel-odometry estimate): exact at t=0, drifting with path length afterwards.
    """
    if not samples:
        return []
    rate = samples[1].t - samples[0].t if len(samples) > 1 else 0.0
    stride = max(1, round(1.0 / (spec.rate_hz * rate))) if rate > 0 else 1
    picked = samples[::stride]

    out = [(picked[0].t, 0.0, 0.0, 0.0)]
    x = y = th = 0.0
    for prev, cur in zip(picked, picked[1:]):
        # Ground-truth relative motion in the previous body frame...
        c, s = math.cos(_yaw(prev)), math.sin(_yaw(prev))
        wx, wy = cur.px - prev.px, cur.py - prev.py
        dx, dy = c * wx + s * wy, -s * wx + c * wy
        dth = _yaw(cur) - _yaw(prev)
        # ...perturbed by the imperfection model, composed onto the odometry pose.
        dt = cur.t - prev.t
        dx, dy = dx * spec.scale, dy * spec.scale
        dth += spec.yaw_rate_bias * dt
        c, s = math.cos(th), math.sin(th)
        x += c * dx - s * dy
        y += s * dx + c * dy
        th += dth
        out.append((cur.t, x, y, th))
    return out


def write_odom_tum(rows: list[tuple], path: Path) -> None:
    """Write the odometry stream in TUM format (`slam-replay --odom`)."""
    with Path(path).open("w") as f:
        f.write("# timestamp tx ty tz qx qy qz qw  (planar wheel odometry)\n")
        for t, x, y, th in rows:
            f.write(
                f"{t:.9f} {x:.9f} {y:.9f} 0.0 "
                f"0.0 0.0 {math.sin(th / 2.0):.9f} {math.cos(th / 2.0):.9f}\n"
            )


# ----------------------------------------------------------------------- laser scans


@dataclass(frozen=True)
class ScanSpec:
    """A planar lidar in a rectangular room enclosing the trajectory.

    The default room leaves 2–3 m of clearance around the default trajectory's
    [0, 3] × [0, 2] footprint; the 270° fan always sees at least two walls, so the
    planar solve is never degenerate.
    """

    rate_hz: float = 10.0
    n_beams: int = 540
    fov_rad: float = 1.5 * math.pi
    range_min: float = 0.05
    range_max: float = 25.0
    noise_m: float = 0.01
    # Room walls: (x_min, x_max, y_min, y_max).
    room: tuple[float, float, float, float] = (-2.0, 5.0, -2.0, 4.0)


def _noise_unit(beam: int, scan: int) -> float:
    """Deterministic pseudo-noise in [-1, 1) — no RNG state, reproducible per beam."""
    v = math.sin(beam * 12.9898 + scan * 78.233) * 43758.5453
    return 2.0 * (v - math.floor(v)) - 1.0


def _raycast_box(px: float, py: float, angle: float, room: tuple) -> float:
    """Distance from (px, py) along `angle` to the enclosing box (always hits)."""
    x_min, x_max, y_min, y_max = room
    dx, dy = math.cos(angle), math.sin(angle)
    best = math.inf
    if dx > 1e-12:
        best = min(best, (x_max - px) / dx)
    elif dx < -1e-12:
        best = min(best, (x_min - px) / dx)
    if dy > 1e-12:
        best = min(best, (y_max - py) / dy)
    elif dy < -1e-12:
        best = min(best, (y_min - py) / dy)
    return best


def generate_scans(samples: list[Sample], spec: ScanSpec = ScanSpec()) -> list[tuple]:
    """Raycast scans at the ground-truth poses → `(t, ranges)` rows."""
    if not samples:
        return []
    rate = samples[1].t - samples[0].t if len(samples) > 1 else 0.0
    stride = max(1, round(1.0 / (spec.rate_hz * rate))) if rate > 0 else 1
    angle_min = -spec.fov_rad / 2.0
    inc = spec.fov_rad / (spec.n_beams - 1)

    out = []
    for k, s in enumerate(samples[::stride]):
        yaw = _yaw(s)
        ranges = []
        for i in range(spec.n_beams):
            r = _raycast_box(s.px, s.py, yaw + angle_min + i * inc, spec.room)
            r += spec.noise_m * _noise_unit(i, k)
            ranges.append(r if spec.range_min <= r <= spec.range_max else math.inf)
        out.append((s.t, ranges))
    return out


def write_scan_csv(scans: list[tuple], path: Path, spec: ScanSpec = ScanSpec()) -> None:
    """Write scans in the `slam-replay --scan` CSV format."""
    angle_min = -spec.fov_rad / 2.0
    inc = spec.fov_rad / (spec.n_beams - 1)
    with Path(path).open("w") as f:
        f.write("# t angle_min angle_increment range_min range_max n r0 .. r(n-1)\n")
        for t, ranges in scans:
            head = (
                f"{t:.9f} {angle_min:.9f} {inc:.9f} "
                f"{spec.range_min} {spec.range_max} {len(ranges)}"
            )
            f.write(head + " " + " ".join(f"{r:.4f}" for r in ranges) + "\n")


def main(argv: list[str] | None = None) -> int:
    import argparse

    p = argparse.ArgumentParser(description="Generate a synthetic trajectory + IMU stream.")
    p.add_argument("--out-dir", type=Path, required=True, help="directory for imu.csv + groundtruth.tum")
    p.add_argument("--duration", type=float, default=TrajectorySpec.duration_s)
    p.add_argument("--rate", type=float, default=TrajectorySpec.rate_hz)
    args = p.parse_args(argv)

    spec = TrajectorySpec(duration_s=args.duration, rate_hz=args.rate)
    samples = generate(spec)
    args.out_dir.mkdir(parents=True, exist_ok=True)
    write_imu_csv(samples, args.out_dir / "imu.csv")
    write_groundtruth_tum(samples, args.out_dir / "groundtruth.tum")
    write_odom_tum(derive_odometry(samples), args.out_dir / "odom.tum")
    write_scan_csv(generate_scans(samples), args.out_dir / "scan.csv")
    print(f"wrote {len(samples)} samples to {args.out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
