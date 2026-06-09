"""Synthetic trajectory + IMU generator.

Produces a *ground-truth* trajectory and an IMU stream that is exactly consistent with
it, with **no downloads and no GPU** — the zero-dependency dataset the CI benchmark runs
on (ADR 0005).

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
    print(f"wrote {len(samples)} samples to {args.out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
