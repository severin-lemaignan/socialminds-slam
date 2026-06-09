"""Drive the Rust engine (`slam-replay`) from Python.

Locates the compiled binary (or builds it on demand) and runs a chosen baseline over an
IMU CSV, producing a TUM trajectory. This is the seam between the harness and the engine:
the harness only ever sees *inputs in, TUM trajectory out*.
"""

from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path

# eval/harness/replay.py -> repo root is two parents up.
REPO_ROOT = Path(__file__).resolve().parents[2]


def find_binary(name: str, env_var: str, package: str, *, build_if_missing: bool = True) -> Path:
    """Locate a workspace binary.

    Resolution order: ``$<env_var>``, then ``PATH``, then the Cargo target dirs (release
    preferred). If still missing and ``build_if_missing``, build ``package``.
    """
    env = os.environ.get(env_var)
    if env and Path(env).exists():
        return Path(env)

    on_path = shutil.which(name)
    if on_path:
        return Path(on_path)

    for profile in ("release", "debug"):
        candidate = REPO_ROOT / "target" / profile / name
        if candidate.exists():
            return candidate

    if build_if_missing:
        subprocess.run(["cargo", "build", "--release", "-p", package], cwd=REPO_ROOT, check=True)
        candidate = REPO_ROOT / "target" / "release" / name
        if candidate.exists():
            return candidate

    raise FileNotFoundError(
        f"{name} binary not found; set {env_var} or run `cargo build -p {package}`."
    )


def find_replay_binary(build_if_missing: bool = True) -> Path:
    return find_binary("slam-replay", "SLAM_REPLAY_BIN", "slam-replay", build_if_missing=build_if_missing)


def find_bag2imu_binary(build_if_missing: bool = True) -> Path:
    return find_binary("slam-bag2imu", "SLAM_BAG2IMU_BIN", "slam-datasets", build_if_missing=build_if_missing)


def find_bag2scan_binary(build_if_missing: bool = True) -> Path:
    return find_binary("slam-bag2scan", "SLAM_BAG2SCAN_BIN", "slam-datasets", build_if_missing=build_if_missing)


def find_bag2csv_binary(build_if_missing: bool = True) -> Path:
    return find_binary("slam-bag2csv", "SLAM_BAG2CSV_BIN", "slam-datasets", build_if_missing=build_if_missing)


def run_baseline(baseline: str, imu_csv: Path, out_tum: Path, *, binary: Path | None = None) -> Path:
    """Run one baseline (`stationary` | `dead-reckoning`) and return the output path."""
    binary = binary or find_replay_binary()
    out_tum = Path(out_tum)
    out_tum.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(
        [
            str(binary),
            "--baseline", baseline,
            "--imu", str(imu_csv),
            "--out", str(out_tum),
        ],
        check=True,
    )
    return out_tum
