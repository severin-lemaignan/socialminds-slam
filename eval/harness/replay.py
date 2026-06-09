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


def find_replay_binary(build_if_missing: bool = True) -> Path:
    """Return a path to the `slam-replay` binary.

    Resolution order: ``SLAM_REPLAY_BIN`` env var, then a binary on ``PATH``, then the
    Cargo target dirs (release preferred). If still missing and ``build_if_missing``, run
    ``cargo build``.
    """
    env = os.environ.get("SLAM_REPLAY_BIN")
    if env and Path(env).exists():
        return Path(env)

    on_path = shutil.which("slam-replay")
    if on_path:
        return Path(on_path)

    for profile in ("release", "debug"):
        candidate = REPO_ROOT / "target" / profile / "slam-replay"
        if candidate.exists():
            return candidate

    if build_if_missing:
        subprocess.run(
            ["cargo", "build", "--release", "-p", "slam-replay"],
            cwd=REPO_ROOT,
            check=True,
        )
        candidate = REPO_ROOT / "target" / "release" / "slam-replay"
        if candidate.exists():
            return candidate

    raise FileNotFoundError(
        "slam-replay binary not found; set SLAM_REPLAY_BIN or run `cargo build -p slam-replay`."
    )


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
