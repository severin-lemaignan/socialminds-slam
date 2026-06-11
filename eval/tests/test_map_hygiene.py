"""End-to-end map hygiene: free-space carving evicts ghosts (ADR 0014).

Runs the real engine over the synthetic *dynamic* sequence (walkers + follower) and
inspects the final TSDF dump: people must not survive as map geometry. This gate runs
maskless by construction — per ADR 0014, no map-quality property may depend on
dynamics masking.
"""

from __future__ import annotations

import math
import re
import subprocess
from pathlib import Path

import numpy as np
import pytest

from harness import datasets, replay, stsd, synthetic

ROOM = synthetic.ScanSpec().room


def _final_person_positions() -> np.ndarray:
    """Where the walkers + follower stand when the sequence ends.

    A person still present at the end is *truthful* map content — only off-wall
    surface far from every final person position counts as a stale ghost.
    """
    spec = synthetic.TrajectorySpec()
    finals = [p.center(spec.duration_s) for p in synthetic.DEFAULT_PEOPLE]
    s = synthetic.generate(spec)[-1]
    yaw = 2.0 * math.atan2(s.qz, s.qw)
    f = synthetic.FollowerSpec()
    finals.append(
        (
            s.px + f.distance_m * math.cos(yaw + f.bearing_rad),
            s.py + f.distance_m * math.sin(yaw + f.bearing_rad),
        )
    )
    return np.array(finals)


def _replay_or_skip() -> Path:
    try:
        return replay.find_replay_binary(build_if_missing=False)
    except FileNotFoundError:
        pytest.skip("slam-replay not built; run `cargo build -p slam-replay`")


def _run_to_dump(binary: Path, scan_csv: Path, out: Path) -> stsd.MapDump:
    subprocess.run(
        [
            str(binary),
            "--baseline", "scan-matching-3d",
            "--scan", str(scan_csv),
            "--out", str(out.with_suffix(".tum")),
            "--map-out", str(out),
        ],
        check=True,
        capture_output=True,
    )
    return stsd.read_stsd(out)


def test_carving_evicts_walker_ghosts(tmp_path):
    binary = _replay_or_skip()
    dyn = datasets.materialize_synthetic_dynamic(tmp_path / "dyn")
    dump = _run_to_dump(binary, dyn.scan_csv, tmp_path / "dynamic.stsd")

    surface = dump.surface()
    c = dump.centres[surface]
    x_min, x_max, y_min, y_max = ROOM
    d_wall = np.minimum.reduce(
        [np.abs(c[:, 0] - x_min), np.abs(c[:, 0] - x_max), np.abs(c[:, 1] - y_min), np.abs(c[:, 1] - y_max)]
    )
    off_wall = c[d_wall > 0.3]
    d_person = np.min(
        np.linalg.norm(off_wall[:, None, :2] - _final_person_positions()[None, :, :], axis=2),
        axis=1,
    )
    # Stale ghosts: off-wall surface that is NOT a person currently standing there.
    # Pre-carving (ADR 0014 measurement): 2384 stale voxels, 70 % of all surface;
    # with carving: ~30 (the last few keyframes' trails, never re-observed).
    stale = (d_person >= 0.5).sum() / max(surface.sum(), 1)
    assert stale < 0.05, f"stale ghost fraction {stale:.3f} — carving regressed"

    # The walls themselves must still be mapped (carving must not eat the room).
    clean = datasets.materialize_synthetic(tmp_path / "clean")
    clean_dump = _run_to_dump(binary, clean.scan_csv, tmp_path / "clean.stsd")
    assert surface.sum() > 0.5 * clean_dump.surface().sum(), (
        "dynamic-run wall coverage collapsed: "
        f"{surface.sum()} vs clean {clean_dump.surface().sum()}"
    )


def test_clean_map_has_no_ghosts(tmp_path):
    binary = _replay_or_skip()
    seq = datasets.materialize_synthetic(tmp_path / "clean")
    dump = _run_to_dump(binary, seq.scan_csv, tmp_path / "clean.stsd")
    assert stsd.ghost_fraction(dump, ROOM) < 0.01


def test_busy_environment_stays_tracked(tmp_path):
    """The registration-field regression gate (ADR 0014).

    60 s in a dense crowd (~29 % of beams hit people, including a lingering "queue"
    walker), with the wheel-odometry prior the robot always has (ADR 0012). With an
    uncarved registration field this scenario collapses (120 s variant measured:
    ATE 114 m scan-only / 1.7 m with odom; carved: 0.90 m). Short dynamic gates
    cannot catch this — ghosts only bite once current people overlap earlier stamps.
    Observed with carving: ATE 0.31, 98 % matched.
    """
    binary = _replay_or_skip()
    seq = datasets.materialize_synthetic_busy(tmp_path / "busy")
    out = tmp_path / "busy.tum"
    proc = subprocess.run(
        [
            str(binary),
            "--baseline", "scan-matching-3d",
            "--scan", str(seq.scan_csv),
            "--odom", str(seq.odom_csv),
            "--out", str(out),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    from harness import metrics as hm

    ate = hm.ate(seq.groundtruth_tum, out, align=False).rmse
    assert ate < 1.0, f"busy-crowd ATE {ate:.2f} m — registration-field hygiene regressed"

    health = next(l for l in proc.stderr.splitlines() if "front-end health" in l)
    m = re.search(r"(\d+) matched / (\d+) coasted", health)
    matched, coasted = int(m.group(1)), int(m.group(2))
    assert matched / (matched + coasted) > 0.9, f"mostly coasting in the crowd: {health}"
