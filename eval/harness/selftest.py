"""End-to-end harness self-test / smoke benchmark.

Generates a synthetic sequence, runs the trivial baselines through the Rust engine, scores
them with ATE/RPE, and **gates** on the expected ordering. This is the M0 acceptance
check (ADR 0005) and the job CI runs:

- the pipeline (generate → engine → TUM → evo) works end to end, GPU-free;
- ``dead-reckoning`` beats ``stationary`` on a moving sequence;
- ``dead-reckoning`` drift stays within an absolute bound.

Run: ``python -m harness.selftest`` (from ``eval/``).
"""

from __future__ import annotations

import argparse
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path

from . import metrics, replay, synthetic


@dataclass(frozen=True)
class Gates:
    # A static estimate must be badly wrong on this moving sequence...
    stationary_ate_min: float = 0.5
    # ...dead-reckoning must beat it by a clear margin...
    dr_beats_stationary_ratio: float = 0.5
    # ...and its absolute drift must stay bounded. Observed ~0.03 m on this sequence;
    # 0.10 leaves headroom for cross-platform float variance while catching regressions.
    dr_ate_max: float = 0.10


def _fmt(stats: metrics.ErrorStats) -> str:
    return f"rmse={stats.rmse:.4f}  mean={stats.mean:.4f}  max={stats.max:.4f}"


def run(workdir: Path, gates: Gates = Gates()) -> bool:
    workdir.mkdir(parents=True, exist_ok=True)

    # 1. Synthesize a known sequence (no download, no GPU).
    spec = synthetic.TrajectorySpec()
    samples = synthetic.generate(spec)
    imu_csv = workdir / "imu.csv"
    gt_tum = workdir / "groundtruth.tum"
    synthetic.write_imu_csv(samples, imu_csv)
    synthetic.write_groundtruth_tum(samples, gt_tum)

    # 2. Run both baselines through the engine.
    binary = replay.find_replay_binary()
    stationary_tum = replay.run_baseline("stationary", imu_csv, workdir / "stationary.tum", binary=binary)
    dr_tum = replay.run_baseline("dead-reckoning", imu_csv, workdir / "dead_reckoning.tum", binary=binary)

    # 3. Score (ATE unaligned — both start at the origin identity, so it is meaningful).
    stat_ate = metrics.ate(gt_tum, stationary_tum, align=False)
    dr_ate = metrics.ate(gt_tum, dr_tum, align=False)
    dr_rpe = metrics.rpe(gt_tum, dr_tum, delta=1.0)

    print(f"sequence: {spec.duration_s:.0f} s @ {spec.rate_hz:.0f} Hz, {len(samples)} samples")
    print(f"  stationary       ATE: {_fmt(stat_ate)}")
    print(f"  dead-reckoning   ATE: {_fmt(dr_ate)}")
    print(f"  dead-reckoning   RPE(1m): {_fmt(dr_rpe)}")

    # 4. Gate.
    checks = [
        (
            "stationary is far off on a moving sequence",
            stat_ate.rmse >= gates.stationary_ate_min,
            f"{stat_ate.rmse:.4f} >= {gates.stationary_ate_min}",
        ),
        (
            "dead-reckoning beats stationary",
            dr_ate.rmse <= gates.dr_beats_stationary_ratio * stat_ate.rmse,
            f"{dr_ate.rmse:.4f} <= {gates.dr_beats_stationary_ratio} * {stat_ate.rmse:.4f}",
        ),
        (
            "dead-reckoning drift bounded",
            dr_ate.rmse <= gates.dr_ate_max,
            f"{dr_ate.rmse:.4f} <= {gates.dr_ate_max}",
        ),
    ]
    ok = True
    print("gates:")
    for name, passed, detail in checks:
        print(f"  [{'PASS' if passed else 'FAIL'}] {name}  ({detail})")
        ok = ok and passed
    return ok


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--keep",
        action="store_true",
        help="keep generated artifacts in eval/_run instead of a temp dir",
    )
    args = p.parse_args(argv)

    if args.keep:
        workdir = Path(__file__).resolve().parents[1] / "_run"
        ok = run(workdir)
    else:
        with tempfile.TemporaryDirectory(prefix="slam-selftest-") as tmp:
            ok = run(Path(tmp))

    print("\nSELF-TEST:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
