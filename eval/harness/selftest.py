"""End-to-end harness self-test / smoke benchmark.

Generates a synthetic sequence, runs the trivial baselines through the Rust engine via the
benchmark machinery, and **gates** on the expected ordering. This is the M0/M1 acceptance
check (ADR 0005) and the job CI runs:

- the pipeline (generate → engine → TUM → evo + compute metrics) works end to end, GPU-free;
- ``dead-reckoning`` beats ``stationary`` on a moving sequence;
- ``dead-reckoning`` drift stays within an absolute bound and runs faster than real time.

Run: ``python -m harness.selftest`` (from ``eval/``).
"""

from __future__ import annotations

import argparse
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path

from . import benchmark, datasets, replay


@dataclass(frozen=True)
class Gates:
    # A static estimate must be badly wrong on this moving sequence...
    stationary_ate_min: float = 0.5
    # ...dead-reckoning must beat it by a clear margin...
    dr_beats_stationary_ratio: float = 0.5
    # ...its absolute drift must stay bounded (observed ~0.03 m; headroom for float variance)...
    dr_ate_max: float = 0.10
    # ...and it must run faster than real time.
    dr_min_real_time_factor: float = 1.0


def run(workdir: Path, gates: Gates = Gates()) -> bool:
    workdir = Path(workdir)
    workdir.mkdir(parents=True, exist_ok=True)

    seq = datasets.materialize_synthetic(workdir / "synthetic")
    replay_bin = replay.find_replay_binary()

    # align=False: both baselines start at the origin identity, so unaligned ATE is the
    # honest global error here (and keeps the gate thresholds meaningful).
    common = dict(workdir=workdir, repeats=1, align=False, replay_bin=replay_bin)
    stat = benchmark.run_case(seq, benchmark.SystemSpec("stationary", "stationary"), **common)
    dr = benchmark.run_case(
        seq, benchmark.SystemSpec("imu_dead_reckoning", "dead-reckoning"), **common
    )

    print(f"sequence: {seq.duration_s:.0f} s synthetic, {seq.source}")
    print(f"  stationary      ATE rmse: {stat.ate_rmse_m.mean:.4f}")
    print(f"  dead-reckoning  ATE rmse: {dr.ate_rmse_m.mean:.4f}  RPE: {dr.rpe_rmse_m.mean:.4f}")
    print(
        f"  dead-reckoning  compute: {dr.real_time_factor.mean:.0f}x real-time, "
        f"p99 {dr.latency_p99_us.mean:.2f} us, peak {dr.peak_rss_mb.mean:.1f} MB"
    )

    checks = [
        (
            "stationary is far off on a moving sequence",
            stat.ate_rmse_m.mean >= gates.stationary_ate_min,
            f"{stat.ate_rmse_m.mean:.4f} >= {gates.stationary_ate_min}",
        ),
        (
            "dead-reckoning beats stationary",
            dr.ate_rmse_m.mean <= gates.dr_beats_stationary_ratio * stat.ate_rmse_m.mean,
            f"{dr.ate_rmse_m.mean:.4f} <= {gates.dr_beats_stationary_ratio} * {stat.ate_rmse_m.mean:.4f}",
        ),
        (
            "dead-reckoning drift bounded",
            dr.ate_rmse_m.mean <= gates.dr_ate_max,
            f"{dr.ate_rmse_m.mean:.4f} <= {gates.dr_ate_max}",
        ),
        (
            "dead-reckoning runs in real time",
            dr.real_time_factor.mean >= gates.dr_min_real_time_factor,
            f"{dr.real_time_factor.mean:.1f} >= {gates.dr_min_real_time_factor}",
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
        ok = run(Path(__file__).resolve().parents[1] / "_run")
    else:
        with tempfile.TemporaryDirectory(prefix="slam-selftest-") as tmp:
            ok = run(Path(tmp))

    print("\nSELF-TEST:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
