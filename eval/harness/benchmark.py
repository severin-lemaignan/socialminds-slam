"""Benchmark matrix + report generation.

Runs a grid of (sequence × system) each ``repeats`` times, scoring **accuracy** (ATE/RPE
via `harness.metrics`) and **compute** (latency / real-time factor / peak RSS via
`harness.compute`), then aggregates to mean ± std and renders a JSON + Markdown report.

Repeating each run and reporting mean ± std (not a single number) is how we handle the
non-determinism of real multi-threaded SLAM (ADR 0005); the trivial baselines are
deterministic in accuracy, so this mostly exercises the machinery for now.
"""

from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path

from evo.core.filters import FilterException
from evo.core.geometry import GeometryException

from . import compute, datasets, metrics, replay


@dataclass(frozen=True)
class MeanStd:
    mean: float
    std: float

    def __str__(self) -> str:
        return f"{self.mean:.4g} ± {self.std:.2g}"


def mean_std(values: list[float]) -> MeanStd:
    # Drop None and NaN (e.g. RPE is undefined for a never-moving baseline).
    vals = [v for v in values if v is not None and v == v]
    if not vals:
        return MeanStd(float("nan"), float("nan"))
    m = sum(vals) / len(vals)
    var = sum((v - m) ** 2 for v in vals) / len(vals)
    return MeanStd(m, math.sqrt(var))


@dataclass(frozen=True)
class SystemSpec:
    """A system to benchmark: a `slam-replay` baseline name."""

    name: str
    baseline: str


@dataclass(frozen=True)
class Aggregate:
    system: str
    sequence: str
    source: str
    repeats: int
    ate_rmse_m: MeanStd
    rpe_rmse_m: MeanStd
    real_time_factor: MeanStd
    latency_p99_us: MeanStd
    peak_rss_mb: MeanStd


def run_case(
    seq: datasets.Sequence,
    system: SystemSpec,
    *,
    workdir: Path,
    repeats: int = 3,
    align: bool = True,
    init_pose_from_groundtruth: bool = False,
    replay_bin: Path | None = None,
) -> Aggregate:
    """Run one (sequence, system) cell ``repeats`` times and aggregate."""
    replay_bin = replay_bin or replay.find_replay_binary()
    init_pose = seq.groundtruth_tum if init_pose_from_groundtruth else None

    ate, rpe, rtf, lat_p99, rss = [], [], [], [], []
    for i in range(repeats):
        out_tum = Path(workdir) / f"{system.name}_{seq.name}_{i}.tum"
        stats = compute.run_with_metrics(
            replay_bin, system.baseline, seq.imu_csv, out_tum, init_pose_tum=init_pose
        )
        try:
            ate.append(metrics.ate(seq.groundtruth_tum, out_tum, align=align).rmse)
        except GeometryException:
            # Alignment is degenerate for a constant trajectory (e.g. the stationary
            # baseline); the unaligned ATE is the meaningful value there.
            ate.append(metrics.ate(seq.groundtruth_tum, out_tum, align=False).rmse)
        try:
            rpe.append(metrics.rpe(seq.groundtruth_tum, out_tum, delta=1.0).rmse)
        except FilterException:
            # No pairs at this distance delta (e.g. a non-moving baseline). RPE undefined.
            rpe.append(float("nan"))
        rtf.append(stats.real_time_factor)
        lat_p99.append(stats.latency_us["p99"])
        rss.append(stats.peak_rss_mb)

    return Aggregate(
        system=system.name,
        sequence=seq.name,
        source=seq.source,
        repeats=repeats,
        ate_rmse_m=mean_std(ate),
        rpe_rmse_m=mean_std(rpe),
        real_time_factor=mean_std(rtf),
        latency_p99_us=mean_std(lat_p99),
        peak_rss_mb=mean_std(rss),
    )


def run_matrix(
    sequences: list[datasets.Sequence],
    systems: list[SystemSpec],
    *,
    workdir: Path,
    repeats: int = 3,
    align: bool = True,
    init_pose_from_groundtruth: bool = False,
) -> list[Aggregate]:
    replay_bin = replay.find_replay_binary()
    results = []
    for seq in sequences:
        for system in systems:
            results.append(
                run_case(
                    seq,
                    system,
                    workdir=workdir,
                    repeats=repeats,
                    align=align,
                    init_pose_from_groundtruth=init_pose_from_groundtruth,
                    replay_bin=replay_bin,
                )
            )
    return results


def score_trajectory(
    groundtruth_tum: Path,
    estimate_tum: Path,
    *,
    system: str,
    sequence: str,
    source: str = "reference",
    align: bool = True,
) -> Aggregate:
    """Score an externally-produced TUM trajectory (accuracy only).

    Used to bring a reference system's output (e.g. RTAB-Map or GLIM, run outside this
    repo — see ``eval/reference/``) into the same report as our engine. Compute metrics are
    left NaN since they are not observed here.
    """
    try:
        ate = metrics.ate(groundtruth_tum, estimate_tum, align=align).rmse
    except GeometryException:
        ate = metrics.ate(groundtruth_tum, estimate_tum, align=False).rmse
    try:
        rpe = metrics.rpe(groundtruth_tum, estimate_tum, delta=1.0).rmse
    except FilterException:
        rpe = float("nan")

    nan = MeanStd(float("nan"), float("nan"))
    return Aggregate(
        system=system,
        sequence=sequence,
        source=source,
        repeats=1,
        ate_rmse_m=MeanStd(ate, 0.0),
        rpe_rmse_m=MeanStd(rpe, 0.0),
        real_time_factor=nan,
        latency_p99_us=nan,
        peak_rss_mb=nan,
    )


def to_markdown(results: list[Aggregate]) -> str:
    header = (
        "| System | Sequence | ATE RMSE (m) | RPE RMSE (m) | Real-time × | "
        "Latency p99 (µs) | Peak RSS (MB) |\n"
        "|---|---|---|---|---|---|---|\n"
    )
    rows = "".join(
        f"| {r.system} | {r.sequence} ({r.source}) | {r.ate_rmse_m} | {r.rpe_rmse_m} | "
        f"{r.real_time_factor} | {r.latency_p99_us} | {r.peak_rss_mb} |\n"
        for r in results
    )
    return header + rows


def _aggregate_to_dict(r: Aggregate) -> dict:
    d = asdict(r)
    # Flatten MeanStd dataclasses already handled by asdict (nested dicts).
    return d


def write_report(results: list[Aggregate], out_dir: Path) -> tuple[Path, Path]:
    """Write ``results.json`` and ``report.md``; return their paths."""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    json_path = out_dir / "results.json"
    md_path = out_dir / "report.md"

    json_path.write_text(json.dumps([_aggregate_to_dict(r) for r in results], indent=2))
    md_path.write_text("# SLAM benchmark report\n\n" + to_markdown(results) + "\n")
    return json_path, md_path


def default_systems() -> list[SystemSpec]:
    return [
        SystemSpec(name="stationary", baseline="stationary"),
        SystemSpec(name="imu_dead_reckoning", baseline="dead-reckoning"),
    ]


def gather_sequences(
    euroc: list[str], openloris: list[str], synthetic: bool, workdir: Path
) -> list[datasets.Sequence]:
    """Materialise the requested sequences under ``workdir``.

    Real datasets are picked up from the `harness.fetch` cache (never downloaded here);
    the synthetic sequence is included when asked for — or as the default when nothing
    real is requested, which keeps the bare `python -m harness.benchmark` download-free.
    """
    from . import fetch

    seqs = []
    if synthetic or not (euroc or openloris):
        seqs.append(datasets.materialize_synthetic(workdir / "synthetic"))
    for name in euroc:
        mav0 = fetch.locate_euroc(name)
        seqs.append(datasets.convert_euroc(mav0, workdir / name, name=name))
    for name in openloris:
        bag, gt = fetch.locate_openloris(name)
        # OpenLORIS bags are bz2-compressed inside, and the split gyro/accel topics mean
        # *two* full decompression passes (~minutes each) — so materialise into the data
        # cache, not the throwaway workdir, and reuse. Delete the dir to force re-extraction.
        cache = fetch.cache_root() / "openloris" / "_materialized" / name
        imu_csv, gt_tum = cache / "imu.csv", cache / "groundtruth.tum"
        if imu_csv.exists() and gt_tum.exists():
            seqs.append(
                datasets.Sequence(
                    name=name,
                    source="openloris",
                    imu_csv=imu_csv,
                    groundtruth_tum=gt_tum,
                    duration_s=datasets._imu_csv_duration(imu_csv),
                    has_gyro=True,
                )
            )
        else:
            print(
                f"extracting IMU from {bag.name} (bz2-compressed, two passes — takes a "
                f"few minutes; cached under {cache} afterwards)",
                flush=True,
            )
            seqs.append(
                datasets.materialize_openloris(
                    bag,
                    gt,
                    cache,
                    name=name,
                    gyro_topic=datasets.OPENLORIS_GYRO_TOPIC,
                    accel_topic=datasets.OPENLORIS_ACCEL_TOPIC,
                )
            )
    return seqs


def main(argv: list[str] | None = None) -> int:
    import argparse
    import tempfile

    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--repeats", type=int, default=3)
    p.add_argument(
        "--out-dir",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "results",
        help="where to write results.json + report.md (default eval/results)",
    )
    p.add_argument(
        "--euroc",
        action="append",
        default=[],
        metavar="SEQ",
        help=f"add a cached EuRoC sequence, e.g. MH_01_easy (one of {sorted(datasets.EUROC_SEQUENCES)}); repeatable",
    )
    p.add_argument(
        "--openloris",
        action="append",
        default=[],
        metavar="SEQ",
        help="add a cached OpenLORIS sequence, e.g. cafe1-1 (bag + ground truth under $SLAM_DATA_DIR); repeatable",
    )
    p.add_argument(
        "--synthetic",
        action="store_true",
        help="include the synthetic sequence alongside real ones (it is the default when none are requested)",
    )
    p.add_argument(
        "--init-pose-from-gt",
        action="store_true",
        help="seed each run with the ground-truth initial pose (gravity-aligns dead-reckoning on real data)",
    )
    args = p.parse_args(argv)

    with tempfile.TemporaryDirectory(prefix="slam-bench-") as tmp:
        tmp = Path(tmp)
        seqs = gather_sequences(args.euroc, args.openloris, args.synthetic, tmp)
        results = run_matrix(
            seqs,
            default_systems(),
            workdir=tmp,
            repeats=args.repeats,
            init_pose_from_groundtruth=args.init_pose_from_gt,
        )

    json_path, md_path = write_report(results, args.out_dir)
    print(to_markdown(results))
    print(f"wrote {json_path}\nwrote {md_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
