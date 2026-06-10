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
import subprocess
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
    """A system to benchmark: a `slam-replay` system name + its primary input stream."""

    name: str
    baseline: str
    # Which sequence stream the system consumes: "imu" or "scan". Sequences lacking it
    # skip the system (e.g. scan matching on the scan-less EuRoC).
    input: str = "imu"


def _sequence_inputs(seq: datasets.Sequence, system: SystemSpec) -> dict:
    """`compute.run_with_metrics` input kwargs for this (sequence, system).

    The system's primary stream is passed — as a CSV, or streamed directly from the
    ROS1 bag when the sequence is bag-backed. Raises if the sequence lacks the stream.
    """
    if system.input == "imu":
        if seq.bag is not None:
            return {
                "bag": seq.bag,
                "gyro_topic": seq.bag_gyro_topic,
                "accel_topic": seq.bag_accel_topic,
            }
        return {"imu_csv": seq.imu_csv}
    if system.input == "scan":
        if seq.bag is not None and seq.bag_scan_topic is not None:
            return {"bag": seq.bag, "scan_topic": seq.bag_scan_topic}
        if seq.scan_csv is None:
            raise ValueError(f"{seq.name} has no scan stream for {system.name}")
        return {"scan_csv": seq.scan_csv}
    raise ValueError(f"unknown system input kind {system.input!r}")


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

    inputs = _sequence_inputs(seq, system)
    imu_csv = inputs.pop("imu_csv", None)
    ate, rpe, rtf, lat_p99, rss = [], [], [], [], []
    for i in range(repeats):
        out_tum = Path(workdir) / f"{system.name}_{seq.name}_{i}.tum"
        stats = compute.run_with_metrics(
            replay_bin,
            system.baseline,
            imu_csv,
            out_tum,
            init_pose_tum=init_pose,
            **inputs,
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
            if system.input == "scan" and seq.scan_csv is None and seq.bag_scan_topic is None:
                print(f"skipping {system.name} on {seq.name}: no scan stream")
                continue
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
        SystemSpec(name="scan_matching", baseline="scan-matching", input="scan"),
        # Scan-to-submap TSDF registration (ADR 0010); the planar matcher above stays
        # in-tree as the parity reference (eval/reference/baselines/m3-planar-frontend).
        SystemSpec(name="scan_matching_3d", baseline="scan-matching-3d", input="scan"),
    ]


def gather_sequences(
    euroc: list[str],
    openloris: list[str],
    synthetic: bool,
    workdir: Path,
    *,
    direct_bag: bool = False,
) -> list[datasets.Sequence]:
    """Materialise the requested sequences under ``workdir``.

    Real datasets are picked up from the `harness.fetch` cache (never downloaded here);
    the synthetic sequence is included when asked for — or as the default when nothing
    real is requested, which keeps the bare `python -m harness.benchmark` download-free.
    With ``direct_bag``, OpenLORIS sequences skip CSV materialisation entirely:
    `slam-replay --bag` streams the topics straight from the bag on every run.
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
        # Not every OpenLORIS scene has a 2D laser: the `market` bags (Scrubber 75
        # robot) are RGB-D/VIO-only. A complete CSV cache answers without touching the
        # (multi-GB) bag; otherwise probe its index once. Scan systems skip cleanly.
        cache = fetch.cache_root() / "openloris" / "_materialized" / name
        imu_csv, gt_tum = cache / "imu.csv", cache / "groundtruth.tum"
        scan_csv = cache / "scan.csv"
        if not direct_bag and imu_csv.exists() and gt_tum.exists() and scan_csv.exists():
            has_scan = True
        else:
            has_scan = datasets.OPENLORIS_SCAN_TOPIC in datasets.bag_topics(bag)
            if not has_scan:
                print(
                    f"{name}: no {datasets.OPENLORIS_SCAN_TOPIC} topic (RGB-D/VIO-only "
                    "bag); scan systems will be skipped",
                    flush=True,
                )
        if direct_bag:
            seqs.append(
                datasets.Sequence(
                    name=name,
                    source="openloris",
                    imu_csv=None,
                    groundtruth_tum=gt,
                    duration_s=datasets._tum_duration(gt),
                    has_gyro=True,
                    bag=bag,
                    bag_gyro_topic=datasets.OPENLORIS_GYRO_TOPIC,
                    bag_accel_topic=datasets.OPENLORIS_ACCEL_TOPIC,
                    bag_scan_topic=datasets.OPENLORIS_SCAN_TOPIC if has_scan else None,
                )
            )
            continue
        # Materialise into the data cache, not the throwaway workdir, and reuse across
        # runs. Delete the dir to force re-extraction.
        if imu_csv.exists() and gt_tum.exists():
            if has_scan and not scan_csv.exists():
                # IMU-only cache from before the scan front-end: add the scan stream.
                print(f"extracting scans from {bag.name}", flush=True)
                subprocess.run(
                    [
                        str(replay.find_bag2scan_binary()),
                        "--bag", str(bag),
                        "--out", str(scan_csv),
                    ],
                    check=True,
                )
            seqs.append(
                datasets.Sequence(
                    name=name,
                    source="openloris",
                    imu_csv=imu_csv,
                    groundtruth_tum=gt_tum,
                    duration_s=datasets._imu_csv_duration(imu_csv),
                    has_gyro=True,
                    scan_csv=scan_csv if has_scan else None,
                )
            )
        else:
            print(f"extracting sensor streams from {bag.name} into {cache}", flush=True)
            seqs.append(
                datasets.materialize_openloris(
                    bag,
                    gt,
                    cache,
                    name=name,
                    gyro_topic=datasets.OPENLORIS_GYRO_TOPIC,
                    accel_topic=datasets.OPENLORIS_ACCEL_TOPIC,
                    extract_scans=has_scan,
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
        "--direct-bag",
        action="store_true",
        help="stream OpenLORIS topics straight from the bag (slam-replay --bag) instead of materialising CSVs",
    )
    p.add_argument(
        "--init-pose-from-gt",
        action="store_true",
        help="seed each run with the ground-truth initial pose (gravity-aligns dead-reckoning on real data)",
    )
    args = p.parse_args(argv)

    with tempfile.TemporaryDirectory(prefix="slam-bench-") as tmp:
        tmp = Path(tmp)
        seqs = gather_sequences(
            args.euroc, args.openloris, args.synthetic, tmp, direct_bag=args.direct_bag
        )
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
