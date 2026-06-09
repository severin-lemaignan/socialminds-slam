"""Trajectory accuracy metrics (ATE / RPE) via the `evo` toolkit.

We standardise on:

- **ATE** — Absolute Trajectory Error, translation RMSE after timestamp association.
  Optional SE(3) Umeyama alignment (`align=True`); scale is known for our sensors, so we
  never use Sim(3). ATE is the global-consistency number loop closure should improve.
- **RPE** — Relative Pose Error over a fixed delta; the local drift number, independent of
  global alignment.

See ADR 0005 for the methodology.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from evo.core import metrics, sync
from evo.core.trajectory import PoseTrajectory3D
from evo.tools import file_interface


@dataclass(frozen=True)
class ErrorStats:
    rmse: float
    mean: float
    median: float
    std: float
    min: float
    max: float

    @classmethod
    def from_metric(cls, m: metrics.PE) -> "ErrorStats":
        return cls(
            rmse=m.get_statistic(metrics.StatisticsType.rmse),
            mean=m.get_statistic(metrics.StatisticsType.mean),
            median=m.get_statistic(metrics.StatisticsType.median),
            std=m.get_statistic(metrics.StatisticsType.std),
            min=m.get_statistic(metrics.StatisticsType.min),
            max=m.get_statistic(metrics.StatisticsType.max),
        )


def _load_pair(ref_tum: Path, est_tum: Path) -> tuple[PoseTrajectory3D, PoseTrajectory3D]:
    ref = file_interface.read_tum_trajectory_file(str(ref_tum))
    est = file_interface.read_tum_trajectory_file(str(est_tum))
    # Associate by timestamp (max 10 ms difference) before comparing.
    ref, est = sync.associate_trajectories(ref, est, max_diff=0.01)
    return ref, est


def ate(ref_tum: Path, est_tum: Path, *, align: bool = False) -> ErrorStats:
    """Absolute Trajectory Error (translation), optionally SE(3)-aligned."""
    ref, est = _load_pair(ref_tum, est_tum)
    if align:
        est.align(ref, correct_scale=False)
    m = metrics.APE(metrics.PoseRelation.translation_part)
    m.process_data((ref, est))
    return ErrorStats.from_metric(m)


def rpe(
    ref_tum: Path,
    est_tum: Path,
    *,
    delta: float = 1.0,
    delta_unit: metrics.Unit = metrics.Unit.meters,
) -> ErrorStats:
    """Relative Pose Error (translation) over a fixed delta (default 1 m)."""
    ref, est = _load_pair(ref_tum, est_tum)
    m = metrics.RPE(
        metrics.PoseRelation.translation_part,
        delta=delta,
        delta_unit=delta_unit,
        all_pairs=False,
    )
    m.process_data((ref, est))
    return ErrorStats.from_metric(m)
