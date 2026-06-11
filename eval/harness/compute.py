"""Run the engine while capturing compute metrics.

Combines the engine's own metrics sidecar (latency / throughput / real-time factor, written
by `slam-replay --metrics`) with externally-observed **peak resident memory**, sampled from
``/proc/<pid>/status`` (Linux). Together these are the compute half of the benchmark report
(ADR 0005); the accuracy half comes from `harness.metrics`.
"""

from __future__ import annotations

import json
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class ComputeStats:
    n_samples: int
    input_span_s: float
    processing_wall_s: float
    throughput_hz: float
    real_time_factor: float
    latency_us: dict
    peak_rss_mb: float | None  # None if /proc is unavailable

    @property
    def is_real_time(self) -> bool:
        return self.real_time_factor >= 1.0


def _read_vmhwm_kb(pid: int) -> int | None:
    """Peak resident set size (VmHWM, kB) of a process, or None if unavailable."""
    try:
        with open(f"/proc/{pid}/status") as f:
            for line in f:
                if line.startswith("VmHWM:"):
                    return int(line.split()[1])
    except (FileNotFoundError, ProcessLookupError, PermissionError, ValueError):
        return None
    return None


def run_with_metrics(
    binary: Path,
    baseline: str,
    imu_csv: Path | None,
    out_tum: Path,
    *,
    scan_csv: Path | None = None,
    odom_csv: Path | None = None,
    bag: Path | None = None,
    gyro_topic: str | None = None,
    accel_topic: str | None = None,
    scan_topic: str | None = None,
    init_pose_tum: Path | None = None,
    poll_interval_s: float = 0.002,
) -> ComputeStats:
    """Run one system, returning its accuracy-agnostic compute metrics.

    Whatever input streams are given (IMU CSV, scan CSV — or topics streamed directly
    from a ROS1 ``bag``) are passed through; the system consumes what it understands.
    Writes the trajectory to ``out_tum`` and a metrics sidecar next to it; samples the
    child's peak RSS while it runs (VmHWM is monotonic, so the last sample is the peak).
    """
    out_tum = Path(out_tum)
    out_tum.parent.mkdir(parents=True, exist_ok=True)
    metrics_json = out_tum.with_suffix(".metrics.json")

    cmd = [
        str(binary),
        "--baseline", baseline,
        "--out", str(out_tum),
        "--metrics", str(metrics_json),
    ]
    if imu_csv is not None:
        cmd += ["--imu", str(imu_csv)]
    if scan_csv is not None:
        cmd += ["--scan", str(scan_csv)]
    if odom_csv is not None:
        cmd += ["--odom", str(odom_csv)]
    if bag is not None:
        cmd += ["--bag", str(bag)]
        if gyro_topic is not None:
            cmd += ["--gyro-topic", gyro_topic, "--accel-topic", accel_topic]
        if scan_topic is not None:
            cmd += ["--scan-topic", scan_topic]
    if init_pose_tum is not None:
        cmd += ["--init-pose-from-tum", str(init_pose_tum)]

    proc = subprocess.Popen(cmd)
    peak_kb: int | None = None
    while proc.poll() is None:
        sample = _read_vmhwm_kb(proc.pid)
        if sample is not None:
            peak_kb = sample if peak_kb is None else max(peak_kb, sample)
        time.sleep(poll_interval_s)
    if proc.returncode != 0:
        raise subprocess.CalledProcessError(proc.returncode, cmd)

    data = json.loads(metrics_json.read_text())
    return ComputeStats(
        n_samples=data["n_samples"],
        input_span_s=data["input_span_s"],
        processing_wall_s=data["processing_wall_s"],
        throughput_hz=data["throughput_hz"],
        real_time_factor=data["real_time_factor"],
        latency_us=data["latency_us"],
        peak_rss_mb=(peak_kb / 1024.0) if peak_kb is not None else None,
    )
