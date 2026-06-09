"""Interactive scan-matching debugger: laser scans + estimated pose + ground truth.

Top-down view with three layers:

- the **accumulated point map**: every k-th scan rendered through its *estimated* pose —
  if the front-end is right, walls come out crisp; drift smears them;
- the **trajectories**: estimate (blue) vs ground truth (green), first-pose aligned by
  default so they overlay;
- the **current scan** (red) through the current estimated pose, with both pose arrows.

Drive it with the slider, ←/→ (±1 scan; shift: ±25) or space (autoplay). ``--save``
renders the final overview headless instead.

Two ways in::

    # explicit files
    python -m harness.viz --scan scan.csv --estimate est.tum --groundtruth gt.tum
    # cached OpenLORIS sequence: runs slam-replay (scan matching) and shows the result
    python -m harness.viz --openloris cafe1-1
"""

from __future__ import annotations

import argparse
import math
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np


# --------------------------------------------------------------------------------------
# Data loading (kept dependency-light: TUM and scan CSV are trivial line formats)
# --------------------------------------------------------------------------------------

@dataclass
class TrajectoryArrays:
    stamps: np.ndarray  # (N,)
    xy: np.ndarray      # (N, 2)
    yaw: np.ndarray     # (N,)

    def index_at(self, t: float) -> int:
        """Index of the pose nearest in time to ``t``."""
        return int(np.clip(np.searchsorted(self.stamps, t), 0, len(self.stamps) - 1))


def load_tum(path: Path) -> TrajectoryArrays:
    rows = []
    for line in Path(path).read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        f = [float(v) for v in line.split()]
        t, x, y, _z, qx, qy, qz, qw = f[:8]
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        rows.append((t, x, y, yaw))
    if not rows:
        raise ValueError(f"{path} contains no poses")
    arr = np.asarray(rows)
    return TrajectoryArrays(stamps=arr[:, 0], xy=arr[:, 1:3], yaw=arr[:, 3])


@dataclass
class Scan:
    stamp: float
    points: np.ndarray  # (M, 2) valid returns, sensor frame


def load_scans(path: Path) -> list[Scan]:
    scans = []
    for line in Path(path).read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        f = line.split()
        stamp, angle_min, inc, rmin, rmax = (float(v) for v in f[:5])
        ranges = np.array(f[6:], dtype=np.float64)
        angles = angle_min + inc * np.arange(len(ranges))
        valid = np.isfinite(ranges) & (ranges >= rmin) & (ranges <= rmax)
        r, a = ranges[valid], angles[valid]
        scans.append(Scan(stamp, np.column_stack((r * np.cos(a), r * np.sin(a)))))
    if not scans:
        raise ValueError(f"{path} contains no scans")
    return scans


# --------------------------------------------------------------------------------------
# Planar geometry
# --------------------------------------------------------------------------------------

def transform_points(points: np.ndarray, x: float, y: float, yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    rot = np.array([[c, -s], [s, c]])
    return points @ rot.T + (x, y)


def first_pose_align(est: TrajectoryArrays, gt: TrajectoryArrays) -> TrajectoryArrays:
    """SE(2)-transform the estimate so its first pose coincides with ground truth's."""
    dyaw = gt.yaw[0] - est.yaw[0]
    c, s = math.cos(dyaw), math.sin(dyaw)
    rot = np.array([[c, -s], [s, c]])
    xy = (est.xy - est.xy[0]) @ rot.T + gt.xy[0]
    return TrajectoryArrays(stamps=est.stamps, xy=xy, yaw=est.yaw + dyaw)


# --------------------------------------------------------------------------------------
# The viewer
# --------------------------------------------------------------------------------------

class Viewer:
    """Matplotlib state: static layers drawn once, current-scan layer updated in place."""

    def __init__(
        self,
        scans: list[Scan],
        estimate: TrajectoryArrays,
        groundtruth: TrajectoryArrays | None,
        map_scans: int = 150,
        map_points_per_scan: int = 250,
    ):
        import matplotlib.pyplot as plt

        self.scans = scans
        self.est = estimate
        self.gt = groundtruth
        self.index = 0
        self.playing = False

        self.fig, self.ax = plt.subplots(figsize=(11, 9))
        self.ax.set_aspect("equal")
        self.ax.grid(True, alpha=0.2)

        # Accumulated map through estimated poses.
        stride = max(1, len(scans) // map_scans)
        blobs = []
        for scan in scans[::stride]:
            i = self.est.index_at(scan.stamp)
            pts = scan.points
            if len(pts) > map_points_per_scan:
                pts = pts[:: len(pts) // map_points_per_scan + 1]
            blobs.append(transform_points(pts, *self.est.xy[i], self.est.yaw[i]))
        cloud = np.vstack(blobs)
        self.ax.scatter(cloud[:, 0], cloud[:, 1], s=0.3, c="0.65", linewidths=0,
                        label="map (scans @ estimate)")
        # Frame the dense part of the map: a few stray long-range returns (windows,
        # glass) otherwise dominate the extent.
        lo, hi = np.percentile(cloud, 1, axis=0), np.percentile(cloud, 99, axis=0)
        margin = 0.05 * float((hi - lo).max()) + 1.0
        self.ax.set_xlim(lo[0] - margin, hi[0] + margin)
        self.ax.set_ylim(lo[1] - margin, hi[1] + margin)

        if self.gt is not None:
            self.ax.plot(self.gt.xy[:, 0], self.gt.xy[:, 1], color="tab:green", lw=1.2,
                         label="ground truth")
        self.ax.plot(self.est.xy[:, 0], self.est.xy[:, 1], color="tab:blue", lw=1.2,
                     label="estimate")

        # Dynamic artists.
        self.scan_artist = self.ax.scatter([], [], s=2.0, c="tab:red", linewidths=0,
                                           label="current scan @ estimate")
        self.est_arrow = self.ax.annotate("", xy=(0, 0), xytext=(0, 0),
                                          arrowprops=dict(color="tab:blue", width=2,
                                                          headwidth=8))
        self.gt_arrow = self.ax.annotate("", xy=(0, 0), xytext=(0, 0),
                                         arrowprops=dict(color="tab:green", width=2,
                                                         headwidth=8))
        self.ax.legend(loc="upper right", fontsize=9)

        self.fig.canvas.mpl_connect("key_press_event", self._on_key)
        self.timer = self.fig.canvas.new_timer(interval=50)
        self.timer.add_callback(self._tick)
        self._slider = self._make_slider(plt)
        self.show_index(0)

    def _make_slider(self, plt):
        from matplotlib.widgets import Slider

        axsl = self.fig.add_axes([0.15, 0.02, 0.6, 0.025])
        slider = Slider(axsl, "scan", 0, len(self.scans) - 1, valinit=0, valstep=1)
        slider.on_changed(lambda v: self.show_index(int(v), from_slider=True))
        return slider

    def _arrow(self, artist, x: float, y: float, yaw: float, length: float = 0.6):
        # Annotation's settable text-anchor property is `xyann` (not `xytext`).
        artist.xyann = (x, y)
        artist.xy = (x + length * math.cos(yaw), y + length * math.sin(yaw))

    def show_index(self, index: int, from_slider: bool = False):
        self.index = int(np.clip(index, 0, len(self.scans) - 1))
        scan = self.scans[self.index]
        i = self.est.index_at(scan.stamp)
        x, y, yaw = self.est.xy[i, 0], self.est.xy[i, 1], self.est.yaw[i]
        self.scan_artist.set_offsets(transform_points(scan.points, x, y, yaw))
        self._arrow(self.est_arrow, x, y, yaw)

        title = f"scan {self.index + 1}/{len(self.scans)}   t = {scan.stamp:.2f} s"
        if self.gt is not None:
            j = self.gt.index_at(scan.stamp)
            gx, gy, gyaw = self.gt.xy[j, 0], self.gt.xy[j, 1], self.gt.yaw[j]
            self._arrow(self.gt_arrow, gx, gy, gyaw)
            err = math.hypot(x - gx, y - gy)
            title += f"   |est − gt| = {err:.3f} m"
        self.ax.set_title(title)

        if not from_slider:
            self._slider.set_val(self.index)  # re-enters show_index once, harmless
        self.fig.canvas.draw_idle()

    def _on_key(self, event):
        step = {"left": -1, "right": 1, "shift+left": -25, "shift+right": 25}.get(event.key)
        if step is not None:
            self.show_index(self.index + step)
        elif event.key == " ":
            self.playing = not self.playing
            (self.timer.start if self.playing else self.timer.stop)()

    def _tick(self):
        if self.playing:
            if self.index + 1 >= len(self.scans):
                self.playing = False
                self.timer.stop()
            else:
                self.show_index(self.index + 1)


# --------------------------------------------------------------------------------------
# Sequence convenience: cached OpenLORIS → run slam-replay → visualise
# --------------------------------------------------------------------------------------

def materialized_openloris(name: str) -> tuple[Path, Path]:
    """(scan_csv, groundtruth_tum) from the benchmark's materialised cache."""
    from . import fetch

    cache = fetch.cache_root() / "openloris" / "_materialized" / name
    scan_csv, gt = cache / "scan.csv", cache / "groundtruth.tum"
    if not scan_csv.exists() or not gt.exists():
        raise FileNotFoundError(
            f"no materialised scans for {name!r} under {cache}; run "
            f"`python -m harness.benchmark --openloris {name}` once to extract them."
        )
    return scan_csv, gt


def run_system(scan_csv: Path, gt_tum: Path, system: str, out_tum: Path) -> Path:
    from . import replay

    out_tum.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(
        [
            str(replay.find_replay_binary()),
            "--baseline", system,
            "--scan", str(scan_csv),
            "--init-pose-from-tum", str(gt_tum),
            "--out", str(out_tum),
        ],
        check=True,
    )
    return out_tum


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--scan", type=Path, help="scan CSV (with --estimate)")
    p.add_argument("--estimate", type=Path, help="estimated trajectory (TUM)")
    p.add_argument("--groundtruth", type=Path, help="ground-truth trajectory (TUM)")
    p.add_argument("--openloris", metavar="SEQ",
                   help="cached OpenLORIS sequence, e.g. cafe1-1 (runs slam-replay)")
    p.add_argument("--system", default="scan-matching",
                   help="system to run with --openloris (default scan-matching)")
    p.add_argument("--no-align", action="store_true",
                   help="skip first-pose alignment of the estimate onto ground truth")
    p.add_argument("--save", type=Path, help="write a static overview PNG and exit")
    args = p.parse_args(argv)

    if args.save:
        import matplotlib

        matplotlib.use("Agg")

    if args.openloris:
        scan_csv, gt_tum = materialized_openloris(args.openloris)
        est_tum = scan_csv.parent / f"viz_{args.system}.tum"
        print(f"running {args.system} over {scan_csv} …", flush=True)
        run_system(scan_csv, gt_tum, args.system, est_tum)
        args.scan, args.estimate, args.groundtruth = scan_csv, est_tum, gt_tum
    elif not (args.scan and args.estimate):
        p.error("pass --openloris SEQ, or --scan + --estimate (+ optional --groundtruth)")

    scans = load_scans(args.scan)
    est = load_tum(args.estimate)
    gt = load_tum(args.groundtruth) if args.groundtruth else None
    if gt is not None and not args.no_align:
        est = first_pose_align(est, gt)

    viewer = Viewer(scans, est, gt)
    if args.save:
        viewer.show_index(len(scans) - 1)
        viewer.fig.savefig(args.save, dpi=130, bbox_inches="tight")
        print(f"wrote {args.save}")
        return 0

    import matplotlib.pyplot as plt

    plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
