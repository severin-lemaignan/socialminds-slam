"""Headless tests for the scan/trajectory visualiser (Agg backend, no display)."""

from __future__ import annotations

import math

import matplotlib

matplotlib.use("Agg")  # before any pyplot import inside harness.viz

import numpy as np
import pytest

from harness import viz


def write_fixtures(tmp_path, n=12):
    """A tiny straight drive: scans of a wall at y=2, est == gt shifted/rotated."""
    scan_lines = ["# t angle_min angle_increment range_min range_max n r..."]
    est_lines, gt_lines = ["# est"], ["# gt"]
    for k in range(n):
        t = float(k)
        # Three beams straight up at a wall 2 m to the left (+Y in sensor frame).
        scan_lines.append(f"{t} {math.pi / 2 - 0.05} 0.05 0.1 25.0 3 2.0 2.0 2.0")
        # Ground truth walks +X from (5, 5) at yaw 0.3; estimate walks +X from origin.
        est_lines.append(f"{t} {0.1 * k} 0.0 0.0 0 0 0 1")
        gyaw = 0.3
        gx, gy = 5.0 + 0.1 * k * math.cos(gyaw), 5.0 + 0.1 * k * math.sin(gyaw)
        gt_lines.append(f"{t} {gx} {gy} 0.0 0 0 {math.sin(gyaw / 2)} {math.cos(gyaw / 2)}")
    scan_csv = tmp_path / "scan.csv"
    est_tum = tmp_path / "est.tum"
    gt_tum = tmp_path / "gt.tum"
    scan_csv.write_text("\n".join(scan_lines) + "\n")
    est_tum.write_text("\n".join(est_lines) + "\n")
    gt_tum.write_text("\n".join(gt_lines) + "\n")
    return scan_csv, est_tum, gt_tum


def test_loaders_parse_the_fixtures(tmp_path):
    scan_csv, est_tum, gt_tum = write_fixtures(tmp_path)
    scans = viz.load_scans(scan_csv)
    est = viz.load_tum(est_tum)
    gt = viz.load_tum(gt_tum)
    assert len(scans) == 12 and scans[0].points.shape == (3, 2)
    assert est.xy.shape == (12, 2)
    assert gt.yaw[0] == pytest.approx(0.3)


def test_first_pose_align_overlays_start_poses(tmp_path):
    _, est_tum, gt_tum = write_fixtures(tmp_path)
    est, gt = viz.load_tum(est_tum), viz.load_tum(gt_tum)
    aligned = viz.first_pose_align(est, gt)
    assert aligned.xy[0] == pytest.approx(gt.xy[0])
    assert aligned.yaw[0] == pytest.approx(gt.yaw[0])
    # The fixture's est is gt expressed in the start frame: after alignment they match.
    assert np.allclose(aligned.xy, gt.xy, atol=1e-9)


def test_save_renders_a_png_headless(tmp_path):
    scan_csv, est_tum, gt_tum = write_fixtures(tmp_path)
    out = tmp_path / "overview.png"
    rc = viz.main([
        "--scan", str(scan_csv),
        "--estimate", str(est_tum),
        "--groundtruth", str(gt_tum),
        "--save", str(out),
    ])
    assert rc == 0
    assert out.stat().st_size > 1000  # a real PNG, not an empty file


def test_missing_inputs_is_a_usage_error(tmp_path, capsys):
    with pytest.raises(SystemExit):
        viz.main(["--estimate", "nope.tum"])
