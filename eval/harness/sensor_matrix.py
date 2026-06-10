"""Sensor-set ablation matrix: run `slam-replay` across sequences × sensor configs,
score the trajectories, and emit a self-contained HTML report.

The sensor sets live as ADR 0013 run configs in `configs/ablations/` — the matrix is
exactly "the same engine, different YAML". One run per cell (the pipeline is
deterministic in accuracy; compute numbers are single-run). Runs are resumable: a
completed cell (its `.tum` exists) is skipped, so re-invoking after an interruption
or with new sequences only does the missing work.

Usage (from `eval/`, venv active):

    python -m harness.sensor_matrix                 # run + score + report
    python -m harness.sensor_matrix --score-only    # re-score / re-report existing runs
    python -m harness.sensor_matrix --seq cafe1-1 --seq market1-1   # subset

Output: `eval/results/stage-matrix/` — per-run {tum,metrics.json,log}, `results.json`,
`report.md`, `report.html` (plots embedded; any `shot-*.png` screenshots dropped in
the output directory are embedded too).
"""

from __future__ import annotations

import argparse
import base64
import io
import json
import re
import subprocess
from pathlib import Path

from . import metrics as hm

REPO = Path(__file__).resolve().parents[2]

DEFAULT_SEQS = [f"office1-{i}" for i in range(1, 8)] + [
    "cafe1-1",
    "cafe1-2",
    "market1-1",
    "market1-2",
    "market1-3",
]
CONFIGS = ["scan", "scan-imu", "scan-odom", "scan-odom-depth", "depth", "odom-depth", "imu-odom-depth"]
CFG_LABEL = {
    "scan": "scan",
    "scan-imu": "scan+imu",
    "scan-odom": "scan+odom",
    "scan-odom-depth": "scan+odom+depth",
    "depth": "depth",
    "odom-depth": "odom+depth",
    "imu-odom-depth": "imu+odom+depth",
}
#: Scenes whose bags carry no 2D laser topic (recorded on a scan-less robot).
SCANLESS_SCENES = {"market"}

#: Per-stage interpretation, shown under "Drift, normalised" in the report.
#: **Update this when regenerating the report for a later stage of the project** —
#: it is analysis of the numbers, not machinery.
STAGE_NOTES = """
<h3>Analysis notes</h3>
<p><b>Why market ATE looks so much worse on the depth configs — and why odometry
doesn't rescue it.</b> Normalised, market depth-only registration is <i>not</i> worse
than cafe: 2.7–2.8 m per 100 m on market1-1/-2 vs 3.4 on cafe1-1. Three effects make
the absolute numbers diverge:</p>
<ul>
<li><b>Path length × zero loop closures.</b> Market trajectories are 145–223 m
(cafe: 30–47 m) and get <b>0 verified loop closures</b> (cafe: 2–5), so drift
accumulates unchecked over 5–7× more path. The proximity-gated loop search is the
bottleneck: by revisit time the estimate has drifted several metres, and the
seed-grid search (±0.5 m) can no longer land a seed inside the 3D field's 15 cm
truncation basin — verification honestly fails. (market1-3's 9 "verified" depth
loops alongside its <i>worst</i> ATE suggest the opposite failure under shelf
aliasing: geometrically self-similar shelf bays can pass the 0.55 inlier gate at the
wrong alignment.) This is precisely what per-submap appearance signatures + the
seed pyramid are queued to fix.</li>
<li><b>The wheel-odometry prior is a registration <i>seed</i>, not a factor — and on
the Scrubber 75 in the market it is biased.</b> The paper's own wheel-odometry
baseline has its worst scene on market (4.26 m scene-mean). Feeding that biased seed
into gradient registration over highly repetitive shelf geometry lands the solver in
aliased minima: RPE <i>improves</i> (0.39 → 0.28 m — locally smoother) while ATE
<i>worsens</i> (market1-1: 2.7 → 5.5 m/100 m) — the classic signature of a
systematically biased prior. Adding the IMU repairs the attitude part of the seed
and recovers most of it (3.0 m/100 m). Promoting odometry from seed to graph factor
(roadmap M3) lets the optimiser weigh it instead of trusting it.</li>
<li><b>People.</b> Market is the paper's most dynamic scene and we mask nothing yet;
people corrupt both the integrated field and registration on every config. Dynamics
masking (roadmap M5, top blocker) is expected to lift all depth columns.</li>
<li><b>This matrix caught a parity regression.</b> The cafe scan path at HEAD
(scan+imu: 0.164/0.150 m) no longer matches the archived planar gate (0.090/0.066 m)
or the 0.039/0.055 m measured during the stage-1+2 migration. An A/B run excludes
loop closure and the pose graph (ATE unchanged with <code>--no-loops</code> /
<code>--no-graph</code>), pointing at one of the later tuning rounds (per-modality
integration diets, field/loop parameters) that was benchmarked on the depth path but
not re-run on cafe scans. Office sequences are unaffected (0.019–0.106 m). Bisecting
this is queued — exactly the failure mode the parity gate exists to catch.</li>
</ul>
<p><b>Where the SLAM stands overall.</b> The engine is a single, real-time, fully-3D
multi-modal pipeline — every column of this matrix is the same binary configured by a
YAML sensor list. With the laser backbone it operates at the centimetre level
(2–10 cm ATE office/cafe, 50–200× real time, sub-millisecond p99) — at or beyond the
envelope of the published OpenLORIS systems, with the standing caveat that the
dataset's ground truth derives from the same lasers. It degrades gracefully, by
design (ADR 0012): dropping the IMU costs centimetres; dropping the lasers leaves a
working RGB-D odometry at roughly 2–6 m per 100 m; dropping everything but depth
still tracks every sequence (no run lost tracking — 0 coasting on market — where most
published systems on this benchmark lose track outright in the market scene).
Its current limits are equally clear, and queued in the roadmap in priority order:
<b>(1) no dynamics masking</b> — people poison depth registration and the map, which
is why depth may not yet update the pose when scans are present and why every depth
column trails its potential; <b>(2) proximity-gated loop closure</b> — fails exactly
where loops matter most (long, drifted revisits; aliased corridors), the cause of
market's unchecked drift; <b>(3) priors enter as seeds, not factors</b> — biased
odometry steers registration instead of being weighed against it; <b>(4) no
re-localization / multi-session capability yet</b> (ADR 0010 stage 4). None of these
is architectural debt: masking, signatures, and graph factors all slot into seams
that already exist.</p>
"""
COLORS = ["#2563eb", "#0e7490", "#0891b2", "#059669", "#d97706", "#dc2626", "#7c3aed"]


def scene_of(seq: str) -> str:
    return re.match(r"([a-z]+)", seq).group(1)


def applicable(seq: str, cfg: str) -> bool:
    return not (scene_of(seq) in SCANLESS_SCENES and cfg.startswith("scan"))


# --------------------------------------------------------------------------- run


def run_matrix(seqs: list[str], out: Path, data: Path, binary: Path) -> None:
    out.mkdir(parents=True, exist_ok=True)
    for seq in seqs:
        for cfg in CONFIGS:
            if not applicable(seq, cfg):
                continue
            tag = f"{seq}.{cfg}"
            tum = out / f"{tag}.tum"
            if tum.exists():
                print(f"skip {tag} (done)")
                continue
            bag = data / f"{seq}.bag"
            gt = data / "groundtruth/per-sequence" / seq / "groundtruth.txt"
            if not bag.exists():
                print(f"skip {tag} (no bag at {bag})")
                continue
            print(f"=== {tag}")
            cmd = [
                "/usr/bin/time",
                "-v",
                str(binary),
                "--baseline",
                "scan-matching-3d",
                "--bag",
                str(bag),
                "--config",
                str(REPO / "configs/ablations" / f"{cfg}.yaml"),
                "--init-pose-from-tum",
                str(gt),
                "--out",
                str(tum) + ".part",
                "--metrics",
                str(out / f"{tag}.metrics.json"),
            ]
            log = out / f"{tag}.log"
            with log.open("w") as lf:
                rc = subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=lf).returncode
            if rc == 0:
                (out / (tag + ".tum.part")).rename(tum)
            else:
                print(f"FAILED {tag} (see {log})")


# ------------------------------------------------------------------------- score


def score_matrix(seqs: list[str], out: Path, data: Path) -> dict:
    rows: dict[str, dict] = {}
    for seq in seqs:
        gt = data / "groundtruth/per-sequence" / seq / "groundtruth.txt"
        for cfg in CONFIGS:
            tag = f"{seq}.{cfg}"
            tum = out / f"{tag}.tum"
            if not tum.exists():
                continue
            entry: dict = {}
            try:
                entry["ate"] = hm.ate(gt, tum, align=True).rmse
            except Exception as e:  # noqa: BLE001 — a bad run must not kill the report
                entry["error"] = str(e)[:120]
            try:
                entry["rpe"] = hm.rpe(gt, tum).rmse
            except Exception:  # noqa: BLE001 — e.g. a near-stationary sequence has no 1 m pairs
                pass
            mj = out / f"{tag}.metrics.json"
            if mj.exists():
                m = json.loads(mj.read_text())
                entry["rtf"] = m.get("real_time_factor")
                entry["p99_us"] = (m.get("latency_us") or {}).get("p99")
                entry["events"] = m.get("n_samples")
            log = out / f"{tag}.log"
            if log.exists():
                t = log.read_text()
                rss = re.search(r"Maximum resident set size \(kbytes\): (\d+)", t)
                if rss:
                    entry["rss_mb"] = int(rss.group(1)) / 1024
                h = re.search(
                    r"health: (\d+) matched / (\d+) coasted / (\d+) skipped /\s+(\d+)"
                    r" degenerate; (\d+) submap hand-overs, (\d+) verified loop",
                    t,
                )
                if h:
                    entry.update(
                        matched=int(h.group(1)),
                        coasted=int(h.group(2)),
                        degenerate=int(h.group(4)),
                        submaps=int(h.group(5)) + 1,
                        loops=int(h.group(6)),
                    )
            rows[tag] = entry
    (out / "results.json").write_text(json.dumps(rows, indent=1))
    return rows


# ------------------------------------------------------------------------ report


def _cell(rows: dict, seq: str, cfg: str, key: str):
    return rows.get(f"{seq}.{cfg}", {}).get(key)


def _scene_mean(rows: dict, seqs: list[str], scene: str, cfg: str, key: str):
    vals = [
        v
        for s in seqs
        if scene_of(s) == scene
        and isinstance(v := _cell(rows, s, cfg, key), (int, float))
    ]
    return sum(vals) / len(vals) if vals else None


def _b64(fig) -> str:
    import matplotlib.pyplot as plt

    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=110, bbox_inches="tight")
    plt.close(fig)
    return base64.b64encode(buf.getvalue()).decode()


def _plots(rows: dict, seqs: list[str], scenes: list[str]) -> dict[str, str]:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    plots = {}
    w = 0.14

    fig, ax = plt.subplots(figsize=(13, 4.5))
    x = np.arange(len(seqs))
    for i, cfg in enumerate(CONFIGS):
        vals = [_cell(rows, s, cfg, "ate") or np.nan for s in seqs]
        ax.bar(x + (i - (len(CONFIGS) - 1) / 2) * w, vals, w, label=CFG_LABEL[cfg], color=COLORS[i])
    ax.set_yscale("log")
    ax.set_ylabel("ATE RMSE (m), log scale")
    ax.set_xticks(x)
    ax.set_xticklabels(seqs, rotation=30, ha="right")
    ax.legend(ncol=6, fontsize=8)
    ax.grid(axis="y", alpha=0.3)
    ax.set_title("Accuracy by sensor set — lower is better")
    plots["ate"] = _b64(fig)

    fig, ax = plt.subplots(figsize=(7, 4))
    x = np.arange(len(scenes))
    for i, cfg in enumerate(CONFIGS):
        vals = [_scene_mean(rows, seqs, sc, cfg, "ate") or np.nan for sc in scenes]
        bars = ax.bar(x + (i - (len(CONFIGS) - 1) / 2) * w * 2.2, vals, w * 2.2, label=CFG_LABEL[cfg], color=COLORS[i])
        for b, v in zip(bars, vals):
            if v == v:
                ax.text(b.get_x() + b.get_width() / 2, v * 1.05, f"{v:.2f}", ha="center", fontsize=6.5)
    ax.set_yscale("log")
    ax.set_ylabel("scene-mean ATE RMSE (m)")
    ax.set_xticks(x)
    ax.set_xticklabels(scenes)
    ax.legend(ncol=3, fontsize=8)
    ax.grid(axis="y", alpha=0.3)
    ax.set_title("Scene averages")
    plots["scene"] = _b64(fig)

    fig, ax = plt.subplots(figsize=(7, 4))
    for i, cfg in enumerate(CONFIGS):
        vals = [_scene_mean(rows, seqs, sc, cfg, "rpe") or np.nan for sc in scenes]
        ax.bar(x + (i - (len(CONFIGS) - 1) / 2) * w * 2.2, vals, w * 2.2, label=CFG_LABEL[cfg], color=COLORS[i])
    ax.set_ylabel("scene-mean RPE@1 m RMSE (m)")
    ax.set_xticks(x)
    ax.set_xticklabels(scenes)
    ax.legend(ncol=3, fontsize=8)
    ax.grid(axis="y", alpha=0.3)
    ax.set_title("Local drift (RPE @ 1 m)")
    plots["rpe"] = _b64(fig)

    fig, axes = plt.subplots(1, 2, figsize=(13, 4))
    for ax_, key, label in [
        (axes[0], "rtf", "real-time factor (×, higher is better)"),
        (axes[1], "p99_us", "latency p99 (µs, log)"),
    ]:
        for i, cfg in enumerate(CONFIGS):
            vals = [_cell(rows, s, cfg, key) or np.nan for s in seqs]
            ax_.bar(np.arange(len(seqs)) + (i - (len(CONFIGS) - 1) / 2) * w, vals, w, label=CFG_LABEL[cfg], color=COLORS[i])
        ax_.set_xticks(np.arange(len(seqs)))
        ax_.set_xticklabels(seqs, rotation=30, ha="right", fontsize=7)
        ax_.set_ylabel(label)
        ax_.grid(axis="y", alpha=0.3)
        if key == "p99_us":
            ax_.set_yscale("log")
    axes[0].legend(ncol=3, fontsize=7)
    fig.suptitle("Compute")
    plots["compute"] = _b64(fig)
    return plots


def _traj_overlays(rows: dict, seqs: list[str], out: Path, data: Path) -> str:
    """Top-down est-vs-GT overlays: one figure per scene, representative sequence,
    best scan-family config vs best depth-family config (by ATE)."""
    import matplotlib.pyplot as plt
    import numpy as np

    figs = []
    for scene in dict.fromkeys(scene_of(s) for s in seqs):
        # the scene's longest sequence shows drift best
        cands = [s for s in seqs if scene_of(s) == scene]
        seq = max(cands, key=lambda s: _gt_path_len(data, s) or 0)
        gt = data / "groundtruth/per-sequence" / seq / "groundtruth.txt"
        if not gt.exists():
            continue
        picks = []
        for fam in (("scan", "scan-imu", "scan-odom", "scan-odom-depth"),
                    ("depth", "odom-depth", "imu-odom-depth")):
            scored = [(v, c) for c in fam
                      if isinstance(v := _cell(rows, seq, c, "ate"), (int, float))]
            if scored:
                picks.append(min(scored)[1])
        if not picks:
            continue
        fig, ax = plt.subplots(figsize=(6.5, 5.5))
        G = np.loadtxt(gt, usecols=(1, 2))
        ax.plot(G[:, 0], G[:, 1], color="#16a34a", lw=2, label="ground truth")
        for cfg, color in zip(picks, ("#2563eb", "#d97706")):
            E = np.loadtxt(out / f"{seq}.{cfg}.tum", usecols=(1, 2))
            ate = _cell(rows, seq, cfg, "ate")
            ax.plot(E[:, 0], E[:, 1], color=color, lw=1.2,
                    label=f"{CFG_LABEL[cfg]} (ATE {ate:.2f} m)")
        ax.set_aspect("equal")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
        ax.set_title(f"{seq} — estimated vs ground truth (top-down)")
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        figs.append(f'<img src="data:image/png;base64,{_b64(fig)}" style="max-width:48%">')
    return "".join(figs)


def _table(rows: dict, seqs: list[str], key: str, spec: str, unit: str) -> str:
    h = "<tr><th>sequence</th>" + "".join(f"<th>{CFG_LABEL[c]}</th>" for c in CONFIGS) + "</tr>"
    body = []
    for s in seqs:
        vals = [_cell(rows, s, c, key) for c in CONFIGS]
        nums = [v for v in vals if isinstance(v, (int, float))]
        best = (
            (min(nums) if key in ("ate", "rpe", "p99_us", "rss_mb") else max(nums)) if nums else None
        )
        tds = []
        for v in vals:
            if not isinstance(v, (int, float)):
                tds.append("<td class=na>—</td>")
            else:
                cls = " class=best" if v == best else ""
                tds.append(f"<td{cls}>{format(v, spec)}</td>")
        body.append(f"<tr><td class=seq>{s}</td>" + "".join(tds) + "</tr>")
    return f"<table><caption>{unit}</caption>{h}{''.join(body)}</table>"


def _gt_path_len(data: Path, seq: str) -> float | None:
    gt = data / "groundtruth/per-sequence" / seq / "groundtruth.txt"
    if not gt.exists():
        return None
    import numpy as np

    pts = np.loadtxt(gt, usecols=(1, 2, 3))
    return float(np.linalg.norm(np.diff(pts, axis=0), axis=1).sum())


def write_report(rows: dict, seqs: list[str], out: Path, data: Path) -> None:
    scenes = sorted({scene_of(s) for s in seqs}, key=lambda sc: [scene_of(s) for s in seqs].index(sc))
    plots = _plots(rows, seqs, scenes)

    git = subprocess.run(
        ["git", "rev-parse", "--short", "HEAD"], capture_output=True, text=True, cwd=REPO
    ).stdout.strip()
    cpuinfo = Path("/proc/cpuinfo").read_text() if Path("/proc/cpuinfo").exists() else ""
    cpu = next((l.split(":", 1)[1].strip() for l in cpuinfo.splitlines() if l.startswith("model name")), "?")
    ncores = cpuinfo.count("processor\t")

    # Reference baselines (best-effort: sections drop out if the archives move).
    base_rows, sota_rows, ours_scan, best_noscan = [], [], "", []
    try:
        m3 = {
            r["sequence"]: r
            for r in json.loads(
                (REPO / "eval/reference/baselines/m3-planar-frontend/results.json").read_text()
            )
            if r["system"] == "scan_matching"
        }
        g0 = {
            (r["system"], r["sequence"]): r
            for r in json.loads((REPO / "eval/reference/baselines/ground0/results.json").read_text())
        }
        for seq in ("cafe1-1", "cafe1-2"):
            if seq not in m3 or not _cell(rows, seq, "scan-imu", "ate"):
                continue
            base_rows.append(
                f"<tr><td class=seq>{seq}</td>"
                f"<td>{g0.get(('stationary', seq), {}).get('ate_rmse_m', {}).get('mean', float('nan')):.2f}</td>"
                f"<td>{m3[seq]['ate_rmse_m']['mean']:.3f} / {m3[seq]['real_time_factor']['mean']:.0f}× / {m3[seq]['latency_p99_us']['mean']:.0f} µs</td>"
                f"<td class=best>{_cell(rows, seq, 'scan-imu', 'ate'):.3f} / {_cell(rows, seq, 'scan-imu', 'rtf'):.0f}× / {_cell(rows, seq, 'scan-imu', 'p99_us'):.0f} µs</td></tr>"
            )
        sota = json.loads((REPO / "eval/reference/sota/openloris-scene-paper.json").read_text())[
            "per_sequence"
        ]["systems"]
        for sysd in sota:
            cells = "".join(
                f"<td>{sysd['ate_rmse_m'][sc]:.3f}</td>" if sc in sysd["ate_rmse_m"] else "<td class=na>—</td>"
                for sc in scenes
            )
            sota_rows.append(f"<tr><td class=seq>{sysd['system']} ({sysd['input']})</td>{cells}</tr>")
        ours_scan = "".join(
            f"<td class=best>{v:.3f}</td>" if (v := _scene_mean(rows, seqs, sc, "scan", "ate")) else "<td class=na>—</td>"
            for sc in scenes
        )
        for sc in scenes:
            v = min(
                filter(
                    None,
                    (_scene_mean(rows, seqs, sc, c, "ate") for c in ("depth", "odom-depth", "imu-odom-depth")),
                ),
                default=None,
            )
            best_noscan.append(f"<td>{v:.3f}</td>" if v else "<td class=na>—</td>")
    except FileNotFoundError:
        pass

    drift_rows = []
    for seq in seqs:
        ln = _gt_path_len(data, seq)
        if not ln:
            continue
        tds = []
        for c in CONFIGS:
            v = _cell(rows, seq, c, "ate")
            loops = _cell(rows, seq, c, "loops")
            if isinstance(v, (int, float)):
                tds.append(f"<td>{100 * v / ln:.2f} <span class=meta>({loops})</span></td>")
            else:
                tds.append("<td class=na>—</td>")
        drift_rows.append(f"<tr><td class=seq>{seq} <span class=meta>{ln:.0f} m</span></td>" + "".join(tds) + "</tr>")
    drift_table = (
        "<table><caption>ATE per 100 m of GT path (m) — verified loop count in parentheses</caption>"
        "<tr><th>sequence (length)</th>"
        + "".join(f"<th>{CFG_LABEL[c]}</th>" for c in CONFIGS)
        + "</tr>"
        + "".join(drift_rows)
        + "</table>"
    )
    notes = STAGE_NOTES

    trajs = _traj_overlays(rows, seqs, out, data)
    shots = "".join(
        f"<figure><img src=\"data:image/png;base64,{base64.b64encode(p.read_bytes()).decode()}\">"
        f"<figcaption>{p.stem.replace('shot-', '').replace('-', ' ')}</figcaption></figure>"
        for p in sorted(out.glob("shot-*.png"))
    )

    html = f"""<!doctype html><html><head><meta charset="utf-8">
<title>socialminds-slam — stage benchmark report</title><style>
body{{font:15px/1.55 system-ui,sans-serif;max-width:1080px;margin:2em auto;padding:0 1em;color:#1e293b}}
h1{{font-size:1.6em}} h2{{margin-top:2em;border-bottom:2px solid #e2e8f0;padding-bottom:.2em}}
table{{border-collapse:collapse;margin:1em 0;font-size:13px;width:100%}}
caption{{text-align:left;font-weight:600;padding-bottom:.4em;color:#475569}}
th,td{{border:1px solid #e2e8f0;padding:.35em .6em;text-align:right}}
th{{background:#f1f5f9}} td.seq{{text-align:left;font-weight:600}}
td.best{{background:#dcfce7;font-weight:600}} td.na{{color:#cbd5e1;text-align:center}}
img{{max-width:100%;border:1px solid #e2e8f0;border-radius:6px;margin:.5em 0}}
figure{{margin:1em 0}} figcaption{{color:#64748b;font-size:.9em}}
code{{background:#f1f5f9;padding:.1em .35em;border-radius:4px;font-size:.9em}}
.meta{{color:#64748b;font-size:.9em}} .note{{background:#fef9c3;padding:.6em 1em;border-radius:6px;font-size:.92em}}
ul{{padding-left:1.3em}} li{{margin:.25em 0}} .next td{{text-align:left}}
</style></head><body>
<h1>socialminds-slam — sensor-set benchmark matrix</h1>
<p class=meta>Commit <code>{git}</code> · {cpu} ({ncores} threads) · single run per cell
(the pipeline is deterministic in accuracy; compute numbers are single-run) ·
OpenLORIS-Scene, {len(seqs)} sequences, {len(rows)} runs.</p>

<h2>1. Current feature set</h2>
<ul>
<li><b>Full-3D SE(3) state</b> with IMU attitude (tilt-compensated scan fans) — IMU strictly optional (ADR 0012, measured cost ≈ 4 cm).</li>
<li><b>TSDF submap registration</b>: 2D laser fans → 2.5 cm gravity-plane field; RGB-D depth clouds → separate 5 cm 3D field; ICP degeneracy guard fills weak directions from the motion prior.</li>
<li><b>Multi-sensor rig</b> from URDF / bag <code>tf_static</code> (ADR 0009); measurements self-identify by frame; multi-lidar fusion.</li>
<li><b>Range-adaptive depth sampling</b> (≈ constant surface spacing at every range) + wheel-odometry motion prior.</li>
<li><b>Geometrically verified, modality-aware loop closure</b> against frozen anchor-relative submaps + <b>GTSAM pose graph</b> (optimise on every verified loop; anchors re-posed, voxels never rewritten).</li>
<li><b>YAML run configuration</b> (ADR 0013) — exactly the mechanism used to express the sensor sets below (<code>configs/ablations/</code>).</li>
<li>Live/recorded <b>rerun</b> visualization: coloured 3D map (CIELAB a*b*), true-size voxel cubes, per-submap TSDF entities.</li>
</ul>

<h2>2. Benchmark setup</h2>
<p>Six sensor sets, expressed as run configs, on every sequence they apply to
(market bags carry no 2D laser — recorded on a different robot):</p>
<table><tr><th>config</th><th>laser scans</th><th>wheel odometry</th><th>RGB-D depth</th><th>IMU</th></tr>
<tr><td class=seq>scan</td><td>✓</td><td></td><td></td><td></td></tr>
<tr><td class=seq>scan+imu</td><td>✓</td><td></td><td></td><td>✓</td></tr>
<tr><td class=seq>scan+odom</td><td>✓</td><td>✓</td><td></td><td></td></tr>
<tr><td class=seq>scan+odom+depth</td><td>✓</td><td>✓</td><td>✓</td><td></td></tr>
<tr><td class=seq>depth</td><td></td><td></td><td>✓</td><td></td></tr>
<tr><td class=seq>odom+depth</td><td></td><td>✓</td><td>✓</td><td></td></tr>
<tr><td class=seq>imu+odom+depth</td><td></td><td>✓</td><td>✓</td><td>✓</td></tr></table>
<p>Each run: <code>slam-replay --baseline scan-matching-3d --bag SEQ.bag --config
configs/ablations/CFG.yaml --init-pose-from-tum GT --out est.tum --metrics m.json</code>,
rig from the bag's <code>/tf_static</code>, loop closure + pose graph on, depth at every
3rd frame. <b>ATE</b> = translation RMSE after SE(3) Umeyama alignment (evo,
harness-standard); <b>RPE</b> @ 1 m; compute from the engine's own per-event latency
clock; peak RSS via <code>/usr/bin/time -v</code> (includes bag decode).</p>
<p class=note>Two caveats. (1) OpenLORIS ground truth is itself produced from the 2D
lasers — scan-based configs are partially evaluated against their own sensor; the
depth-only columns are the honest cross-modal numbers. (2) When scans are present,
depth integrates into the map but does not update the pose (gated until dynamics
masking lands), so scan+odom+depth ≈ scan+odom in accuracy by design.</p>

<h2>3. Results</h2>
<h3>Accuracy</h3>
{_table(rows, seqs, "ate", ".3f", "ATE RMSE (m) — SE(3)-aligned, lower is better; green = best per sequence")}
{_table(rows, seqs, "rpe", ".4f", "RPE @ 1 m RMSE (m) — local drift")}
<h3>Compute</h3>
{_table(rows, seqs, "rtf", ".0f", "Real-time factor (×, input span ÷ processing wall time; higher is better)")}
{_table(rows, seqs, "p99_us", ".0f", "Per-event latency p99 (µs)")}
{_table(rows, seqs, "rss_mb", ".0f", "Peak RSS (MB) — includes bag decode buffers")}
<h3>Loop closure activity</h3>
{_table(rows, seqs, "loops", "d", "Verified loop closures (count)")}

<h2>4. Plots</h2>
<img src="data:image/png;base64,{plots['ate']}">
<img src="data:image/png;base64,{plots['scene']}">
<img src="data:image/png;base64,{plots['rpe']}">
<img src="data:image/png;base64,{plots['compute']}">

<h2>5. Trajectories & maps, visually</h2>
{trajs}
{shots}

<h2>6. Drift, normalised</h2>
<p>ATE is a <i>global</i> error: under pure odometric drift it scales with path length,
and verified loop closures are what cap it. Reading accuracy per 100 m of ground-truth
path alongside the loop count separates "registers badly" from "drifts unchecked":</p>
{drift_table}
{notes}

<h2>7. Against the baselines</h2>
<table><caption>cafe ATE (m) / RTF / p99 — trivial floor → archived planar parity gate → current 3D pipeline (scan config)</caption>
<tr><th>sequence</th><th>stationary floor</th><th>M3 planar gate</th><th>current (scan+imu)</th></tr>
{''.join(base_rows)}</table>
<table><caption>Scene-mean ATE (m) vs the OpenLORIS-Scene paper (ICRA 2020, Fig. 2; per-scene
averages — the paper's systems are camera/IMU/odometry-based and do <b>not</b> use the lasers;
read jointly with their correct-rate, see eval/reference/sota/)</caption>
<tr><th>system</th>{''.join(f'<th>{sc}</th>' for sc in scenes)}</tr>
{''.join(sota_rows)}
<tr><td class=seq><b>ours — scan (laser; GT caveat)</b></td>{ours_scan}</tr>
<tr><td class=seq><b>ours — best no-scan config</b></td>{''.join(best_noscan)}</tr></table>

<h2>8. Next steps & expected improvements</h2>
<table class=next><tr><th>item</th><th>expectation (grounded in measurements)</th></tr>
<tr><td class=seq>Re-establish cafe scan parity (regression bisect)</td><td>Caught by this
matrix: cafe scan+imu 0.164/0.150 m vs the 0.090/0.066 m gate; loops/graph ruled out by
A/B. Expected outcome: restore ≤ 0.09 m on cafe while keeping the depth-path gains.</td></tr>
<tr><td class=seq>Dynamics masking (top blocker)</td><td>Un-masked people dominate depth error:
depth→pose fusion measured 0.16→3.0 m when ungated. Masking should unlock
<code>depth_updates_pose</code> and pull depth-inclusive configs toward the laser-band
contribution measured at 0.16 m — i.e. several-fold ATE improvement on the no-scan
columns, and depth finally <i>helping</i> the scan configs.</td></tr>
<tr><td class=seq>Per-submap appearance signatures (MapClosures-style)</td><td>Replaces proximity
gating: loop closures in repetitive corridors + re-localization &lt; 1 s (ADR 0010 stage 4);
prerequisite for the corridor scenes.</td></tr>
<tr><td class=seq>Depth loop-closure seed pyramid</td><td>Decouples verification from the 3D field's
truncation: lets a 2.5 cm field recover near-range accuracy (measured 0.46 vs 0.81 m
open-loop) <i>without</i> losing loop verification.</td></tr>
<tr><td class=seq>Odometry as a graph factor</td><td>Currently a motion prior only; as a factor it
constrains the graph between loops — expected to firm up market-style long loops.</td></tr>
<tr><td class=seq>Hybrid per-point fan registration</td><td>Laser fans use the 3D field where camera
coverage is dense; removes the 2D-field/3D-field accuracy split as RGB-D coverage grows.</td></tr>
<tr><td class=seq>OpenVDB backend + voxel a*b* colour channel</td><td>reMap interop and a coloured,
illumination-invariant map; no accuracy change expected (conformance-gated).</td></tr>
</table>
</body></html>"""
    (out / "report.html").write_text(html)

    # Compact markdown sibling (tables only) for quick diffing.
    md = ["# Sensor-set benchmark matrix", ""]
    for key, spec, title in [
        ("ate", ".3f", "ATE RMSE (m)"),
        ("rpe", ".4f", "RPE@1m RMSE (m)"),
        ("rtf", ".0f", "Real-time factor (x)"),
        ("p99_us", ".0f", "Latency p99 (us)"),
        ("rss_mb", ".0f", "Peak RSS (MB)"),
        ("loops", "d", "Verified loop closures"),
    ]:
        md += [f"## {title}", "", "| sequence | " + " | ".join(CFG_LABEL[c] for c in CONFIGS) + " |", "|---|" + "---|" * len(CONFIGS)]
        for s in seqs:
            cells = [
                format(v, spec) if isinstance(v := _cell(rows, s, c, key), (int, float)) else "—"
                for c in CONFIGS
            ]
            md.append(f"| {s} | " + " | ".join(cells) + " |")
        md.append("")
    (out / "report.md").write_text("\n".join(md))
    print(f"report: {out / 'report.html'}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--seq", action="append", help="sequence (repeatable; default: the full list)")
    ap.add_argument("--out-dir", type=Path, default=REPO / "eval/results/stage-matrix")
    ap.add_argument("--data-dir", type=Path, default=REPO / "data/openloris")
    ap.add_argument("--bin", type=Path, default=REPO / "target/release/slam-replay")
    ap.add_argument("--score-only", action="store_true", help="skip running; re-score + re-report")
    args = ap.parse_args()
    seqs = args.seq or DEFAULT_SEQS
    if not args.score_only:
        run_matrix(seqs, args.out_dir, args.data_dir, args.bin)
    rows = score_matrix(seqs, args.out_dir, args.data_dir)
    print(f"{len(rows)} runs scored")
    write_report(rows, seqs, args.out_dir, args.data_dir)


if __name__ == "__main__":
    main()
