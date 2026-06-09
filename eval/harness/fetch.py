"""Download + cache benchmark datasets.

Datasets are large (OpenLORIS scenes are 6–33 GB), so they live in a **cache** outside the
repo and are fetched on demand — never committed, never downloaded in CI (ADR 0003/0005).
This module is the one place that knows dataset URLs; the `Makefile` wraps it as build
steps (`make data-openloris SCENE=office1`).

Cache root: ``$SLAM_DATA_DIR`` if set, else ``<repo>/data`` (git-ignored). Downloads are
resumable (via ``curl -C -`` when available) and skipped if already complete.
"""

from __future__ import annotations

import os
import shutil
import subprocess
import urllib.request
import zipfile
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]


def cache_root() -> Path:
    root = os.environ.get("SLAM_DATA_DIR")
    return Path(root) if root else REPO_ROOT / "data"


# --------------------------------------------------------------------------------------
# Dataset URL registries
# --------------------------------------------------------------------------------------

from . import datasets  # noqa: E402  (reuse EuRoC URL builder)

# OpenLORIS-Scene on Hugging Face (Google Drive mirror deprecated 2025-12).
OPENLORIS_HF_BASE = "https://huggingface.co/datasets/shixuesong/openloris-scene/resolve/main"

# Per-scene rosbag artifacts. Some scenes ship as a single multi-sequence tar; market1 is
# raw per-sequence bags. Names verified against the HF tree listing.
OPENLORIS_ROSBAG_FILES: dict[str, list[str]] = {
    "office1": ["rosbag/office1-1_7-rosbag.tar"],
    "corridor1": ["rosbag/corridor1-1_2-rosbag.tar", "rosbag/corridor1-3_5-rosbag.tar"],
    "home1": ["rosbag/home1-1_5-rosbag.tar"],
    "cafe1": ["rosbag/cafe1-1_2-rosbag.tar"],
    "market1": ["rosbag/market1-1.bag", "rosbag/market1-2.bag", "rosbag/market1-3.bag"],
}
OPENLORIS_GROUNDTRUTH = "package/groundtruth.zip"  # ~11 MB, all sequences, TUM format


# --------------------------------------------------------------------------------------
# Download primitive
# --------------------------------------------------------------------------------------

def download_file(url: str, dest: Path, *, resume: bool = True) -> Path:
    """Download ``url`` to ``dest`` (resumable, skipped if a ``.done`` marker exists)."""
    dest = Path(dest)
    dest.parent.mkdir(parents=True, exist_ok=True)
    done = dest.with_suffix(dest.suffix + ".done")
    if done.exists() and dest.exists():
        print(f"cached: {dest}")
        return dest

    curl = shutil.which("curl")
    if curl:
        cmd = [curl, "-fL", "-o", str(dest), url]
        if resume:
            cmd[1:1] = ["-C", "-"]
        print(f"$ {' '.join(cmd)}")
        subprocess.run(cmd, check=True)
    else:
        print(f"downloading {url} -> {dest}")
        urllib.request.urlretrieve(url, dest)

    done.touch()
    return dest


# --------------------------------------------------------------------------------------
# High-level fetchers
# --------------------------------------------------------------------------------------

def fetch_euroc(seq_name: str, *, root: Path | None = None) -> Path:
    """Download an EuRoC sequence zip into the cache; return its path."""
    root = root or cache_root()
    url = datasets.euroc_download_url(seq_name)
    return download_file(url, root / "euroc" / f"{seq_name}.zip")


def fetch_openloris_scene(scene: str, *, root: Path | None = None) -> list[Path]:
    """Download every rosbag artifact for an OpenLORIS scene; return their paths."""
    if scene not in OPENLORIS_ROSBAG_FILES:
        raise KeyError(f"unknown OpenLORIS scene {scene!r}; known: {sorted(OPENLORIS_ROSBAG_FILES)}")
    root = root or cache_root()
    paths = []
    for rel in OPENLORIS_ROSBAG_FILES[scene]:
        url = f"{OPENLORIS_HF_BASE}/{rel}"
        paths.append(download_file(url, root / "openloris" / Path(rel).name))
    return paths


def fetch_openloris_groundtruth(*, root: Path | None = None) -> Path:
    """Download + extract the small OpenLORIS ground-truth bundle; return its directory."""
    root = root or cache_root()
    zip_path = download_file(f"{OPENLORIS_HF_BASE}/{OPENLORIS_GROUNDTRUTH}", root / "openloris" / "groundtruth.zip")
    out_dir = root / "openloris" / "groundtruth"
    out_dir.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(zip_path) as zf:
        zf.extractall(out_dir)
    return out_dir


def main(argv: list[str] | None = None) -> int:
    import argparse

    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest="target", required=True)

    pe = sub.add_parser("euroc", help="download an EuRoC sequence")
    pe.add_argument("sequence", help=f"one of {sorted(datasets.EUROC_SEQUENCES)}")

    po = sub.add_parser("openloris", help="download an OpenLORIS scene (large)")
    po.add_argument("scene", help=f"one of {sorted(OPENLORIS_ROSBAG_FILES)}")

    sub.add_parser("openloris-gt", help="download the OpenLORIS ground-truth bundle (~11 MB)")

    args = p.parse_args(argv)
    print(f"cache root: {cache_root()}")
    if args.target == "euroc":
        print("done:", fetch_euroc(args.sequence))
    elif args.target == "openloris":
        for path in fetch_openloris_scene(args.scene):
            print("done:", path)
    elif args.target == "openloris-gt":
        print("done:", fetch_openloris_groundtruth())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
