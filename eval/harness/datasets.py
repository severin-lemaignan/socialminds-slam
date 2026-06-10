"""Datasets as a uniform interface.

A [`Sequence`] is everything the harness needs to benchmark one run: an IMU stream and a
ground-truth trajectory, materialised on disk in the engine's input formats (the
`slam-replay` IMU CSV and a TUM ground-truth file). Adapters convert each source dataset
into that shape so the rest of the harness is dataset-agnostic.

Sources, by what we can run *today* (IMU-only baselines) vs. the roadmap:

- `synthetic` — generated, no download, no GPU; the CI dataset.
- `euroc` — EuRoC MAV (ETH ASL): real 200 Hz gyro+accel IMU + Vicon/Leica ground truth,
  in small CSVs. The first *real-data* benchmark we can run with the IMU baseline.
- `openloris`, `tum_rgbd` — the robot-relevant indoor/dynamic datasets; their adapters
  need the RGB-D / lidar front-ends (M3+) to be meaningful, so only download helpers and
  format notes live here for now.

Frame/convention notes for EuRoC are documented in `convert_euroc`.
"""

from __future__ import annotations

import csv
import shutil
import subprocess
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class Sequence:
    """A benchmark-ready sequence: CSVs materialised on disk, or a bag streamed direct."""

    name: str
    source: str
    imu_csv: Path | None
    groundtruth_tum: Path
    duration_s: float
    has_gyro: bool
    # Planar laser scans (scan CSV), when the source records a 2D lidar (M3 front-end).
    scan_csv: Path | None = None
    # Direct-bag replay: `slam-replay --bag` streams these topics straight from the ROS1
    # bag, skipping CSV materialisation entirely (the split IMU is merged in Rust).
    bag: Path | None = None
    bag_gyro_topic: str | None = None
    bag_accel_topic: str | None = None
    bag_scan_topic: str | None = None


def _ns_to_seconds_str(ns: int) -> str:
    """Exact nanoseconds → decimal-seconds string (no float rounding).

    Mirrors `slam_types::Stamp::from_seconds_str`, so high-rate stamps stay exact.
    """
    sign = "-" if ns < 0 else ""
    ns = abs(ns)
    return f"{sign}{ns // 1_000_000_000}.{ns % 1_000_000_000:09d}"


# --------------------------------------------------------------------------------------
# Synthetic
# --------------------------------------------------------------------------------------

def materialize_synthetic(workdir: Path, spec=None) -> Sequence:
    """Generate the synthetic sequence into ``workdir`` (see `harness.synthetic`)."""
    from . import synthetic

    spec = spec or synthetic.TrajectorySpec()
    samples = synthetic.generate(spec)
    workdir = Path(workdir)
    workdir.mkdir(parents=True, exist_ok=True)
    imu_csv = workdir / "imu.csv"
    gt_tum = workdir / "groundtruth.tum"
    synthetic.write_imu_csv(samples, imu_csv)
    synthetic.write_groundtruth_tum(samples, gt_tum)
    return Sequence(
        name="synthetic",
        source="synthetic",
        imu_csv=imu_csv,
        groundtruth_tum=gt_tum,
        duration_s=spec.duration_s,
        has_gyro=True,
    )


# --------------------------------------------------------------------------------------
# EuRoC MAV (ETH ASL format: a `mav0/` directory)
# --------------------------------------------------------------------------------------

# EuRoC column layouts (ASL "dataset" format). Both files are timestamped in ns.
#   imu0/data.csv:   ts, w_x, w_y, w_z, a_x, a_y, a_z              (gyro rad/s, accel m/s^2)
#   state_groundtruth_estimate0/data.csv:
#                    ts, p_x, p_y, p_z, q_w, q_x, q_y, q_z, ...     (rest: v, biases)

def convert_euroc(mav0_dir: Path, workdir: Path, name: str = "euroc") -> Sequence:
    """Convert an EuRoC ``mav0/`` directory to a [`Sequence`].

    Conventions: EuRoC's accelerometer reports specific force in the IMU body frame
    (gravity included), matching our convention; the world frame is gravity-aligned (Z up),
    matching the engine's ``g_vec = (0, 0, −g)``. Ground truth ``p_RS_R`` / ``q_RS`` is the
    IMU body pose in the reference frame, written out in TUM order (quaternion reordered
    ``w,x,y,z`` → ``x,y,z,w``). The IMU and ground-truth streams are sampled at different
    rates; the metrics layer associates them by timestamp.
    """
    mav0_dir = Path(mav0_dir)
    workdir = Path(workdir)
    workdir.mkdir(parents=True, exist_ok=True)

    imu_src = mav0_dir / "imu0" / "data.csv"
    gt_src = mav0_dir / "state_groundtruth_estimate0" / "data.csv"
    if not imu_src.exists():
        raise FileNotFoundError(f"EuRoC IMU file not found: {imu_src}")
    if not gt_src.exists():
        raise FileNotFoundError(f"EuRoC ground-truth file not found: {gt_src}")

    imu_csv = workdir / "imu.csv"
    gt_tum = workdir / "groundtruth.tum"

    first_ns: int | None = None
    last_ns: int | None = None

    with imu_src.open() as fsrc, imu_csv.open("w") as fdst:
        fdst.write("# t gx gy gz ax ay az  (seconds, rad/s, m/s^2)  [from EuRoC imu0]\n")
        for row in csv.reader(fsrc):
            if not row or row[0].lstrip().startswith("#"):
                continue
            ns = int(row[0])
            gx, gy, gz, ax, ay, az = (float(v) for v in row[1:7])
            first_ns = ns if first_ns is None else first_ns
            last_ns = ns
            fdst.write(f"{_ns_to_seconds_str(ns)} {gx:.9f} {gy:.9f} {gz:.9f} {ax:.9f} {ay:.9f} {az:.9f}\n")

    with gt_src.open() as fsrc, gt_tum.open("w") as fdst:
        fdst.write("# timestamp tx ty tz qx qy qz qw  [from EuRoC state_groundtruth]\n")
        for row in csv.reader(fsrc):
            if not row or row[0].lstrip().startswith("#"):
                continue
            ns = int(row[0])
            px, py, pz, qw, qx, qy, qz = (float(v) for v in row[1:8])
            fdst.write(
                f"{_ns_to_seconds_str(ns)} {px:.9f} {py:.9f} {pz:.9f} "
                f"{qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}\n"
            )

    duration = 0.0 if first_ns is None or last_ns is None else (last_ns - first_ns) / 1e9
    return Sequence(
        name=name,
        source="euroc",
        imu_csv=imu_csv,
        groundtruth_tum=gt_tum,
        duration_s=duration,
        has_gyro=True,
    )


# EuRoC is distributed via the ETH Research Collection as one zip *per collection* (the old
# per-sequence ASL links are retired). Each collection zip bundles several sequences in ASL
# (mav0) format. Download by bitstream UUID:
#   https://www.research-collection.ethz.ch/server/api/core/bitstreams/<uuid>/content
RESEARCH_COLLECTION_BITSTREAM = (
    "https://www.research-collection.ethz.ch/server/api/core/bitstreams/{uuid}/content"
)
EUROC_COLLECTIONS: dict[str, str] = {
    "machine_hall": "7b2419c1-62b5-4714-b7f8-485e5fe3e5fe",
    "vicon_room1": "02ecda9a-298f-498b-970c-b7c44334d880",
    "vicon_room2": "ea12bc01-3677-4b4c-853d-87c7870b8c44",
    "calibration_datasets": "5732e864-10f1-49e7-befb-669ee29ff770",
}
# Sequence → collection it lives in.
EUROC_SEQUENCES: dict[str, str] = {
    "MH_01_easy": "machine_hall",
    "MH_02_easy": "machine_hall",
    "MH_03_medium": "machine_hall",
    "MH_04_difficult": "machine_hall",
    "MH_05_difficult": "machine_hall",
    "V1_01_easy": "vicon_room1",
    "V1_02_medium": "vicon_room1",
    "V1_03_difficult": "vicon_room1",
    "V2_01_easy": "vicon_room2",
    "V2_02_medium": "vicon_room2",
    "V2_03_difficult": "vicon_room2",
}


def euroc_collection(seq_name: str) -> str:
    """Return the collection a sequence belongs to."""
    if seq_name not in EUROC_SEQUENCES:
        raise KeyError(f"unknown EuRoC sequence {seq_name!r}; known: {sorted(EUROC_SEQUENCES)}")
    return EUROC_SEQUENCES[seq_name]


def euroc_download_url(seq_name: str) -> str:
    """Return the ETH Research Collection download URL for the *collection* zip that
    contains ``seq_name``."""
    uuid = EUROC_COLLECTIONS[euroc_collection(seq_name)]
    return RESEARCH_COLLECTION_BITSTREAM.format(uuid=uuid)


def _euroc_seq_prefix(seq_name: str) -> str:
    """Short id used to match a sequence's directory, e.g. ``MH_01_easy`` → ``MH_01``."""
    return "_".join(seq_name.split("_")[:2])


def locate_euroc_mav0(extracted_root: Path, seq_name: str) -> Path:
    """Find a sequence's ``mav0/`` directory inside an extracted collection.

    Handles both layouts seen in the wild: a per-sequence directory containing ``mav0``,
    or a nested per-sequence zip that must be unpacked first.
    """
    import zipfile

    root = Path(extracted_root)
    prefix = _euroc_seq_prefix(seq_name)

    def matches(path: Path) -> bool:
        return any(p == seq_name or p.startswith(prefix) for p in path.parts)

    for mav0 in sorted(root.rglob("mav0")):
        if mav0.is_dir() and matches(mav0):
            return mav0

    for nested in sorted(root.rglob("*.zip")):
        if nested.stem == seq_name or nested.stem.startswith(prefix):
            out = nested.with_suffix("")
            with zipfile.ZipFile(nested) as zf:
                zf.extractall(out)
            for mav0 in sorted(out.rglob("mav0")):
                if mav0.is_dir():
                    return mav0

    raise FileNotFoundError(f"no mav0/ for {seq_name!r} under {root}")


# --------------------------------------------------------------------------------------
# OpenLORIS-Scene (the robot's twin) — IMU path via the Rust ROS1 bag reader
# --------------------------------------------------------------------------------------

def _imu_csv_duration(imu_csv: Path) -> float:
    """Span (seconds) between the first and last sample in an IMU CSV."""
    first = last = None
    with Path(imu_csv).open() as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            t = float(line.split()[0])
            first = t if first is None else first
            last = t
    return 0.0 if first is None or last is None else last - first


# OpenLORIS bags carry RealSense-style *split* IMU streams: gyro and accel arrive as
# separate sensor_msgs/Imu topics at different rates, per device. We use the d400 (the
# RGB-D camera, the robot-relevant device); its gyro is the denser stream, so it is the
# merge time base.
OPENLORIS_GYRO_TOPIC = "/d400/gyro/sample"
OPENLORIS_ACCEL_TOPIC = "/d400/accel/sample"
# The Hokuyo UTM-30LX planar lidar.
OPENLORIS_SCAN_TOPIC = "/scan"


def _tum_duration(tum_path: Path) -> float:
    """Span (seconds) between the first and last pose of a TUM trajectory file."""
    # Same line shape as an IMU CSV (timestamp first), so the parser is shared.
    return _imu_csv_duration(tum_path)


def _parse_imu_rows(path: Path) -> list[list[str]]:
    """IMU CSV → raw column strings (timestamps must survive a merge bit-exact)."""
    rows = []
    with Path(path).open() as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            rows.append(line.split())
    return rows


def merge_split_imu(gyro_csv: Path, accel_csv: Path, out_csv: Path) -> None:
    """Merge RealSense-style split IMU streams into one 6-axis IMU CSV.

    Gyro samples are the time base; accel is linearly interpolated at each gyro
    timestamp. Gyro samples outside the accel time span are dropped (no extrapolation).
    The gyro columns (timestamp included) pass through verbatim, keeping stamps exact.
    """
    gyro = _parse_imu_rows(gyro_csv)
    accel = [[float(v) for v in row] for row in _parse_imu_rows(accel_csv)]
    if not gyro or not accel:
        raise ValueError(f"empty IMU stream: gyro={len(gyro)} accel={len(accel)} samples")

    i = 0
    with Path(out_csv).open("w") as f:
        f.write("# t gx gy gz ax ay az  (seconds, rad/s, m/s^2)  [merged split IMU]\n")
        for row in gyro:
            t = float(row[0])
            if t < accel[0][0] or t > accel[-1][0]:
                continue
            while i + 1 < len(accel) and accel[i + 1][0] < t:
                i += 1
            a0, a1 = accel[i], accel[min(i + 1, len(accel) - 1)]
            w = 0.0 if a1[0] == a0[0] else (t - a0[0]) / (a1[0] - a0[0])
            ax, ay, az = (a0[k] + w * (a1[k] - a0[k]) for k in (4, 5, 6))
            f.write(f"{row[0]} {row[1]} {row[2]} {row[3]} {ax:.9f} {ay:.9f} {az:.9f}\n")


def materialize_openloris(
    bag_path: Path,
    groundtruth_txt: Path,
    workdir: Path,
    *,
    name: str = "openloris",
    imu_topic: str | None = None,
    gyro_topic: str | None = None,
    accel_topic: str | None = None,
    extract_scans: bool = True,
    scan_topic: str = OPENLORIS_SCAN_TOPIC,
    bag2imu_bin: Path | None = None,
    bag2csv_bin: Path | None = None,
) -> Sequence:
    """Materialise an OpenLORIS sequence from one ROS1 ``.bag`` + its ground-truth file.

    Sensor streams are extracted by the Rust bag tools (no ROS install) — in **one
    decompression pass** (``slam-bag2csv``) when the topics are explicit, since
    decompressing the bz2 bag dominates the cost. Real OpenLORIS bags split the IMU per
    RealSense convention — pass ``gyro_topic`` + ``accel_topic`` (e.g. the
    ``OPENLORIS_*_TOPIC`` defaults) to extract both and merge them; ``imu_topic`` covers
    single-topic bags (auto-selected when ``None``, via ``slam-bag2imu``). OpenLORIS
    ground truth is *already* TUM-formatted, so it is used directly. RGB-D extraction
    lands with the visual front-end.
    """
    from . import replay

    if (gyro_topic is None) != (accel_topic is None):
        raise ValueError("pass both gyro_topic and accel_topic, or neither")
    if imu_topic and gyro_topic:
        raise ValueError("imu_topic and gyro_topic/accel_topic are mutually exclusive")

    bag_path = Path(bag_path)
    groundtruth_txt = Path(groundtruth_txt)
    workdir = Path(workdir)
    workdir.mkdir(parents=True, exist_ok=True)

    imu_csv = workdir / "imu.csv"
    scan_csv = workdir / "scan.csv" if extract_scans else None

    if gyro_topic and accel_topic:
        # The real OpenLORIS path: everything in one pass.
        gyro_csv = workdir / "gyro.csv"
        accel_csv = workdir / "accel.csv"
        binary = Path(bag2csv_bin) if bag2csv_bin else replay.find_bag2csv_binary()
        cmd = [
            str(binary),
            "--bag", str(bag_path),
            "--imu", f"{gyro_topic}={gyro_csv}",
            "--imu", f"{accel_topic}={accel_csv}",
        ]
        if scan_csv is not None:
            cmd += ["--scan", f"{scan_topic}={scan_csv}"]
        subprocess.run(cmd, check=True)
        merge_split_imu(gyro_csv, accel_csv, imu_csv)
    else:
        # Single/auto IMU topic (fixtures, non-RealSense bags): per-stream tools.
        binary = Path(bag2imu_bin) if bag2imu_bin else replay.find_bag2imu_binary()
        cmd = [str(binary), "--bag", str(bag_path), "--out", str(imu_csv)]
        if imu_topic:
            cmd += ["--imu-topic", imu_topic]
        subprocess.run(cmd, check=True)
        if scan_csv is not None:
            subprocess.run(
                [
                    str(replay.find_bag2scan_binary()),
                    "--bag", str(bag_path),
                    "--scan-topic", scan_topic,
                    "--out", str(scan_csv),
                ],
                check=True,
            )

    gt_tum = workdir / "groundtruth.tum"
    shutil.copyfile(groundtruth_txt, gt_tum)

    return Sequence(
        name=name,
        source="openloris",
        imu_csv=imu_csv,
        groundtruth_tum=gt_tum,
        duration_s=_imu_csv_duration(imu_csv),
        has_gyro=True,
        scan_csv=scan_csv,
    )


# --------------------------------------------------------------------------------------
# Roadmap datasets (full adapters await the RGB-D / lidar front-ends, M3+)
# --------------------------------------------------------------------------------------

# TUM RGB-D dynamic sequences — the cheap dynamic-environment baseline (ADR 0005). Freely
# downloadable; the ground-truth trajectory is a TUM file already, but the useful signal
# (RGB-D) needs the visual front-end, so we only provide the fetch here.
TUM_RGBD_BASE = "https://cvg.cit.tum.de/rgbd/dataset"
TUM_RGBD_SEQUENCES = {
    "fr3_walking_xyz": "freiburg3/rgbd_dataset_freiburg3_walking_xyz.tgz",
    "fr3_walking_halfsphere": "freiburg3/rgbd_dataset_freiburg3_walking_halfsphere.tgz",
    "fr3_sitting_xyz": "freiburg3/rgbd_dataset_freiburg3_sitting_xyz.tgz",
}


def tum_rgbd_download_url(seq_name: str) -> str:
    if seq_name not in TUM_RGBD_SEQUENCES:
        raise KeyError(f"unknown TUM RGB-D sequence {seq_name!r}; known: {sorted(TUM_RGBD_SEQUENCES)}")
    return f"{TUM_RGBD_BASE}/{TUM_RGBD_SEQUENCES[seq_name]}"


def download_tum_rgbd(seq_name: str, dest_dir: Path) -> Path:
    """Download + extract a TUM RGB-D sequence; return its directory. Operator step."""
    import tarfile
    import urllib.request

    dest_dir = Path(dest_dir)
    dest_dir.mkdir(parents=True, exist_ok=True)
    url = tum_rgbd_download_url(seq_name)
    tgz = dest_dir / f"{seq_name}.tgz"
    if not tgz.exists():
        print(f"downloading {url} -> {tgz}")
        urllib.request.urlretrieve(url, tgz)
    with tarfile.open(tgz) as tf:
        tf.extractall(dest_dir)
    extracted = next((p for p in dest_dir.iterdir() if p.is_dir() and p.name.startswith("rgbd_dataset")), None)
    if extracted is None:
        raise FileNotFoundError(f"no rgbd_dataset_* directory under {dest_dir}")
    return extracted

# OpenLORIS-Scene download lives in `harness.fetch` (it is freely hosted on Hugging Face);
# its IMU adapter is `materialize_openloris` above.
