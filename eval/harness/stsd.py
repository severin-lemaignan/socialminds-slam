"""Reader for `slam-replay --map-out` STSD voxel dumps.

Format (little-endian): `STSD` magic, u32 version, f64 voxel size, u64 count, then per
voxel `i32 ix, i32 iy, i32 iz, f32 tsdf, f32 weight`. Voxel `i` is centred at
`(i + 0.5) · voxel_size`.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from pathlib import Path

import numpy as np

_RECORD = np.dtype([("index", "<i4", 3), ("tsdf", "<f4"), ("weight", "<f4")])


@dataclass(frozen=True)
class MapDump:
    voxel_size: float
    voxels: np.ndarray  # structured array: index (3×i4), tsdf (f4), weight (f4)

    @property
    def centres(self) -> np.ndarray:
        """World-frame voxel centres, shape (n, 3)."""
        return (self.voxels["index"].astype(float) + 0.5) * self.voxel_size

    def surface(self) -> np.ndarray:
        """Mask of near-surface voxels (|tsdf| < voxel size)."""
        return np.abs(self.voxels["tsdf"]) < self.voxel_size


def read_stsd(path: Path) -> MapDump:
    with Path(path).open("rb") as f:
        magic = f.read(4)
        if magic != b"STSD":
            raise ValueError(f"{path}: not an STSD dump (magic {magic!r})")
        (version,) = struct.unpack("<I", f.read(4))
        if version != 1:
            raise ValueError(f"{path}: unsupported STSD version {version}")
        (voxel_size,) = struct.unpack("<d", f.read(8))
        (count,) = struct.unpack("<Q", f.read(8))
        voxels = np.frombuffer(f.read(), dtype=_RECORD)
        if len(voxels) != count:
            raise ValueError(f"{path}: header says {count} voxels, found {len(voxels)}")
    return MapDump(voxel_size=voxel_size, voxels=voxels)


def ghost_fraction(dump: MapDump, room: tuple[float, float, float, float], clearance: float = 0.3) -> float:
    """Fraction of surface voxels farther than `clearance` from every wall of the
    rectangular `room` (x_min, x_max, y_min, y_max) — the synthetic ghost metric
    (ADR 0014): in a wall-only world, off-wall surface is by definition phantom.
    """
    surface = dump.surface()
    if not surface.any():
        return 0.0
    c = dump.centres[surface]
    x_min, x_max, y_min, y_max = room
    d_wall = np.minimum.reduce(
        [np.abs(c[:, 0] - x_min), np.abs(c[:, 0] - x_max), np.abs(c[:, 1] - y_min), np.abs(c[:, 1] - y_max)]
    )
    return float((d_wall > clearance).mean())
