"""Score an externally-produced trajectory against a dataset's ground truth.

This is how a *reference system* (RTAB-Map, GLIM, …) — run outside this repo, on a machine
with ROS/GPU and the dataset (see ``eval/reference/``) — gets the same ATE/RPE numbers as
our engine, so its result can serve as the "number to beat".

    python -m harness.score --groundtruth gt.tum --estimate rtabmap.tum \\
        --system rtabmap --sequence office1-1 --out eval/reference/baselines/office1-1.json
"""

from __future__ import annotations

import argparse
import json
from dataclasses import asdict
from pathlib import Path

from . import benchmark


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--groundtruth", type=Path, required=True, help="ground-truth TUM file")
    p.add_argument("--estimate", type=Path, required=True, help="estimated TUM trajectory")
    p.add_argument("--system", required=True, help="system name, e.g. rtabmap")
    p.add_argument("--sequence", required=True, help="sequence name, e.g. office1-1")
    p.add_argument("--source", default="reference", help="dataset/source label")
    p.add_argument("--no-align", action="store_true", help="disable SE(3) Umeyama alignment")
    p.add_argument("--out", type=Path, help="write a single-row results.json here")
    args = p.parse_args(argv)

    result = benchmark.score_trajectory(
        args.groundtruth,
        args.estimate,
        system=args.system,
        sequence=args.sequence,
        source=args.source,
        align=not args.no_align,
    )
    print(benchmark.to_markdown([result]))
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps([asdict(result)], indent=2))
        print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
