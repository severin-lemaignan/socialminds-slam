# M3 planar front-end baseline — the parity bar for the 3D migration

Benchmark of the **SE(2) PLICP scan front-end** (multi-lidar-capable, ADR 0007/0009) on
cafe1-1 + cafe1-2, frozen immediately before the move to full-3D TSDF submap
registration (ADR 0010). **The 3D pipeline must match these numbers** — that is a
standing instruction, not an aspiration: each migration stage is benchmarked against
this file, and any regression (accuracy *or* compute) must be explicitly justified and
accepted, never silently absorbed.

| System | Sequence | ATE RMSE (m) | RPE RMSE (m) | Real-time × | p99 (µs) | RSS (MB) |
|---|---|---|---|---|---|---|
| scan_matching | cafe1-1 | **0.0897** | 0.0326 | 27.2 | 3405 | 11.0 |
| scan_matching | cafe1-2 | **0.0665** | 0.0657 | 22.4 | 4273 | 15.2 |

(Full table incl. trivial baselines: [`report.md`](report.md) / [`results.json`](results.json).)

## How to compare

```bash
python -m harness.benchmark --openloris cafe1-1 --openloris cafe1-2 --init-pose-from-gt
```

- **Accuracy:** ATE/RPE are deterministic for this front-end (std = 0 across repeats) —
  compare directly. "Same performance" = within run-to-run noise once the system becomes
  non-deterministic (multi-threaded); then compare mean ± std overlap.
- **Compute:** RTF / latency / RSS are machine-dependent — compare only against a fresh
  planar run **on the same machine**, not against the absolute numbers below when on
  different hardware. Re-run the planar system (it stays in-tree as a baseline) in the
  same session to re-anchor.
- The dual-lidar synthetic harness (`slam-frontend-scan/tests/dual_lidar.rs`) guards the
  multi-lidar accuracy bounds in CI; this baseline guards the real-data end-to-end.

## Provenance

- Commit: `ffd6ee9` (multi-lidar rig front-end), 2026-06-10, 3 repeats,
  `--init-pose-from-gt`, 2-stage (materialised CSV) input path.
- Machine: Intel Core Ultra 9 185H (22 threads), rustc 1.94.1, release profile.
- Caveat (from `eval/reference/sota/`): OpenLORIS cafe ground truth is itself
  laser-based; ATE here partly correlates with GT error. The parity comparison is
  unaffected (both pipelines face the same GT).
