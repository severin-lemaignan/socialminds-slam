# Archived reference baselines

Scored results of reference systems (RTAB-Map, GLIM, …) on benchmark sequences — the
numbers our engine aims to beat. Each file is the JSON written by `python -m harness.score`
(see [`../README.md`](../README.md)), one array of aggregate records:

```json
[
  {
    "system": "rtabmap",
    "sequence": "office1-1",
    "source": "reference",
    "repeats": 1,
    "ate_rmse_m": { "mean": 0.0, "std": 0.0 },
    "rpe_rmse_m": { "mean": 0.0, "std": 0.0 },
    "real_time_factor": { "mean": NaN, "std": NaN },
    "latency_p99_us": { "mean": NaN, "std": NaN },
    "peak_rss_mb": { "mean": NaN, "std": NaN }
  }
]
```

Compute fields are `NaN` because reference systems are run externally where we don't capture
them. Name files `<sequence>_<system>.json` (e.g. `office1-1_rtabmap.json`). These are small
and **committed** (unlike the datasets and our own generated reports).
