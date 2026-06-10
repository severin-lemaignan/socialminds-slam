# Published SotA results — the literature's numbers on our benchmark data

Machine-readable results of established SLAM systems on **OpenLORIS-Scene**, extracted
verbatim from the dataset paper (Shi et al., ICRA 2020, arXiv:1911.05603v2 —
[`docs/openloris-scene.pdf`](../../../docs/openloris-scene.pdf)). Unlike
[`../baselines/`](../baselines/) (reference systems *we* run on *our* robot/data), these
are the authors' published numbers — the state-of-the-art context our engine is measured
against.

**Data:** [`openloris-scene-paper.json`](openloris-scene-paper.json), three blocks:

| Block | Paper source | What it measures |
|---|---|---|
| `per_sequence` | Fig. 2 | Each sequence run independently: tracking robustness (CR∞) + accuracy (ATE RMSE) |
| `lifelong` | Fig. 3 | Sequences fed consecutively: re-localization into a prior session's map |
| `relocalization` | Table III | Re-localization score on `office` pairs isolating one changing factor each |

Tested systems: ORB-SLAM2 (stereo-fisheye, RGB-D), DS-SLAM, DSO, VINS-Mono (color,
fisheye), InfiniTAMv2, ElasticFusion, and the robot's wheel odometry.

## How to read the numbers (caveats that matter)

- **Per-scene averages, not per-sequence.** The paper publishes only scene-level values,
  averaged over each scene's 2–7 sequences (weighted by time span for CR, by pose count
  for ATE). Our harness reports per-sequence ATE, so e.g. our `cafe1-1` number compares
  against the paper's `cafe` average (cafe1-1 + cafe1-2) — directionally fine, not exact.
- **ATE is computed only over poses the algorithm actually estimated.** A system that
  tracks 9 % of a sequence accurately gets a *better* ATE than one that survives the whole
  run (the paper notes this negative ATE↔CR correlation explicitly). **Never compare ATE
  without its CR.** ElasticFusion's CR=100 % everywhere is tracking-success only (CR∞),
  with huge ATEs in large scenes.
- **No tested system uses the 2D lidars — and the ground truth itself is laser-based**
  (a hector_mapping variant localizing in a pre-built map for cafe/corridor). Our scan
  front-end estimates from the *same sensor modality* the GT was derived from, so its
  errors can correlate with the GT's and a very low lidar ATE is partly expected — treat
  it as platform potential, not algorithmic superiority over the camera-based systems.
- The paper interpolates ground truth to each estimate's timestamp (laser-based GT is
  low-rate); `evo` matches nearest timestamps. Negligible at our error scales.
- Lifelong CR uses ATE thresholds of 1/3/5 m (small/medium/large scenes) + AOE ≤ 30°;
  the paper itself flags large-scene values as noisy (alignment artifacts in
  `corridor-1`/`market-1`).

## Reference points for our engine

- **Best per-sequence robustness:** VINS-Mono — the only system >70 % CR∞ in `corridor`
  (the perceptual-aliasing scene our lidar front-end targets). Wheel odometry alone
  out-tracks every visual system.
- **Accuracy to beat (per-sequence, with CR context):** `office` ~0.07–0.16 m,
  `cafe` 0.25 m (VINS-fisheye, 95 % CR), `corridor` ~1.3–3.4 m — *our scan front-end's
  0.09 m on cafe1-1 already sits well under the cafe SotA*.
- **Lifelong re-localization is where everything collapses** (most CRs < 50 %; changed
  viewpoint and low light defeat all systems in Table III) — the gap our loop-closure
  priority is aimed at.
