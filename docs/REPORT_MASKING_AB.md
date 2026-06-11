# Dynamics masking A/B on cached OpenLORIS data (ADR 0015)

**Date:** 2026-06-11 · **Machine:** 22-core CPU, **no GPU** (ONNX Runtime CPU EP) ·
**Model:** `onnx/yolo11s-seg-rect.onnx` (fp16, 640×480, camera-shaped export) ·
**Engine:** `slam-replay --baseline scan-matching-3d`, flags identical to
`harness.sensor_matrix` (`--init-pose-from-tum`, loop closure + pose graph on,
depth every 3rd frame). One run per cell — the pipeline is deterministic
(verified: a rebuild reproduced `cafe1-1.depth` bit-identically, `depth.rebuilt`).
ATE = translation RMSE after SE(3) Umeyama alignment (evo, harness-standard);
RPE @ 1 m. Masking per ADR 0015 defaults: conf 0.2, dilate 8 px; class sets as
tagged. Reproduce with `eval/masking-ab/run-ab.sh` + `run-controls.sh`
(resumable; outputs land in the git-ignored `eval/results/masking-ab/`).

## Questions asked

1. Does the ONNX/`ort` route work on a GPU-less machine? *(the integration was
   developed against a GPU machine)*
2. Does masking improve the depth-driven pose path? *(ROADMAP M5: depth-only
   2.8 m, depth→pose 0.16→3.0 m, laser-band 0.164→0.357 m were the numbers to move)*
3. Can the two gated bridges (`depth_updates_pose`, `reg_band_tolerance`) be
   unlocked now that masking exists?

## Results

### ATE (m, RMSE) — masked vs unmasked

| sequence · config | unmasked | masked (dynamic) | masked (person) |
|---|---|---|---|
| cafe1-1 · depth | **0.925** | 1.057 | 1.000 |
| cafe1-1 · odom+depth | **0.892** | 1.134 | 0.932 |
| cafe1-2 · depth | **0.846** | 0.900 | 0.863 |
| cafe1-2 · odom+depth | **0.836** | 0.914 | 0.852 |
| market1-1 · full (depth drives pose) | **4.927** | 5.978 | 5.957 |
| cafe1-1 · full (scan-driven) | 0.169 | 0.169 (bit-identical) | — |
| cafe1-2 · full (scan-driven) | 0.151 | 0.151 (bit-identical) | — |

RPE @ 1 m moves the same direction as ATE on every depth cell (e.g. cafe1-1
depth 0.337 → 0.407): **local registration is genuinely worse with masking**,
not just the global alignment.

Controls: `depth-color` (colour stream, no masking) is **bit-identical** to
`depth` on both cafe sequences — the colour pairing contributes nothing; the
delta is entirely the masking. Person-only masking (7.8–11 % mean coverage vs
14.6–21.4 % for the dynamic set) roughly halves the damage but never beats
unmasked.

### The gated bridges (ADR 0015's stated unlock)

| sequence · bridge | bridge off | bridge on, unmasked | bridge on + masked |
|---|---|---|---|
| cafe1-1 · `--depth-updates-pose` | **0.169** | 0.951 | 1.942 (person: 2.134) |
| cafe1-2 · `--depth-updates-pose` | **0.151** | 1.280 | 6.777 (person: 3.205) |
| cafe1-1 · `--reg-band-tolerance 0.15` | 0.169 | 0.170 | **0.166** |
| cafe1-2 · `--reg-band-tolerance 0.15` | 0.151 | 0.149 | **0.148** |

- **`depth_updates_pose` must stay gated** — and masking makes it *worse*, not
  better (cafe1-2: 6.8 m masked vs 1.3 m unmasked). With the bridge on, verified
  loop counts explode (95 → 213–282); the masked, sparser 3D fields apparently
  pass the 0.55-inlier verification gate at wrong alignments more easily, and the
  bad loops drag the graph. This is the corridor-aliasing failure mode of the
  loop verifier surfacing on depth fields, not a masking bug per se.
- **`reg_band_tolerance 0.15` is no longer a regression** (the archived
  0.164→0.357 predates free-space carving and current registration): neutral
  unmasked, and *the best cafe numbers in this whole matrix with masking on*
  (0.166/0.148). The win is ~2 % — real but marginal.

### Why masking doesn't help the depth path (interpretation)

The motivating numbers predate ADR 0014: free-space carving + robust
registration have already absorbed most of the dynamics damage (depth-only
cafe1-1 was 2.8 m then; it is **0.925 m unmasked today**). What masking adds on
top is mostly *constraint sparsification*: at 640×480 the dynamic class set
discards 15–21 % of pixels — in a café that includes the chairs and tables that
are the nearest, highest-parallax registrable geometry; the survivors
concentrate on far walls and floor, which condition the solve worse. Person-only
masking discards less and correspondingly hurts less, but people points were
already being handled (carved, down-weighted by robust kernels), so there is no
upside left to collect at this operating point.

### CPU-EP compute (the second question of this session)

- `slam-dynamics` builds and its smoke test passes on this GPU-less machine
  (ort 2.0.0-rc.12 downloads its prebuilt CPU runtime at first build; cached).
- Inference ≈ **0.2 s/frame** (640×480 fp16, CPU EP): cafe1-1 (569 frames)
  wall 36 s → 151 s; market1-1 (2047 frames) 5:43 → 6:51. Peak RSS +~280 MB.
- The engine's own real-time factor is unaffected (masking runs in the ingest
  pass), but end-to-end a masked replay is ~1.8× the input span on CPU at
  10 Hz depth — fine for offline replay, **not** robot-real-time on CPU; the
  robot path remains TensorRT (~2.5 ms/frame, per the survey).

## Conclusions

1. **CPU route: validated.** Build, smoke test, and 14 full masked replays ran
   on the CPU EP. (Also fixed in this pass: the smoke test and a config test
   still pointed at the replaced square `yolo11s-seg.onnx` — the smoke test had
   been silently skipping since the rect model landed.)
2. **Keep `depth_updates_pose` gated** — masking does not unlock it; the
   blocker has moved from "people corrupt the registration" to "false-verified
   depth loops corrupt the graph" (→ per-submap appearance signatures / stricter
   depth-loop gating, already the roadmap's next item).
3. **`reg_band_tolerance 0.15` + masking is enable-able** (best cafe ATE,
   0.166/0.148) but the margin is ~2 %; defer flipping the default until the
   loop-verification work lands and it can be re-measured.
4. **Masking stays an enhancer, not a foundation (ADR 0014/0015 reaffirmed)** —
   the default path is provably untouched (bit-identical), and the map-side
   value (people never enter the TSDF/nav map; 14–21 % of pixels rejected at
   ingest) stands independently of these pose numbers, but pose-side it
   currently *costs* accuracy on every depth-driven configuration tested.
5. Operating-point follow-ups if pose-side gains are pursued: per-class
   thresholds (person 0.25 / chair 0.1), smaller dilation, mask propagation,
   and re-balancing `max_registration_points` sampling on masked clouds.
