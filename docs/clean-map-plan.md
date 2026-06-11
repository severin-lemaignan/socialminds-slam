# Clean-map project plan

Plan for the TODO item *"Clean-3D-map project: filter depth to remove outliers;
post-hoc model filtering; … → target: a compact map suitable for downstream tasks
like semantic segmentation."*

Grounded in the ADRs (0002 sensor roles, 0004 map trait, 0005 evaluation-first,
0010 TSDF submaps) and the dense-RGB-D-reconstruction literature surveyed in
[`3d-rgbd-reconstruction.pdf`](3d-rgbd-reconstruction.pdf) (LoopSplat, Zhu et al.
2024, and its related work: KinectFusion lineage, voxel-hashing, Voxgraph,
Loopy-SLAM, Point-SLAM).

## Goal

A **measurably accurate, artifact-free, compact** 3D map product:

- **Accurate:** re-rendered depth from the TSDF matches held-out sensor depth
  (Depth-L1); surface within τ of ground truth on synthetic scenes (F1@5 cm).
- **Clean:** no flying-pixel streaks at silhouette edges, no floaters, no
  ghost geometry from people/doors, no doubled walls after loop closure.
- **Compact:** weight-culled, component-filtered, half-float narrow band; an
  exported **mesh (PLY)** with optional per-voxel RGB as the hand-off artifact
  for semantic segmentation and reMap.

## What the literature says, filtered through our ADRs

The survey's main lessons that transfer:

1. **Submap + pose-graph global consistency is the dominant lever for map
   quality at scale** (LoopSplat, Loopy-SLAM, Voxgraph, BundleFusion). We
   already committed to this (ADR 0010); the map-quality work must therefore be
   *evaluated with and without loop closure*, not treated as independent of it.
2. **Frame-to-model tracking + per-pixel masking of unreliable observations**
   (LoopSplat's alpha/inlier masks, Point-SLAM's density-adaptive sampling) is
   how coupled systems keep the model clean — the cheap classical equivalents
   are depth-edge rejection, range-dependent noise weighting, and
   incidence-angle weighting, all standard since KinectFusion.
3. **Evaluation methodology:** Depth-L1 (render depth from the map at known
   poses vs sensor depth) works on real data without a GT mesh; mesh
   precision/recall/F1 at a τ threshold needs ground-truth geometry → synthetic
   scenes. We adopt both.
4. **What we do NOT adopt:** 3DGS/NeRF map representations. ADR 0004 already
   rejected them (8 GB shared VRAM, rendering ≠ planning representation;
   LoopSplat profiles on an RTX A6000 and is *coupled* — tracking depends on
   the GPU map, violating ADR 0003's CPU-fallback rule). We keep the TSDF and
   borrow only the evaluation and masking ideas. Notably, LoopSplat's core
   registration insight — derive loop edges by registering directly against the
   map representation instead of FPFH+ICP on point clouds — is something our
   point-to-SDF submap registration already does natively.

## Current state (measured 2026-06-10)

- Depth ingest (`slam-replay/src/config.rs`): stride 4, range clip 0.3–6.0 m,
  NaN/inf removal — **no other filtering**.
- Integration (`slam-map/src/sparse.rs:173`): per-ray projective TSDF,
  **constant weight 1 per hit**, no range or incidence weighting; weight cap
  100 000 in the local field (early wrong geometry is effectively frozen
  forever), 64 in the output map.
- Storage: 8 B/voxel (f32 tsdf + f32 weight), blocked hash grid, no pruning,
  no decay.
- Export: STSD voxel dump only. **No mesh extraction. No map-quality metric
  anywhere in the harness** — ATE/RPE and compute only.
- Known measured damage: un-masked people send laser-band depth contribution
  0.164→0.357 ATE on cafe1-1 (gated bridges held off because of it);
  market1-1 reads 0 matched depth registrations (stride spacing > voxel at
  4–6 m range).

## Phases

Ordered so each phase is independently land-able and benchmark-gated.
Per ADR 0005 discipline, **Phase 0 (measurement) blocks everything else**.

### Phase 0 — Map-quality metrics + mesh extraction (the new gate)

We cannot claim "cleaner" without a number. Deliverables:

1. **Marching cubes in `slam-map`** (pure Rust, over the narrow band; classic
   MC is ~a day of work at our scale). `slam-replay --mesh-out FILE.ply`, plus
   an offline STSD→PLY converter so archived dumps stay scoreable.
2. **`harness/map_quality.py`** scoring a run:
   - **Depth-L1**: raycast the final TSDF from GT-aligned held-out poses
     (every Nth frame excluded from integration), compare predicted vs
     measured depth. Runs on real OpenLORIS sequences — the primary metric.
   - **Accuracy / completeness / F1@τ** (τ = 5 cm) of mesh vertices vs ground
     truth — synthetic scenes only.
   - **Hygiene stats** on the STSD dump: voxel count, bytes, weight histogram,
     isolated-surface-voxel fraction, connected-component size distribution.
3. **Synthetic depth-camera scenario** (existing TODO, promoted to
   prerequisite): raycast depth images from the 2.5D world, clean + noisy
   variants (range noise, dropout, *flying-pixel simulation at edges* —
   blended-depth pixels straddling silhouettes, the dominant real artifact),
   plus a walking-person variant. This is the only place F1 is exact and the
   CI guard for everything below.
4. **Archive the baseline** (`eval/reference/baselines/m4-map-quality/`):
   current pipeline's Depth-L1 + hygiene on cafe1-1/-2 and the synthetic
   scenario — the floor to beat, ground0-style.

Gate for all later phases: map metrics improve, **and** the ADR 0010 parity
gate holds (ATE, p99, RSS on cafe1).

### Phase 1 — Input-side depth filtering (cheapest, biggest expected win)

The KinectFusion-lineage standard treatment, all CPU-trivial at our
stride-sampled rates (~13 k points/frame at 10 Hz):

1. **Flying-pixel / depth-edge rejection**: discard samples whose local depth
   gradient across the stride neighborhood exceeds a threshold (relative to
   z — edges scale with range). Kills the smeared streaks at object
   silhouettes that structured-light/stereo sensors (D435) produce; these are
   the single largest outlier class in the literature and visibly the main
   contamination in our rerun dumps.
2. **Range-dependent noise model**: σ(z) quadratic for RealSense-class
   sensors. Used twice: integration weight ∝ 1/σ²(z), and truncation scaled
   with σ(z) (constant 7.5 cm truncation is too tight at 6 m, too loose at
   0.5 m).
3. **Incidence-angle weighting**: w·max(cos θ, ε) from the local depth-image
   normal (already computed for the edge filter — shared stencil). Grazing
   observations of floors/walls stop dragging the surface.
4. **Range-adaptive sampling** (existing TODO, folded in): per-row stride
   chosen so sample spacing ≤ voxel size at the measured depth. Unblocks
   market1-1's 0-matched problem and gives the first market odometry numbers.
5. *Decide by measurement, default to no:* bilateral/median prefilter. At
   stride 4 the edge filter likely captures most of the benefit; only add the
   smoother if Depth-L1 says so.

Config: a `depth.filter` YAML block (ADR 0013): `edge_threshold`,
`noise_model: {none|quadratic}`, `angle_weighting: bool`. Defaults flip on
once the parity gate passes.

Expected side benefit: cleaner depth feeds the 3D registration field directly
→ the two gated bridges (`depth_updates_pose`, `reg_band_tolerance`) can be
re-measured with filtering alone, quantifying how much of the 0.164→0.357
regression is sensor outliers vs people (informs how much Phase 2's masking
hook must carry).

### Phase 2 — Post-hoc model filtering + compaction (the "compact map" deliverable)

Offline/at-freeze passes — zero hot-path risk, so this lands *before* the
riskier fusion changes:

1. **Weight-threshold culling**: drop voxels with weight < k (k ≈ 2–3
   observations) — removes one-shot outliers that survived Phase 1.
2. **Connected-component floater removal**: components of surface voxels
   (|tsdf| < voxel) below N voxels are deleted.
3. **Narrow-band re-trim + empty-block compaction** after culling.
4. **Half-float TSDF** (existing TODO): 8→4 B/voxel; conformance-suite
   tolerance check covers the precision loss.
5. Where it runs: (a) at **submap freeze** (frozen submaps are immutable —
   filter exactly once, ADR 0010's lifecycle gives the natural hook), and
   (b) as `slam-replay --map-filter` / standalone STSD tool for post-hoc use
   on any dump.
6. **Voxel RGB** (existing TODO, config-gated) rides along here as the
   semantic-segmentation enabler: surface-voxel color accumulation, exported
   per-vertex on the mesh. Implementation shares the color-decode path with
   dynamics masking (ADR 0002) — build the decode once.

### Phase 3 — Fusion-side robustness: free-space carving + decay

The TSDF realization of the occupancy-decay commitment (ADR 0002/0004),
evicting people/door ghosts that input filtering cannot catch:

1. **Asymmetric free-space carving** — ✅ **done** (ADR 0014, 2026-06-11):
   multiplicative weight decay along free segments, **every active field**,
   block-skip ray walk; measured 98.7 % stale-ghost eviction on the synthetic
   dynamic variant, and the difference between collapse (114 m) and tracking
   (0.90 m) on the 120 s busy-crowd scenario; p99 0.9 → ~3.5 ms accepted.
   CI-gated (`eval/tests/test_map_hygiene.py` incl. the 60 s busy gate, maskless
   by construction). Depth rays carve the same fields automatically.
2. **Weight-cap sanity**: the local field's cap of 100 000 means early wrong
   geometry is unrevisable; tune toward the literature's 64–256 so the map
   stays plastic. Benchmark-gated — the high cap may be load-bearing for
   registration stability (suspected reason it was set high; measure, don't
   assume).
3. **Mask hook in depth ingest**: a per-pixel mask interface applied at decode
   time, fed initially from *sidecar mask files* (precomputed offline) so we
   can measure the dynamics-masking ceiling on cafe/market **before** the
   YOLO-seg pipeline (ADR 0002, separate TODO) exists. When live masking
   lands it plugs into this hook; the gated bridges then flip on.

Phase 3 item 3 is the formal interface boundary with the *dynamics masking*
TODO — that project supplies the masks; this project consumes them.

### Phase 4 — Global consistency of the exported map

Rides the existing Stage 3b roadmap (GTSAM pose graph re-posing frozen
submaps) rather than duplicating it; the clean-map additions are:

1. **Overlap-aware lazy fusion at export**: when re-posed submaps disagree in
   overlap regions, blend by weight (and recency at freeze) instead of
   last-writer-wins — removes doubled walls after closure.
2. **Extend the ADR 0005 loop-closure criterion to the map**: report Depth-L1
   and hygiene with/without loop closure, alongside the existing
   ATE-with/without.

## What we deliberately do not do

- **3DGS / NeRF / neural-implicit map** — rejected by ADR 0004 (VRAM, CPU
  fallback, planning representation); revisit only per that ADR.
- **Per-frame heavy nets** (Mask R-CNN, learned depth completion) — ADR 0002's
  real-time CPU rule.
- **Voxel rewrites on loop closure** — submap re-posing only (ADR 0010).

## Sequencing, gates, paperwork

| Order | Phase | Depends on | Gate |
|-------|-------|-----------|------|
| 1 | 0 metrics + mesh + synthetic depth scenario | — | metrics reproduce; baseline archived |
| 2 | 1 input filtering | 0 | Depth-L1 ↓, hygiene ↓, parity gate holds |
| 3 | 2 post-hoc filter + compaction | 0 | F1 ↑ on synthetic, bytes ↓, no ATE change |
| 4 | 3 carving/decay + mask hook | 0 (1 recommended) | ghost eviction on synthetic walking-person; parity holds |
| 5 | 4 export fusion | Stage 3b | Depth-L1 with-loop ≤ without-loop |

- One ADR: **map-quality evaluation methodology + clean-map pipeline**
  (extends ADR 0005; records the noise model, filter defaults, decay rule).
- Every phase: small commits, benchmarked against
  `eval/reference/baselines/m3-planar-frontend/` + the new
  `m4-map-quality` baseline; regressions justified explicitly per ADR 0010.
- CI: synthetic depth scenario (clean + noisy) asserts absolute bounds and
  bounded noisy/clean degradation ratios, per ADR 0010 decision 6.
