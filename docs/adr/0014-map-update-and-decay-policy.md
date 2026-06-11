# ADR 0014: Map update & decay policy — contradiction-driven carving, no time decay, masking never load-bearing

- **Status:** accepted
- **Date:** 2026-06-11
- **Deciders:** Séverin Lemaignan

## Context

The synthetic dynamic-scan variant (walkers + a body-frame follower, ~18 % of beams)
made the map-hygiene problem measurable: after a 20 s run, **70 % of the final map's
surface voxels are ghosts** left by people (2384 of 3411; the clean run has 0), at a
median weight of 21. Registration is unaffected (ATE 0.0613 dynamic vs 0.0614 clean —
the truncation band rejects ghost residuals), but for every *map consumer* — Nav2
costmaps, reMap, semantic segmentation, the mesh export — those voxels are phantom
obstacles.

Mechanically, ghosts are permanent today for three compounding reasons
(`slam-map/src/sparse.rs`):

1. Integration only touches the ±truncation band around each **new hit**; the free
   segment of the ray is never traversed, so the evidence that a voxel is now empty is
   discarded before it is collected.
2. The running average cannot evict: a weight-1 observation against a weight-21 ghost
   moves it by 1/22.
3. The local field's weight cap (100 000) makes geometry effectively write-once.

Two prior measurements constrain the solution space:

- **Long memory is load-bearing for registration.** Cutting `max_weight` to 8 made
  ATE 15× worse on cafe1-2 ("the map follows its own drift", commit b15b19a). Any
  policy that erodes *unobserved* geometry — time decay, low weight caps — re-creates
  that failure.
- **Robust people-masking cannot be assumed.** The real robot's RGB-D cameras sit
  close to floor level; clean person recognition/segmentation from that viewpoint is
  doubtful. M5's masking remains worth building (it measurably dominates the *depth*
  error), but **no map-quality or accuracy strategy may uniquely or critically depend
  on it**. The 2D lidars cannot be semantically masked at all. Geometry-driven
  eviction must carry the dynamics load on its own.

One policy currently serves three memories with different jobs:

| Memory | Job | Policy it wants |
|---|---|---|
| Active registration fields (2D + 3D) | stable reference for the pose solve | **stability** — long memory (measured) |
| Map product (mesh → Nav2 / reMap / semantics) | truthful *current* geometry | **eviction** of contradicted geometry |
| Frozen submaps (loop verification, re-localization) | stable *historical* geometry | **immutability** (ADR 0010); filter once at freeze |

## Decision

1. **Free-space carving, driven by contradiction, not time.** During integration,
   each ray also traverses its free segment (sensor → one truncation before the hit).
   Every *allocated* voxel found there is observed strongly free; its weight is
   decayed **multiplicatively** (`weight *= carve_factor`, default 0.5), and below
   weight 1 the voxel reverts to unobserved. The TSDF value is never edited — carving
   is an evidence-removal rule, not a fusion rule. Voxels never observed again keep
   their memory forever; only actively contradicted geometry dies. Config-gated in
   `TsdfConfig` (`carve_factor`, 1.0 = off), **on for every active field —
   registration fields included.** The free-segment walk probes at half-block
   strides and descends to voxel resolution only across allocated spans, so empty
   corridors cost a handful of hash probes per ray.

   *Correction (same day):* the first version of this ADR exempted the registration
   fields, on two measurements that did not survive scrutiny. The "carving the reg
   field costs p99 0.9 → 15 ms" figure was a mis-attribution — the 15 ms tail is
   loop-closure events (present with carving off too); the true cost of carving all
   fields is p99 0.9 → ~3.5 ms / −25 % RTF (no-loops protocol, cafe1). And "ghosts
   are inert for registration" only holds on *short* runs: a 120 s busy scenario
   (~29 % of beams on people, including a lingering "queue" walker) collapses an
   uncarved registration field — **ATE 114 m scan-only / 1.66 m with the odom
   prior, half the scans coasting** — because current people eventually overlap
   earlier stamps and become moving attractors. Carved: 2.7 m / 0.90 m, tracking
   held (98 % matched). Accuracy parity on cafe1 is unaffected (0.039/0.054).
   CI-gated: `test_busy_environment_stays_tracked` (60 s busy + odom prior,
   maskless).
2. **No time-based decay in the TSDF.** ADR 0004's "occupancy decay" commitment is
   realised *as carving*: decay-by-contradiction, not decay-by-clock. Uniform
   forgetting is the `max_weight: 8` failure spelled differently and is hostile to
   lifelong mapping. (A separate fast-decay occupancy layer for Nav2 remains possible
   later — as its own product, never as the SLAM map's policy.)
3. **The weight cap stays high in the registration fields.** With eviction handled by
   carving, the cap's only job is averaging inertia — which registration measurably
   needs. The cap/carve pair replaces the cap-only compromise.
4. **Frozen submaps are never carved or decayed.** They are immutable (ADR 0010);
   their hygiene hook is one-shot filtering *at freeze time* (clean-map plan Phase 2:
   weight culling, floater removal, compaction).
5. **Masking is an enhancer, not a foundation.** When M5's masking exists it removes
   people *before* integration on the depth path — fewer ghosts to carve, faster
   convergence — but every gate must also pass with masking absent or wrong
   (floor-level cameras). CI enforces this by construction: the dynamic-scan gates run
   maskless.

## Consequences

- **Easier:** the map product self-heals — revisited space evicts people, moved
  chairs, opened doors without any semantic model; the Nav2/reMap/mesh consumers see
  current geometry; the `max_weight` tension dissolves; registration survives busy
  social environments (the robot's actual deployment) instead of collapsing after
  ~a minute of crowd exposure.
- **Harder / costs:** integration now walks full free segments — bounded by visiting
  only allocated blocks (hash-skip). Measured on cafe1 (no-loops protocol): keyframe
  p99 0.9 → ~3.5 ms, RTF 53× → ~38× — accepted explicitly per the ADR 0010 parity
  discipline; the absolute budget (≥ 40 ms per scan at 25 Hz) keeps 10× headroom,
  and per-beam subsampling of the carve is the ready knob if it tightens. Grazing
  rays can transiently carve true-wall band voxels; continuous reinforcement
  out-heals the 0.5 decay in practice — guarded by the parity gate (clean accuracy
  must hold) and the dynamic-variant gates.
- **Risks accepted:** carving trusts the pose — under gross drift it could erode true
  geometry along mis-projected rays; bounded by the keyframe integration diet and by
  carving only the active submap (frozen history is safe). Revisit if the loop-closure
  stress sequences show closure-rate degradation attributable to carved bands.
- **Revisit when:** the OpenVDB backend lands (carving must join the conformance
  suite); when masking lands (re-measure how much carving load it removes on depth);
  if Nav2 integration wants a faster-forgetting occupancy product.

## Alternatives considered

- **Time-based weight decay** (every voxel fades unless re-observed): erodes
  never-revisited corridors exactly as fast as ghosts; measured proxy (`max_weight` 8)
  was 15× worse ATE. Rejected as the TSDF policy.
- **Low weight cap only** (the current "precursor" stance): cannot evict (dilution is
  1/weight) without also destabilising registration. Rejected by measurement.
- **Masking-first** (rely on M5 segmentation to keep people out): unavailable for the
  2D lidars, unreliable from floor-level cameras, and violates the new constraint that
  no strategy critically depends on masking. Kept strictly as an additive enhancer.
- **Per-voxel occupancy statistics (log-odds hit/miss)**: duplicates what the TSDF
  weight already is, at +memory; carving achieves the same eviction with the existing
  8 B/voxel layout.
- **Carve-by-overwrite (integrate free-space observations as +truncation updates)**:
  the symmetric-update standard (KinectFusion lineage) — but it *allocates* free-space
  voxels (narrow-band memory budget gone) and still evicts at dilution speed. The
  asymmetric weight-decay rule keeps the band sparse and evicts geometrically.
