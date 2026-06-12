# ADR 0016: Masking altitude — semantics gate durable products, geometry guards the ephemeral estimate

- **Status:** accepted (implementation pending — lands with the appearance-signature work)
- **Date:** 2026-06-12
- **Deciders:** Séverin Lemaignan

## Context

ADR 0015 wired dynamics masking as **rejection at depth ingest**: pixels are
suppressed before back-projection, so every downstream consumer — registration,
TSDF integration, the colour channel, loop closure — inherits the same policy
from a single hook. That uniformity was the point: remove the points once, for
everyone. The expectation was that masking would then unlock the two depth→pose
bridges (`depth_updates_pose`, `reg_band_tolerance`) that had been gated on it.

The same-day A/B (`docs/REPORT_MASKING_AB.md`) falsified that expectation, and
the visual inspection (`mask_dump`) explains *why* in a way that is structural,
not a tuning accident. The problem decomposes into three observations.

**1. The pose estimate did not need the help.** Masking worsens ATE *and* RPE
on every depth-driven pose configuration tested (cafe depth-only 0.925→1.057 m,
market1-1 4.93→5.98 m; person-only class set hurts less, never wins). The
motivating measurements for the bridges (depth-only 2.8 m; depth→pose
0.16→3.0 m) predate free-space carving — ADR 0014's contradiction-driven
eviction, the robust registration kernels, the wheel-odometry prior and the
lidar backbone *already absorb* the dynamics damage on the estimation path.
Registration asks "is this point rigid for the next ~100 ms?" — a **geometric**
question at a timescale where redundancy is huge, outliers self-announce as
large residuals, and any residual error is corrected by the next solve or the
graph. A briefly-stationary person is, within one solve window, a *valid
constraint*. What the estimation path is actually short of is constraints:
the masks are well-aligned (the masker works as designed), but the dynamic
class set removes 16–48 % of a café frame — empty stools, benches, the close
high-parallax structure registration leans on — and in market crowds the
people *are* most of the in-range geometry, so even person-only masking
starves the solve. Worse, detection degrades and misplaces under motion blur:
masking is least dependable on exactly the fast-rotation frames where
registration is most fragile.

**2. The durable products cannot be protected any other way.** The map we hand
to Nav2, the per-submap appearance signatures (the queued M4 corridor-aliasing
and re-localization work, ADR 0010's < 1 s requirement), and the live
re-localization query are different in kind from the pose estimate. Three
properties flip simultaneously:

- *Errors are not self-announcing.* A person baked into a signature produces
  no residual until a wrong loop candidate has already been committed — and a
  wrong loop is the catastrophic failure mode. The A/B measured precisely
  this shape: with depth→pose on, "verified" loop counts explode (95→238)
  and drag the graph to 6.8 m. There is no robust kernel at the descriptor
  level; a single unmasked person can dominate a compressed signature.
- *The comparison spans a dynamics timescale.* Registration compares
  observations 100 ms apart; place recognition compares minutes, hours, or
  sessions apart. At revisit time the people **will** be gone and the chairs
  **will** have moved. The class-set verdict therefore *inverts* per stage:
  a chair is a fine registration constraint today and guaranteed descriptor
  noise next week — the full dynamic set, wrong for registration, is exactly
  right for signatures (which pool over a whole submap and tolerate losing
  20 % of support; registration does not).
- *Carving cannot reach them.* Carving needs revisit evidence — a later beam
  through the voxel. Signatures are computed at submap freeze, often for
  places just left and not re-observed before freezing (the newborn-submap
  bias of ADR 0014 is the same blind spot). A cross-session re-localization
  query is the *first* observation of its session: no temporal evidence
  exists or ever will. Masking is the only mechanism that works one-shot.

**3. One hook, two opposite policies.** The ingest-side rejection point forces
a single masking policy onto consumers whose needs are now measured to be
opposite. The hook is at the wrong altitude: the question is not *whether* to
mask but *which stage* each masked/unmasked view of the cloud feeds.

## Decision

1. **Re-home the mask from rejection to tagging.** Depth frames back-project
   in full; the `PixelMask` travels with the cloud (per-point dynamic tag or
   sidecar mask) instead of deleting points at `parse_depth_image`. Each
   consumer applies its own policy.
2. **Estimation stays geometric and unmasked.** Scan/depth registration and
   the motion prior consume the full cloud — the configuration that won the
   A/B (carving + robust kernels + odometry prior). The bridge gates of
   ADR 0015 are unchanged (`depth_updates_pose` off; `reg_band_tolerance`
   default off).
3. **Durable products are semantically filtered with the full dynamic set:**
   - **TSDF integration:** masked points never integrate. This complements
     carving instead of replacing it — it keeps people out of *newborn*
     submaps before any contradiction evidence exists, the one regime where
     ADR 0014's geometric mechanisms structurally lag.
   - **Appearance signatures:** computed over masked, persistence-weighted
     geometry (voxels that survived carving *and* are not semantically
     dynamic are the durable structure worth describing).
   - **Re-localization queries:** always masked — the one-shot case where
     masking is irreplaceable. Query keyframes should additionally be
     quality-gated (blur/rotation), since mask reliability collapses on
     blurred frames.
4. **ADR 0014's invariant is preserved per stage.** Inference failure, a
   missing colour stream, or a stale mask degrade that stage to today's
   unmasked behaviour (integration that carving cleans up later; an unmasked
   signature that geometric verification still gates). Every accuracy and map
   gate must still pass maskless. Enhancer, never foundation.

## Consequences

- `slam-datasets::parse_depth_image` stops consuming the mask destructively;
  the mask (or a per-point tag) becomes part of the cloud's payload. The
  `masking:` YAML section and `--mask-*` flags keep their meaning; what
  changes is *where* the rejection applies.
- The A/B matrix gains a third arm — "registration unmasked + integration
  masked" — with a concrete acceptance bar: pose accuracy must match the
  unmasked baseline (bit-identical is the expectation, since registration
  inputs are untouched) while map cleanliness matches or beats the
  ingest-masked arm (stale-ghost eviction is already CI-gated maskless;
  newborn-submap people counts are the new observable).
- The signature-side benefit is asserted, not yet measured — it becomes
  measurable only with the appearance-signature implementation, and that
  evaluation (signature precision/recall with/without masking, per ADR 0005)
  must ship with that work. Until then this ADR's estimation-side half rests
  on `docs/REPORT_MASKING_AB.md` and the signature-side half on the three
  structural arguments above.
- Masking inference cost moves off the replay hot loop only conceptually —
  it still runs per kept depth frame (CPU EP ≈ 0.2 s/frame offline, TensorRT
  ≈ 2.5 ms on the robot) but its output now gates integration/signatures
  rather than the cloud itself.
- ADR 0015 remains the runtime/model decision (yolo11s-seg, `ort`, CPU EP
  default, feature-gating, AGPL caveat); this ADR supersedes only its
  *application point* (decision 4 there: "masking applies at depth ingest").

## Alternatives considered

- **Keep ingest-side rejection (status quo):** measured against and falsified
  for the estimation path; retains the single-hook elegance at the cost of
  pose accuracy on every depth-driven configuration.
- **Soft semantic weighting in registration** (down-weight rather than drop
  dynamic-tagged points): adds a tuning surface to a path whose geometric
  robustness already does this job at the right timescale; revisit only if a
  dataset shows registration failing *because of* dynamics that carving and
  kernels miss.
- **Carving-only for the durable products too:** fails structurally — no
  revisit evidence at submap freeze (newborn bias), none ever for
  cross-session re-localization queries.
- **Per-stage class sets beyond static/dynamic** (e.g. person-only for
  integration, full set for signatures): deferred — plausible refinement,
  but the simple split (estimation unmasked / durable products full set)
  should be measured first.
