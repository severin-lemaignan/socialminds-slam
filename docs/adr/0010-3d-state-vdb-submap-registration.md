# ADR 0010: Full-3D base state; registration against TSDF submaps (OpenVDB-backed); re-localization as a first-class capability

- **Status:** accepted
- **Date:** 2026-06-10
- **Deciders:** Séverin Lemaignan

## Context

REQUIREMENTS.md gained five requirements with deep design impact (2026-06-10):
production-grade ambitions, **fast re-localization**, the base modelled as a **3D body**
(the lidar scan plane tilts under acceleration), **sensor registration in a 3D map**
(OpenVDB, anticipating reMap), at **building scale** (100 × 100 × 12 m, three floors),
with **clean + noisy synthetic test data** for every multi-sensor setup.

These collide with two standing assumptions:

1. **ADR 0007 made SE(2) the scan front-end's native group.** A real base pitches and
   rolls when accelerating (order 0.5–2° on suspension at 1–2 m/s²). At 10 m range, 1° of
   tilt moves a beam's hit point ~17 cm vertically: against vertical walls this mostly
   slides the point *along* the wall (small range error — why the planar model worked at
   cafe scale), but against clutter, stair edges and across floors it is real error. The
   lidar cannot observe its own tilt; **the IMU can** (gravity direction at 1 kHz).
2. **ADR 0004 made the map an *output*** (navigation + reMap interop), registered-to
   only implicitly via keyframes. The new requirements make the 3D map the **registration
   substrate** itself.

What survives unchanged: ADR 0009's rig (it already stores full SE(3) extrinsics — only
the front-end's consumption was planar), the lock-free pipeline shape, GTSAM back-end,
the lidars' role as planar-geometry/loop-closure backbone (ADR 0002), and evaluation-first
discipline (ADR 0005).

Scale math that drives the map design (5 cm voxels, three floors): walls + floors +
ceilings + clutter ≈ 10⁵ m² of surface → ~4·10⁷ surface voxels × a ±3-voxel narrow band
≈ 2.4·10⁸ allocated voxels. At 4 B TSDF + 4 B weight that is ~2 GB — tolerable on the
robot, **but a single global grid is untenable under loop closure**: a corridor-loop
correction would rewrite hundreds of millions of voxels. Submaps re-posed rigidly by the
pose graph are the standard and correct answer.

## Decision

**1. The base state is SE(3) everywhere; SE(2) becomes an optimisation, never an
assumption.** The IMU's gravity estimate continuously provides roll/pitch; each 2D scan
is lifted to a **3D fan of points** through `T_world_base(t) · T_base_sensor` (full SE(3)
rig extrinsic — no more planar projection at ingest). Lidar factors constrain the
gravity-aligned (x, y, yaw); roll, pitch and z come from IMU (and later RGB-D / floor
priors). ADR 0007's principle "the lidar never invents out-of-plane motion" survives in
this sharpened form. The PLICP machinery is retained as the planar seed and as the
CI-cheap baseline.

**2. Registration happens against TSDF submaps behind the `Map` trait.** Scan-to-keyframe
is replaced by **scan-to-submap**: incoming fans minimise point-to-SDF residuals
(Gauss–Newton, analytic SDF gradients) against the *local submap's* narrow-band TSDF.
This is the natural completion of ADR 0009's "shared local map": both lidars — and later
both RGB-D depth streams — fuse into and register against the **same submap**, giving
cross-modal coupling for free (a lidar localises against structure a camera built, which
is exactly what the people-occlude-cameras scenario needs). The arithmetic is comfortable:
two lidars ≈ 90 k points/s, three orders of magnitude below the 3D-lidar workloads this
method serves elsewhere — CPU-only real-time is not in doubt (ADR 0003 holds).

**3. Submap architecture for 100 × 100 × 12 m.** One TSDF grid per submap; a submap is
born at a keyframe pose-graph node, grows for ~15–25 m of travel or N keyframes (with
overlap), then freezes. **Loop closure re-poses submap anchors** — voxels are never
rewritten by graph corrections. Frozen submaps are immutable → trivially shareable across
threads (the multi-threading requirement), compressible, and evictable under an LRU
budget (half-float TSDF halves the ~2 GB ceiling; dormant-floor submaps live serialized).
The global map for Nav2/reMap export is a lazy fusion of re-posed submaps (ADR 0004's
layers consume it as before). Voxel size **5 cm** (matches lidar/RGB-D noise; Nav2
costmap downsamples; final value is benchmark-gated, not dogma); narrow band ±3 voxels.

**4. Dual backend: a pure-Rust sparse TSDF is the default/CI backend; OpenVDB is the
production backend.** Both implement the same batch-level `Map` trait (ADR 0004) and must
pass one conformance suite (tolerance-based, not bit-exact). The trait boundary keeps
~75–80 % of the map subsystem backend-agnostic — TSDF fusion math, point-to-SDF
registration, the whole submap lifecycle — while the duplicated layer is only the sparse
container itself (~500 lines of blocked hash-grid on the Rust side). The OpenVDB backend
is **feature-gated** and links the **system-packaged OpenVDB 10.x** (Ubuntu
`libopenvdb-dev`) through a thin `cxx` shim — *not* vendored (unlike GTSAM/ADR 0006):
the robot and reMap hosts install it from apt, and reMap consumes the grids **in-process**
(memory-based integration, pinned at review). FFI calls are batched (one call per
integration/sampling pass, never per voxel). Build order is also sequencing: the Rust
backend lands first and unblocks all algorithm work; the VDB backend lands with the reMap
integration, validated by the already-existing conformance suite.

**5. Fast re-localization is a designed capability, not an emergent one.** Every frozen
submap carries a compact place signature (MapClosures-style 2D density images of its
gravity-aligned structure — the same machinery as loop-closure detection, deliberately).
Cold start or tracking loss: gravity from the IMU pins roll/pitch; indoor z is quantized
by floor candidates; the search is therefore ~3-DoF (x, y, yaw) over submap signatures,
then 6-DoF point-to-SDF refinement, **always geometrically verified** before adoption
(the ADR 0002 rule). Budget target: **≤ 1 s** to verified re-localization in a mapped
area. Scored with the OpenLORIS lifelong protocol (CR, CS-R) already archived in
`eval/reference/sota/` — where every published system collapses (< 52 % CR; 0 under
viewpoint change and low light). This is the gap the project exists to close.

**6. Clean + noisy synthetic data for every multi-sensor capability.** The raycast
harness and the Python generator grow a **noise suite**, each scenario in clean and noisy
variants: (a) extrinsic perturbation (exists — dual-lidar test), (b) **transient
scan-plane tilt** — roll/pitch pulses during acceleration, raycast against a 2.5D world
with finite-height walls so a tilted fan reports the slant ranges a real tilted lidar
would, (c) walking dynamic objects (exists single-lidar; becomes standard), (d) range
noise + dropout. CI asserts both absolute bounds on the noisy runs and bounded
degradation ratios noisy/clean. The 3D-tilt scenario is the acceptance test for
Decision 1.

## Consequences

- **Easier:** loop closure and re-localization share one mechanism (submap signatures +
  geometric verification); RGB-D integration lands into an already-3D registration
  pipeline instead of forcing a second migration; reMap interop becomes "hand over VDB
  grids"; graph corrections are O(submaps), not O(voxels).
- **Harder:** the front-end becomes attitude-dependent (IMU fusion moves from "later" to
  "prerequisite for tilt compensation"); two map backends + conformance suite to
  maintain, with drift between them held down by the suite and by keeping VDB-only
  conveniences out of the core trait (extension trait only). ADR 0001's "GTSAM is the
  only mandatory C++ dependency" formally becomes "…the only *unconditionally* mandatory
  one": OpenVDB is a feature-gated **system** dependency (apt, not vendored), present on
  robot/reMap hosts and the gated CI job only — dev machines and default CI stay C++-free
  via the Rust backend.
- **Risks:** point-to-SDF registration of a sparse 2D fan can be under-constrained in
  long corridors (the classic degenerate direction) — mitigated by the IMU prior, the
  covariance gate PLICP already has, and per-direction information checks before trusting
  the solve. Submap overlap consistency (two submaps disagree after odometry drift) —
  standard answer: registration only ever targets *one* active submap; overlaps are
  reconciled by the pose graph, never by voxel blending at write time.
- **Performance gates (production-readiness):** the governing gate, set at review
  (2026-06-10): **the 3D pipeline must match the planar front-end's performance** —
  accuracy *and* compute — on the archived baseline
  (`eval/reference/baselines/m3-planar-frontend/`: ATE 0.090/0.066 m, 27×/22× real-time,
  p99 3.4/4.3 ms on cafe1). Every migration stage is benchmarked against it (planar runs
  re-anchored on the same machine for compute numbers); a regression must be explicitly
  justified and accepted, never silently absorbed. Additionally: scan-to-submap
  registration ≥ 10× sensor rate CPU-only; submap freeze/serialize never blocks the hot
  path (pipeline rule: drop, never stall); re-localization ≤ 1 s verified; the benchmark
  table grows map-op metrics (integrate/query/freeze) via criterion.
- **Pinned by review (2026-06-10):** reMap integration is **memory-based** (shared
  in-process VDB grids — reMap currently runs 10 cm cells but adjusts easily, so SLAM
  owns the resolution); the **1–2 GB map RAM budget is confirmed**; **< 1 s is the hard
  upper bound** for verified re-localization.
- **Revisit when:** RGB-D lands (does depth fuse into the same 5 cm submaps or a finer
  local grid?).

## Alternatives considered

- **Stay SE(2) + post-hoc tilt correction.** Cheaper now, but it bakes the planar
  assumption ever deeper while the requirements explicitly name it false; and it cannot
  serve 3-floor maps (z is structural, not noise). Rejected.
- **One global VDB grid, no submaps.** Simplest mentally; loop closure then either
  rewrites voxels (minutes of work at building scale) or is deferred to "rebuild the map
  offline" — both unacceptable given loop closure is the top requirement. Rejected.
- **ESDF/occupancy registration instead of TSDF.** ESDF is derived state (more expensive
  to maintain incrementally); TSDF narrow band is the registration-native form and Nav2's
  ESDF stays a derived export (ADR 0004). Rejected as primary.
- **OpenVDB as the only backend.** One backend fewer, but every contributor and every CI
  run takes the system C++ dependency, and all algorithm unit tests run through FFI —
  against the ADR 0003 spirit that made development on any machine possible. Rejected
  in favour of dual (decision pinned at review, 2026-06-10).
- **Vendoring OpenVDB (the GTSAM/ADR 0006 pattern).** Hermetic, but OpenVDB hard-requires
  TBB (a second vendored lib) and the robot/reMap hosts run Ubuntu with `libopenvdb-dev`
  10.x packaged anyway; pinning to the distro package is the lighter contract. Rejected
  at review in favour of the system library.
- **Pure-Rust only (no OpenVDB).** Avoids the C++ build permanently but walls off reMap
  (VDB-native) and discards two decades of sparse-volume engineering; Rust VDB writers
  are immature. Rejected for production; retained as the dev/CI backend.
- **Per-sensor keyframe matching forever (status quo).** Proven at cafe scale (0.066 m),
  but it is two independent 2D maps in disguise — no cross-sensor structure sharing, no
  3D, no persistence story for re-localization. Superseded by design.
