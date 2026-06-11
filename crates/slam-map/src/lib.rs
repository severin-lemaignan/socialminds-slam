//! 3D map substrate (ADR 0010): a narrow-band **TSDF** behind a batch-level trait.
//!
//! Registration and fusion code sees only [`TsdfMap`]; the storage backend is
//! swappable. This crate ships the **pure-Rust sparse backend** ([`SparseTsdf`], a
//! blocked hash grid) — the default for dev/CI. The OpenVDB backend (production,
//! in-process reMap grid sharing) implements the same trait behind a feature flag and
//! must pass the same conformance suite (tolerance-based, not bit-exact).
//!
//! The trait API is deliberately **batch-level** (integrate a sweep, sample a batch):
//! per-voxel virtual calls would forbid backend-internal optimisations (accessor
//! caching, block memoisation) and make FFI backends unusable.

#![forbid(unsafe_code)]

mod sparse;

pub use sparse::SparseTsdf;

use slam_types::Vec3;

/// TSDF tuning (ADR 0010 defaults: 5 cm voxels, ±3-voxel narrow band).
#[derive(Debug, Clone)]
pub struct TsdfConfig {
    /// Voxel edge length (m).
    pub voxel_size: f64,
    /// Truncation distance (m): the half-width of the band updated around a surface
    /// hit. Values are clamped to ±truncation.
    pub truncation: f64,
    /// Integration weight cap — averaging inertia for registration stability.
    /// Eviction of stale geometry is carving's job, not the cap's (ADR 0014).
    pub max_weight: f32,
    /// Free-space carving (ADR 0014): each ray's free segment multiplies the weight
    /// of every allocated voxel it crosses by this factor (contradiction-driven
    /// eviction — a beam passing through a voxel proves it empty); below weight 1
    /// the voxel reverts to unobserved. `1.0` disables carving.
    pub carve_factor: f32,
    /// Proportional carve margin: a voxel only counts as contradicted when the
    /// beam overshoots it by more than this fraction of its range (floored at
    /// 2·truncation). Grazing beams of the *same oblique surface* overshoot by a
    /// few % of range — without the margin, floors and oblique walls erode between
    /// revisits (measured: 55 % of cafe1-1+depth surface voxels) — while a
    /// transient's voxels are overshot by metres and still carve. `0.0` keeps the
    /// aggressive eviction the *registration* fields measurably need in crowds
    /// (busy-gate ATE 0.31 vs 1.03 with the margin).
    pub carve_relative_margin: f64,
}

impl Default for TsdfConfig {
    fn default() -> Self {
        TsdfConfig {
            voxel_size: 0.05,
            truncation: 0.15,
            max_weight: 64.0,
            carve_factor: 0.5,
            carve_relative_margin: 0.1,
        }
    }
}

/// One SDF observation at a queried point.
#[derive(Debug, Clone, Copy)]
pub struct SdfSample {
    /// Interpolated truncated signed distance (m): positive on the observed (free)
    /// side of the surface, negative behind it.
    pub sdf: f64,
    /// SDF spatial gradient (≈ unit surface normal towards free space, where the
    /// field is clean).
    pub gradient: Vec3,
}

/// A truncated signed distance field that can fuse sensor sweeps and answer
/// interpolated SDF queries — everything scan-to-map registration needs.
pub trait TsdfMap {
    fn config(&self) -> &TsdfConfig;

    /// Fuse one sensor sweep: each `point` is a surface hit seen from `origin`
    /// (world frame). Voxels in the truncation band along each ray are updated with
    /// the projective signed distance.
    fn integrate_points(&mut self, origin: Vec3, points: &[Vec3]);

    /// Like [`integrate_points`](Self::integrate_points), but each point carries an
    /// sRGB colour (`colors[i]` ↔ `points[i]`) that is running-averaged into the
    /// voxels at the surface hit — a visualization/semantic channel that **never**
    /// affects the SDF or registration. Mismatched lengths fall back to the
    /// colourless path. The default impl ignores colour; backends with colour
    /// storage (the sparse grid) override it.
    fn integrate_points_colored(&mut self, origin: Vec3, points: &[Vec3], _colors: &[[u8; 3]]) {
        self.integrate_points(origin, points);
    }

    /// Sample SDF + gradient at each point; `None` where the field is unobserved.
    /// `out` is cleared and refilled (reusable allocation — hot path).
    fn sample_batch(&self, points: &[Vec3], out: &mut Vec<Option<SdfSample>>);

    /// One-off variant of [`sample_batch`](Self::sample_batch).
    fn sample(&self, point: Vec3) -> Option<SdfSample>;

    /// Number of allocated voxels (capacity/diagnostics, the memory-budget signal).
    fn allocated_voxels(&self) -> usize;

    /// Visit every observed voxel as `(ix, iy, iz, tsdf, weight)` (global voxel
    /// indices; centre of voxel i at `(i + 0.5) · voxel_size`). Export/viz path —
    /// not for the registration hot loop.
    fn visit_voxels(&self, visit: &mut dyn FnMut(i32, i32, i32, f32, f32));

    /// Colour-aware [`visit_voxels`](Self::visit_voxels): the trailing argument is the
    /// voxel's running-averaged sRGB surface colour, or `None` for voxels never
    /// touched by a coloured integration (scan-only geometry). The default impl
    /// reports every voxel as colourless; the sparse grid overrides it.
    fn visit_voxels_colored(
        &self,
        visit: &mut dyn FnMut(i32, i32, i32, f32, f32, Option<[u8; 3]>),
    ) {
        self.visit_voxels(&mut |ix, iy, iz, tsdf, w| visit(ix, iy, iz, tsdf, w, None));
    }
}
