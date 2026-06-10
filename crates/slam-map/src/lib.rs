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
    /// Integration weight cap — bounds memory of stale geometry and lets the map
    /// keep adapting (the precursor of occupancy decay, ADR 0004).
    pub max_weight: f32,
}

impl Default for TsdfConfig {
    fn default() -> Self {
        TsdfConfig {
            voxel_size: 0.05,
            truncation: 0.15,
            max_weight: 64.0,
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
}
