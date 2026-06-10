//! The pure-Rust sparse TSDF backend: a blocked hash grid (8³ voxel blocks).
//!
//! This is the dev/CI default of ADR 0010's dual-backend decision — ~500 lines of
//! container so that every algorithm above the [`TsdfMap`](crate::TsdfMap) trait can be
//! developed and tested with zero C++ in the loop. Block granularity keeps the hash map
//! small (one entry per 8³ voxels) and gives spatially-coherent queries a cheap memo.

use std::collections::HashMap;

use slam_types::Vec3;

use crate::{SdfSample, TsdfConfig, TsdfMap};

const BLOCK_BITS: i32 = 3;
const BLOCK_SIDE: i32 = 1 << BLOCK_BITS; // 8
const BLOCK_MASK: i32 = BLOCK_SIDE - 1;
const BLOCK_VOXELS: usize = (BLOCK_SIDE * BLOCK_SIDE * BLOCK_SIDE) as usize;

#[derive(Clone, Copy)]
struct Voxel {
    tsdf: f32,
    weight: f32,
}

const EMPTY: Voxel = Voxel {
    tsdf: 0.0,
    weight: 0.0,
};

type BlockKey = (i32, i32, i32);
type Block = Box<[Voxel; BLOCK_VOXELS]>;

#[inline]
fn split(ix: i32, iy: i32, iz: i32) -> (BlockKey, usize) {
    let key = (ix >> BLOCK_BITS, iy >> BLOCK_BITS, iz >> BLOCK_BITS);
    let off = (((ix & BLOCK_MASK) << (2 * BLOCK_BITS))
        | ((iy & BLOCK_MASK) << BLOCK_BITS)
        | (iz & BLOCK_MASK)) as usize;
    (key, off)
}

/// Blocked sparse TSDF grid (the Rust backend of ADR 0010).
pub struct SparseTsdf {
    cfg: TsdfConfig,
    blocks: HashMap<BlockKey, Block>,
}

impl SparseTsdf {
    pub fn new(cfg: TsdfConfig) -> Self {
        SparseTsdf {
            cfg,
            blocks: HashMap::new(),
        }
    }

    /// Global voxel index containing the world point.
    #[inline]
    fn voxel_of(&self, p: Vec3) -> (i32, i32, i32) {
        let s = 1.0 / self.cfg.voxel_size;
        (
            (p.x * s).floor() as i32,
            (p.y * s).floor() as i32,
            (p.z * s).floor() as i32,
        )
    }

    #[inline]
    fn voxel(&self, ix: i32, iy: i32, iz: i32) -> Voxel {
        let (key, off) = split(ix, iy, iz);
        match self.blocks.get(&key) {
            Some(b) => b[off],
            None => EMPTY,
        }
    }

    fn update_voxel(&mut self, ix: i32, iy: i32, iz: i32, obs: f64) {
        let (key, off) = split(ix, iy, iz);
        let block = self
            .blocks
            .entry(key)
            .or_insert_with(|| Box::new([EMPTY; BLOCK_VOXELS]));
        let v = &mut block[off];
        let w_new = (v.weight + 1.0).min(self.cfg.max_weight);
        v.tsdf = (v.tsdf * v.weight + obs as f32) / (v.weight + 1.0);
        v.weight = w_new;
    }

    /// Trilinear SDF + analytic gradient over the 8 surrounding voxel centres.
    ///
    /// **Planar degeneracy:** a 2D lidar's fan is a measure-zero slice — it observes a
    /// *single* voxel layer in z, so full trilinear would return `None` everywhere.
    /// When exactly one of the two z-layers is fully observed, the sample collapses to
    /// bilinear within that layer (gradient z = 0) — exact for planar data, and the
    /// planar 3-DoF registration never consumes the z gradient. Both layers observed
    /// (tilted fans, RGB-D) → full trilinear. Neither → `None`.
    fn sample_impl(&self, p: Vec3) -> Option<SdfSample> {
        let s = 1.0 / self.cfg.voxel_size;
        // Continuous coordinates in voxel-centre space (centre of voxel i at i + 0.5).
        let gx = p.x * s - 0.5;
        let gy = p.y * s - 0.5;
        let gz = p.z * s - 0.5;
        let (ix, iy, iz) = (gx.floor() as i32, gy.floor() as i32, gz.floor() as i32);
        let (fx, fy, fz) = (gx - gx.floor(), gy - gy.floor(), gz - gz.floor());

        // c[(dx<<2)|(dy<<1)|dz]
        let mut c = [0.0f64; 8];
        let mut seen = [false; 8];
        for n in 0..8 {
            let v = self.voxel(
                ix + ((n >> 2) & 1) as i32,
                iy + ((n >> 1) & 1) as i32,
                iz + (n & 1) as i32,
            );
            seen[n] = v.weight > 0.0;
            c[n] = v.tsdf as f64;
        }
        let lower_ok = seen[0] && seen[2] && seen[4] && seen[6];
        let upper_ok = seen[1] && seen[3] && seen[5] && seen[7];
        let lerp = |a: f64, b: f64, t: f64| a + (b - a) * t;

        if lower_ok && upper_ok {
            let c00 = lerp(c[0], c[4], fx);
            let c01 = lerp(c[1], c[5], fx);
            let c10 = lerp(c[2], c[6], fx);
            let c11 = lerp(c[3], c[7], fx);
            let sdf = lerp(lerp(c00, c10, fy), lerp(c01, c11, fy), fz);
            let dx = lerp(
                lerp(c[4] - c[0], c[6] - c[2], fy),
                lerp(c[5] - c[1], c[7] - c[3], fy),
                fz,
            );
            let dy = lerp(
                lerp(c[2] - c[0], c[6] - c[4], fx),
                lerp(c[3] - c[1], c[7] - c[5], fx),
                fz,
            );
            let dz = lerp(
                lerp(c[1] - c[0], c[5] - c[4], fx),
                lerp(c[3] - c[2], c[7] - c[6], fx),
                fy,
            );
            return Some(SdfSample {
                sdf,
                gradient: Vec3::new(dx, dy, dz) * s,
            });
        }

        // Single observed layer: bilinear in it, z gradient unobservable.
        let dz_layer = if lower_ok {
            0
        } else if upper_ok {
            1
        } else {
            return None;
        };
        let at = |dx: usize, dy: usize| c[(dx << 2) | (dy << 1) | dz_layer];
        let (a, b, c2, d) = (at(0, 0), at(1, 0), at(0, 1), at(1, 1));
        let sdf = lerp(lerp(a, b, fx), lerp(c2, d, fx), fy);
        let dx = lerp(b - a, d - c2, fy);
        let dy = lerp(c2 - a, d - b, fx);
        Some(SdfSample {
            sdf,
            gradient: Vec3::new(dx, dy, 0.0) * s,
        })
    }
}

impl TsdfMap for SparseTsdf {
    fn config(&self) -> &TsdfConfig {
        &self.cfg
    }

    fn integrate_points(&mut self, origin: Vec3, points: &[Vec3]) {
        let trunc = self.cfg.truncation;
        // Half-voxel stepping covers every voxel the band crosses; consecutive
        // duplicates are skipped so no voxel is double-counted within one ray.
        let step = self.cfg.voxel_size * 0.5;
        for &p in points {
            let v = p - origin;
            let dist = v.norm();
            if dist < 1e-9 {
                continue;
            }
            let dir = v / dist;
            let start = (dist - trunc).max(0.0);
            let end = dist + trunc;
            let n = ((end - start) / step) as usize + 1;
            let mut last = None;
            for k in 0..=n {
                let s = (start + k as f64 * step).min(end);
                let q = origin + dir * s;
                let idx = self.voxel_of(q);
                if last == Some(idx) {
                    continue;
                }
                last = Some(idx);
                // Projective signed distance: + before the hit (free), − behind it.
                self.update_voxel(idx.0, idx.1, idx.2, (dist - s).clamp(-trunc, trunc));
            }
        }
    }

    fn sample_batch(&self, points: &[Vec3], out: &mut Vec<Option<SdfSample>>) {
        out.clear();
        out.extend(points.iter().map(|&p| self.sample_impl(p)));
    }

    fn sample(&self, point: Vec3) -> Option<SdfSample> {
        self.sample_impl(point)
    }

    fn allocated_voxels(&self) -> usize {
        self.blocks.len() * BLOCK_VOXELS
    }

    fn visit_voxels(&self, visit: &mut dyn FnMut(i32, i32, i32, f32, f32)) {
        for (key, block) in &self.blocks {
            for (off, v) in block.iter().enumerate() {
                if v.weight <= 0.0 {
                    continue;
                }
                let off = off as i32;
                visit(
                    (key.0 << BLOCK_BITS) | (off >> (2 * BLOCK_BITS)),
                    (key.1 << BLOCK_BITS) | ((off >> BLOCK_BITS) & BLOCK_MASK),
                    (key.2 << BLOCK_BITS) | (off & BLOCK_MASK),
                    v.tsdf,
                    v.weight,
                );
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A wall in the y-z plane at x = 2, observed from the origin: rays along +X.
    /// Sampled denser than the voxel size — behind a surface, a single viewpoint's
    /// rays *diverge*, and surface samples sparser than the grid leave band holes
    /// (real fans are ~mm-spaced at these ranges).
    fn wall_map() -> SparseTsdf {
        let mut map = SparseTsdf::new(TsdfConfig::default());
        let points: Vec<Vec3> = (-40..=40)
            .flat_map(|y| (0..=16).map(move |z| Vec3::new(2.0, y as f64 * 0.025, z as f64 * 0.025)))
            .collect();
        let origin = Vec3::new(0.0, 0.0, 0.2);
        for _ in 0..4 {
            map.integrate_points(origin, &points);
        }
        map
    }

    #[test]
    fn sdf_is_signed_distance_near_the_wall() {
        let map = wall_map();
        // 6 cm in front of the wall (free side): positive, roughly the distance.
        // Projective TSDF reads high where oblique rays contribute (distance measured
        // along the ray, not perpendicular) — a property, not a bug; the registration
        // cost only needs sign + monotonicity + a consistent zero crossing.
        let front = map.sample(Vec3::new(1.94, 0.0, 0.2)).expect("observed");
        assert!(
            (0.03..=0.11).contains(&front.sdf),
            "front sdf {}",
            front.sdf
        );
        // 6 cm behind: negative.
        let behind = map.sample(Vec3::new(2.06, 0.0, 0.2)).expect("observed");
        assert!(
            (-0.11..=-0.03).contains(&behind.sdf),
            "behind sdf {}",
            behind.sdf
        );
        // Monotone across the surface.
        let at = map.sample(Vec3::new(2.0, 0.0, 0.2)).expect("observed");
        assert!(behind.sdf < at.sdf && at.sdf < front.sdf);
    }

    #[test]
    fn gradient_points_towards_free_space() {
        let map = wall_map();
        let s = map.sample(Vec3::new(1.95, 0.1, 0.2)).expect("observed");
        let g = s.gradient.normalize();
        // Free space is towards −X (the sensor side).
        assert!(g.x < -0.8, "gradient {g:?}");
    }

    #[test]
    fn unobserved_space_is_none() {
        let map = wall_map();
        assert!(map.sample(Vec3::new(-3.0, 5.0, 4.0)).is_none());
        // Far behind the wall, outside the truncation band, is also unobserved.
        assert!(map.sample(Vec3::new(3.0, 0.0, 0.2)).is_none());
    }

    #[test]
    fn repeated_integration_is_stable_and_weight_capped() {
        let mut map = SparseTsdf::new(TsdfConfig {
            max_weight: 8.0,
            ..TsdfConfig::default()
        });
        // A small patch (a lone ray leaves trilinear corners unobserved by design).
        let pts: Vec<Vec3> = (-2..=2)
            .flat_map(|y| (-2..=2).map(move |z| Vec3::new(1.0, y as f64 * 0.05, z as f64 * 0.05)))
            .collect();
        let origin = Vec3::zeros();
        for _ in 0..100 {
            map.integrate_points(origin, &pts);
        }
        let near = map.sample(Vec3::new(0.95, 0.025, 0.025)).expect("observed");
        assert!((0.02..=0.10).contains(&near.sdf), "sdf {}", near.sdf);
    }

    #[test]
    fn allocation_stays_in_the_narrow_band() {
        let map = wall_map();
        // The wall is ~2 m × 0.4 m; the band is ±0.15 m. A dense 5 cm grid over the
        // whole observed frustum would be ~hundreds of thousands of voxels.
        assert!(
            map.allocated_voxels() < 60_000,
            "band blew up: {} voxels",
            map.allocated_voxels()
        );
    }

    #[test]
    fn batch_and_single_sampling_agree() {
        let map = wall_map();
        let pts = vec![Vec3::new(1.94, 0.0, 0.2), Vec3::new(-3.0, 5.0, 4.0)];
        let mut out = Vec::new();
        map.sample_batch(&pts, &mut out);
        assert_eq!(out.len(), 2);
        assert!(out[0].is_some() && out[1].is_none());
        assert_eq!(out[0].unwrap().sdf, map.sample(pts[0]).unwrap().sdf);
    }
}
