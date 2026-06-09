//! Trimmed point-to-line ICP (PLICP) for planar scans (ADR 0007).
//!
//! Point-to-line is the right metric for indoor lidar: consecutive returns on a wall
//! define the wall locally, and the residual is the distance *to that line*, not to the
//! nearest sampled point — quadratic convergence on straight geometry (Censi 2008).
//! Robustness comes from a correspondence distance gate plus *trimming*: the worst
//! residuals are dropped every iteration, so dynamic objects (people) shed as outliers
//! without any semantic masking at this layer.

use nalgebra::{Matrix3, Vector3};
use slam_types::Vec2;

use crate::se2::Se2;

/// Tuning for one [`match_scans`] call.
#[derive(Debug, Clone)]
pub struct MatchConfig {
    /// Gauss-Newton / correspondence refresh iterations.
    pub max_iterations: usize,
    /// Correspondences farther than this (m) are discarded outright.
    pub max_correspondence_distance: f64,
    /// Fraction of the *gated* correspondences dropped as worst-residual outliers.
    pub trim_ratio: f64,
    /// Converged when one step moves less than this (m / rad).
    pub translation_epsilon: f64,
    pub rotation_epsilon: f64,
    /// Give up when fewer surviving correspondences than this.
    pub min_correspondences: usize,
}

impl Default for MatchConfig {
    fn default() -> Self {
        MatchConfig {
            max_iterations: 30,
            max_correspondence_distance: 1.0,
            trim_ratio: 0.2,
            translation_epsilon: 1e-6,
            rotation_epsilon: 1e-7,
            min_correspondences: 20,
        }
    }
}

/// Outcome of a scan match. `transform` maps current-scan coordinates into the
/// reference-scan frame — i.e. it *is* the current sensor pose in the reference frame.
#[derive(Debug, Clone, Copy)]
pub struct MatchResult {
    pub transform: Se2,
    pub iterations: usize,
    /// Mean |point-to-line| residual over the kept correspondences (m).
    pub mean_residual: f64,
    /// Kept correspondences / current-scan points. The health signal: low values mean
    /// occlusion, dynamics, or a bad match.
    pub inlier_fraction: f64,
    /// False when the iteration budget ran out before the step shrank below epsilon.
    pub converged: bool,
}

/// Uniform-grid 2-nearest-neighbour index over the reference points.
///
/// Cell size = the correspondence gate, so the 3×3 neighbourhood of a query's cell
/// provably contains every point within the gate. O(1) build per point, O(1) query on
/// indoor scan densities — no tree, no heap allocations per query.
struct Grid {
    cell: f64,
    map: std::collections::HashMap<(i32, i32), Vec<u32>>,
}

impl Grid {
    fn build(points: &[Vec2], cell: f64) -> Grid {
        let mut map: std::collections::HashMap<(i32, i32), Vec<u32>> =
            std::collections::HashMap::with_capacity(points.len());
        for (i, p) in points.iter().enumerate() {
            map.entry(Self::key(p, cell)).or_default().push(i as u32);
        }
        Grid { cell, map }
    }

    #[inline]
    fn key(p: &Vec2, cell: f64) -> (i32, i32) {
        ((p.x / cell).floor() as i32, (p.y / cell).floor() as i32)
    }

    /// Indices of the two nearest reference points within the gate, nearest first.
    fn two_nearest(&self, points: &[Vec2], q: Vec2) -> Option<(u32, Option<u32>, f64)> {
        // (squared distance, point index) of a nearest-so-far candidate.
        type Candidate = Option<(f64, u32)>;
        let (kx, ky) = Self::key(&q, self.cell);
        let gate2 = self.cell * self.cell;
        let (mut best, mut second): (Candidate, Candidate) = (None, None);
        for dx in -1..=1 {
            for dy in -1..=1 {
                let Some(bucket) = self.map.get(&(kx + dx, ky + dy)) else {
                    continue;
                };
                for &i in bucket {
                    let d2 = (points[i as usize] - q).norm_squared();
                    if d2 > gate2 {
                        continue;
                    }
                    if best.is_none_or(|(bd, _)| d2 < bd) {
                        second = best;
                        best = Some((d2, i));
                    } else if second.is_none_or(|(sd, _)| d2 < sd) {
                        second = Some((d2, i));
                    }
                }
            }
        }
        best.map(|(d2, i)| (i, second.map(|(_, j)| j), d2))
    }
}

/// One correspondence: current point index, line normal, signed residual.
struct Correspondence {
    point: Vec2,
    normal: Vec2,
    residual: f64,
}

/// Align `current` onto `reference` starting from `initial`.
///
/// Returns `None` when there are never enough gated correspondences to solve (scan
/// content mismatch, bad initial guess beyond the gate, or degenerate geometry where the
/// normal equations lose rank).
pub fn match_scans(
    reference: &[Vec2],
    current: &[Vec2],
    initial: Se2,
    cfg: &MatchConfig,
) -> Option<MatchResult> {
    if reference.len() < cfg.min_correspondences || current.len() < cfg.min_correspondences {
        return None;
    }
    let grid = Grid::build(reference, cfg.max_correspondence_distance);

    let mut transform = initial;
    let mut mean_residual = f64::INFINITY;
    let mut inlier_fraction = 0.0;
    let mut converged = false;
    let mut iterations = 0;

    for iter in 0..cfg.max_iterations {
        iterations = iter + 1;

        // ---- Correspondences at the current estimate --------------------------------
        let mut corr: Vec<Correspondence> = Vec::with_capacity(current.len());
        for &p in current {
            let q = transform.apply(p);
            let Some((i, j, _)) = grid.two_nearest(reference, q) else {
                continue;
            };
            let a = reference[i as usize];
            // Local line through the two nearest reference points; fall back to
            // point-to-point when they coincide (isolated return).
            let normal = match j {
                Some(j) => {
                    let tangent = reference[j as usize] - a;
                    let n = Vec2::new(-tangent.y, tangent.x);
                    let len = n.norm();
                    if len < 1e-9 {
                        let d = q - a;
                        let dn = d.norm();
                        if dn < 1e-12 {
                            continue;
                        }
                        d / dn
                    } else {
                        n / len
                    }
                }
                None => {
                    let d = q - a;
                    let dn = d.norm();
                    if dn < 1e-12 {
                        continue;
                    }
                    d / dn
                }
            };
            corr.push(Correspondence {
                point: p,
                normal,
                residual: normal.dot(&(q - a)),
            });
        }

        // ---- Trim the worst residuals ------------------------------------------------
        let keep = ((corr.len() as f64) * (1.0 - cfg.trim_ratio)).floor() as usize;
        if keep < cfg.min_correspondences {
            return None;
        }
        corr.select_nth_unstable_by(keep - 1, |a, b| {
            a.residual.abs().total_cmp(&b.residual.abs())
        });
        corr.truncate(keep);

        // ---- Gauss-Newton step on (dx, dy, dθ), left-applied -------------------------
        let mut h = Matrix3::<f64>::zeros();
        let mut g = Vector3::<f64>::zeros();
        let mut abs_sum = 0.0;
        for c in &corr {
            let q = transform.apply(c.point);
            // d(R(θ)p + t)/dθ at the current estimate = (−q_y, q_x) about the origin.
            let jac = Vector3::new(c.normal.x, c.normal.y, c.normal.x * -q.y + c.normal.y * q.x);
            h += jac * jac.transpose();
            g += jac * c.residual;
            abs_sum += c.residual.abs();
        }
        mean_residual = abs_sum / keep as f64;
        inlier_fraction = keep as f64 / current.len() as f64;

        let delta = h.cholesky().map(|ch| ch.solve(&(-g)))?;

        // Left-multiply the increment (it was linearised in the reference frame).
        transform = Se2::new(delta.x, delta.y, delta.z).compose(&transform);

        if delta.x.hypot(delta.y) < cfg.translation_epsilon && delta.z.abs() < cfg.rotation_epsilon
        {
            converged = true;
            break;
        }
    }

    Some(MatchResult {
        transform,
        iterations,
        mean_residual,
        inlier_fraction,
        converged,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Points sampled on the walls of a 10×6 rectangle centred at the origin.
    fn rectangle_points(step: f64) -> Vec<Vec2> {
        let (hw, hh) = (5.0, 3.0);
        let mut pts = Vec::new();
        let mut x = -hw;
        while x <= hw {
            pts.push(Vec2::new(x, -hh));
            pts.push(Vec2::new(x, hh));
            x += step;
        }
        let mut y = -hh + step;
        while y < hh {
            pts.push(Vec2::new(-hw, y));
            pts.push(Vec2::new(hw, y));
            y += step;
        }
        pts
    }

    fn transform_all(points: &[Vec2], t: &Se2) -> Vec<Vec2> {
        points.iter().map(|p| t.apply(*p)).collect()
    }

    #[test]
    fn recovers_a_known_rigid_transform() {
        let reference = rectangle_points(0.05);
        // The "sensor" moved by (0.15, -0.08, 0.04): points seen from the new pose are
        // the world points pulled back through its inverse.
        let motion = Se2::new(0.15, -0.08, 0.04);
        let current = transform_all(&reference, &motion.inverse());

        let result = match_scans(
            &reference,
            &current,
            Se2::identity(),
            &MatchConfig::default(),
        )
        .expect("match must succeed");
        assert!(result.converged);
        let err = motion.inverse().compose(&result.transform);
        assert!(
            err.translation_norm() < 1e-3 && err.theta.abs() < 1e-4,
            "residual motion {err:?}"
        );
        assert!(result.mean_residual < 1e-3);
    }

    #[test]
    fn trimming_survives_a_dynamic_blob() {
        let reference = rectangle_points(0.05);
        let motion = Se2::new(0.1, 0.05, -0.03);
        let mut current = transform_all(&reference, &motion.inverse());
        // A "person": a dense cluster that exists only in the current scan.
        for i in 0..30 {
            current.push(Vec2::new(1.0 + 0.01 * i as f64, 0.5));
        }

        let result = match_scans(
            &reference,
            &current,
            Se2::identity(),
            &MatchConfig::default(),
        )
        .expect("match must succeed");
        let err = motion.inverse().compose(&result.transform);
        assert!(
            err.translation_norm() < 5e-3 && err.theta.abs() < 5e-4,
            "residual motion {err:?}"
        );
    }

    #[test]
    fn fails_cleanly_on_too_few_points() {
        let reference = rectangle_points(0.05);
        let current = vec![Vec2::new(0.0, 0.0); 5];
        assert!(match_scans(
            &reference,
            &current,
            Se2::identity(),
            &MatchConfig::default()
        )
        .is_none());
    }
}
