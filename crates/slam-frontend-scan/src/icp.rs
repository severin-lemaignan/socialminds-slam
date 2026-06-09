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
/// Dense CSR layout over the reference bounding box: cell size = the correspondence
/// gate, so the 3×3 neighbourhood of a query's cell provably contains every point within
/// the gate. A lidar's extent is sensor-bounded (`range_max`, tens of metres) so the
/// dense grid is a few thousand cells — lookups are plain array indexing, no hashing.
struct Grid {
    cell: f64,
    min_x: f64,
    min_y: f64,
    nx: i32,
    ny: i32,
    /// CSR: cell `c` holds `items[starts[c] .. starts[c + 1]]`.
    starts: Vec<u32>,
    items: Vec<u32>,
}

impl Grid {
    fn build(points: &[Vec2], cell: f64) -> Grid {
        let (mut min_x, mut min_y) = (f64::INFINITY, f64::INFINITY);
        let (mut max_x, mut max_y) = (f64::NEG_INFINITY, f64::NEG_INFINITY);
        for p in points {
            min_x = min_x.min(p.x);
            min_y = min_y.min(p.y);
            max_x = max_x.max(p.x);
            max_y = max_y.max(p.y);
        }
        if points.is_empty() {
            return Grid {
                cell,
                min_x: 0.0,
                min_y: 0.0,
                nx: 0,
                ny: 0,
                starts: vec![0],
                items: Vec::new(),
            };
        }
        let nx = ((max_x - min_x) / cell).floor() as i64 + 1;
        let ny = ((max_y - min_y) / cell).floor() as i64 + 1;
        let ncells = (nx * ny) as usize;
        assert!(
            ncells <= 1 << 24,
            "scan extent {nx}x{ny} cells is not sensor-bounded data"
        );
        let (nx, ny) = (nx as i32, ny as i32);

        let cell_of = |p: &Vec2| -> usize {
            let cx = ((p.x - min_x) / cell).floor() as i32;
            let cy = ((p.y - min_y) / cell).floor() as i32;
            cx as usize * ny as usize + cy as usize
        };

        // Counting sort into CSR.
        let mut starts = vec![0u32; ncells + 1];
        for p in points {
            starts[cell_of(p) + 1] += 1;
        }
        for c in 0..ncells {
            starts[c + 1] += starts[c];
        }
        let mut cursor = starts.clone();
        let mut items = vec![0u32; points.len()];
        for (i, p) in points.iter().enumerate() {
            let c = cell_of(p);
            items[cursor[c] as usize] = i as u32;
            cursor[c] += 1;
        }

        Grid {
            cell,
            min_x,
            min_y,
            nx,
            ny,
            starts,
            items,
        }
    }

    /// Indices of the two nearest reference points within the gate, nearest first.
    fn two_nearest(&self, points: &[Vec2], q: Vec2) -> Option<(u32, Option<u32>, f64)> {
        // (squared distance, point index) of a nearest-so-far candidate.
        type Candidate = Option<(f64, u32)>;
        let kx = ((q.x - self.min_x) / self.cell).floor() as i32;
        let ky = ((q.y - self.min_y) / self.cell).floor() as i32;
        // More than one cell outside the bounding box → nothing can be within the gate.
        if kx < -1 || kx > self.nx || ky < -1 || ky > self.ny {
            return None;
        }
        let gate2 = self.cell * self.cell;
        let (mut best, mut second): (Candidate, Candidate) = (None, None);
        for cx in (kx - 1).max(0)..=(kx + 1).min(self.nx - 1) {
            for cy in (ky - 1).max(0)..=(ky + 1).min(self.ny - 1) {
                let c = cx as usize * self.ny as usize + cy as usize;
                let bucket = &self.items[self.starts[c] as usize..self.starts[c + 1] as usize];
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

/// A reference scan, indexed once and matched against many times.
///
/// This is the scan-to-keyframe shape (ADR 0007): one keyframe serves dozens of incoming
/// scans, so the neighbour index is built when the keyframe is adopted — not per match.
pub struct ScanMatcher {
    cfg: MatchConfig,
    reference: Vec<Vec2>,
    grid: Grid,
}

impl ScanMatcher {
    /// Take ownership of the reference points and index them.
    pub fn new(reference: Vec<Vec2>, cfg: MatchConfig) -> Self {
        let grid = Grid::build(&reference, cfg.max_correspondence_distance);
        ScanMatcher {
            cfg,
            reference,
            grid,
        }
    }

    pub fn reference(&self) -> &[Vec2] {
        &self.reference
    }

    /// Align `current` onto the reference starting from `initial`.
    ///
    /// Returns `None` when there are never enough gated correspondences to solve (scan
    /// content mismatch, bad initial guess beyond the gate, or degenerate geometry where
    /// the normal equations lose rank).
    pub fn match_to(&self, current: &[Vec2], initial: Se2) -> Option<MatchResult> {
        let (cfg, reference, grid) = (&self.cfg, &self.reference[..], &self.grid);
        if reference.len() < cfg.min_correspondences || current.len() < cfg.min_correspondences {
            return None;
        }

        let mut transform = initial;
        let mut mean_residual = f64::INFINITY;
        let mut inlier_fraction = 0.0;
        let mut converged = false;
        let mut iterations = 0;

        for iter in 0..cfg.max_iterations {
            iterations = iter + 1;

            // ---- Correspondences at the current estimate ----------------------------
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

            // ---- Trim the worst residuals --------------------------------------------
            let keep = ((corr.len() as f64) * (1.0 - cfg.trim_ratio)).floor() as usize;
            if keep < cfg.min_correspondences {
                return None;
            }
            corr.select_nth_unstable_by(keep - 1, |a, b| {
                a.residual.abs().total_cmp(&b.residual.abs())
            });
            corr.truncate(keep);

            // ---- Gauss-Newton step on (dx, dy, dθ), left-applied ---------------------
            let mut h = Matrix3::<f64>::zeros();
            let mut g = Vector3::<f64>::zeros();
            let mut abs_sum = 0.0;
            for c in &corr {
                let q = transform.apply(c.point);
                // d(R(θ)p + t)/dθ at the current estimate = (−q_y, q_x) about the origin.
                let jac =
                    Vector3::new(c.normal.x, c.normal.y, c.normal.x * -q.y + c.normal.y * q.x);
                h += jac * jac.transpose();
                g += jac * c.residual;
                abs_sum += c.residual.abs();
            }
            mean_residual = abs_sum / keep as f64;
            inlier_fraction = keep as f64 / current.len() as f64;

            let delta = h.cholesky().map(|ch| ch.solve(&(-g)))?;

            // Left-multiply the increment (it was linearised in the reference frame).
            transform = Se2::new(delta.x, delta.y, delta.z).compose(&transform);

            if delta.x.hypot(delta.y) < cfg.translation_epsilon
                && delta.z.abs() < cfg.rotation_epsilon
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
}

/// One-shot convenience over [`ScanMatcher`]: index `reference`, match once.
///
/// Pays the indexing cost every call — odometry-style repeated matching against the same
/// reference should hold a [`ScanMatcher`] instead.
pub fn match_scans(
    reference: &[Vec2],
    current: &[Vec2],
    initial: Se2,
    cfg: &MatchConfig,
) -> Option<MatchResult> {
    ScanMatcher::new(reference.to_vec(), cfg.clone()).match_to(current, initial)
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
