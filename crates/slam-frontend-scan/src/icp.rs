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
    /// Translation observability gate: when the smaller eigenvalue of the 2×2
    /// translation information block falls below this fraction of the larger one, the
    /// solution along that eigenvector is unconstrained by the geometry (the corridor
    /// case: all surviving normals near-parallel) and is reported in
    /// [`MatchResult::degenerate_direction`] instead of being silently trusted.
    pub degeneracy_eigenvalue_ratio: f64,
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
            degeneracy_eigenvalue_ratio: 0.02,
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
    /// Unit direction (reference frame) along which translation was *unobservable*
    /// (see [`MatchConfig::degeneracy_eigenvalue_ratio`]). The transform's component
    /// along it is the solver's guess, not a measurement — callers must replace it
    /// with their motion prior. `None` = fully constrained.
    pub degenerate_direction: Option<Vec2>,
}

/// How many grid cells subdivide the correspondence gate.
///
/// Cells finer than the gate matter on real indoor scans: returns bunch on nearby walls
/// (hundreds of points per gate-sized cell), and the expanding ring search below visits
/// the dense region in near-to-far order with early exit instead of scanning it whole.
const CELLS_PER_GATE: i32 = 4;

/// Uniform-grid 2-nearest-neighbour index over the reference points.
///
/// Dense CSR layout over the reference bounding box, cell = gate / [`CELLS_PER_GATE`].
/// Queries search outward ring by ring and stop as soon as no unseen cell can beat the
/// current second-nearest (a cell at Chebyshev ring `r` holds points ≥ `(r−1)·cell`
/// away). A lidar's extent is sensor-bounded (`range_max`), so the dense grid stays
/// small — lookups are plain array indexing, no hashing.
struct Grid {
    cell: f64,
    gate2: f64,
    min_x: f64,
    min_y: f64,
    nx: i32,
    ny: i32,
    /// CSR: cell `c` holds `items[starts[c] .. starts[c + 1]]`.
    starts: Vec<u32>,
    items: Vec<u32>,
}

impl Grid {
    fn build(points: &[Vec2], gate: f64) -> Grid {
        let cell = gate / CELLS_PER_GATE as f64;
        let gate2 = gate * gate;
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
                gate2,
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
            gate2,
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
        // Covering the gate from anywhere inside the query's cell needs this many rings.
        let max_ring = CELLS_PER_GATE + 1;
        if kx < -max_ring || kx > self.nx + max_ring || ky < -max_ring || ky > self.ny + max_ring {
            return None;
        }

        let (mut best, mut second): (Candidate, Candidate) = (None, None);
        let scan_cell = |cx: i32, cy: i32, best: &mut Candidate, second: &mut Candidate| {
            if cx < 0 || cx >= self.nx || cy < 0 || cy >= self.ny {
                return;
            }
            let c = cx as usize * self.ny as usize + cy as usize;
            let bucket = &self.items[self.starts[c] as usize..self.starts[c + 1] as usize];
            for &i in bucket {
                let d2 = (points[i as usize] - q).norm_squared();
                if d2 > self.gate2 {
                    continue;
                }
                if best.is_none_or(|(bd, _)| d2 < bd) {
                    *second = *best;
                    *best = Some((d2, i));
                } else if second.is_none_or(|(sd, _)| d2 < sd) {
                    *second = Some((d2, i));
                }
            }
        };

        for r in 0..=max_ring {
            if r == 0 {
                scan_cell(kx, ky, &mut best, &mut second);
            } else {
                // The ring's perimeter: top + bottom rows, then the side columns.
                for cx in kx - r..=kx + r {
                    scan_cell(cx, ky - r, &mut best, &mut second);
                    scan_cell(cx, ky + r, &mut best, &mut second);
                }
                for cy in ky - r + 1..=ky + r - 1 {
                    scan_cell(kx - r, cy, &mut best, &mut second);
                    scan_cell(kx + r, cy, &mut best, &mut second);
                }
            }
            // Any point in ring r+1 or beyond is at least r·cell away: once the
            // second-nearest beats that bound, farther rings cannot change the answer.
            if let Some((sd, _)) = second {
                let unseen_min = r as f64 * self.cell;
                if sd <= unseen_min * unseen_min {
                    break;
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
    /// Correspondence scratch space, reused across iterations and calls (hot path:
    /// no allocation after the first match).
    scratch: Vec<Correspondence>,
}

impl ScanMatcher {
    /// Take ownership of the reference points and index them.
    pub fn new(reference: Vec<Vec2>, cfg: MatchConfig) -> Self {
        let grid = Grid::build(&reference, cfg.max_correspondence_distance);
        ScanMatcher {
            cfg,
            reference,
            grid,
            scratch: Vec::new(),
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
    pub fn match_to(&mut self, current: &[Vec2], initial: Se2) -> Option<MatchResult> {
        // The scratch buffer moves in and out by value so the hot loop pushes into a
        // plain local Vec — zero allocation after warm-up, zero indirection.
        let corr = std::mem::take(&mut self.scratch);
        let (mut corr, result) = self.match_inner(current, initial, corr);
        corr.clear();
        self.scratch = corr;
        result
    }

    fn match_inner(
        &self,
        current: &[Vec2],
        initial: Se2,
        mut corr: Vec<Correspondence>,
    ) -> (Vec<Correspondence>, Option<MatchResult>) {
        let (cfg, reference, grid) = (&self.cfg, &self.reference[..], &self.grid);
        if reference.len() < cfg.min_correspondences || current.len() < cfg.min_correspondences {
            return (corr, None);
        }

        let mut transform = initial;
        let mut mean_residual = f64::INFINITY;
        let mut inlier_fraction = 0.0;
        let mut converged = false;
        let mut iterations = 0;
        let mut h_translation = [0.0; 3]; // (h00, h01, h11) of the last iteration

        for iter in 0..cfg.max_iterations {
            iterations = iter + 1;

            // ---- Correspondences at the current estimate ----------------------------
            corr.clear();
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
                return (corr, None);
            }
            corr.select_nth_unstable_by(keep - 1, |a, b| {
                a.residual.abs().total_cmp(&b.residual.abs())
            });
            corr.truncate(keep);

            // ---- Gauss-Newton step on (dx, dy, dθ), left-applied ---------------------
            let mut h = Matrix3::<f64>::zeros();
            let mut g = Vector3::<f64>::zeros();
            let mut abs_sum = 0.0;
            for c in corr.iter() {
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
            h_translation = [h[(0, 0)], h[(0, 1)], h[(1, 1)]];

            let Some(delta) = h.cholesky().map(|ch| ch.solve(&(-g))) else {
                return (corr, None);
            };

            // Left-multiply the increment (it was linearised in the reference frame).
            transform = Se2::new(delta.x, delta.y, delta.z).compose(&transform);

            if delta.x.hypot(delta.y) < cfg.translation_epsilon
                && delta.z.abs() < cfg.rotation_epsilon
            {
                converged = true;
                break;
            }
        }

        (
            corr,
            Some(MatchResult {
                transform,
                iterations,
                mean_residual,
                inlier_fraction,
                converged,
                degenerate_direction: weak_translation_direction(
                    h_translation,
                    cfg.degeneracy_eigenvalue_ratio,
                ),
            }),
        )
    }
}

/// Eigen-analysis of the symmetric 2×2 translation information block `(h00, h01, h11)`:
/// the unit eigenvector of the smaller eigenvalue, when that eigenvalue is below
/// `ratio` × the larger one (translation unobservable along it — e.g. a corridor's
/// axis when every surviving normal faces the walls).
fn weak_translation_direction([h00, h01, h11]: [f64; 3], ratio: f64) -> Option<Vec2> {
    let half_trace = 0.5 * (h00 + h11);
    let d = (0.25 * (h00 - h11).powi(2) + h01 * h01).sqrt();
    let (lo, hi) = (half_trace - d, half_trace + d);
    if lo > ratio * hi {
        return None;
    }
    // Eigenvector for `lo`: rows of (H − lo·I) are orthogonal to it.
    let v = if (h00 - lo).abs() > h01.abs() {
        Vec2::new(-h01, h00 - lo)
    } else if h01.abs() > 1e-300 {
        Vec2::new(lo - h11, h01)
    } else {
        // Diagonal H: the weak axis is whichever diagonal entry is smaller.
        if h00 <= h11 {
            Vec2::new(1.0, 0.0)
        } else {
            Vec2::new(0.0, 1.0)
        }
    };
    let n = v.norm();
    (n > 0.0).then(|| v / n)
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
mod scratch_tests {
    use super::*;

    #[test]
    fn scratch_buffer_is_reused_across_matches() {
        let pts: Vec<Vec2> = (0..100)
            .map(|i| Vec2::new(i as f64 * 0.05, (i % 2) as f64 * 0.01))
            .collect();
        let mut matcher = ScanMatcher::new(pts.clone(), MatchConfig::default());
        matcher.match_to(&pts, Se2::identity()).unwrap();
        let cap = matcher.scratch.capacity();
        assert!(cap >= 80, "scratch should retain capacity: {cap}");
        matcher.match_to(&pts, Se2::identity()).unwrap();
        assert_eq!(matcher.scratch.capacity(), cap, "no reallocation on reuse");
    }
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
