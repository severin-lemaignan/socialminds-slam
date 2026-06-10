//! GTSAM-backed implementation of the front-end's [`AnchorGraph`] seam (ADR 0010
//! stage 3b): submap anchors as SE(3) nodes (planar, z = 0), odometry between-edges,
//! verified loop edges; Levenberg–Marquardt distributes loop corrections across the
//! whole anchor chain instead of snapping the latest pose.

use slam_backend::{FactorGraph, Key, PoseNoise};
use slam_frontend_scan::{AnchorGraph, Se2};

/// Noise scales: odometry drifts (loose), verified loops are trusted (tight), the
/// first anchor pins the gauge.
pub struct GtsamAnchorGraph {
    odometry_noise: PoseNoise,
    loop_noise: PoseNoise,
    prior_noise: PoseNoise,
}

impl Default for GtsamAnchorGraph {
    fn default() -> Self {
        GtsamAnchorGraph {
            odometry_noise: PoseNoise::isotropic(0.05, 0.20),
            loop_noise: PoseNoise::isotropic(0.01, 0.05),
            prior_noise: PoseNoise::isotropic(1e-4, 1e-4),
        }
    }
}

impl AnchorGraph for GtsamAnchorGraph {
    fn optimize(
        &mut self,
        anchors: &[Se2],
        odometry: &[Se2],
        loops: &[(usize, usize, Se2)],
    ) -> Option<Vec<Se2>> {
        if anchors.len() < 2 || loops.is_empty() {
            return None;
        }
        let mut graph = FactorGraph::new();
        for (i, a) in anchors.iter().enumerate() {
            graph.insert_pose(Key::pose(i as u64), &a.to_pose());
        }
        graph.add_pose_prior(Key::pose(0), &anchors[0].to_pose(), &self.prior_noise);
        for (i, rel) in odometry.iter().enumerate().take(anchors.len() - 1) {
            graph.add_between(
                Key::pose(i as u64),
                Key::pose(i as u64 + 1),
                &rel.to_pose(),
                &self.odometry_noise,
            );
        }
        for &(from, to, rel) in loops {
            if from >= anchors.len() || to >= anchors.len() || from == to {
                continue;
            }
            graph.add_between(
                Key::pose(from as u64),
                Key::pose(to as u64),
                &rel.to_pose(),
                &self.loop_noise,
            );
        }
        graph.optimize(50).ok()?;
        let mut out = Vec::with_capacity(anchors.len());
        for i in 0..anchors.len() {
            let pose = graph.pose(Key::pose(i as u64)).ok()?;
            out.push(Se2::planar_projection_of(&pose).0);
        }
        Some(out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn distributes_a_loop_correction_over_the_chain() {
        // Three anchors 10 m apart with +0.2 m of y-drift per hop; the loop edge says
        // anchor 2 truly sits at (20, 0) relative to anchor 0 (rel measured through
        // matching: target-relative ∘ active-relative⁻¹ = identity offsets here).
        let anchors = vec![
            Se2::new(0.0, 0.0, 0.0),
            Se2::new(10.0, 0.2, 0.0),
            Se2::new(20.0, 0.4, 0.0),
        ];
        let odometry = vec![Se2::new(10.0, 0.2, 0.0), Se2::new(10.0, 0.2, 0.0)];
        let loops = vec![(0usize, 2usize, Se2::new(20.0, 0.0, 0.0))];
        let out = GtsamAnchorGraph::default()
            .optimize(&anchors, &odometry, &loops)
            .expect("optimises");
        assert!(out[2].y.abs() < 0.03, "end anchor corrected: {}", out[2].y);
        assert!(
            out[1].y > 0.02 && out[1].y < 0.18,
            "middle anchor partially corrected: {}",
            out[1].y
        );
    }
}
